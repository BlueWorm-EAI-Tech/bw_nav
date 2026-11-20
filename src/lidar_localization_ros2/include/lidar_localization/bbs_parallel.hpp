#ifndef BBS_PARALLEL_HPP_
#define BBS_PARALLEL_HPP_

// Multi-threaded variant of the BBS_2D branch-and-bound search in
// bbs_branch_and_bound.hpp.
//
// Why: on the real PGO map the reference search spends >99% of its time in the
// single global best-first loop (docs/dev/g2_bbs_speedup.md). Its heap interleaves
// nodes across *all* yaw samples, so the loop is inherently serial. Yaw samples
// do not interact until the very end (each has its own offset cache and scores
// only nodes of its own yaw), so each shard can run the same best-first search
// independently and only the candidate merge needs the global view.
//
// Each shard runs the reference search with a *shard-local* NMS and keeps its
// `local_candidates` best distinct leaves. The shard lists are then merged and
// reduced by one global NMS with the reference radius. This is the same greedy
// suppression rule as the reference, just applied in two stages.
//
// Approximation boundary: a leaf kept by the reference can be suppressed in a
// shard by a higher-scoring same-yaw neighbour that the global NMS then
// suppresses with a third candidate, so a shard-local NMS can lose a candidate
// that the global pass would have kept (the chain case). Keeping several times
// `max_candidates` distinct leaves per yaw makes this rare in practice, and the
// top hypotheses — the ones the startup walk and consensus need — are
// unaffected on the field map. With an effectively unlimited shard cap the
// result matches the sequential reference for every candidate above the k-th
// score on the golden fixtures, and only equal-scoring k-th-boundary ties can
// differ (the reference stops on the first tie its heap visits; the merge picks
// the sort-key-best tie). The BBS regression suite asserts that contract. A single-pass raw-score cap was rejected outright: on the field map
// each yaw's top leaves cluster around the same few places, so the global NMS
// collapsed the merged list to one or two hypotheses.
//
// This header is pure C++/STL like the reference; `num_threads <= 1` delegates
// to branch_and_bound_candidates() so disabling the parallel path is exact.

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <exception>
#include <limits>
#include <map>
#include <mutex>
#include <queue>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "lidar_localization/bbs_branch_and_bound.hpp"

namespace lidar_localization {
namespace bbs {

namespace detail {

// Mirrors the per-yaw offset-cache construction in branch_and_bound_candidates.
// Dedup order does not affect the gathered integer sum.
inline std::vector<std::vector<OffsetCell>> build_yaw_offset_cache(
  const std::vector<std::array<double, 2>> & scan_grid,
  double yaw_rad,
  int n_levels)
{
  const int n_points = static_cast<int>(scan_grid.size());
  const double c = std::cos(yaw_rad);
  const double s = std::sin(yaw_rad);
  std::vector<double> rx(n_points), ry(n_points);
  for (int p = 0; p < n_points; ++p) {
    rx[p] = c * scan_grid[p][0] - s * scan_grid[p][1];
    ry[p] = s * scan_grid[p][0] + c * scan_grid[p][1];
  }

  std::vector<std::vector<OffsetCell>> cache(n_levels);
  for (int level = 0; level < n_levels; ++level) {
    const double factor = static_cast<double>(1 << level);
    std::map<std::pair<int, int>, int> dedup;  // (qy, qx) -> count
    for (int p = 0; p < n_points; ++p) {
      const int qx = static_cast<int>(std::floor((0.5 + rx[p]) / factor));
      const int qy = static_cast<int>(std::floor((0.5 + ry[p]) / factor));
      ++dedup[{qy, qx}];
    }
    auto & cells = cache[level];
    cells.reserve(dedup.size());
    for (const auto & kv : dedup) {
      cells.push_back({kv.first.first, kv.first.second, kv.second});
    }
  }
  return cache;
}

// A zero halo turns out-of-map samples into regular loads. All yaws share
// one byte per cell: a 2x2 mask whose low bit is the original cell.
struct PaddedGrid
{
  int stride = 0;
  int pad = 0;
  std::vector<uint8_t> data;  // 2x2 child mask; least significant bit is the original cell
};

inline std::vector<PaddedGrid> pad_bound_grids(
  const std::vector<Grid> & bounds, const std::vector<std::array<double, 2>> & scan_grid)
{
  double radius = 0.0;
  for (const auto & p : scan_grid) {
    const double length = std::hypot(p[0], p[1]);
    if (!std::isfinite(length) || length > 4096) return {};
    radius = std::max(radius, length);
  }
  std::vector<PaddedGrid> result;
  result.reserve(bounds.size());
  // Bound shared scratch space, not per-thread memory. The checked path
  // remains available when long rays or still larger maps exceed this budget.
  const bool large_grid = !bounds.empty() && bounds[0].data.size() >= (1 << 20);
  int64_t remaining_cells = (large_grid ? 192 : 8) * 1024 * 1024;
  for (size_t level = 0; level < bounds.size(); ++level) {
    const auto & g = bounds[level];
    if (g.width <= 0 || g.height <= 0) return {};
    // Two extra cells cover the floor(0.5 + offset) convention and the
    // right/bottom sibling load, even when that sibling will be discarded.
    const int pad = static_cast<int>(std::ceil(radius / (1 << level))) + 2;
    const int64_t w = static_cast<int64_t>(g.width) + 2 * pad;
    const int64_t h = static_cast<int64_t>(g.height) + 2 * pad;
    if (h <= 0 || w > remaining_cells / h) return {};
    remaining_cells -= w * h;
    PaddedGrid pg{static_cast<int>(w), pad, std::vector<uint8_t>(w * h)};
    for (int y = -1; y < g.height; ++y) {
      const auto * upper = y >= 0 ? g.data.data() + static_cast<size_t>(y) * g.width : nullptr;
      const auto * lower = y + 1 < g.height ?
        g.data.data() + static_cast<size_t>(y + 1) * g.width : nullptr;
      const size_t row = static_cast<size_t>(y + pad) * pg.stride + pad;
      for (int x = -1; x < g.width; ++x) {
        const bool left = x >= 0;
        const bool right = x + 1 < g.width;
        pg.data[row + x] = (upper && left && upper[x]) | ((upper && right && upper[x + 1]) << 1) |
          ((lower && left && lower[x]) << 2) | ((lower && right && lower[x + 1]) << 3);
      }
    }
    result.push_back(std::move(pg));
  }
  return result;
}

// Half-open intervals of occupied cells; only used on the coarse levels.
using Runs = std::vector<std::vector<std::array<int, 2>>>;

inline Runs occupancy_runs(const Grid & g)
{
  Runs rows(g.height);
  for (int y = 0; y < g.height; ++y) {
    for (int x = 0; x < g.width;) {
      if (!g.at(y, x)) {++x; continue;}
      const int begin = x++;
      while (x < g.width && g.at(y, x)) ++x;
      rows[y].push_back({begin, x});
    }
  }
  return rows;
}

inline std::vector<uint16_t> correlate_runs(
  const Grid & g, const Runs & runs, const std::vector<OffsetCell> & offsets)
{
  // Shift each occupied interval by the negative scan offset. Difference
  // and prefix sums can wrap mod 2^16: the final count never exceeds n_points.
  std::vector<uint16_t> hits(g.data.size());
  std::vector<uint16_t> delta(g.width + 1);
  for (int y = 0; y < g.height; ++y) {
    std::fill(delta.begin(), delta.end(), 0);
    for (const auto & o : offsets) {
      const int sy = y + o.qy;
      if (sy < 0 || sy >= g.height) continue;
      if (o.qx <= -g.width || o.qx >= g.width) continue;
      const int lo = std::max(0, o.qx);
      const int hi = std::min(g.width, o.qx + g.width);
      if (lo >= hi) continue;
      for (const auto & run : runs[sy]) {
        if (run[1] <= lo) continue;
        if (run[0] >= hi) break;
        delta[std::max(lo, run[0]) - o.qx] += o.count;
        delta[std::min(hi, run[1]) - o.qx] -= o.count;
      }
    }
    uint16_t prefix = 0;
    auto * dst = hits.data() + static_cast<size_t>(y) * g.width;
    for (int x = 0; x < g.width; ++x) {
      prefix += delta[x];
      dst[x] = prefix;
    }
  }
  return hits;
}

struct LinearOffset { int index, count; };

inline const uint8_t * base_at(const PaddedGrid & g, int tx, int ty, int level)
{
  return g.data.data() + ((ty >> level) + g.pad) * g.stride + (tx >> level) + g.pad;
}

inline int gather_padded(const uint8_t * base, const std::vector<LinearOffset> & offsets)
{
  int hit = 0;
  for (const auto & o : offsets) hit += (base[o.index] & 1) * o.count;
  return hit;
}

// Four 16-bit hit counts in one 64-bit accumulator, provided the scan
// has <= 65535 points. The 2x2 occupancy pattern is shared by all yaws.
inline std::array<int, 4> gather_children(
  const uint8_t * base, const std::vector<LinearOffset> & offsets)
{
  static constexpr auto lanes = [] {
    std::array<uint64_t, 16> values{};
    for (int mask = 0; mask < 16; ++mask) {
      for (int i = 0; i < 4; ++i) values[mask] |= uint64_t((mask >> i) & 1) << (16 * i);
    }
    return values;
  }();
  uint64_t hits = 0;
  for (const auto & o : offsets) hits += lanes[base[o.index]] * o.count;
  return {int(hits & 0xffff), int((hits >> 16) & 0xffff),
    int((hits >> 32) & 0xffff), int(hits >> 48)};
}

// Exact priority: descending integer hit count, then FIFO within a tie.
// This mirrors the old (-double(hit)/n_points, sequence) heap ordering.
struct ShardNode { int tx, ty, level; };
class HitQueue
{
public:
  explicit HitQueue(int max_hits)
  : buckets_(max_hits <= 4096 ? static_cast<size_t>(max_hits) + 1 : 0) {}

  void push(int hits, ShardNode n)
  {
    if (buckets_.empty()) {
      heap_.push({-static_cast<double>(hits), sequence_++, n.level, n.tx, n.ty, 0});
      return;
    }
    auto & bucket = buckets_[hits];
    // Reclaim a consumed prefix before growing, without disturbing tie order.
    if (bucket.nodes.size() == bucket.nodes.capacity() &&
      bucket.next > 0 && bucket.next >= bucket.nodes.size() / 2)
    {
      bucket.nodes.erase(bucket.nodes.begin(), bucket.nodes.begin() + bucket.next);
      bucket.next = 0;
    }
    bucket.nodes.push_back(n);
    highest_ = std::max(highest_, hits);
  }

  bool pop(ShardNode & n, int & hits)
  {
    if (buckets_.empty()) {
      if (heap_.empty()) return false;
      const auto node = heap_.top();
      heap_.pop();
      n = {node.tx_cell, node.ty_cell, node.level};
      hits = static_cast<int>(-node.neg_bound);
      return true;
    }
    while (highest_ >= 0) {
      auto & bucket = buckets_[highest_];
      if (bucket.next < bucket.nodes.size()) {
        n = bucket.nodes[bucket.next++];
        hits = highest_;
        if (bucket.next == bucket.nodes.size()) {
          bucket.nodes.clear();
          bucket.next = 0;
        }
        return true;
      }
      --highest_;
    }
    return false;
  }
private:
  struct Bucket { std::vector<ShardNode> nodes; size_t next = 0; };
  std::vector<Bucket> buckets_;
  int highest_ = -1;
  // Scans much larger than the runtime point cap avoid O(n_points) buckets.
  std::priority_queue<HeapNode, std::vector<HeapNode>, HeapCmp> heap_;
  long sequence_ = 0;
};

// One yaw's slice of the reference search: identical best-first order and
// pruning, but the leaf list is NMS-filtered within the shard and capped.
inline std::vector<BbsGridCandidate> search_single_yaw(
  const std::vector<Grid> & ub,
  const std::vector<PaddedGrid> & padded,
  const std::vector<Runs> & runs,
  const std::vector<std::array<double, 2>> & scan_grid,
  const std::vector<double> & yaws,
  int yaw_index,
  int start_level,
  int nms_radius_cells,
  int local_candidates)
{
  const int height = ub[0].height;
  const int width = ub[0].width;
  const int n_points = static_cast<int>(scan_grid.size());
  const int n_levels = static_cast<int>(ub.size());
  const auto cache = build_yaw_offset_cache(scan_grid, yaws[yaw_index], n_levels);

  std::vector<std::vector<uint16_t>> hit_maps(ub.size());
  for (size_t l = 0; l < runs.size(); ++l) {
    if (!runs[l].empty()) hit_maps[l] = correlate_runs(ub[l], runs[l], cache[l]);
  }
  std::vector<std::vector<LinearOffset>> linear(padded.size());
  for (size_t level = 0; level < padded.size(); ++level) {
    linear[level].reserve(cache[level].size());
    for (const auto & o : cache[level]) {
      linear[level].push_back({o.qy * padded[level].stride + o.qx, o.count});
    }
  }
  auto gather = [&](int level, int tx, int ty) {
    if (!hit_maps[level].empty()) {
      return int(hit_maps[level][size_t(ty >> level) * ub[level].width + (tx >> level)]);
    }
    return padded.empty() ? gather_hits(ub[level], cache[level], level, tx, ty) :
      gather_padded(base_at(padded[level], tx, ty, level), linear[level]);
  };
  const int initial_step = 1 << start_level;
  HitQueue queue(n_points);
  for (int ty = 0; ty < height; ty += initial_step) {
    for (int tx = 0; tx < width; tx += initial_step) {
      queue.push(gather(start_level, tx, ty), {tx, ty, start_level});
    }
  }

  std::vector<BbsGridCandidate> best;
  best.reserve(std::min(local_candidates, 4096));
  double kth_score = -std::numeric_limits<double>::infinity();
  const double eps = 1.0e-12;

  ShardNode node;
  int hit = 0;
  while (queue.pop(node, hit)) {
    const double bound = static_cast<double>(hit) / static_cast<double>(n_points);
    if (static_cast<int>(best.size()) >= local_candidates && bound <= kth_score + eps)
      break;

    if (node.level == 0) {
      BbsGridCandidate cand;
      cand.tx_cell = node.tx;
      cand.ty_cell = node.ty;
      cand.yaw_index = yaw_index;
      cand.yaw_rad = yaws[yaw_index];
      cand.score = bound;
      cand.hit_count = hit;
      cand.point_count = n_points;

      if (nms_radius_cells > 0) {
        bool suppressed = false;
        for (size_t i = 0; i < best.size(); ++i) {
          if (std::abs(best[i].tx_cell - cand.tx_cell) <= nms_radius_cells &&
            std::abs(best[i].ty_cell - cand.ty_cell) <= nms_radius_cells)
          {
            if (cand.score > best[i].score) best[i] = cand;
            suppressed = true;
            break;
          }
        }
        if (suppressed) continue;
      }
      best.push_back(cand);
      std::stable_sort(best.begin(), best.end(), candidate_less);
      if (static_cast<int>(best.size()) > local_candidates) best.resize(local_candidates);
      if (static_cast<int>(best.size()) >= local_candidates) kth_score = best.back().score;
      continue;
    }

    const int child_level = node.level - 1;
    const int child_step = 1 << child_level;
    std::array<int, 4> hits{};
    if (!padded.empty() && hit_maps[child_level].empty()) {
      const auto & g = padded[child_level];
      const auto index = base_at(g, node.tx, node.ty, child_level) - g.data.data();
      hits = gather_children(g.data.data() + index, linear[child_level]);
    }
    for (int dy = 0; dy < 2; ++dy) {
      const int child_ty = node.ty + dy * child_step;
      if (child_ty >= height) continue;
      for (int dx = 0; dx < 2; ++dx) {
        const int child_tx = node.tx + dx * child_step;
        if (child_tx >= width) continue;
        const int child_hit = padded.empty() || !hit_maps[child_level].empty() ?
          gather(child_level, child_tx, child_ty) : hits[2 * dy + dx];
        const double child_bound = static_cast<double>(child_hit) / static_cast<double>(n_points);
        if (static_cast<int>(best.size()) >= local_candidates && child_bound < kth_score - eps) {
          continue;
        }
        queue.push(child_hit, {child_tx, child_ty, child_level});
      }
    }
  }
  return best;
}

// Sort every shard's leaves by the reference key, then apply the reference's
// greedy (tx, ty)-neighbourhood NMS once over the whole result.
inline std::vector<BbsGridCandidate> merge_and_nms(
  std::vector<std::vector<BbsGridCandidate>> shards,
  int max_candidates,
  int nms_radius_cells)
{
  std::vector<BbsGridCandidate> merged;
  size_t total = 0;
  for (const auto & shard : shards) total += shard.size();
  merged.reserve(total);
  for (auto & shard : shards) {
    for (auto & cand : shard) merged.push_back(cand);
  }
  std::stable_sort(merged.begin(), merged.end(), candidate_less);

  std::vector<BbsGridCandidate> best;
  for (const auto & cand : merged) {
    if (nms_radius_cells > 0) {
      bool suppressed = false;
      for (size_t i = 0; i < best.size(); ++i) {
        if (std::abs(best[i].tx_cell - cand.tx_cell) <= nms_radius_cells &&
          std::abs(best[i].ty_cell - cand.ty_cell) <= nms_radius_cells)
        {
          if (cand.score > best[i].score) best[i] = cand;
          suppressed = true;
          break;
        }
      }
      if (suppressed) continue;
    }
    best.push_back(cand);
  }
  std::stable_sort(best.begin(), best.end(), candidate_less);
  if (static_cast<int>(best.size()) > max_candidates) best.resize(max_candidates);
  return best;
}

}  // namespace detail

// Parallel counterpart of branch_and_bound_candidates() with the same yaw
// window semantics. num_threads 0 or 1 (or a single searched yaw) delegates to
// the sequential reference; num_threads < 0 auto-detects the core count (all
// but one, so localization leaves a core for FAST-LIO/NDT).
// local_candidates <= 0 uses max(2 * max_candidates, 16) distinct leaves per
// yaw for the merge.
inline std::vector<BbsGridCandidate> branch_and_bound_candidates_parallel(
  const Grid & occupancy,
  const std::vector<std::array<double, 2>> & scan_xy_m,
  double resolution_m,
  double angular_resolution_rad,
  int pyramid_depth,
  int max_candidates,
  int nms_radius_cells = 0,
  int num_threads = 0,
  int local_candidates = 0,
  double yaw_center_rad = 0.0,
  double yaw_half_window_rad = M_PI)
{
  if (max_candidates <= 0) return {};
  if (resolution_m <= 0.0) throw std::invalid_argument("resolution_m must be positive");
  if (angular_resolution_rad <= 0.0)
    throw std::invalid_argument("angular_resolution_rad must be positive");
  if (scan_xy_m.empty()) return {};

  std::vector<std::array<double, 2>> scan_grid(scan_xy_m.size());
  for (size_t i = 0; i < scan_xy_m.size(); ++i) {
    scan_grid[i] = {scan_xy_m[i][0] / resolution_m, scan_xy_m[i][1] / resolution_m};
  }

  const std::vector<Grid> pyramid = build_occupancy_pyramid(occupancy, pyramid_depth);
  const std::vector<Grid> ub = build_upper_bound_pyramid(pyramid);
  const int start_level = static_cast<int>(ub.size()) - 1;
  const std::vector<double> yaws = yaw_samples(angular_resolution_rad);
  const int n_yaws = static_cast<int>(yaws.size());

  std::vector<int> searched_yaws;
  searched_yaws.reserve(n_yaws);
  for (int yi = 0; yi < n_yaws; ++yi) {
    if (yaw_half_window_rad >= M_PI ||
      std::abs(py_normalize_angle_rad(yaws[yi] - yaw_center_rad)) <= yaw_half_window_rad)
    {
      searched_yaws.push_back(yi);
    }
  }

  int threads = num_threads;
  if (threads < 0) {
    threads = static_cast<int>(std::thread::hardware_concurrency()) - 1;
    if (threads < 1) threads = 1;
  }
  if (threads <= 1 || searched_yaws.size() <= 1) {
    return branch_and_bound_candidates(
      occupancy, scan_xy_m, resolution_m, angular_resolution_rad, pyramid_depth,
      max_candidates, nms_radius_cells, yaw_center_rad, yaw_half_window_rad);
  }
  threads = std::min(threads, static_cast<int>(searched_yaws.size()));

  if (local_candidates <= 0) {
    local_candidates = std::max(2 * max_candidates, 16);
  }
  local_candidates = std::max(local_candidates, max_candidates);

  // Packed lanes must never carry into one another. Oversized scans and maps
  // still use the same search with ordinary, bounds-checked gathering.
  const auto padded = scan_grid.size() <= std::numeric_limits<uint16_t>::max() ?
    detail::pad_bound_grids(ub, scan_grid) : std::vector<detail::PaddedGrid>{};
  std::vector<detail::Runs> runs(ub.size());
  if (!padded.empty() && occupancy.data.size() >= (1 << 20)) {
    // Cap per-yaw hit-map scratch, and avoid row correlation on highly
    // fragmented grids where walking many runs per offset costs more.
    for (size_t l = 2; l < ub.size(); ++l) {
      const auto & g = ub[l];
      if (g.data.size() > (1 << 20)) continue;
      auto rows = detail::occupancy_runs(g);
      size_t count = 0;
      for (const auto & row : rows) count += row.size();
      if (count <= g.data.size() / 8) runs[l] = std::move(rows);
    }
  }
  std::vector<std::vector<BbsGridCandidate>> shards(searched_yaws.size());
  std::atomic<size_t> next_task{0};
  std::exception_ptr worker_error;
  std::mutex error_mutex;

  auto worker = [&]() {
    try {
      for (;;) {
        const size_t task = next_task.fetch_add(1);
        if (task >= searched_yaws.size()) return;
        shards[task] = detail::search_single_yaw(
          ub, padded, runs, scan_grid, yaws, searched_yaws[task], start_level,
          nms_radius_cells, local_candidates);
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(error_mutex);
      if (!worker_error) worker_error = std::current_exception();
      next_task.store(searched_yaws.size());
    }
  };

  std::vector<std::thread> pool;
  pool.reserve(static_cast<size_t>(threads));
  for (int t = 0; t < threads; ++t) pool.emplace_back(worker);
  for (auto & thread : pool) thread.join();
  if (worker_error) std::rethrow_exception(worker_error);

  auto merged = detail::merge_and_nms(
    std::move(shards), max_candidates, nms_radius_cells);
  if (static_cast<int>(merged.size()) < max_candidates) {
    // Degenerate map where every hypothesis sits inside one NMS neighbourhood:
    // the sequential search is cheap here, and exact.
    return branch_and_bound_candidates(
      occupancy, scan_xy_m, resolution_m, angular_resolution_rad, pyramid_depth,
      max_candidates, nms_radius_cells, yaw_center_rad, yaw_half_window_rad);
  }
  return merged;
}

}  // namespace bbs
}  // namespace lidar_localization

#endif  // BBS_PARALLEL_HPP_
