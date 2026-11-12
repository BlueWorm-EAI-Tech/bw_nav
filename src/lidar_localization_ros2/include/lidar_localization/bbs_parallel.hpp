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
// the sort-key-best tie). test/test_bbs_branch_and_bound.cpp asserts that
// contract. A single-pass raw-score cap was rejected outright: on the field map
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

// One yaw's slice of the reference search: identical best-first order and
// pruning, but the leaf list is NMS-filtered within the shard and capped.
inline std::vector<BbsGridCandidate> search_single_yaw(
  const std::vector<Grid> & ub,
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

  const int initial_step = 1 << start_level;
  std::priority_queue<HeapNode, std::vector<HeapNode>, HeapCmp> heap;
  long sequence = 0;
  const auto & seed_offsets = cache[start_level];
  for (int ty = 0; ty < height; ty += initial_step) {
    for (int tx = 0; tx < width; tx += initial_step) {
      const int hit = gather_hits(ub[start_level], seed_offsets, start_level, tx, ty);
      const double bound = static_cast<double>(hit) / static_cast<double>(n_points);
      heap.push({-bound, sequence, start_level, tx, ty, yaw_index});
      ++sequence;
    }
  }

  std::vector<BbsGridCandidate> best;
  best.reserve(std::min(local_candidates, 4096));
  double kth_score = -std::numeric_limits<double>::infinity();
  const double eps = 1.0e-12;

  while (!heap.empty()) {
    const HeapNode node = heap.top();
    heap.pop();
    const double bound = -node.neg_bound;
    if (static_cast<int>(best.size()) >= local_candidates && bound <= kth_score + eps)
      break;

    if (node.level == 0) {
      const int hit = gather_hits(
        ub[0], cache[0], 0, node.tx_cell, node.ty_cell);
      BbsGridCandidate cand;
      cand.tx_cell = node.tx_cell;
      cand.ty_cell = node.ty_cell;
      cand.yaw_index = yaw_index;
      cand.yaw_rad = yaws[yaw_index];
      cand.score = static_cast<double>(hit) / static_cast<double>(n_points);
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
    for (int dy = 0; dy <= child_step; dy += child_step) {
      const int child_ty = node.ty_cell + dy;
      if (child_ty >= height) continue;
      for (int dx = 0; dx <= child_step; dx += child_step) {
        const int child_tx = node.tx_cell + dx;
        if (child_tx >= width) continue;
        const int hit = gather_hits(
          ub[child_level], cache[child_level], child_level, child_tx, child_ty);
        const double child_bound = static_cast<double>(hit) / static_cast<double>(n_points);
        if (static_cast<int>(best.size()) >= local_candidates &&
          child_bound < kth_score - eps)
        {
          continue;
        }
        heap.push({-child_bound, sequence, child_level, child_tx, child_ty, yaw_index});
        ++sequence;
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
          ub, scan_grid, yaws, searched_yaws[task], start_level,
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
