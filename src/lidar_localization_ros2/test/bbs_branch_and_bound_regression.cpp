// Bit-exact verification of the C++ BBS port (bbs_branch_and_bound.hpp) against
// the Python reference (scripts/make_bbs_relocalization_attempts.py).
//
// The golden fixtures in bbs_golden_fixtures.inc are the frozen output of the
// Python search (regenerate with scripts/generate_bbs_golden_fixtures.py). This
// test runs the C++ port on the same inputs and asserts every candidate tuple
// (tx, ty, yaw_index, hit_count) matches -- proving the port is bit-exact without
// needing an idle machine (docs/dev/g2_bbs_speedup.md). Pure C++/STL, so:
//
//   g++ -std=c++17 -pthread -I include test/bbs_branch_and_bound_regression.cpp \
//     -o /tmp/bbs_branch_and_bound_regression && /tmp/bbs_branch_and_bound_regression

// Keep regression assertions active in Release builds as well.
#ifdef NDEBUG
#undef NDEBUG
#endif

#include "lidar_localization/bbs_branch_and_bound.hpp"
#include "lidar_localization/bbs_parallel.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "bbs_golden_fixtures.inc"

namespace bbs = lidar_localization::bbs;

namespace {

bbs::Grid grid_from_string(int height, int width, const std::string & flat)
{
  bbs::Grid g(height, width);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      g.set(y, x, flat[static_cast<size_t>(y) * width + x] == '1' ? 1 : 0);
    }
  }
  return g;
}

void check_fixture(const GoldenFixture & gf)
{
  const bbs::Grid occ = grid_from_string(gf.height, gf.width, gf.occ);
  const auto got = bbs::branch_and_bound_candidates(
    occ, gf.scan, gf.resolution_m, gf.ares_rad, gf.depth, gf.max_c, gf.nms);

  if (got.size() != gf.expected.size()) {
    std::fprintf(stderr, "[%s] candidate count %zu != expected %zu\n",
                 gf.name, got.size(), gf.expected.size());
    assert(false);
  }
  for (size_t i = 0; i < got.size(); ++i) {
    const auto & g = got[i];
    const auto & e = gf.expected[i];
    const bool ok = g.tx_cell == e.tx && g.ty_cell == e.ty &&
                    g.yaw_index == e.yaw_index && g.hit_count == e.hit;
    if (!ok) {
      std::fprintf(stderr,
        "[%s] candidate %zu mismatch: got (tx=%d ty=%d yaw=%d hit=%d) "
        "expected (tx=%d ty=%d yaw=%d hit=%d)\n",
        gf.name, i, g.tx_cell, g.ty_cell, g.yaw_index, g.hit_count,
        e.tx, e.ty, e.yaw_index, e.hit);
      assert(false);
    }
    // Score must be the exact integer ratio the Python candidate carried.
    const double expect_score =
      static_cast<double>(e.hit) / static_cast<double>(gf.scan.size());
    assert(g.score == expect_score);
    assert(g.point_count == static_cast<int>(gf.scan.size()));
  }
  std::printf("  [%s] %zu candidates match\n", gf.name, got.size());
}

// A couple of direct invariants on the helpers, independent of the golden.
void test_yaw_samples_count()
{
  // 5 degrees -> ceil(2pi / (5 deg)) = 72 yaw samples, first is 0.
  const auto yaws = bbs::yaw_samples(M_PI / 36.0);
  assert(yaws.size() == 72);
  assert(std::abs(yaws[0]) < 1e-12);
}

void test_upper_bound_level0_is_exact_occupancy()
{
  bbs::Grid occ(4, 4);
  occ.set(1, 1, 1);
  const auto pyr = bbs::build_occupancy_pyramid(occ, 2);
  const auto ub = bbs::build_upper_bound_pyramid(pyr);
  // Level 0 of the upper-bound pyramid must equal the occupancy (no dilation),
  // which is what lets the search score the exact map at level 0.
  assert(ub[0].data == occ.data);
}

// Exercise the zero halo on an odd-sized map with rays extending beyond all
// four edges. Include the computed right/bottom sibling of the last cell.
void test_packed_gather_matches_checked_gather()
{
  const auto occ = grid_from_string(3, 5, "010010100101010");
  const auto bounds = bbs::build_upper_bound_pyramid(bbs::build_occupancy_pyramid(occ, 3));
  const std::vector<std::array<double, 2>> scan = {
    {-6.0, -2.0}, {-3.0, 1.0}, {0.0, 0.0}, {0.0, 0.0}, {5.0, 2.0}, {2.0, 3.0}};
  const auto padded = bbs::detail::pad_bound_grids(bounds, scan);
  assert(padded.size() == bounds.size());
  for (double yaw : bbs::yaw_samples(M_PI / 6.0)) {
    const auto cache = bbs::detail::build_yaw_offset_cache(scan, yaw, padded.size());
    for (size_t l = 0; l < padded.size(); ++l) {
      const auto & g = padded[l];
      std::vector<bbs::detail::LinearOffset> offsets;
      for (const auto & o : cache[l]) offsets.push_back({o.qy * g.stride + o.qx, o.count});
      for (int y = 0; y < bounds[l].height; ++y) {
        for (int x = 0; x < bounds[l].width; ++x) {
          const auto * base = bbs::detail::base_at(g, x << l, y << l, l);
          const auto hits = bbs::detail::gather_children(
            base, offsets);
          for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
              assert(hits[2 * dy + dx] == bbs::detail::gather_hits(
                bounds[l], cache[l], l, (x + dx) << l, (y + dy) << l));
            }
          }
          assert(hits[0] == bbs::detail::gather_padded(base, offsets));
        }
      }
    }
  }
  assert(bbs::detail::pad_bound_grids(bounds, {{{-5000.0, 0.0}}}).empty());
}

void test_run_correlation_matches_checked_gather()
{
  const auto occ = grid_from_string(3, 5, "010010100101010");
  const auto runs = bbs::detail::occupancy_runs(occ);
  // Include negative shifts, clipped intervals, offsets outside the map, and
  // cancellation between differences with a total weight of exactly 65535.
  const std::vector<bbs::OffsetCell> offsets = {
    {0, 0, 30000}, {0, -1, 26000}, {1, 3, 1500}, {-1, -3, 7500},
    {0, -5, 300}, {0, 5, 200}, {-3, 0, 35}};
  const auto counts = bbs::detail::correlate_runs(occ, runs, offsets);
  for (int y = 0; y < occ.height; ++y) {
    for (int x = 0; x < occ.width; ++x) {
      assert(counts[static_cast<size_t>(y) * occ.width + x] ==
        bbs::detail::gather_hits(occ, offsets, 0, x, y));
    }
  }
}

void test_hit_queue_preserves_ties_with_new_higher_bounds()
{
  for (int capacity : {5, 65536}) {  // FIFO buckets and binary-heap fallback
    bbs::detail::HitQueue queue(capacity);
    using Node = bbs::detail::ShardNode;
    Node n{};
    int hit = 0;
    queue.push(2, {1, 0, 1});
    queue.push(2, {2, 0, 1});
    queue.push(1, {3, 0, 1});
    assert(queue.pop(n, hit) && n.tx == 1 && hit == 2);
    queue.push(2, {4, 0, 0});
    queue.push(4, {5, 0, 0});
    assert(queue.pop(n, hit) && n.tx == 5 && hit == 4);
    assert(queue.pop(n, hit) && n.tx == 2 && hit == 2);
    assert(queue.pop(n, hit) && n.tx == 4 && hit == 2);
    assert(queue.pop(n, hit) && n.tx == 3 && hit == 1);
    assert(!queue.pop(n, hit));
  }
}

// With an effectively unlimited shard cap the parallel decomposition must
// reproduce the sequential reference, with one documented exception: the
// reference stops as soon as the heap top ties the k-th score, so among
// candidates tied at that boundary it keeps whichever its global heap happened
// to visit first. The parallel merge visits every leaf, so at the k-th score it
// picks the sort-key-best tie instead. Everything strictly above the k-th score
// is deterministic and must match exactly, as must the score sequence. (The
// production shard cap is small for speed; see bbs_parallel.hpp for the
// same-yaw suppression chain that cap can lose.)
void check_fixture_parallel(const GoldenFixture & gf)
{
  const bbs::Grid occ = grid_from_string(gf.height, gf.width, gf.occ);
  const auto expected = bbs::branch_and_bound_candidates(
    occ, gf.scan, gf.resolution_m, gf.ares_rad, gf.depth, gf.max_c, gf.nms);

  for (int threads : {2, 3, 8}) {
    const auto got = bbs::branch_and_bound_candidates_parallel(
      occ, gf.scan, gf.resolution_m, gf.ares_rad, gf.depth, gf.max_c, gf.nms,
      threads, 1000000);
    if (got.size() != expected.size()) {
      std::fprintf(stderr,
        "[%s] parallel(%d) candidate count %zu != sequential %zu\n",
        gf.name, threads, got.size(), expected.size());
      assert(false);
    }
    const double kth_score = expected.empty() ? 0.0 : expected.back().score;
    for (size_t i = 0; i < got.size(); ++i) {
      assert(got[i].score == expected[i].score);
      assert(got[i].point_count == expected[i].point_count);
      const bool at_boundary = got[i].score == kth_score;
      if (!at_boundary) {
        const bool ok = got[i].tx_cell == expected[i].tx_cell &&
                        got[i].ty_cell == expected[i].ty_cell &&
                        got[i].yaw_index == expected[i].yaw_index &&
                        got[i].hit_count == expected[i].hit_count;
        if (!ok) {
          std::fprintf(stderr,
            "[%s] parallel(%d) candidate %zu above the k-th score mismatch\n",
            gf.name, threads, i);
          assert(false);
        }
      } else {
        assert(got[i].hit_count == expected[i].hit_count);
      }
    }
  }
  std::printf("  [%s] parallel matches sequential above the k-th score\n", gf.name);
}

// A yaw-windowed search must return exactly the in-window prefix of a full search
// deep enough to contain it (nms off, so candidates do not suppress each other).
void test_yaw_window_matches_filtered_full_search(const GoldenFixture & gf)
{
  const bbs::Grid occ = grid_from_string(gf.height, gf.width, gf.occ);
  const double center = 0.3;
  const double half_window = M_PI / 6.0;
  const int k = 4;
  const auto windowed = bbs::branch_and_bound_candidates(
    occ, gf.scan, gf.resolution_m, gf.ares_rad, gf.depth, k, 0, center, half_window);
  const auto full = bbs::branch_and_bound_candidates(
    occ, gf.scan, gf.resolution_m, gf.ares_rad, gf.depth, 100000, 0);
  std::vector<bbs::BbsGridCandidate> expected;
  for (const auto & c : full) {
    if (std::abs(bbs::py_normalize_angle_rad(c.yaw_rad - center)) <= half_window &&
      static_cast<int>(expected.size()) < k)
    {
      expected.push_back(c);
    }
  }
  assert(!expected.empty());
  assert(expected.size() < full.size());  // the window must exclude some yaws
  assert(windowed.size() == expected.size());
  for (size_t i = 0; i < windowed.size(); ++i) {
    assert(windowed[i].tx_cell == expected[i].tx_cell);
    assert(windowed[i].ty_cell == expected[i].ty_cell);
    assert(windowed[i].yaw_index == expected[i].yaw_index);
    assert(windowed[i].hit_count == expected[i].hit_count);
  }
  std::printf("  [%s] yaw window matches the filtered full search\n", gf.name);
}

}  // namespace

int main()
{
  test_yaw_samples_count();
  test_upper_bound_level0_is_exact_occupancy();
  test_packed_gather_matches_checked_gather();
  test_hit_queue_preserves_ties_with_new_higher_bounds();
  test_run_correlation_matches_checked_gather();
  const auto fixtures = golden_fixtures();
  assert(!fixtures.empty());
  for (const auto & gf : fixtures) {
    check_fixture(gf);
    check_fixture_parallel(gf);
  }
  std::printf("all %zu golden fixtures bit-exact\n", fixtures.size());
  test_yaw_window_matches_filtered_full_search(fixtures.back());  // 12 yaws
  return 0;
}
