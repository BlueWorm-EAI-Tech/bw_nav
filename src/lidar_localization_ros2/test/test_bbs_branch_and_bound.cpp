// Bit-exact verification of the C++ BBS port (bbs_branch_and_bound.hpp) against
// the Python reference (scripts/make_bbs_relocalization_attempts.py).
//
// The golden fixtures in bbs_golden_fixtures.inc are the frozen output of the
// Python search (regenerate with scripts/generate_bbs_golden_fixtures.py). This
// test runs the C++ port on the same inputs and asserts every candidate tuple
// (tx, ty, yaw_index, hit_count) matches -- proving the port is bit-exact without
// needing an idle machine (docs/dev/g2_bbs_speedup.md). Pure C++/STL, so:
//
//   g++ -std=c++17 -I include test/test_bbs_branch_and_bound.cpp -o /tmp/t && /tmp/t

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
