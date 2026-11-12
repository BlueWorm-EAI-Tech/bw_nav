# G2 BBS query speedup: profiling and the C++ port plan

> **Note:** this is a development record. Most scripts, tests and experiment
> directories it names were removed with the offline tooling in 6825f05; run them
> from `git checkout 6825f05^`. The current benchmark is [benchmark.md](../benchmark.md).

The G3 live closed-loop work ([g3_live_closed_loop.md](g3_live_closed_loop.md))
isolated **query latency** as the dominant blocker to live recovery: a BBS query
takes ~5–23 s, so the candidate it returns is stale by the time the supervisor
publishes it. [Seed motion compensation](g3_live_closed_loop.md) (lever 2)
mitigates this from the consumer side; the root fix is to make the query itself
fast. This page records what actually costs the time and the resulting port plan,
so the implementation does not start from guesswork.

## Profiling the real map (Koide outdoor_hard)

`branch_and_bound_candidates` was profiled on the real occupancy map
(`outdoor_hard_occupancy`, 1439×1063 cells at 0.2 m, 581 306 occupied), a 512-point
scan, `angular_resolution 5°`, `pyramid_depth 4`, `max_candidates 16`. The machine
was loaded, so absolute times are inflated (~2× the idle ~8 s baseline noted in the
development plan); the **relative breakdown is the load-independent finding**:

| Component | cumulative share | calls | what it is |
| --- | --- | --- | --- |
| `gather_hits` (cold path) | **~61 %** | **250 554** | per-node integer gather over the rotated-scan offset cells, in numpy |
| `full_hit_map` (FFT) | ~18 % | 223 | one-shot FFT cross-correlation for hot (yaw, level) pairs |
| heap `heappop`/`heappush` | ~11 % | 564 k / 2 247 k | the best-first branch-and-bound queue |

The decisive correction to the development plan's earlier note ("the remaining
floor is the Python heap loop"): the heap loop is only ~11 %. The real floor is the
**250 k `gather_hits` calls**, each paying Python+numpy per-call overhead (array
alloc, `np.any`, fancy-index, `sum`) on a *tiny* array of offset cells. It is
interpreter/call overhead, not arithmetic.

## What does NOT work: tuning the FFT-switch threshold

`gather_hits` already switches a hot (yaw, level) pair from per-node gather to a
single FFT hit map once it has been queried `max(64, H·W/256)` times. The obvious
cheap hypothesis was: lower that threshold so more pairs use the fast FFT map and
fewer pay the 250 k cold gathers. **Tested and refuted** — every threshold gives
the same wall time within noise, and raising the FFT usage too far makes it
*worse* (the large-grid FFTs cost more than the gathers they replace):

| FFT-switch threshold | wall (loaded) | candidates identical? |
| --- | --- | --- |
| default `max(64, H·W/256)` | 18.2 s | — |
| 2048 | 25.6 s | yes |
| 1024 | 21.8 s | yes |
| 512 | 20.2 s | yes |
| 256 | 17.9 s | yes |
| 128 | 18.7 s | yes |
| 64 | 18.2 s | yes |

Two things are confirmed: the adaptive threshold is already near-optimal (no free
Python win), and the FFT path is **bit-exact** to the direct gather (every row's
candidates are identical — the integer-rounding argument holds). There is no cheap
algorithmic shortcut left in Python; sub-second requires leaving the interpreter.

## The port plan, and why it is simpler than it looks

The key insight from the profile: **the FFT path exists only to work around
Python's slow per-cell gather.** In C++ the direct gather is a tight loop over a
few hundred integer offsets with no per-call overhead — fast enough that the FFT
machinery is not needed at all. So the C++ port is *smaller* than the Python
module, not larger:

Port to C++ (pure, no external FFT dependency):

1. `build_occupancy_pyramid` + `build_upper_bound_pyramid` (max-pool + 1-cell dilate)
2. the per-(yaw, level) offset cache (rotate scan, floor to cells, dedup with counts)
3. the best-first heap search with k-th-best pruning and optional NMS
4. the direct integer `gather_hits` (no FFT branch)

Keep out of the port: the FFT `full_hit_map` (dropped — unneeded in C++), and all
map loading / CSV / ROS plumbing (stays in Python; the C++ core takes plain arrays).

Bit-exactness strategy: the C++ core computes the *same exact integer hit counts*
as Python, so the candidate list must be identical. This is verifiable without an
idle machine — run the existing `test_bbs_relocalization_attempts` fixtures and a
few medium synthetic maps through both and assert identical `(tx, ty, yaw, hit)`
tuples. Only the final wall-clock *speed* claim needs an idle machine.

Integration: expose the core via a small pybind11 module built by the existing
ament/CMake setup (PCL/Eigen already linked), imported by
`global_localization_node.py` with a Python fallback to the current implementation
when the extension is absent — same opt-in, no behavior change by default.

Expected payoff: removing the 61 % gather overhead and the 11 % Python heap loop
(72 % of the time) by moving them to compiled code should bring the idle ~8 s
query toward the sub-second target the roadmap's G1 optimization note calls for,
which — combined with lever 2 motion compensation — closes the live-recovery
latency gap.

## Status

- Profiling and the threshold refutation: **done** (this page).
- C++ core implementation + bit-exact fixture verification: **done**.
  `include/lidar_localization/bbs_branch_and_bound.hpp` is the pure-C++ port
  (pyramid + offset cache + heap + direct gather, no FFT). It is verified
  bit-exact against the Python reference by `test/test_bbs_branch_and_bound.cpp`,
  which replays four golden fixtures (single-yaw, multi-yaw, deep-pyramid, and
  NMS) frozen from the Python search by `scripts/generate_bbs_golden_fixtures.py`
  — every `(tx, ty, yaw_index, hit_count)` tuple matches.
- Directional speed (same machine, same load, same input — the **real**
  `outdoor_hard` map, 512-pt scan, 5° yaw, depth 4, NMS 8): the C++ core ran the
  query in **2.6 s vs the Python search's 14.6 s — ~5.5×** — and returned the
  identical top candidate (hit 499 at cell (397, 175), yaw 8), confirming
  bit-exactness on the full map too. The machine was loaded (the development
  plan's idle Python baseline is ~8 s), so this ratio — not the absolute time —
  is the load-independent result. Scaling the idle baseline by ~5.5× puts the
  ported query near ~1.5 s, and with a coarser yaw step (which the plan notes
  roughly halves the Python query) into the sub-second range that — combined with
  lever-2 seed motion compensation — closes the live-recovery latency gap.
- pybind11 runtime wiring into `global_localization_node.py` (opt-in, Python
  fallback): **done**. `src/bbs_pybind.cpp` exposes the C++ core as the Python
  module `bbs_cpp` (numpy in, candidate objects out with the same fields as the
  Python dataclass). `GlobalLocalizationEngine` resolves the backend once: with
  `use_cpp_backend=true` it imports `bbs_cpp` and uses it, and on `ImportError`
  it logs once and falls back to the pure-Python search — so a stock build with
  the default `use_cpp_backend=false` is unchanged. CMake gates the module behind
  `find_package(pybind11 QUIET)` and installs it next to the node in
  `lib/${PROJECT_NAME}`; when pybind11 is absent the module is simply not built.
  `test/test_bbs_cpp_backend_parity.py` re-checks the binding's numpy↔struct
  plumbing against the Python reference on the four fixture shapes (and skips
  cleanly when the module is not present).
- Wall-clock benchmark of the **wired runtime path** (through `bbs_cpp`, on the
  dilated matching grid the node actually searches, real `outdoor_hard_bbs` map,
  512-pt scan, 5° yaw, depth 4, NMS 10 cells): the C++ backend ran the query in
  **~8.3 s vs the Python backend's ~38 s — ~4.6×**, returning the identical top
  candidate (cell (718, 530), hit 512/512). The Python reps ran first at lower
  load and the C++ reps under the load climbing toward the gate, so 4.6× is a
  **conservative lower bound** (the C++ side was penalised), consistent with the
  earlier 5.5× core microbenchmark. Reproduce with
  `scripts/benchmark_bbs_backends.py` (records per-rep load; flags any rep at
  load ≥ 5 as invalid). Note: the run that produced these numbers crossed the
  load gate partway through, so the absolute seconds are approximate and a
  quiet-machine re-run is still worthwhile; the 4.6× ratio is the
  load-independent takeaway.
- Important honest correction to the earlier projection: the **runtime** query is
  **not** sub-second at the default settings. The 2.6 s figure above is the C++
  *core* on the **raw** occupancy; the node searches the **dilated** grid
  (`dilate_cells=1`, ~3× more occupied cells), which raises every upper bound and
  weakens pruning, so the real query is several seconds even in C++. Sub-second
  for live recovery therefore needs the search-cost levers, not the port alone:
  coarser yaw (`g2_angular_resolution_deg`), fewer scan points
  (`g2_max_scan_points`), and/or less dilation — all already exposed as launch
  args. `scripts/benchmark_bbs_backends.py` also times the 10° / 256-pt C++
  variants to quantify those levers.

## Parallel yaw shards (multi-threaded BBS)

The C++ port made the per-node gather cheap but left the search in one global
best-first loop, which was measured at **>99 % of the query time** on the real
PGO map (main loop ~6.86 s of a ~6.88 s search; pyramid + upper bound + offset
cache + seeding together were ~25 ms). That loop is serial because its heap
interleaves all yaw samples. Yaw samples, however, are independent: each has its
own offset cache and scores only cells of its own rotation. `bbs_parallel.hpp`
therefore splits the yaw samples across `std::thread` workers, runs the same
best-first search inside each shard, and merges the shard candidates once.

Each shard keeps its `local_candidates` best *distinct* leaves (default
`max(2 * max_candidates, 16)`), then the merged list is reduced by one global
NMS with the reference's greedy `(tx, ty)` neighbourhood rule. Applying NMS in
two stages is an approximation: a leaf kept by the reference can be suppressed
in a shard by a higher-scoring same-yaw neighbour that the global NMS then
suppresses with a third candidate, so the chain survivor is lost. Keeping
several times `max_candidates` distinct leaves per yaw makes this rare, and the
top hypotheses used by the startup walk were unaffected on the field map. With
an effectively unlimited cap the parallel result matches the sequential
reference for every candidate strictly above the k-th score; at the k-th score
itself the reference stops on the first tie its heap visits while the merge
picks the sort-key-best tie. The golden-fixture test asserts exactly that
boundary. A single-pass raw-score cap (no shard NMS) was rejected: on the field
map each yaw's top leaves cluster around the same few places, so the global NMS
collapsed the merged list to one or two hypotheses.

Runtime wiring: `GlobalLocalizationConfig.bbs_num_threads` /
`bbs_local_candidates`, declared by `global_localization_node.py` and exposed as
`g2_bbs_num_threads` / `g2_bbs_local_candidates` in the launch files. `0` keeps
the sequential backend, `<0` uses all but one core, `>0` sets the exact count.
The deployment launch (`mid360_nav2_localization.launch.py`) defaults to `-1`;
the generic quickstart/recovery launches default to `0` so their behavior is
unchanged unless opted in. If the installed `bbs_cpp` predates the parallel entry
point, the engine keeps the exact sequential C++ search and reports the reason in
`backend_error` instead of failing startup.

Measured on the production grid used by the field PGO map (381x601 cells,
1024-point scan, 36 yaw samples at 10°, NMS 40 cells, 16-core host): the
sequential core took ~6.9 s; the parallel search took ~1.3 s with the default
16-leaf shard cap and distinct top candidates (top-1..3 identical to the
sequential reference), about **5x**. At 5° (72 shards) the parallel search is
what makes the half-degree phase fix affordable: replaying all 109
mapping-session scans through the full production chain (BBS → NDT → startup
gates → `decide_startup`), parallel queries averaged **2.95 s** (max 4.27 s)
versus **9.70 s** for the sequential backend at the same settings (~3.3x), and
**104/109 frames** published a pose (mean 2.75 attempts; published error mean
0.046 m / 0.44°, max 0.13 m / 2.2°). Frames 64 and 72, which alias at 10°,
recover in 2 and 4 attempts. The five failures (39, 49, 70, 88, 101) exhaust 8
attempts in the sequential backend too: every query's top-1 is within
0.09 m / 2.2° of ground truth, but `decide_startup` keeps returning
`ambiguous_candidate_retry` because `min_score_margin=0.03` is never met and
`max_registration_fitness_ratio=0` disables the NDT-distinctiveness escape.
That is the decision-policy issue analysed separately, not candidate loss from
the parallel merge.

## Reproduce

Bit-exactness (committed, machine-independent):

```bash
python3 scripts/generate_bbs_golden_fixtures.py   # refresh the golden from Python
g++ -std=c++17 -pthread -I include -I test \
    test/test_bbs_branch_and_bound.cpp -o /tmp/t && /tmp/t
```

Runtime C++ backend (opt-in; needs pybind11 at build time):

```bash
colcon build --packages-select lidar_localization   # builds bbs_cpp if pybind11 is found
# then run the node with the backend enabled:
ros2 run lidar_localization global_localization_node.py \
    --ros-args -p occupancy_yaml:=<map>.yaml -p use_cpp_backend:=true
# the startup log and each query summary report backend=cpp (or python on fallback)
```

The profiling/threshold/timing harness is ad-hoc (`/tmp`); the inputs are the
committed `outdoor_hard_occupancy` map and a seeded 512-point scan sampled from
occupied cells within 30 m of the map-centre pose. Re-derive the profile with
`cProfile` over `branch_and_bound_candidates`; re-derive the speed ratio by
dumping that map+scan and running it through both the Python search and the C++
core on the same idle machine.
