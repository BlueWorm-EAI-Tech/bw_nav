"""Aggregate the per-round candidate lists and report accuracy vs (rounds, candidates).

Without --ndt-module-dir this reproduces the vote-only tables from data/rounds.json.
With --ndt-module-dir pointing at a built g2_ndt_score extension it also reranks
the top-3 vote clusters by NDT fitness (the deployed decision rule).

Usage:
  python3 evaluate.py --rounds data/rounds.json \\
      --ndt-map /path/pgo_session/map_consistent.pcd \\
      --ndt-module-dir /path/to/install/lib/lidar_localization_ros2
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from common import (cluster_candidates, cluster_pose, is_correct,  # noqa: E402
                    pose_error, rank_clusters)


def read_pcd_xyz(path: Path) -> np.ndarray:
    with open(path, "rb") as handle:
        header = {}
        while True:
            line = handle.readline().decode("ascii", "replace").strip()
            if not line:
                continue
            header[line.split()[0]] = line
            if line.startswith("DATA"):
                break
        count = int(header["POINTS"].split()[1])
        payload = handle.read()
    fields = header["FIELDS"].split()[1:]
    types = header["TYPE"].split()[1:]
    sizes = [int(v) for v in header["SIZE"].split()[1:]]
    dtype = np.dtype([(name, "<" + ("f" if t == "F" else "u") + str(size))
                      for name, t, size in zip(fields, types, sizes)])
    points = np.frombuffer(payload, dtype=dtype, count=count)
    return np.stack([points["x"], points["y"], points["z"]], axis=1).astype(np.float64)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rounds", default=str(HERE / "data" / "rounds.json"))
    parser.add_argument("--frames", type=int, default=0, help="0 = every frame in the file")
    parser.add_argument("--max-rounds", type=int, default=6)
    parser.add_argument("--scans", default=None, help="scan dir, required for NDT reranking")
    parser.add_argument("--ndt-map", default=None)
    parser.add_argument("--ndt-module-dir", default=None)
    parser.add_argument("--ndt-resolution", type=float, default=1.0)
    args = parser.parse_args()

    records = json.loads(Path(args.rounds).read_text())
    frames = sorted({item["frame"] for item in records})
    if args.frames:
        frames = frames[: args.frames]
    by_frame = {}
    for item in records:
        by_frame.setdefault(item["frame"], {})[item["round"]] = item
    max_rounds = min(args.max_rounds, max(len(v) for v in by_frame.values()))

    scorer = None
    if args.ndt_map and args.ndt_module_dir:
        sys.path.insert(0, args.ndt_module_dir)
        import g2_ndt_score
        scorer = g2_ndt_score.MapNdtScorer(args.ndt_map, args.ndt_resolution, 0.1, 0.01, 30, 4,
                                           0.5, 0.2, 150.0, 100)

    print("=== single round: top1 / recall@k (mean over rounds) ===")
    for k in (1, 3, 4, 6, 8):
        hits = [any(is_correct(c, by_frame[f][r]["truth"]) for c in by_frame[f][r]["cands"][:k])
                for f in frames for r in range(max_rounds)]
        print(" recall@%d %.3f" % (k, sum(hits) / len(hits)))
    top1 = [is_correct(by_frame[f][r]["cands"][0], by_frame[f][r]["truth"])
            for f in frames for r in range(max_rounds)]
    round0 = [is_correct(by_frame[f][0]["cands"][0], by_frame[f][0]["truth"]) for f in frames]
    print(" top1 mean-over-rounds %.3f   round0(production) %.3f" % (
        sum(top1) / len(top1), sum(round0) / len(round0)))
    times = [item["dt"] for item in records]
    print(" BBS round time under the 12-way pool (contended; single process ~4s): mean %.2fs p90 %.2fs" % (
        statistics.mean(times), sorted(times)[int(0.9 * (len(times) - 1))]))

    print("\n=== vote aggregation accuracy (cluster 2m/20deg, medians published) ===")
    header = "       " + "".join("  k=%d " % k for k in (4, 6, 8))
    print(header)
    for rounds in range(2, max_rounds + 1):
        row = []
        for k in (4, 6, 8):
            hits = 0
            for f in frames:
                truth = by_frame[f][0]["truth"]
                candidates = [[*c, r] for r in range(rounds) for c in by_frame[f][r]["cands"][:k]]
                clusters = rank_clusters(cluster_candidates(candidates))
                hits += is_correct(cluster_pose(clusters[0]), truth)
            row.append(hits / len(frames))
        print(" R=%d  " % rounds + "".join(" %.3f" % v for v in row))

    if scorer is None:
        print("\n(NDT reranking skipped: pass --ndt-map and --ndt-module-dir)")
        return 0

    print("\n=== vote + NDT rerank of the top-3 clusters ===")
    print("  R   k  vote  vote+NDT  unanimity(votes==R): coverage / precision")
    for k in (4, 6, 8):
        for rounds in range(2, max_rounds + 1):
            vote_hits = ndt_hits = unan = unan_ok = 0
            for f in frames:
                truth = by_frame[f][0]["truth"]
                candidates = [[*c, r] for r in range(rounds) for c in by_frame[f][r]["cands"][:k]]
                clusters = rank_clusters(cluster_candidates(candidates))
                top = clusters[:3]
                vote_hits += is_correct(cluster_pose(top[0]), truth)
                if len(top) == 1:
                    ndt_hits += is_correct(cluster_pose(top[0]), truth)
                else:
                    points = read_pcd_xyz(Path(args.scans) / sorted(
                        Path(args.scans).glob("*.pcd"))[f].name)
                    best = None
                    for cluster in top:
                        x, y, yaw = cluster_pose(cluster)
                        result = scorer.score_candidate(points, x, y, 0.0, math.radians(yaw))
                        fitness = (float(result.fitness)
                                   if result.converged and math.isfinite(result.fitness)
                                   else float("inf"))
                        if best is None or fitness < best[0]:
                            best = (fitness, cluster)
                    ndt_hits += is_correct(cluster_pose(best[1]), truth)
                if len(top[0]["rounds"]) == rounds:
                    unan += 1
                    unan_ok += is_correct(cluster_pose(top[0]), truth)
            print("  %d   %d  %.3f  %.3f      %d/%d  %.3f" % (
                rounds, k, vote_hits / len(frames), ndt_hits / len(frames),
                unan, len(frames), unan_ok / max(1, unan)))

    print("\n=== final pose error of the vote+NDT winner (R=5, k=8) ===")
    errors = []
    for f in frames:
        truth = by_frame[f][0]["truth"]
        candidates = [[*c, r] for r in range(min(5, max_rounds)) for c in by_frame[f][r]["cands"]]
        clusters = rank_clusters(cluster_candidates(candidates))[:3]
        points = read_pcd_xyz(Path(args.scans) / sorted(Path(args.scans).glob("*.pcd"))[f].name)
        best = None
        for cluster in clusters:
            x, y, yaw = cluster_pose(cluster)
            result = scorer.score_candidate(points, x, y, 0.0, math.radians(yaw))
            fitness = (float(result.fitness)
                       if result.converged and math.isfinite(result.fitness) else float("inf"))
            if best is None or fitness < best[0]:
                best = (fitness, (x, y, yaw))
        errors.append(pose_error(best[1], truth))
    translations = sorted(e[0] for e in errors)
    yaws = sorted(e[1] for e in errors)
    print(" translation: median %.3f p90 %.3f" % (
        translations[len(translations) // 2], translations[int(0.9 * (len(translations) - 1))]))
    print(" yaw: median %.1f p90 %.1f" % (
        yaws[len(yaws) // 2], yaws[int(0.9 * (len(yaws) - 1))]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
