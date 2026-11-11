"""Compare the deployed ranking strategies on the recorded rounds.

Variants:
  bbs       rank the candidate list by BBS score, publish the raw grid pose
  ndt_raw   rank by NDT fitness on the raw grid pose, publish the raw pose
  ndt_ref   rank by NDT fitness, publish the NDT-refined pose (registration_refine_candidates=True)
  vote      cluster + vote R rounds, publish the cluster median
  vote_ref  cluster + vote R rounds, publish the NDT-refined cluster median
  vote+ndt  cluster + vote R rounds, NDT-rerank the top-3 clusters, publish the refined pose
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
from common import cluster_candidates, cluster_pose, is_correct, pose_error, rank_clusters  # noqa: E402


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
    parser.add_argument("--scans", required=True)
    parser.add_argument("--ndt-map", required=True)
    parser.add_argument("--ndt-module-dir", required=True)
    parser.add_argument("--ndt-resolution", type=float, default=1.0)
    parser.add_argument("--output", default=str(HERE / "data" / "variants.json"))
    args = parser.parse_args()

    sys.path.insert(0, args.ndt_module_dir)
    import g2_ndt_score
    scorer = g2_ndt_score.MapNdtScorer(args.ndt_map, args.ndt_resolution, 0.1, 0.01, 30, 4,
                                       0.5, 0.2, 150.0, 100)
    records = json.loads(Path(args.rounds).read_text())
    by = {}
    for item in records:
        by.setdefault(item["frame"], {})[item["round"]] = item
    frames = sorted(by)
    scans = sorted(Path(args.scans).glob("*.pcd"))
    cloud = {f: read_pcd_xyz(scans[f]) for f in frames}

    def score(points, pose):
        result = scorer.score_candidate(points, pose[0], pose[1], 0.0, math.radians(pose[2]))
        fitness = (float(result.fitness)
                   if result.converged and math.isfinite(result.fitness) else float("inf"))
        refined = (float(result.refined_x), float(result.refined_y),
                   math.degrees(float(result.refined_yaw)))
        return fitness, refined

    def candidates(frame, rounds, k):
        return [[*c, r] for r in range(rounds) for c in by[frame][r]["cands"][:k]]

    variants = [
        ("bbs", 1, 8), ("ndt_raw", 1, 8), ("ndt_ref", 1, 8),
        ("vote", 4, 6), ("vote_ref", 4, 6), ("vote+ndt", 4, 6),
        ("vote", 5, 6), ("vote_ref", 5, 6), ("vote+ndt", 5, 6),
        ("vote", 6, 4), ("vote_ref", 6, 4), ("vote+ndt", 6, 4),
    ]
    rows = []
    for name, rounds, k in variants:
        hits = 0
        errors = []
        for frame in frames:
            truth = by[frame][0]["truth"]
            if name.startswith("ndt"):
                pool = [tuple(c[:3]) for c in by[frame][0]["cands"][:k]]
                ranked = sorted(pool, key=lambda p: score(cloud[frame], p)[0])
                pose = ranked[0]
                published = score(cloud[frame], pose)[1] if name == "ndt_ref" else pose
            else:
                clusters = rank_clusters(cluster_candidates(candidates(frame, rounds, k)))
                pose = cluster_pose(clusters[0])
                if name in ("bbs", "vote"):
                    published = pose
                elif name == "vote_ref":
                    published = score(cloud[frame], pose)[1]
                else:
                    best = None
                    for cluster in clusters[:3]:
                        cluster_pose_value = cluster_pose(cluster)
                        fitness, refined = score(cloud[frame], cluster_pose_value)
                        if best is None or fitness < best[0]:
                            best = (fitness, refined)
                    published = best[1]
            if is_correct(published, truth):
                hits += 1
                errors.append(pose_error(published, truth)[0])
        errors.sort()
        rows.append(dict(variant=name, rounds=rounds, candidates=k, accuracy=hits / len(frames),
                         median_error_m=errors[len(errors) // 2] if errors else None,
                         p90_error_m=errors[int(0.9 * (len(errors) - 1))] if errors else None))
        print("%-8s R=%d k=%d  acc %.3f  median %.3fm  p90 %.3fm" % (
            name, rounds, k, hits / len(frames),
            rows[-1]["median_error_m"] or float("nan"), rows[-1]["p90_error_m"] or float("nan")), flush=True)
    Path(args.output).write_text(json.dumps(rows, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
