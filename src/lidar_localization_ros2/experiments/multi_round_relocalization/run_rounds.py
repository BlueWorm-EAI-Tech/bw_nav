"""Run R perturbed BBS rounds per frame and record the candidate lists.

Perturbation per round r of R: a fresh random subsample of the 0.05 m voxel-unique
scan points plus a yaw-grid offset of r*angular_resolution/R (the scan is rotated
and the candidate yaw is un-rotated, so the returned poses stay in map frame).
Both perturbations are free: no extra scans, no motion compensation.

Usage:
  python3 run_rounds.py --frames 87 --rounds 6 \\
      --map-yaml /path/mid360_pgo_csf.yaml \\
      --poses /path/pgo_session/optimized_poses.txt \\
      --scans /path/pgo_session/Scans
"""
from __future__ import annotations

import argparse
import json
import math
import multiprocessing as mp
import sys
import time
from pathlib import Path

import numpy as np

PKG = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PKG / "scripts"))
import make_bbs_relocalization_attempts as bbs  # noqa: E402

_G: dict = {}
ARGS = None


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


def init(args):
    global ARGS
    ARGS = args
    occupancy = bbs.load_occupancy_map(args.map_yaml)
    _G["occupancy"] = occupancy
    _G["grid"] = bbs._dilate_one_cell(occupancy.occupied)
    _G["resolution"] = occupancy.resolution_m
    _G["poses"] = np.loadtxt(args.poses).reshape(-1, 3, 4)
    _G["scans"] = sorted(Path(args.scans).glob("*.pcd"))


def scan_base(points: np.ndarray) -> np.ndarray:
    p = np.asarray(points, dtype=np.float64)
    finite = np.isfinite(p).all(axis=1)
    band = (p[:, 2] >= ARGS.z_min_m) & (p[:, 2] <= ARGS.z_max_m)
    reach = np.hypot(p[:, 0], p[:, 1]) >= ARGS.min_range_m
    return bbs.voxel_downsample_2d(p[finite & band & reach, :2], _G["resolution"])


def rotate(xy: np.ndarray, angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.stack([c * xy[:, 0] - s * xy[:, 1], s * xy[:, 0] + c * xy[:, 1]], axis=1)


def truth_score(scan_xy_grid: np.ndarray, tx: int, ty: int):
    best, best_yaw = -1.0, 0.0
    for k in range(0, 360, 1):
        yaw = math.radians(k)
        score = bbs.bbs_exact_score(_G["grid"], scan_xy_grid, tx, ty, yaw)
        if score > best:
            best, best_yaw = score, yaw
    return best, best_yaw


def run(task):
    frame, rnd = task
    occupancy, resolution = _G["occupancy"], _G["resolution"]
    base = scan_base(read_pcd_xyz(_G["scans"][frame]))
    rng = np.random.default_rng(1000 * frame + rnd)
    if base.shape[0] > ARGS.scan_points:
        scan = base[rng.choice(base.shape[0], size=ARGS.scan_points, replace=False)]
    else:
        scan = base
    delta = math.radians(rnd * ARGS.angular_resolution_deg / ARGS.rounds)
    rotated = rotate(scan, delta) if delta else scan
    start = time.monotonic()
    grid_candidates = bbs.branch_and_bound_candidates(
        _G["grid"], rotated, resolution, math.radians(ARGS.angular_resolution_deg),
        ARGS.pyramid_depth, ARGS.candidates, int(round(ARGS.nms_radius_m / resolution)))
    elapsed = time.monotonic() - start
    pose = _G["poses"][frame]
    x, y = pose[0, 3], pose[1, 3]
    yaw = math.atan2(pose[1, 0], pose[0, 0])
    candidates = []
    for candidate in grid_candidates:
        wx, wy = occupancy.grid_cell_center_to_world(candidate.tx_cell, candidate.ty_cell)
        wyaw = bbs.normalize_angle_rad(occupancy.origin_yaw_rad + candidate.yaw_rad + delta)
        candidates.append([round(wx, 3), round(wy, 3), round(math.degrees(wyaw), 2),
                           round(candidate.score, 5)])
    cell = (int(math.floor((x - occupancy.origin_x_m) / resolution)),
            int(math.floor((y - occupancy.origin_y_m) / resolution)))
    score, best_yaw = truth_score(scan / resolution, cell[0], cell[1])
    return dict(frame=frame, round=rnd, dyaw=round(math.degrees(delta), 2),
                dt=round(elapsed, 2), npts=int(scan.shape[0]),
                truth=[round(x, 3), round(y, 3), round(math.degrees(yaw), 2)],
                truth_score=round(float(score), 5), truth_best_yaw=round(math.degrees(best_yaw), 2),
                cands=candidates)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map-yaml", required=True)
    parser.add_argument("--poses", required=True)
    parser.add_argument("--scans", required=True)
    parser.add_argument("--frames", type=int, default=87)
    parser.add_argument("--rounds", type=int, default=6)
    parser.add_argument("--candidates", type=int, default=8)
    parser.add_argument("--scan-points", type=int, default=256)
    parser.add_argument("--angular-resolution-deg", type=float, default=10.0)
    parser.add_argument("--pyramid-depth", type=int, default=4)
    parser.add_argument("--nms-radius-m", type=float, default=2.0)
    parser.add_argument("--z-min-m", type=float, default=0.5)
    parser.add_argument("--z-max-m", type=float, default=5.0)
    parser.add_argument("--min-range-m", type=float, default=1.0)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--output", default="data/rounds.json")
    args = parser.parse_args()
    tasks = [(frame, rnd) for frame in range(args.frames) for rnd in range(args.rounds)]
    print("tasks=%d rounds=%d" % (len(tasks), args.rounds), flush=True)
    started = time.time()
    results = []
    with mp.Pool(args.workers, initializer=init, initargs=(args,)) as pool:
        for index, result in enumerate(pool.imap_unordered(run, tasks, chunksize=2)):
            results.append(result)
            if (index + 1) % 50 == 0:
                print("%d/%d %.0fs" % (index + 1, len(tasks), time.time() - started), flush=True)
    results.sort(key=lambda item: (item["frame"], item["round"]))
    Path(args.output).write_text(json.dumps(results))
    print("wrote %s in %.0fs" % (args.output, time.time() - started))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
