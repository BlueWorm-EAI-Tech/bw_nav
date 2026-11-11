"""Shared helpers for the multi-round BBS relocalization experiment."""
from __future__ import annotations

import math
from typing import Iterable, List, Sequence

import numpy as np

TOL_TRANSLATION_M = 0.75
TOL_YAW_DEG = 12.0
CLUSTER_RADIUS_M = 2.0
CLUSTER_YAW_DEG = 20.0


def angle_diff_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def pose_error(pose: Sequence[float], truth: Sequence[float]):
    translation = math.hypot(pose[0] - truth[0], pose[1] - truth[1])
    return translation, angle_diff_deg(pose[2], truth[2])


def is_correct(pose, truth, tol_m=TOL_TRANSLATION_M, tol_deg=TOL_YAW_DEG) -> bool:
    translation, yaw = pose_error(pose, truth)
    return translation <= tol_m and yaw <= tol_deg


def cluster_candidates(candidates: Iterable[Sequence[float]], radius_m=CLUSTER_RADIUS_M,
                       yaw_deg=CLUSTER_YAW_DEG) -> List[dict]:
    """Greedy spatial vote clustering.

    Each candidate is ``[x, y, yaw_deg, bbs_score, round_index]``.  Candidates
    are visited in descending score order so the best-scoring member defines the
    cluster representative.
    """
    clusters: List[dict] = []
    for candidate in sorted(candidates, key=lambda item: -item[3]):
        for cluster in clusters:
            rep = cluster["rep"]
            if (math.hypot(candidate[0] - rep[0], candidate[1] - rep[1]) <= radius_m
                    and angle_diff_deg(candidate[2], rep[2]) <= yaw_deg):
                cluster["members"].append(candidate)
                cluster["rounds"].add(candidate[4])
                break
        else:
            clusters.append({"rep": candidate, "members": [candidate], "rounds": {candidate[4]}})
    return clusters


def cluster_pose(cluster: dict):
    xs = [c[0] for c in cluster["members"]]
    ys = [c[1] for c in cluster["members"]]
    yaws = [c[2] for c in cluster["members"]]
    return (float(np.median(xs)), float(np.median(ys)), float(np.median(yaws)))


def rank_clusters(clusters: List[dict]) -> List[dict]:
    return sorted(clusters, key=lambda c: (-len(c["rounds"]), -max(m[3] for m in c["members"])))
