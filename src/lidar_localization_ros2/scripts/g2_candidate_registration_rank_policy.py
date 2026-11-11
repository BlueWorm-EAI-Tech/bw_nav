"""Pure policy for ranking G2 candidates by NDT registration fitness.

BBS occupancy score does not separate perceptually aliased poses from the true
location (see docs/dev/g3_live_closed_loop.md). When NDT fitness is available, the
supervisor-facing score and candidate order should follow registration quality
(lower fitness is better).
"""

from __future__ import annotations

from typing import Optional

import math
from collections.abc import Iterable, Sequence
from dataclasses import dataclass


@dataclass(frozen=True)
class RankedG2Candidate:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    bbs_score: float
    registration_fitness: float | None
    score: float
    hit_count: int = 0
    point_count: int = 0
    registration_converged: Optional[bool] = None
    # Diagnostic provenance of the supervisor-facing pose.  raw_* is the
    # original coarse BBS cell; refinement_used says whether x_m/y_m/yaw_rad
    # actually came from NDT.  Defaults keep older constructors valid.
    raw_x_m: Optional[float] = None
    raw_y_m: Optional[float] = None
    raw_z_m: Optional[float] = None
    raw_yaw_rad: Optional[float] = None
    refinement_used: bool = False
    refined_bbs_score: Optional[float] = None


def registration_fitness_to_supervisor_score(
    fitness: float | None,
    score_gate: float = 6.0,
) -> float:
    """Map NDT fitness (lower is better) to a [0, 1] supervisor score."""
    if fitness is None or not math.isfinite(fitness) or score_gate <= 0.0:
        return 0.0
    if fitness <= 0.0:
        return 1.0
    return max(0.0, min(1.0, 1.0 - fitness / score_gate))


def _fitness_sort_key(fitness: float | None) -> float:
    if fitness is None or not math.isfinite(fitness):
        return float("inf")
    return fitness


def rank_candidates_by_registration(
    candidates: Sequence[RankedG2Candidate],
) -> list[RankedG2Candidate]:
    """Sort by registration fitness ascending, then BBS score descending."""
    return sorted(
        candidates,
        key=lambda candidate: (
            _fitness_sort_key(candidate.registration_fitness),
            -candidate.bbs_score,
        ),
    )


def candidate_pose_from_registration(
    candidate: RankedG2Candidate,
    *,
    converged: bool,
    fitness: float | None,
    refined_x: float,
    refined_y: float,
    refined_z: float,
    refined_yaw: float,
) -> tuple[float, float, float, float]:
    """Return supervisor-facing pose; use NDT refinement only when converged."""
    if converged and fitness is not None and math.isfinite(fitness):
        return refined_x, refined_y, refined_z, refined_yaw
    return candidate.x_m, candidate.y_m, candidate.z_m, candidate.yaw_rad


def apply_registration_ranking(
    candidates: Sequence[RankedG2Candidate],
    score_gate: float = 6.0,
) -> list[RankedG2Candidate]:
    """Recompute supervisor scores from fitness and return ranked candidates."""
    updated = []
    for candidate in candidates:
        fitness = candidate.registration_fitness
        updated.append(
            RankedG2Candidate(
                x_m=candidate.x_m,
                y_m=candidate.y_m,
                z_m=candidate.z_m,
                yaw_rad=candidate.yaw_rad,
                bbs_score=candidate.bbs_score,
                registration_fitness=fitness,
                score=registration_fitness_to_supervisor_score(fitness, score_gate),
                hit_count=candidate.hit_count,
                point_count=candidate.point_count,
                registration_converged=candidate.registration_converged,
                raw_x_m=candidate.raw_x_m,
                raw_y_m=candidate.raw_y_m,
                raw_z_m=candidate.raw_z_m,
                raw_yaw_rad=candidate.raw_yaw_rad,
                refinement_used=candidate.refinement_used,
                refined_bbs_score=candidate.refined_bbs_score,
            ))
    return rank_candidates_by_registration(updated)


def bbs_only_candidates(
    raw_candidates: Iterable[tuple[float, float, float, float, float, int, int]],
) -> list[RankedG2Candidate]:
    """Wrap BBS-only candidates where score == bbs_score."""
    wrapped = []
    for x_m, y_m, z_m, yaw_rad, bbs_score, hit_count, point_count in raw_candidates:
        wrapped.append(
            RankedG2Candidate(
                x_m=x_m,
                y_m=y_m,
                z_m=z_m,
                yaw_rad=yaw_rad,
                bbs_score=bbs_score,
                registration_fitness=None,
                score=bbs_score,
                hit_count=hit_count,
                point_count=point_count,
                registration_converged=None,
            )
        )
    return list(wrapped)
