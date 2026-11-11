#!/usr/bin/env python3
"""Regression guard for the multi-round BBS aggregation experiment.

These numbers come from 87 scans of maps/pgo_session scored against the
production 2D map; they pin the published comparison so a change in the
clustering/voting helpers cannot silently move the reported gains.
"""

import json
from pathlib import Path

from experiments.multi_round_relocalization.common import cluster_candidates
from experiments.multi_round_relocalization.common import cluster_pose
from experiments.multi_round_relocalization.common import is_correct
from experiments.multi_round_relocalization.common import rank_clusters

DATA = Path(__file__).resolve().parents[1] / "experiments" / "multi_round_relocalization" / "data" / "rounds.json"


def load_rounds():
    records = json.loads(DATA.read_text())
    by_frame = {}
    for item in records:
        by_frame.setdefault(item["frame"], {})[item["round"]] = item
    return by_frame


def vote_top1(by_frame, rounds, candidates):
    frames = sorted(by_frame)
    hits = 0
    for frame in frames:
        truth = by_frame[frame][0]["truth"]
        pool = [[*c, r] for r in range(rounds) for c in by_frame[frame][r]["cands"][:candidates]]
        clusters = rank_clusters(cluster_candidates(pool))
        hits += is_correct(cluster_pose(clusters[0]), truth)
    return hits / len(frames)


def test_single_round_baseline():
    by_frame = load_rounds()
    frames = sorted(by_frame)
    hits = sum(is_correct(by_frame[f][0]["cands"][0], by_frame[f][0]["truth"]) for f in frames)
    assert abs(hits / len(frames) - 0.724) < 0.02


def test_vote_aggregation_recovers_failed_frames():
    by_frame = load_rounds()
    assert abs(vote_top1(by_frame, 1, 8) - 0.724) < 0.02
    assert abs(vote_top1(by_frame, 4, 6) - 0.920) < 0.02
    assert abs(vote_top1(by_frame, 5, 6) - 0.943) < 0.02


def test_union_recall_reaches_one():
    by_frame = load_rounds()
    frames = sorted(by_frame)
    for rounds, expected in ((1, 0.966), (2, 0.989), (4, 1.0)):
        covered = sum(
            any(is_correct(c, by_frame[f][r]["truth"])
                for r in range(rounds) for c in by_frame[f][r]["cands"])
            for f in frames)
        assert abs(covered / len(frames) - expected) < 0.02


if __name__ == "__main__":
    test_single_round_baseline()
    test_vote_aggregation_recovers_failed_frames()
    test_union_recall_reaches_one()
    print("multi-round relocalization experiment checks passed")
