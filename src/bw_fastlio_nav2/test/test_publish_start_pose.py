import json
import math
from pathlib import Path
import sys

import pytest


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_DIR = PACKAGE_DIR / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from publish_start_pose import (  # noqa: E402
    load_named_point,
    point_to_body_pose,
)


def _write_points(path: Path, points: dict) -> None:
    path.write_text(json.dumps(points, ensure_ascii=False), encoding="utf-8")


def _point(**overrides):
    point = {
        "frame_id": "map",
        "child_frame_id": "base_footprint",
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
    }
    point.update(overrides)
    return point


def test_load_named_point_normalizes_quaternion(tmp_path):
    _write_points(tmp_path / "points.json", {"start": _point(qw=2.0)})

    point = load_named_point(
        tmp_path / "points.json", "start", "map", "base_footprint"
    )

    assert point["x"] == pytest.approx(1.0)
    assert point["qw"] == pytest.approx(1.0)
    assert point["qz"] == pytest.approx(0.0)


def test_load_named_point_rejects_missing_name(tmp_path):
    _write_points(tmp_path / "points.json", {"home": _point()})

    with pytest.raises(ValueError, match="not found"):
        load_named_point(tmp_path / "points.json", "start", "map", "base_footprint")


def test_load_named_point_rejects_wrong_frames(tmp_path):
    _write_points(
        tmp_path / "points.json",
        {"start": _point(frame_id="odom")},
    )

    with pytest.raises(ValueError, match="frame_id"):
        load_named_point(tmp_path / "points.json", "start", "map", "base_footprint")


def test_point_to_body_pose_undoes_aligned_translation():
    pose = {
        "x": 1.0, "y": 2.0, "z": 0.0,
        "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0,
    }

    body = point_to_body_pose(pose, (-0.14, 0.0, -0.18))

    assert body["x"] == pytest.approx(1.14)
    assert body["y"] == pytest.approx(2.0)
    assert body["z"] == pytest.approx(0.18)
    assert body["qw"] == pytest.approx(1.0)


def test_point_to_body_pose_rotates_the_offset_with_heading():
    half_yaw = math.pi / 4.0
    pose = {
        "x": 0.0, "y": 0.0, "z": 0.0,
        "qx": 0.0, "qy": 0.0,
        "qz": math.sin(half_yaw), "qw": math.cos(half_yaw),
    }

    body = point_to_body_pose(pose, (-0.14, 0.0, 0.0))

    # The -0.14 m body->footprint offset now points along map -y.
    assert body["x"] == pytest.approx(0.0, abs=1e-9)
    assert body["y"] == pytest.approx(0.14)
    assert body["z"] == pytest.approx(0.0)
