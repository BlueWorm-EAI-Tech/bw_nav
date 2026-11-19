"""Offline CLI regressions; no ROS node, recordings, NumPy or Python PCL needed.

After building bw_fastlio_nav2, from the colcon workspace root run:
  REBUILD_PGO_MAP="$PWD/build/bw_fastlio_nav2/rebuild_pgo_consistent_map" \
    python3 -m pytest -q src/bw_fastlio_nav2/test/test_rebuild_pgo_freedom.py
"""

import os
import struct
import subprocess
from collections import Counter
from pathlib import Path

import pytest


_XYZI = struct.Struct("<4f")
_WALL = 2
_STATIC_PANEL = 3
_FIRST_STOP = 10
_SECOND_STOP = 11
_SINGLE_FRAME = 20
# Non-identity T_body_lidar in KITTI row-major order. T_world_body uses a
# +90-degree yaw, so the two rotations compose to the identity. Deliberately
# place the body origin >10 m behind the LiDAR: casting from the body origin
# would put the wall beyond FreeDOM's raycast range and leave the ghosts.
_EXTRINSIC = "0 1 0 0.031 -1 0 0 -12.023 0 0 1 0.041\n"


def _write_pcd(path, points):
    header = (
        "VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\n"
        "TYPE F F F F\nCOUNT 1 1 1 1\n"
        f"WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA binary\n"
    )
    path.write_bytes(header.encode("ascii") + b"".join(_XYZI.pack(*p) for p in points))


def _read_map(path):
    header, data = path.read_bytes().split(b"DATA binary\n", 1)
    fields = dict(
        line.split(maxsplit=1)
        for line in header.splitlines()
        if line and not line.startswith(b"#")
    )
    assert fields[b"FIELDS"].split() == [b"x", b"y", b"z", b"intensity"]
    assert fields[b"SIZE"].split() == [b"4"] * 4
    assert fields[b"TYPE"].split() == [b"F"] * 4
    assert fields[b"COUNT"].split() == [b"1"] * 4
    assert len(data) == _XYZI.size * int(fields[b"POINTS"])
    return Counter(_XYZI.iter_unpack(data))


def _make_scene(root, *, moving):
    scans = root / "Scans"
    scans.mkdir(parents=True)
    poses = []
    for index in range(16):
        # Points on the wall/panels are the nearest collisions on an angular
        # lattice, never a second return through a foreground obstacle.
        ox = 0.18 + 0.19 * (index % 6)
        oy = -0.24 + 0.11 * (index % 5)
        oz = 0.43
        bx, by, bz = ox - 12.023, oy - 0.031, oz - 0.041
        poses.append(f"0 -1 0 {bx:.6f} 1 0 0 {by:.6f} 0 0 1 {bz:.6f}\n")
        panels = [(4.13, 1.26, 1.58, 0.14, 1.24, _STATIC_PANEL)]
        if moving and index < 4:
            panels.insert(0, (3.33, -0.94, -0.26, 0.22, 1.12, _FIRST_STOP))
        elif moving and index < 8:
            panels.insert(0, (3.83, 0.18, 0.86, 0.22, 1.12, _SECOND_STOP))
        panels.append((6.03, -3.2, 3.2, 0.02, 2.12, _WALL))
        points = []
        for ay in range(-56, 57):
            for az in range(-13, 31):
                for x, y_min, y_max, z_min, z_max, intensity in panels:
                    y = oy + (x - ox) * ay * 0.012
                    z = oz + (x - ox) * az * 0.012
                    if y_min <= y <= y_max and z_min <= z <= z_max:
                        # Scan points and poses are in the IMU body frame, as
                        # in production, rather than already in LiDAR frame.
                        # A varying return on the static panel also exercises
                        # preservation of the baseline's averaged intensity.
                        if intensity == _STATIC_PANEL:
                            intensity += (index % 3) * 0.125
                        points.append((y - by, bx - x, z - bz, intensity))
                        break
        if index == 0:
            # One isolated return outside the panel/wall footprint, within
            # range and height; it must not bypass --min-observations 2.
            points.append((4.19 - by, bx - 3.17, 1.47 - bz, _SINGLE_FRAME))
        _write_pcd(scans / f"{index:06d}.pcd", points)
    (root / "optimized_poses_batch.txt").write_text("".join(poses), encoding="ascii")
    (root / "extrinsic.txt").write_text(_EXTRINSIC, encoding="ascii")
    return root


@pytest.fixture(scope="module")
def rebuild():
    path = os.environ.get("REBUILD_PGO_MAP")
    if not path:
        pytest.skip("Set REBUILD_PGO_MAP to the built CLI (see module docstring)")
    binary = Path(path).resolve()
    assert binary.is_file() and os.access(binary, os.X_OK), f"Not executable: {binary}"
    return binary


@pytest.fixture(scope="module")
def moving_scene(tmp_path_factory):
    return _make_scene(tmp_path_factory.mktemp("moving_scene"), moving=True)


def _run(rebuild, scene, output, *, extrinsic=None, voxel_size="0.05"):
    args = [
        str(rebuild),
        "--scans-directory", str(scene / "Scans"),
        "--poses", str(scene / "optimized_poses_batch.txt"),
        "--output", str(output),
        "--voxel-size", voxel_size,
        "--min-observations", "2",
        "--threads", "1",
    ]
    if extrinsic is not None:
        args += ["--freedom-extrinsic", str(extrinsic)]
    return subprocess.run(args, capture_output=True, text=True, timeout=60)


def _successful_map(rebuild, scene, output, *, freedom=False):
    extrinsic = scene / "extrinsic.txt" if freedom else None
    result = _run(rebuild, scene, output, extrinsic=extrinsic)
    assert result.returncode == 0, result.stdout + result.stderr
    return _read_map(output)


def _label_counts(points):
    counts = Counter()
    for (_, _, _, intensity), copies in points.items():
        counts[int(intensity)] += copies
    return counts


def test_freedom_only_deletes_motion_ghosts(rebuild, moving_scene, tmp_path):
    baseline = _successful_map(rebuild, moving_scene, tmp_path / "baseline.pcd")
    filtered = _successful_map(rebuild, moving_scene, tmp_path / "filtered.pcd", freedom=True)
    before, after = _label_counts(baseline), _label_counts(filtered)
    # Check the old pipeline retains both multi-frame ghosts with the flag off.
    for ghost in (_FIRST_STOP, _SECOND_STOP):
        assert before[ghost] >= 100, before
        assert after[ghost] <= before[ghost] * 0.10, (before, after)
    for control in (_WALL, _STATIC_PANEL):
        assert before[control] >= 100, before
        assert after[control] >= before[control] * 0.99, (before, after)
    assert before[_SINGLE_FRAME] == after[_SINGLE_FRAME] == 0
    # Every surviving record (including weighted intensity) must already exist
    # in the baseline, irrespective of PCD record order. No re-voxelization.
    assert not (filtered - baseline)


def test_freedom_preserves_static_scene(rebuild, tmp_path):
    scene = _make_scene(tmp_path / "static_scene", moving=False)
    baseline = _successful_map(rebuild, scene, tmp_path / "baseline.pcd")
    filtered = _successful_map(rebuild, scene, tmp_path / "filtered.pcd", freedom=True)
    assert _label_counts(baseline)[_STATIC_PANEL] >= 100
    assert filtered == baseline


@pytest.mark.parametrize(
    ("problem", "diagnostic"),
    [
        ("malformed-extrinsic", "extrinsic.txt"),
        ("wrong-resolution", "--voxel-size 0.05"),
        ("missing-scan", "matching scan/pose counts"),
        ("extra-scan", "matching scan/pose counts"),
        ("wrong-scan-id", "000001.pcd"),
    ],
)
def test_freedom_rejects_bad_inputs_without_writing(rebuild, tmp_path, problem, diagnostic):
    # Start with a valid three-frame snapshot. Even with a scan missing, two
    # observations remain, so the legacy reconstruction would still succeed.
    scans = tmp_path / "Scans"
    scans.mkdir()
    points = [(3.01, 0.12, 0.23, 2.0), (3.01, 0.28, 0.23, 2.0)]
    for index in range(3):
        _write_pcd(scans / f"{index:06d}.pcd", points)
    (tmp_path / "optimized_poses_batch.txt").write_text(
        "1 0 0 0 0 1 0 0 0 0 1 0\n" * 3, encoding="ascii"
    )
    extrinsic = tmp_path / "extrinsic.txt"
    extrinsic.write_text(
        "1 0 0 0\n" if problem == "malformed-extrinsic" else _EXTRINSIC,
        encoding="ascii",
    )
    voxel_size = "0.10" if problem == "wrong-resolution" else "0.05"
    if problem in ("missing-scan", "wrong-scan-id"):
        (scans / "000001.pcd").unlink()
    if problem in ("extra-scan", "wrong-scan-id"):
        _write_pcd(scans / "000003.pcd", points)

    legacy_output = tmp_path / "baseline.pcd"
    result = _run(rebuild, tmp_path, legacy_output, voxel_size=voxel_size)
    assert result.returncode == 0, result.stdout + result.stderr
    assert _read_map(legacy_output)

    output = tmp_path / "map.pcd"
    result = _run(rebuild, tmp_path, output, extrinsic=extrinsic, voxel_size=voxel_size)
    assert result.returncode > 0, result.stdout + result.stderr
    assert diagnostic in result.stderr, result.stderr
    assert not output.exists()

    # This prior map differs from the baseline so accidental overwrite is
    # detectable even if the failing process writes a valid-looking PCD.
    _write_pcd(output, [(8.01, 5.12, 0.93, 42.0)])
    previous = output.read_bytes()
    result = _run(rebuild, tmp_path, output, extrinsic=extrinsic, voxel_size=voxel_size)
    assert result.returncode > 0, result.stdout + result.stderr
    assert diagnostic in result.stderr, result.stderr
    assert output.read_bytes() == previous
