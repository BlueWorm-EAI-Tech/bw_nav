"""Generate both Nav2 occupancy grids from one consistent point cloud.

Two maps, on purpose:

  mid360_pgo_csf -- the navigation map.  Ray-cast free space seeded from the
      trajectory, obstacle returns grouped into solid clusters (a 0.10 m disk
      closing), isolated blobs below 20 cells removed, a 4 m geodesic cap
      around the path, wall-enclosed rooms kept as unreachable free islands,
      and every remaining free/unknown boundary sealed with a wall.  The
      planner sees a closed world it cannot cut corners in.

  mid360_pgo_bbs -- the map the global (BBS) relocaliser scores scans against.
      The raw, thin obstacle layer and no sealing: sealed cells are invented
      walls up to several metres away from any real return, and BBS only reads
      the occupied layer, so they would pull scan endpoints towards places that
      do not exist.  Clustering is nav-only for the same reason: the BBS
      distance field wants the exact returns, not merged blobs.

Wall bridging is off on purpose.  The 1D closing connects any two obstacle
cells within 2*length along four fixed directions; on dense recordings the
walls are already continuous, so it mostly chains unrelated clutter dots into
long thin synthetic walls (and smears obstacle blobs) instead of repairing real
gaps.  Neither map uses it; the nav map instead groups each object cluster with
a disk closing, which by construction only fills gaps narrower than 2*radius and
cannot draw long lines between separate objects.

Both runs share the CSF / projection parameters that were tuned on
maps/pgo_session/map_consistent.pcd.

Usage:
  ros2 launch bw_fastlio_nav2 generate_nav2_maps.launch.py
  ros2 launch bw_fastlio_nav2 generate_nav2_maps.launch.py \
      pcd_map:=/abs/map.pcd poses:=/abs/poses.txt output_dir:=/abs/out
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

# Ground segmentation / projection parameters, tuned on map_consistent.pcd.
COMMON = [
    "--voxel-size", "0.08",
    "--cloth-resolution", "0.2",
    "--rigidness", "3",
    "--iterations", "500",
    "--classification-threshold", "0.18",
    "--time-step", "0.65",
    "--slope-smoothing", "false",
    "--min-height", "0.25",
    "--max-height", "1.8",
    "--ground-reference-radius", "0.75",
    "--ground-reference-neighbors", "5",
    "--outlier-radius", "0.2",
    "--outlier-min-neighbors", "6",
    "--resolution", "0.05",
    "--padding", "1",
    "--occupied-radius", "0",
    "--min-obstacle-neighbors", "2",
    # Keep the raw obstacle layer: bridging invents thin walls between clutter
    # dots and smears obstacle blobs (see the module docstring).
    "--wall-bridge-length", "0.0",
    "--wall-bridge-directions", "4",
    "--unknown-value", "127",
]

NAV_ONLY = [
    # Join the returns of one object (chair legs, desk clutter) into a solid
    # blob, then drop the leftover specks that are only a few cells across.
    # 20 cells sits in the size gap of this recording: after clustering the
    # leftovers top out at 19 cells, and a floor of 10 still kept a handful of
    # 10-16 cell specks (several inside free space or across a doorway).  The
    # next component is 23 cells with 42 returns 0.18-0.60 m high.  Small
    # blobs in unknown space cannot affect planning either -- unknown is
    # impassable in nav2_omni.yaml.
    "--obstacle-cluster-radius", "0.10",
    "--min-obstacle-component-cells", "20",
    "--free-radius", "0.4",
    "--free-mode", "raycast",
    "--free-max-range", "40",
    "--free-max-offset", "4",
    "--free-islands", "enclosed",
    "--free-island-wall-ratio", "0.4",
    "--free-island-max-area", "25",
    "--min-free-component-cells", "100",
    "--seal-mode", "free",
]

# Only walls matter for BBS; the free layer is irrelevant there, so keep it
# cheap (a 5 cm disk around the floor) and skip ray casting entirely.
BBS_ONLY = [
    "--min-obstacle-component-cells", "1",
    "--free-radius", "0.08",
    "--free-mode", "euclid",
    "--seal-mode", "off",
]


def _resolve(context, name):
    raw = LaunchConfiguration(name).perform(context)
    return Path(os.path.expanduser(raw)).resolve()


def _launch(context):
    pcd_map = _resolve(context, "pcd_map")
    poses = _resolve(context, "poses")
    output_dir = _resolve(context, "output_dir")
    for label, path in (("pcd_map", pcd_map), ("poses", poses)):
        if not path.is_file():
            raise RuntimeError(f"{label} does not exist: {path}")
    output_dir.mkdir(parents=True, exist_ok=True)

    nav_prefix = str(output_dir / "mid360_pgo_csf")
    bbs_prefix = str(output_dir / "mid360_pgo_bbs")

    # pcd_to_nav2_map is a plain CLI, not a ROS node: launch it as a process so
    # that no --ros-args / __node:= remapping is appended to its argv.
    executable = str(
        Path(get_package_prefix("bw_fastlio_nav2")) / "lib" / "bw_fastlio_nav2" / "pcd_to_nav2_map"
    )
    nav_map = ExecuteProcess(
        cmd=[executable, "--input", str(pcd_map), "--output-prefix", nav_prefix,
             "--poses", str(poses)] + COMMON + NAV_ONLY,
        name="pcd_to_nav2_map_nav",
        output="screen",
    )
    bbs_map = ExecuteProcess(
        cmd=[executable, "--input", str(pcd_map), "--output-prefix", bbs_prefix]
        + COMMON + BBS_ONLY,
        name="pcd_to_nav2_map_bbs",
        output="screen",
        condition=IfCondition(LaunchConfiguration("bbs")),
    )
    return [nav_map, bbs_map]


def generate_launch_description():
    share = get_package_share_directory("bw_fastlio_nav2")
    workspace = Path(share).resolve().parents[3]
    return LaunchDescription([
        DeclareLaunchArgument(
            "pcd_map",
            default_value=str(workspace / "maps/pgo_session/map_consistent.pcd"),
            description="Consistent point cloud to rasterise.",
        ),
        DeclareLaunchArgument(
            "poses",
            default_value=str(workspace / "maps/pgo_session/optimized_poses_batch.txt"),
            description="Trajectory used to seed the ray casting free space.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=str(workspace / "maps/nav2/pgm_v2"),
            description="Where mid360_pgo_csf.* and mid360_pgo_bbs.* are written.",
        ),
        DeclareLaunchArgument(
            "bbs",
            default_value="true",
            description="Also generate the relocalisation map.",
        ),
        OpaqueFunction(function=_launch),
    ])
