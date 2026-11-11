"""Guards for the field-of-view gate that feeds the two costmaps.

The original ground/self-filtered cloud stays on /obstacle_points_live; only
the FOV-limited copy is consumed by the global and local costmaps. These tests
keep the node parameters, the launch arguments and the costmap sources in sync.
"""

import re
from pathlib import Path

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
NODE_SOURCE = PACKAGE_DIR / "src" / "laser_map_ground_grid_node.cpp"
FOV_HEADER = PACKAGE_DIR / "include" / "bw_fastlio_nav2" / "fov_filter.hpp"
NAV_LAUNCH = PACKAGE_DIR / "launch" / "mid360_nav2_localization.launch.py"
NAV_CONFIG = PACKAGE_DIR / "config" / "nav2_omni.yaml"

FOV_PARAMETERS = (
    "fov_filter_enabled",
    "fov_obstacle_topic",
    "fov_min_azimuth_deg",
    "fov_max_azimuth_deg",
    "fov_min_elevation_deg",
    "fov_max_elevation_deg",
    "fov_min_range",
    "fov_max_range",
    "fov_mount_roll_deg",
    "fov_mount_pitch_deg",
    "fov_mount_yaw_deg",
)


def test_costmaps_read_the_fov_topic_and_monitor_keeps_the_full_cloud():
    config = yaml.safe_load(NAV_CONFIG.read_text())
    for costmap_name in ("global_costmap", "local_costmap"):
        parameters = config[costmap_name][costmap_name]["ros__parameters"]
        source = parameters["obstacle_layer"]["live_obstacles"]
        assert source["topic"] == "/obstacle_points_fov"
        assert source["data_type"] == "PointCloud2"
        assert source["marking"] is True
        assert source["clearing"] is True

    monitor = config["collision_monitor"]["ros__parameters"]
    assert monitor["live_obstacles"]["topic"] == "/obstacle_points_live"


def test_navigation_launch_exposes_every_fov_parameter():
    launch = NAV_LAUNCH.read_text()
    declared = set(
        re.findall(r'DeclareLaunchArgument\(\s*"([^"]+)"', launch)
    )
    for name in FOV_PARAMETERS:
        assert name in declared, name

    assert '"fov_obstacle_topic": LaunchConfiguration("fov_obstacle_topic")' in launch
    for name in FOV_PARAMETERS:
        if name == "fov_obstacle_topic":
            continue
        assert f'"{name}": ParameterValue(' in launch, name

    # Defaults must match the requested "forward 270 deg, LiDAR plane and below"
    # window, and the costmap-facing topic name.
    for expected in (
        'default_value="/obstacle_points_fov"',
        'default_value="-135.0"',
        'default_value="135.0"',
        'default_value="-90.0"',
        'default_value="0.0"',
    ):
        assert expected in launch, expected


def test_node_declares_and_applies_the_fov_gate_after_ground_filtering():
    source = NODE_SOURCE.read_text()
    header = FOV_HEADER.read_text()
    declared = set(
        re.findall(r'declare_parameter<[^>]+>\("(fov_[a-z_]+)"', source)
    )
    assert declared == set(FOV_PARAMETERS)

    assert "Eigen::AngleAxisf" in header
    assert "point_inside_fov" in header
    assert "filter_cloud_by_fov" in header
    assert "validate_fov_config(fov_config_)" in source
    # The gate must consume the already filtered obstacle cloud and publish it
    # on the separate topic, leaving the original publications untouched.
    assert "filter_cloud_by_fov(\n            *segmented.obstacles" in source
    assert 'get_parameter("fov_obstacle_topic")' in source
    assert 'create_publisher<sensor_msgs::msg::PointCloud2>(\n              get_parameter("fov_obstacle_topic")' in source


def test_fov_output_keeps_the_lidar_stamp_for_the_costmaps():
    """The costmaps resolve TF at the cloud stamp, so the FOV copy must not
    be restamped to host time.

    spatio_temporal_voxel_layer wraps each source in a tf2_ros::MessageFilter
    and nav2_costmap_2d does the same transform at cloud.header.stamp. FAST-LIO
    stamps camera_init->body with lidar_end_time, and the LiDAR clock runs
    hours behind the host clock, so a host-time stamp makes every message
    un-transformable and the layers silently mark nothing. The full cloud on
    /obstacle_points_live stays host-stamped because collision_monitor compares
    its stamp against now().
    """
    source = NODE_SOURCE.read_text()
    launch = NAV_LAUNCH.read_text()

    assert 'declare_parameter<bool>("restamp_fov_output_to_now", false);' in source
    assert "fov_header.stamp = receive_time;" in source
    assert (
        "publish_cloud(*fov_obstacles, fov_header, fov_obstacle_publisher_);"
        in source
    )

    assert (
        'DeclareLaunchArgument(\n            "restamp_fov_output_to_now", '
        'default_value="false")' in launch
    )
    assert '"restamp_fov_output_to_now": ParameterValue(' in launch
