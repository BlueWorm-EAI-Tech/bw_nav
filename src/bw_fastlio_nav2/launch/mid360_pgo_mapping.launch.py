import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory("fast_lio")
    livox_share = get_package_share_directory("livox_ros_driver2")
    integration_share = get_package_share_directory("bw_fastlio_nav2")
    workspace = Path(integration_share).resolve().parents[3]
    save_directory = LaunchConfiguration("save_directory")
    rviz = LaunchConfiguration("rviz")
    obstacle_grid = LaunchConfiguration("obstacle_grid")
    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")
    config_file = LaunchConfiguration("config_file")

    # MID-360 and MID-360S ship different driver launches (and config blocks).
    # install.sh rewrites this name after detecting the model; 360S stays the
    # default so a workspace that never ran install.sh behaves as before.
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360s_launch.py")
        )
    )

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="laser_mapping",
        output="screen",
        parameters=[
            PathJoinSubstitution([config_path, config_file]),
            {
                "use_sim_time": use_sim_time,
                # Always save the map inside this workspace, even when the
                # process is started from another directory.
                "map_file_path": str(workspace / "maps" / "mid360_map.pcd"),
            },
        ],
    )

    pgo = Node(
        package="fastlio_lc_pgo",
        executable="pgo_node",
        name="laserPGO",
        output="screen",
        parameters=[
            {
                "use_sim_time": False,
                "save_directory": save_directory,
                "cloud_topic": "/cloud_registered_body",
                "odom_topic": "/Odometry",
                "map_frame": "camera_init",
                "keyframe_meter_gap": 0.5,
                "keyframe_deg_gap": 10.0,
                "sc_dist_thres": 0.3,
                "sc_max_radius": 50.0,
                "historyKeyframeSearchRadius": 5.0,
                "historyKeyframeSearchTimeDiff": 30.0,
                "historyKeyframeSearchNum": 25,
                "loopClosureFrequency": 2.0,
                "graphUpdateFrequency": 1.0,
                "graphUpdateTimes": 5,
                "loopNoiseScore": 0.1,
                "loopFitnessScoreThreshold": 0.3,
                "vizmapFrequency": 0.2,
                "vizPathFrequency": 5.0,
                "mapviz_filter_size": 0.2,
                "speedFactor": 1.0,
            }
        ],
    )

    live_ground_grid = Node(
        package="bw_fastlio_nav2",
        executable="laser_map_ground_grid",
        name="live_patchwork_ground_segmentation",
        output="screen",
        condition=IfCondition(obstacle_grid),
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/cloud_registered_body",
                "grid_topic": "/obstacle_grid_live",
                "ground_topic": "/ground_points_live",
                "obstacle_topic": "/obstacle_points_live",
                "voxel_leaf_size": 0.05,
                "sensor_height": 0.18,
                "num_iter": 3,
                "num_lpr": 20,
                "num_min_points": 10,
                "seed_threshold": 0.12,
                "distance_threshold": 0.08,
                "min_range": 0.05,
                "max_range": 15.0,
                "uprightness_threshold": 0.866,
                "enable_reflected_noise_removal": False,
                "obstacle_min_height": 0.10,
                "obstacle_max_height": 1.80,
                "outlier_radius": 0.20,
                "outlier_min_neighbors": 3,
                "resolution": 0.05,
                "min_obstacle_neighbors": 3,
                "min_update_period_sec": 0.10,
                "self_filter_enabled": True,
                "self_filter_min_x": -0.50,
                "self_filter_max_x": 0.17,
                "self_filter_min_y": -0.35,
                "self_filter_max_y": 0.35,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(integration_share, "rviz", "mid360_pgo.rviz")],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "save_directory",
                default_value=str(workspace / "maps/pgo_session"),
                description="PGO keyframes, pose graph and optimized map output directory.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with raw and PGO map displays.",
            ),
            DeclareLaunchArgument(
                "obstacle_grid",
                default_value="true",
                description="Run Patchwork++ on live body-frame scans for visualization.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock.",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value=os.path.join(fast_lio_share, "config"),
                description="FAST-LIO configuration directory.",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value="mid360.yaml",
                description="FAST-LIO configuration file.",
            ),
            livox_driver,
            TimerAction(period=3.0, actions=[fast_lio]),
            TimerAction(period=4.0, actions=[live_ground_grid]),
            TimerAction(period=5.0, actions=[pgo, rviz_node]),
        ]
    )
