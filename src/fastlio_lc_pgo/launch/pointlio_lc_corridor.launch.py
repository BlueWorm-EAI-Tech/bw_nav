import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    point_lio_share = get_package_share_directory('point_lio')
    pgo_share = get_package_share_directory('fastlio_lc_pgo')
    default_rviz_cfg = os.path.join(pgo_share, 'rviz', 'fastlio_lc.rviz')

    save_directory = LaunchConfiguration('save_directory')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory', default_value='/home/yoha/Lidar/run_corridor_pointlio_lc/pgo_output/',
        description='Directory where PGO writes optimized poses, odom poses, times and keyframe scans (its Scans/ subfolder is wiped on startup)'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with both the raw Point-LIO view and the loop-closure (PGO) view pre-configured.'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path (reuses the fastlio_lc rviz layout - Point-LIO publishes the same '
                     '/cloud_registered, /path, /Laser_map topic names as FAST-LIO does)'
    )

    # Point-LIO's own launch file is hardcoded to config/legkilo.yaml (no
    # config_file arg like fast_lio's mapping.launch.py), so just include it
    # directly - it already targets corridor's /points_raw + /imu_raw topics.
    point_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_share, 'launch', 'mapping_legkilo.launch.py')
        ),
        launch_arguments={
            'rviz': rviz,
        }.items()
    )

    pgo_node = Node(
        package='fastlio_lc_pgo',
        executable='pgo_node',
        name='laserPGO',
        output='screen',
        parameters=[{
            'save_directory': save_directory,
            'cloud_topic': '/cloud_registered_body',
            # Point-LIO (odom_only=false) publishes odometry on
            # /aft_mapped_to_init instead of FAST-LIO's /Odometry.
            'odom_topic': '/aft_mapped_to_init',

            # same tuning as the FAST-LIO corridor LC run, for a fair
            # apples-to-apples comparison
            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 10.0,

            'sc_dist_thres': 0.4,
            'sc_max_radius': 20.0,

            'historyKeyframeSearchRadius': 1.5,
            'historyKeyframeSearchTimeDiff': 30.0,
            'historyKeyframeSearchNum': 20,
            'speedFactor': 1.0,
            'loopClosureFrequency': 4.0,
            'graphUpdateFrequency': 2.0,
            'graphUpdateTimes': 5,
            'loopNoiseScore': 0.1,
            'vizmapFrequency': 10.0,
            'loopFitnessScoreThreshold': 0.3,
        }],
    )

    ld = LaunchDescription()
    ld.add_action(declare_save_directory_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_cfg_cmd)
    ld.add_action(point_lio_launch)
    ld.add_action(pgo_node)
    return ld
