import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory('fast_lio')
    pgo_share = get_package_share_directory('fastlio_lc_pgo')
    default_rviz_cfg = os.path.join(pgo_share, 'rviz', 'fastlio_lc.rviz')

    save_directory = LaunchConfiguration('save_directory')
    rviz = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    declare_save_directory_cmd = DeclareLaunchArgument(
        'save_directory', default_value='/home/yoha/Lidar/run_corridor_lc/pgo_output/',
        description='Directory where PGO writes optimized poses, odom poses, times and keyframe scans (its Scans/ subfolder is wiped on startup)'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 with both the raw FAST-LIO view and the loop-closure (PGO) view pre-configured '
                     '(/aft_pgo_map, /aft_pgo_path, /loop_closure_constraints).'
    )
    declare_rviz_cfg_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_cfg,
        description='RViz config file path'
    )

    fast_lio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, 'launch', 'mapping.launch.py')
        ),
        launch_arguments={
            'config_file': 'legkilo.yaml',
            'rviz': rviz,
            'rviz_cfg': rviz_cfg,
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
            'odom_topic': '/Odometry',

            # keyframe selection
            'keyframe_meter_gap': 1.0,
            'keyframe_deg_gap': 10.0,

            # Scan Context (indoor corridor -> smaller radius than outdoor default)
            'sc_dist_thres': 0.4,
            'sc_max_radius': 20.0,

            # loop closure
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
    ld.add_action(fast_lio_launch)
    ld.add_action(pgo_node)
    return ld
