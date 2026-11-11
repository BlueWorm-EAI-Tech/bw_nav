import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    livox_share = get_package_share_directory("livox_ros_driver2")
    fast_lio_share = get_package_share_directory("fast_lio")
    rviz = LaunchConfiguration("rviz")

    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch_ROS2", "msg_MID360_launch.py")
        )
    )

    fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fast_lio_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "config_file": "mid360.yaml",
            "rviz": rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Start RViz with the FAST-LIO mapping display.",
            ),
            livox_driver,
            TimerAction(period=3.0, actions=[fast_lio]),
        ]
    )
