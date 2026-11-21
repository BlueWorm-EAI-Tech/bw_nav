from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('zones_file', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='bw_planning_zones', executable='planning_zone_server', output='screen',
             parameters=[{
                 'map_yaml': ParameterValue(LaunchConfiguration('map_yaml'), value_type=str),
                 'zones_file': ParameterValue(LaunchConfiguration('zones_file'), value_type=str),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
             }]),
    ])
