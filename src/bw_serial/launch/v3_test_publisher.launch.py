from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name = LaunchConfiguration("port_name")
    baud_rate = LaunchConfiguration("baud_rate")
    publish_frequency = LaunchConfiguration("publish_frequency")
    protocol_version = LaunchConfiguration("protocol_version")
    auto_enable = LaunchConfiguration("auto_enable")
    test_publish_rate = LaunchConfiguration("test_publish_rate")
    active_joint = LaunchConfiguration("active_joint")
    joint_amplitude = LaunchConfiguration("joint_amplitude")
    joint_period_sec = LaunchConfiguration("joint_period_sec")

    args = [
        DeclareLaunchArgument("port_name", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud_rate", default_value="115200"),
        DeclareLaunchArgument("publish_frequency", default_value="10"),
        DeclareLaunchArgument("protocol_version", default_value="v3"),
        DeclareLaunchArgument("auto_enable", default_value="true"),
        DeclareLaunchArgument("test_publish_rate", default_value="10.0"),
        DeclareLaunchArgument("active_joint", default_value="-1"),
        DeclareLaunchArgument("joint_amplitude", default_value="0.0"),
        DeclareLaunchArgument("joint_period_sec", default_value="4.0"),
    ]

    comm_node = Node(
        package="bw_serial",
        executable="mantis_comm_node",
        name="mantis_comm_node",
        parameters=[
            {
                "port_name": port_name,
                "baud_rate": baud_rate,
                "publish_frequency": publish_frequency,
                "protocol_version": protocol_version,
            }
        ],
        output="screen",
    )

    test_node = Node(
        package="bw_serial",
        executable="v3_test_publisher.py",
        name="v3_test_publisher",
        parameters=[
            {
                "publish_rate": test_publish_rate,
                "auto_enable": auto_enable,
                "ctrl_src_value": 1,
                "active_joint": active_joint,
                "joint_amplitude": joint_amplitude,
                "joint_period_sec": joint_period_sec,
            }
        ],
        output="screen",
    )

    return LaunchDescription(args + [comm_node, test_node])
