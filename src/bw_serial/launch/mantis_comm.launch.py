from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ############## Parameters ##############
    baud_rate = LaunchConfiguration("baud_rate")
    port_name = LaunchConfiguration("port_name")
    publish_frequency = LaunchConfiguration("publish_frequency")
    protocol_version = LaunchConfiguration("protocol_version")
    left_gripper_invert = LaunchConfiguration("left_gripper_invert")
    right_gripper_invert = LaunchConfiguration("right_gripper_invert")

    declare_port_name_cmd = DeclareLaunchArgument(
        "port_name",
        # default_value="/dev/LiXin_USB2TTL",
        default_value="/dev/ttyACM0",
        description="USB-TTL device name",
    )
    declare_baud_rate_cmd = DeclareLaunchArgument(
        "baud_rate",
        default_value="2000000",
        description="baud rate of serial communication",
    )
    declare_publish_frequency_cmd = DeclareLaunchArgument(
        "publish_frequency",
        default_value="1000",
        description="communication frequency between PC and MCU",
    )
    declare_protocol_version_cmd = DeclareLaunchArgument(
        "protocol_version",
        default_value="v2",
        description="mantis communication protocol version: v1/v2/v3",
    )
    declare_left_gripper_invert_cmd = DeclareLaunchArgument(
        "left_gripper_invert",
        default_value="false",
        description="Invert left gripper command before sending to MCU",
    )
    declare_right_gripper_invert_cmd = DeclareLaunchArgument(
        "right_gripper_invert",
        default_value="false",
        description="Invert right gripper command before sending to MCU",
    )

    ############## Nodes ##############
    # Test: Only Transmit Command Frame when ALLOWED.
    mantis_comm_once_node = Node(
        package="bw_serial",
        executable="mantis_comm_once_node",
        name="mantis_comm_once_node",
        parameters=[
            {
                "port_name": port_name,
                "baud_rate": baud_rate,
                "publish_frequency": publish_frequency,
                "protocol_version": protocol_version,
                "left_gripper_invert": left_gripper_invert,
                "right_gripper_invert": right_gripper_invert,
            }
        ],
        output="screen",
    )

    mantis_comm_node = Node(
        package="bw_serial",
        executable="mantis_comm_node",
        name="mantis_comm_node",
        parameters=[
            {
                "port_name": port_name,
                "baud_rate": baud_rate,
                "publish_frequency": publish_frequency,
                "protocol_version": protocol_version,
                "left_gripper_invert": left_gripper_invert,
                "right_gripper_invert": right_gripper_invert,
            }
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(declare_port_name_cmd)
    ld.add_action(declare_baud_rate_cmd)
    ld.add_action(declare_publish_frequency_cmd)
    ld.add_action(declare_protocol_version_cmd)
    ld.add_action(declare_left_gripper_invert_cmd)
    ld.add_action(declare_right_gripper_invert_cmd)
    ld.add_action(mantis_comm_node)
    # ld.add_action(mantis_comm_once_node)

    return ld
