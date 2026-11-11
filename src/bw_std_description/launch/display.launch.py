from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SUPPORTED_ROBOT_MODELS = (
    "std_auto",
    "std_v1",
    "std_v2",
    "std_v3",
    "std_v4",
    "std_v5",
    "std_v6",
)


def launch_robot(context, package_share):
    robot_model = LaunchConfiguration("robot_model").perform(context).lower()
    model_override = LaunchConfiguration("model").perform(context)

    if robot_model not in SUPPORTED_ROBOT_MODELS:
        raise ValueError(
            f"Unsupported robot model '{robot_model}'. Choose one of: "
            + ", ".join(SUPPORTED_ROBOT_MODELS)
        )

    model = Path(model_override or package_share / "urdf" / f"{robot_model}.urdf")
    robot_description = model.read_text(encoding="utf-8")

    gui = LaunchConfiguration("gui")
    use_rviz = LaunchConfiguration("use_rviz")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen",
            condition=IfCondition(gui),
            arguments=[str(model)],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="screen",
            condition=UnlessCondition(gui),
            arguments=[str(model)],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", LaunchConfiguration("rviz_config")],
            condition=IfCondition(use_rviz),
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("bw_std_description"))

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_model",
                default_value="std_v1",
                choices=list(SUPPORTED_ROBOT_MODELS),
                description=(
                    "Display model: std_v1 (full), std_v2 (dual arm), "
                    "std_v3 (patrol), std_v4 (engineering), "
                    "std_v5 (single arm), std_v6 (chassis), or std_auto "
                    "for the runtime full-body contract"
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="Optional absolute URDF path overriding robot_model",
            ),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Start the joint state publisher GUI",
            ),
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(package_share / "rviz" / "std.rviz"),
                description="Absolute path to an RViz2 configuration file",
            ),
            OpaqueFunction(function=launch_robot, args=[package_share]),
        ]
    )
