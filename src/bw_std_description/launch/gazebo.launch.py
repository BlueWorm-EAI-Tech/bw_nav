from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    robot = ET.parse(model).getroot()
    package_prefix = "package://bw_std_description/"
    for mesh in robot.findall(".//mesh"):
        uri = mesh.attrib.get("filename", "")
        if uri.startswith(package_prefix):
            resource = package_share / uri.removeprefix(package_prefix)
            mesh.attrib["filename"] = resource.resolve().as_uri()

    robot_description = ET.tostring(robot, encoding="unicode")

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {"robot_description": robot_description},
                {"use_sim_time": True},
            ],
        ),
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_std_robot",
            output="screen",
            arguments=[
                "-entity",
                LaunchConfiguration("entity_name"),
                "-topic",
                "robot_description",
                "-x",
                LaunchConfiguration("x"),
                "-y",
                LaunchConfiguration("y"),
                "-z",
                LaunchConfiguration("z"),
                "-Y",
                LaunchConfiguration("yaw"),
            ],
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("bw_std_description"))
    gazebo_share = Path(get_package_share_directory("gazebo_ros"))

    return LaunchDescription(
        [
            SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
            DeclareLaunchArgument(
                "robot_model",
                default_value="std_v1",
                choices=list(SUPPORTED_ROBOT_MODELS),
                description=(
                    "Model: std_v1 (full), std_v2 (dual arm), std_v3 (patrol), "
                    "std_v4 (engineering), std_v5 (single arm), std_v6 "
                    "(chassis), or std_auto (runtime full-body contract)"
                ),
            ),
            DeclareLaunchArgument(
                "model",
                default_value="",
                description="Optional absolute URDF path overriding robot_model",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="std_robot",
                description="Gazebo entity name",
            ),
            DeclareLaunchArgument(
                "gazebo_gui",
                default_value="true",
                description="Start the Gazebo client window",
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument(
                "z",
                default_value="0.05",
                description="Initial height above the ground plane",
            ),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(gazebo_share / "launch" / "gazebo.launch.py")
                ),
                launch_arguments={
                    "gui": LaunchConfiguration("gazebo_gui")
                }.items(),
            ),
            OpaqueFunction(function=launch_robot, args=[package_share]),
        ]
    )
