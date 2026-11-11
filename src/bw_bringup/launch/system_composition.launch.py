import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from bw_utils import get_robot_sn
from bw_utils.robot_profile import (
    load_robot_profile,
    normalize_robot_version,
    robot_profile_to_pelvis_params,
)


def _load_smoother_ros_params(param_file_path: str, robot_sn: str) -> dict:
    """从参数文件中提取 smoother_node 的 ros__parameters。"""
    with open(param_file_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    if not isinstance(config, dict):
        raise RuntimeError(f"Smoother 参数文件格式错误: {param_file_path}")

    # 支持常见写法，优先精确匹配 SN 对应节点
    candidates = [
        f"/{robot_sn}/smoother_node",
        f"{robot_sn}/smoother_node",
        "smoother_node",
    ]

    for key in candidates:
        node_cfg = config.get(key)
        if isinstance(node_cfg, dict):
            ros_params = node_cfg.get("ros__parameters")
            if isinstance(ros_params, dict):
                return ros_params

    # 支持嵌套写法: <sn>: { smoother_node: { ros__parameters: ... } }
    for sn_key in (robot_sn, f"/{robot_sn}"):
        sn_cfg = config.get(sn_key)
        if isinstance(sn_cfg, dict):
            node_cfg = sn_cfg.get("smoother_node")
            if isinstance(node_cfg, dict):
                ros_params = node_cfg.get("ros__parameters")
                if isinstance(ros_params, dict):
                    return ros_params

    raise RuntimeError(
        f"无法在 {param_file_path} 中找到 smoother_node.ros__parameters，"
        f"请检查节点键是否为 smoother_node 或 /{robot_sn}/smoother_node"
    )


def _load_teleop_ros_params(param_file_path: str, robot_sn: str) -> dict:
    """从参数文件中提取 teleop_component 的 ros__parameters。"""
    with open(param_file_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f) or {}

    if not isinstance(config, dict):
        raise RuntimeError(f"Teleop 参数文件格式错误: {param_file_path}")

    candidates = [
        f"/{robot_sn}/teleop_component",
        f"{robot_sn}/teleop_component",
        "teleop_component",
    ]

    for key in candidates:
        node_cfg = config.get(key)
        if isinstance(node_cfg, dict):
            ros_params = node_cfg.get("ros__parameters")
            if isinstance(ros_params, dict):
                return ros_params

    for sn_key in (robot_sn, f"/{robot_sn}"):
        sn_cfg = config.get(sn_key)
        if isinstance(sn_cfg, dict):
            node_cfg = sn_cfg.get("teleop_component")
            if isinstance(node_cfg, dict):
                ros_params = node_cfg.get("ros__parameters")
                if isinstance(ros_params, dict):
                    return ros_params

    raise RuntimeError(
        f"无法在 {param_file_path} 中找到 teleop_component.ros__parameters，"
        f"请检查节点键是否为 teleop_component 或 /{robot_sn}/teleop_component"
    )


def launch_setup(context, *args, **kwargs):
    """启动所有输入链路，mode 仅作为 control_router 的默认输入源。"""

    teleop_params = LaunchConfiguration("teleop_params").perform(context)
    mode = LaunchConfiguration('mode').perform(context)
    port_name = LaunchConfiguration('port_name').perform(context)
    baud_rate = int(LaunchConfiguration('baud_rate').perform(context))
    mantis_protocol_version = LaunchConfiguration('mantis_protocol_version').perform(context)
    robot_version = normalize_robot_version(
        LaunchConfiguration('robot_version').perform(context)
    )
    publish_frequency = int(LaunchConfiguration('publish_frequency').perform(context))
    joystick_deadzone = float(LaunchConfiguration('joystick_deadzone').perform(context))
    force_handle_joy_max_abs = float(
        LaunchConfiguration('force_handle_joy_max_abs').perform(context)
    )
    force_handle_joystick_deadzone = float(
        LaunchConfiguration('force_handle_joystick_deadzone').perform(context)
    )
    sdk_backend = LaunchConfiguration('sdk_backend').perform(context)
    left_gripper_invert = LaunchConfiguration('left_gripper_invert').perform(context).lower() in ('true', '1', 'yes')
    right_gripper_invert = LaunchConfiguration('right_gripper_invert').perform(context).lower() in ('true', '1', 'yes')
    smoother_params = LaunchConfiguration('smoother_params').perform(context)
    smoother_output_frequency = LaunchConfiguration('smoother_output_frequency').perform(context)
    ik_joint_continuity_weight = float(
        LaunchConfiguration('ik_joint_continuity_weight').perform(context)
    )
    ik_continuity_reference_max_age_ms = float(
        LaunchConfiguration('ik_continuity_reference_max_age_ms').perform(context)
    )
    ros_ip = LaunchConfiguration('ros_ip').perform(context)
    audio_dir = LaunchConfiguration('audio_dir').perform(context)
    remote_agent_enable = LaunchConfiguration('remote_agent_enable').perform(context).lower() in ('true', '1', 'yes')
    control_plane_url = LaunchConfiguration('control_plane_url').perform(context)
    wg_server_public_key = LaunchConfiguration('wg_server_public_key').perform(context)
    wg_server_endpoint = LaunchConfiguration('wg_server_endpoint').perform(context)
    remote_agent_robot_key = LaunchConfiguration('remote_agent_robot_key').perform(context)
    remote_agent_robot_key_path = LaunchConfiguration('remote_agent_robot_key_path').perform(context)
    wg_private_key_path = LaunchConfiguration('wg_private_key_path').perform(context)
    wg_public_key_path = LaunchConfiguration('wg_public_key_path').perform(context)
    wg_config_path = LaunchConfiguration('wg_config_path').perform(context)
    wg_apply_mode = LaunchConfiguration('wg_apply_mode').perform(context)
    public_endpoint = LaunchConfiguration('public_endpoint').perform(context)
    action_recorder_enable = LaunchConfiguration('action_recorder_enable').perform(context).lower() in ('true', '1', 'yes')
    vr_frequency_monitor_enable = LaunchConfiguration('vr_frequency_monitor_enable').perform(context).lower() in ('true', '1', 'yes')
    action_recorder_action_dir = LaunchConfiguration('action_recorder_action_dir').perform(context)
    action_recorder_sample_rate_hz = float(LaunchConfiguration('action_recorder_sample_rate_hz').perform(context))
    force_handle_bind_ip = LaunchConfiguration('force_handle_bind_ip').perform(context)
    force_handle_right_bind_port = int(
        LaunchConfiguration('force_handle_right_bind_port').perform(context)
    )
    force_handle_left_bind_port = int(
        LaunchConfiguration('force_handle_left_bind_port').perform(context)
    )
    force_handle_feedback_target_port = int(
        LaunchConfiguration('force_handle_feedback_target_port').perform(context)
    )

    robot_sn = get_robot_sn()
    bw_core_dir = get_package_share_directory("bw_core")
    print("-" * 60)
    print(f"core路径：{bw_core_dir}")
    print("-" * 60)

    protocol_key = mantis_protocol_version.lower()
    force_handle_feedback_joint_state_topic = "joint_states_fdb"
    if protocol_key == "v2":
        force_handle_feedback_joint_state_topic = "joint_states_fdb_V2"
    elif protocol_key == "v3":
        force_handle_feedback_joint_state_topic = "joint_states_fdb_V3"

    if smoother_params == "":
        default_smoother_name = "smoother.yaml"
        actual_smoother_params = os.path.join(bw_core_dir, "config", default_smoother_name)
    else:
        actual_smoother_params = smoother_params

    smoother_ros_params = dict(_load_smoother_ros_params(actual_smoother_params, robot_sn))
    smoother_ros_params["input_topic"] = "ctrl/selected_joint_angle_solution"
    smoother_ros_params["output_topic"] = "Teleop/joint_angle_solution/smooth"
    smoother_ros_params["motion_profile_topic"] = "ctrl/selected_motion_profile"
    smoother_ros_params["reset_status_topic"] = "system/is_resetting"
    if smoother_output_frequency != "":
        smoother_ros_params["output_frequency"] = float(smoother_output_frequency)
    print(
        f"[smoother] router_default_source={mode}, param_file={actual_smoother_params}, "
        f"output_frequency={smoother_ros_params.get('output_frequency')}, sn={robot_sn}"
    )

    if teleop_params == "":
        actual_teleop_params = os.path.join(bw_core_dir, "config", "teleop.yaml")
    else:
        actual_teleop_params = teleop_params
    teleop_ros_params = dict(_load_teleop_ros_params(actual_teleop_params, robot_sn))

    for key in (
        "ee_reset_x",
        "left_ee_reset_y",
        "right_ee_reset_y",
        "ee_reset_z",
        "left_ee_reset_qx",
        "left_ee_reset_qy",
        "left_ee_reset_qz",
        "left_ee_reset_qw",
        "right_ee_reset_qx",
        "right_ee_reset_qy",
        "right_ee_reset_qz",
        "right_ee_reset_qw",
    ):
        raw = LaunchConfiguration(key).perform(context)
        if raw != "":
            teleop_ros_params[key] = float(raw)

    gripper_mode_converter_ros_params = {
        "ee_reset_x": teleop_ros_params.get("ee_reset_x", 0.27),
        "left_ee_reset_y": teleop_ros_params.get("left_ee_reset_y", 0.25),
        "right_ee_reset_y": teleop_ros_params.get("right_ee_reset_y", -0.25),
        "ee_reset_z": teleop_ros_params.get("ee_reset_z", -0.15),
    }
    print(f"[teleop] param_file={actual_teleop_params}, sn={robot_sn}")

    robot_profile = load_robot_profile(robot_version, bw_core_share=bw_core_dir)
    robot_version = robot_profile.robot_version
    ik_urdf_relative_path = robot_profile.ik_urdf_relative_path
    mantis_comm_pelvis_params = robot_profile_to_pelvis_params(robot_profile)
    print(
        f"[robot_profiles] robot_version={robot_version}, "
        f"pelvis_default={robot_profile.pelvis_default_height_mm:.0f}mm, "
        f"ik_urdf={ik_urdf_relative_path}, protocol={mantis_protocol_version}"
    )

    composable_nodes = []
    extra_actions = []

    if sdk_backend == "sim":
        extra_actions.append(
            Node(
                package="bw_sim2real",
                executable="sdk_sim_backend_node",
                name="mantis_comm_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "protocol_version": mantis_protocol_version,
                        "robot_version": robot_version,
                        "publish_frequency": publish_frequency,
                        **mantis_comm_pelvis_params,
                    }
                ],
                output="screen",
            )
        )
    else:
        composable_nodes.append(
            ComposableNode(
                package="bw_serial",
                plugin="bw_serial::MantisCommNode",
                name="mantis_comm_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "port_name": port_name,
                        "baud_rate": baud_rate,
                        "publish_frequency": publish_frequency,
                        "protocol_version": mantis_protocol_version,
                        "robot_version": robot_version,
                        "left_gripper_invert": left_gripper_invert,
                        "right_gripper_invert": right_gripper_invert,
                        **mantis_comm_pelvis_params,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    composable_nodes.extend(
        [
            ComposableNode(
                package="bw_vr_bridge",
                plugin="bw_vr_bridge::VrBridgeNode",
                name="vr_bridge_node",
                namespace=robot_sn,
                parameters=[{"ip": "0.0.0.0", "port": 12345, "frame_id": "world"}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_vr_input",
                plugin="bw_vr_input::VrInputNode",
                name="vr_input_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "protocol_version": mantis_protocol_version,
                        "joystick_deadzone": joystick_deadzone,
                        "left_pose_raw_topic": "input/vr/left_pose/raw",
                        "right_pose_raw_topic": "input/vr/right_pose/raw",
                        "head_pose_raw_topic": "input/vr/head_pose/raw",
                        "left_ctrl_topic": "input/vr/ctrl_cmd_left",
                        "right_ctrl_topic": "input/vr/ctrl_cmd_right",
                        "head_pose_topic": "input/vr/head_pose",
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_force_handle_bridge",
                plugin="bw_force_handle_bridge::ForceHandleBridgeNode",
                name="force_handle_bridge_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "bind_ip": force_handle_bind_ip,
                        "right_bind_port": force_handle_right_bind_port,
                        "left_bind_port": force_handle_left_bind_port,
                        "joy_max_abs": force_handle_joy_max_abs,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_force_handle_bridge",
                plugin="bw_force_handle_bridge::ForceHandleFeedbackNode",
                name="force_handle_feedback_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "source_endpoint_topic": "_force_handle/source_endpoint",
                        "feedback_topic": force_handle_feedback_joint_state_topic,
                        "target_port": force_handle_feedback_target_port,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_force_handle_input",
                plugin="bw_force_handle_input::ForceHandleInputNode",
                name="force_handle_input_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "joy_max_abs": force_handle_joy_max_abs,
                        "joystick_deadzone": force_handle_joystick_deadzone,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_teleop",
                plugin="bw_teleop::TeleopComponent",
                name="teleop_component",
                namespace=robot_sn,
                parameters=[
                    teleop_ros_params,
                    {
                        "gripper_output_topic": "input/teleop/gripper_pos",
                        "cmd_vel_output_topic": "input/teleop/cmd_vel",
                        "pelvis_output_topic": "input/teleop/pelvis_speed",
                        "waist_output_topic": "input/teleop/waist_speed",
                    },
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_teleop",
                plugin="bw_teleop::node::GripperModeConverterComponent",
                name="gripper_mode_converter_component",
                namespace=robot_sn,
                parameters=[
                    gripper_mode_converter_ros_params,
                    {
                        "left_fixed_orientation_x": 0.5,
                        "left_fixed_orientation_y": 0.5,
                        "left_fixed_orientation_z": -0.5,
                        "left_fixed_orientation_w": 0.5,
                        "right_fixed_orientation_x": -0.5,
                        "right_fixed_orientation_y": 0.5,
                        "right_fixed_orientation_z": 0.5,
                        "right_fixed_orientation_w": 0.5,
                    }
                ],
            ),
            ComposableNode(
                package="bw_core",
                plugin="bw_core::ArmCommandResolverNode",
                name="arm_command_resolver_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "input_topic": "ctrl/selected_arm_command",
                        "output_topic": "ctrl/selected_joint_angle_solution",
                        "status_topic": "sdk/arm_command_status",
                        "continuity_reference_topic": "Teleop/joint_angle_solution/smooth",
                        "continuity_reference_max_age_ms": ik_continuity_reference_max_age_ms,
                        "ik_joint_continuity_weight": ik_joint_continuity_weight,
                        "ik_urdf_relative_path": ik_urdf_relative_path,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="bw_core",
                plugin="bw_core::SmootherNode",
                name="smoother_node",
                namespace=robot_sn,
                parameters=[smoother_ros_params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ]
    )

    extra_actions.extend(
        [
            Node(
                package="bw_status_machine",
                executable="state_machine",
                name="state_machine",
                namespace=robot_sn,
                parameters=[{"mode": mode, "protocol_version": mantis_protocol_version}],
                output="screen",
            ),
            Node(
                package="bw_utils",
                executable="sn_publisher_node",
                name="sn_publisher_node",
                namespace="",
                output="screen",
            ),
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="ros_tcp_endpoint",
                namespace=robot_sn,
                parameters=[{"ROS_IP": ros_ip}],
                output="screen",
                respawn=True,
                respawn_delay=1.0,
            ),
            Node(
                package="bw_sdk_input",
                executable="sdk_input_node",
                name="sdk_input_node",
                namespace=robot_sn,
                parameters=[{
                    "auto_enable": mode == "sdk",
                    "protocol_version": mantis_protocol_version,
                }],
                output="screen",
            ),
            Node(
                package="bw_control_router",
                executable="control_router_node",
                name="control_router_node",
                namespace=robot_sn,
                parameters=[{"default_source": mode}],
                output="screen",
            ),
            Node(
                package="bw_control_router",
                executable="teleop_input_selector_node",
                name="teleop_input_selector_node",
                namespace=robot_sn,
                parameters=[{"default_source": mode}],
                output="screen",
            ),
            Node(
                package="bw_control_router",
                executable="robot_command_dispatcher_node",
                name="robot_command_dispatcher_node",
                namespace=robot_sn,
                parameters=[{"input_topic": "ctrl/selected_robot_command"}],
                output="screen",
            ),
            Node(
                package="bw_core",
                executable="sdk_arm_command_tracker",
                name="sdk_arm_command_tracker",
                namespace=robot_sn,
                parameters=[
                    {
                        "status_topic": "sdk/arm_command_status",
                        "feedback_topics": [
                            "joint_states_fdb",
                            "joint_states_fdb_V2",
                            "joint_states_fdb_V3",
                        ],
                        "legacy_feedback_uses_urdf_direction": sdk_backend != "sim",
                    }
                ],
                output="screen",
            ),
        ]
    )

    if remote_agent_enable:
        extra_actions.append(
            Node(
                package="bw_remote",
                executable="remote_wg_agent",
                name="remote_wg_agent",
                namespace=robot_sn,
                parameters=[
                    {
                        "control_plane_url": control_plane_url,
                        "wg_server_public_key": wg_server_public_key,
                        "wg_server_endpoint": wg_server_endpoint,
                        "robot_key": remote_agent_robot_key,
                        "robot_key_path": remote_agent_robot_key_path,
                        "wg_private_key_path": wg_private_key_path,
                        "wg_public_key_path": wg_public_key_path,
                        "wg_config_path": wg_config_path,
                        "wg_apply_mode": wg_apply_mode,
                        "public_endpoint": public_endpoint,
                    }
                ],
                output="screen",
            )
        )

    if action_recorder_enable:
        extra_actions.append(
            Node(
                package="bw_action_recorder",
                executable="recording_node",
                name="action_recorder_node",
                namespace=robot_sn,
                parameters=[
                    {
                        "action_dir": action_recorder_action_dir,
                        "sample_rate_hz": action_recorder_sample_rate_hz,
                    }
                ],
                output="screen",
            )
        )

    if vr_frequency_monitor_enable:
        extra_actions.append(
            Node(
                package="bw_status_machine",
                executable="topic_frequency_monitor",
                name="vr_topic_frequency_monitor",
                namespace=robot_sn,
                parameters=[
                    {
                        "protocol_version": mantis_protocol_version,
                        "report_period_sec": 2.0,
                        "stale_timeout_sec": 1.0,
                        "only_report_when_vr_active": True,
                        "log_to_console": False,
                        "publish_report": True,
                    }
                ],
                output="screen",
            )
        )

    if mode == "motion":
        extra_actions.append(
            Node(
                package="bw_action_module",
                executable="service_proxy_node",
                name="service_proxy_node",
                namespace=robot_sn,
                output="screen",
            )
        )

    container = ComposableNodeContainer(
        name="system_container",
        namespace=robot_sn,
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=composable_nodes,
        output="screen",
    )

    return [container] + extra_actions

def generate_launch_description():
    """生成包含三种模式选择的启动描述。"""
    ip_address = "120.26.207.141"
    # 声明启动参数
    return LaunchDescription([
        DeclareLaunchArgument(
            'mode',
            default_value='vr',
            description='默认输入源: vr/sdk/action_studio/web/file/force_handle/none；web 等价于 action_studio'
        ),
        DeclareLaunchArgument(
            'ros_ip',
            default_value='192.168.1.111',
            description='VR模式下 TCP Endpoint 使用的 IP 地址'
        ),
        DeclareLaunchArgument(
            'smoother_params',
            default_value='',
            description='Smoother 节点的参数文件路径 (如果不提供，将使用统一 smoother.yaml)'
        ),
        DeclareLaunchArgument(
            'smoother_output_frequency',
            default_value='',
            description='Smoother 输出频率 (Hz)，覆盖 smoother.yaml 中的 output_frequency；留空则使用参数文件默认值'
        ),
        DeclareLaunchArgument(
            'ik_joint_continuity_weight',
            default_value='1.0',
            description='IK 当前关节构型连续性代价权重；越大越倾向选择接近当前构型的解'
        ),
        DeclareLaunchArgument(
            'ik_continuity_reference_max_age_ms',
            default_value='100.0',
            description='IK 使用 smoother 当前关节构型作为参考的最大允许数据年龄 (ms)'
        ),
        DeclareLaunchArgument(
            'teleop_params',
            default_value='',
            description='Teleop 节点的参数文件路径 (如果不提供，将使用默认 teleop.yaml)'
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='/dev/ttyACM0',
            description='串口设备名称'
        ),
        DeclareLaunchArgument(
            'baud_rate',
            default_value='2000000',
            description='串口波特率'
        ),
        DeclareLaunchArgument(
            'publish_frequency',
            default_value='500',
            description='串口通信频率'
        ),
        DeclareLaunchArgument(
            'robot_version',
            default_value='2.0',
            description='机器人型号：1.0（研发预留）/ 2.0 / 3.0；决定滑台默认高度、IK/RViz 模型'
        ),
        DeclareLaunchArgument(
            'mantis_protocol_version',
            default_value='v3',
            description='下位机通信协议版本：v1 / v2 / v3（与 robot_version 独立）'
        ),
        DeclareLaunchArgument(
            'sdk_backend',
            default_value='real',
            description='VR/SDK 模式执行后端：real（真实串口）/ sim（仿真后端）'
        ),
        DeclareLaunchArgument(
            'joystick_deadzone',
            default_value='0.5',
            description='VR手柄摇杆死区 (0.0-1.0)，用于防止手抖误触'
        ),
        DeclareLaunchArgument(
            'force_handle_joy_max_abs',
            default_value='1000.0',
            description='力反馈手柄摇杆原始值满量程绝对值（ESP32 通常为 ±1000）'
        ),
        DeclareLaunchArgument(
            'force_handle_joystick_deadzone',
            default_value='0.15',
            description='力反馈手柄摇杆死区 (0.0-1.0)，与 VR 死区分设'
        ),
        DeclareLaunchArgument(
            'left_gripper_invert',
            default_value='false',
            description='是否在发送给下位机前反相左夹爪命令'
        ),
        DeclareLaunchArgument(
            'right_gripper_invert',
            default_value='false',
            description='是否在发送给下位机前反相右夹爪命令'
        ),
        DeclareLaunchArgument('ee_reset_x', default_value='', description='覆盖初始末端 X 位置（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('left_ee_reset_y', default_value='', description='覆盖左臂初始末端 Y 位置（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('right_ee_reset_y', default_value='', description='覆盖右臂初始末端 Y 位置（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('ee_reset_z', default_value='', description='覆盖初始末端 Z 位置（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('left_ee_reset_qx', default_value='', description='覆盖左臂初始末端姿态四元数 x（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('left_ee_reset_qy', default_value='', description='覆盖左臂初始末端姿态四元数 y（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('left_ee_reset_qz', default_value='', description='覆盖左臂初始末端姿态四元数 z（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('left_ee_reset_qw', default_value='', description='覆盖左臂初始末端姿态四元数 w（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('right_ee_reset_qx', default_value='', description='覆盖右臂初始末端姿态四元数 x（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('right_ee_reset_qy', default_value='', description='覆盖右臂初始末端姿态四元数 y（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('right_ee_reset_qz', default_value='', description='覆盖右臂初始末端姿态四元数 z（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument('right_ee_reset_qw', default_value='', description='覆盖右臂初始末端姿态四元数 w（空字符串表示使用 teleop 参数文件）'),
        DeclareLaunchArgument(
            'audio_dir',
            default_value='',
            description='预制音频目录（空为默认 ~/bw_teleoperate_ws/robot_admin_system/data/audio）'
        ),
        DeclareLaunchArgument(
            'remote_agent_enable',
            default_value='true',
            description='是否启动机器人公网接入代理节点 remote_wg_agent'
        ),
        DeclareLaunchArgument(
            'control_plane_url',
            default_value=f'http://{ip_address}:5433',
            description='控制面 API 基地址'
        ),
        DeclareLaunchArgument(
            'wg_server_public_key',
            default_value='HVVN7eSWQToDYRUTWGEtTJL12qIGh4eJNhKA4sjkQTk=',
            description='服务器 WireGuard 公钥（remote_wg_agent 必填）'
        ),
        DeclareLaunchArgument(
            'wg_server_endpoint',
            default_value=f'{ip_address}:51820',
            description='服务器 WireGuard 入口（ip:port，remote_wg_agent 必填）'
        ),
        DeclareLaunchArgument(
            'remote_agent_robot_key',
            default_value='69fa727f5b38c614645b350b1e06b343e212dc8728b43a130cd81f9a17129eab',
            description='机器人登录控制面使用的 robot key（联调方便，生产更建议使用文件路径）'
        ),
        DeclareLaunchArgument(
            'remote_agent_robot_key_path',
            default_value='',
            description='机器人 robot key 文件路径（remote_wg_agent 推荐）'
        ),
        DeclareLaunchArgument(
            'wg_private_key_path',
            default_value='/etc/wireguard/robot.key',
            description='机器人 WireGuard 私钥路径'
        ),
        DeclareLaunchArgument(
            'wg_public_key_path',
            default_value='/etc/wireguard/robot.pub',
            description='机器人 WireGuard 公钥路径（缺失时自动派生）'
        ),
        DeclareLaunchArgument(
            'wg_config_path',
            default_value='/etc/wireguard/wg0.conf',
            description='机器人 WireGuard 配置写入路径'
        ),
        DeclareLaunchArgument(
            'wg_apply_mode',
            default_value='wg-quick' if os.getenv('container') == 'docker' or os.path.exists('/.dockerenv') else 'none',
            description='WireGuard 应用方式：systemd/wg-quick/none'
        ),
        DeclareLaunchArgument(
            'public_endpoint',
            default_value='',
            description='机器人公网入口（可选，NAT 场景可留空）'
        ),
        DeclareLaunchArgument(
            'action_recorder_enable',
            default_value='true',
            description='是否启动 VR 菜单动作录制节点 action_recorder_node'
        ),
        DeclareLaunchArgument(
            'action_recorder_action_dir',
            default_value='/home/lanchong/BW/bw_teleoperate_ws/robot_admin_system/data/action',
            description='动作录制输出目录，包含 actions.json 和 packages/'
        ),
        DeclareLaunchArgument(
            'action_recorder_sample_rate_hz',
            default_value='50.0',
            description='轨迹录制采样频率'
        ),
        DeclareLaunchArgument(
            'vr_frequency_monitor_enable',
            default_value='true',
            description='是否启动 VR 控制链路 topic 频率监控节点'
        ),
        DeclareLaunchArgument(
            'force_handle_bind_ip',
            default_value='0.0.0.0',
            description='力反馈手柄 UDP bridge 监听地址'
        ),
        DeclareLaunchArgument(
            'force_handle_right_bind_port',
            default_value='12346',
            description='右手力反馈手柄 UDP bridge 监听端口'
        ),
        DeclareLaunchArgument(
            'force_handle_left_bind_port',
            default_value='12347',
            description='左手力反馈手柄 UDP bridge 监听端口'
        ),
        DeclareLaunchArgument(
            'force_handle_feedback_target_port',
            default_value='12346',
            description='ESP32 接收 ROS feedback 的 UDP 端口'
        ),
        # 使用 OpaqueFunction 来实现根据参数动态选择节点
        OpaqueFunction(function=launch_setup)
    ])
