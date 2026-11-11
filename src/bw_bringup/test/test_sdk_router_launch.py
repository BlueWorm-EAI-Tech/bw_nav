from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / "launch" / "system_composition.launch.py"
TEXT = LAUNCH.read_text(encoding="utf-8")


def test_bringup_routes_one_smoother_after_control_router():
    assert 'smoother_ros_params["input_topic"] = "ctrl/selected_joint_angle_solution"' in TEXT
    assert 'smoother_ros_params["output_topic"] = "Teleop/joint_angle_solution/smooth"' in TEXT
    assert "stage2_output_topic" not in TEXT
    assert 'smoother_ros_params["reset_status_topic"] = "system/is_resetting"' in TEXT
    assert 'input_mode_state_topic' not in TEXT
    assert 'vr_profile_config_file' not in TEXT
    assert 'sdk_profile_config_file' not in TEXT


def test_bringup_supports_smoother_output_frequency_override():
    assert "'smoother_output_frequency'" in TEXT
    assert 'smoother_ros_params["output_frequency"] = float(smoother_output_frequency)' in TEXT


def test_bringup_uses_resolver_as_the_only_main_ik_entry():
    assert "bw_core::IKNode" not in TEXT
    assert "input/teleop/joint_angle_solution" not in TEXT
    assert 'plugin="bw_core::ArmCommandResolverNode"' in TEXT
    assert '"input_topic": "ctrl/selected_arm_command"' in TEXT
    assert '"input_topic": "ctrl/selected_robot_command"' in TEXT
    assert '"gripper_output_topic": "input/teleop/gripper_pos"' in TEXT
    assert '"cmd_vel_output_topic": "input/teleop/cmd_vel"' in TEXT


def test_bringup_always_starts_sdk_bridge_and_router_uses_mode_as_default_source():
    assert 'package="bw_sdk_input"' in TEXT
    assert 'executable="sdk_input_node"' in TEXT
    assert 'name="sdk_input_node"' in TEXT
    assert '"auto_enable": mode == "sdk"' in TEXT
    assert '"protocol_version": mantis_protocol_version' in TEXT
    assert 'package="bw_control_router"' in TEXT
    assert 'executable="control_router_node"' in TEXT
    assert 'executable="teleop_input_selector_node"' in TEXT
    assert 'executable="robot_command_dispatcher_node"' in TEXT
    assert 'parameters=[{"default_source": mode}]' in TEXT
    assert "vr/sdk/action_studio/web/file/force_handle/none" in TEXT


def test_bringup_loads_force_handle_chain_as_components():
    assert 'package="bw_force_handle_bridge"' in TEXT
    assert 'plugin="bw_force_handle_bridge::ForceHandleBridgeNode"' in TEXT
    assert 'plugin="bw_force_handle_bridge::ForceHandleFeedbackNode"' in TEXT
    assert 'package="bw_force_handle_input"' in TEXT
    assert 'plugin="bw_force_handle_input::ForceHandleInputNode"' in TEXT
    assert 'executable="force_handle_bridge_node"' not in TEXT
    assert 'executable="force_handle_input_node"' not in TEXT


def test_force_handle_joy_scale_is_applied_before_bridge_vr_message():
    bridge_index = TEXT.index('plugin="bw_force_handle_bridge::ForceHandleBridgeNode"')
    input_index = TEXT.index('plugin="bw_force_handle_input::ForceHandleInputNode"')
    bridge_block = TEXT[bridge_index:input_index]

    assert '"joy_max_abs": force_handle_joy_max_abs' in bridge_block


def test_force_handle_feedback_topic_follows_mantis_protocol_version():
    assert "force_handle_feedback_topic" not in TEXT
    assert 'protocol_key = mantis_protocol_version.lower()' in TEXT
    assert 'force_handle_feedback_joint_state_topic = "joint_states_fdb"' in TEXT
    assert 'force_handle_feedback_joint_state_topic = "joint_states_fdb_V2"' in TEXT
    assert 'force_handle_feedback_joint_state_topic = "joint_states_fdb_V3"' in TEXT
    assert '"feedback_topic": force_handle_feedback_joint_state_topic' in TEXT
