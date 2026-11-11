from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
COMM_NODE = PKG / "src" / "mantis_comm_node.cpp"
V3_TEST_PUBLISHER = PKG / "scripts" / "v3_test_publisher.py"
V3_TEST_LAUNCH = PKG / "launch" / "v3_test_publisher.launch.py"


def test_v3_system_status_power_on_uses_software_power_bit_only():
    text = COMM_NODE.read_text(encoding="utf-8")

    assert "power_on_value = 0b01;" in text
    assert "power_on_value = 0b00011111;" not in text
    assert "power_on_value = this->requested_v3_control_flag_;" in text
    assert "this->requested_v3_control_flag_ != 0" not in text


def test_v3_set_ctrl_src_is_serialized_with_transmit_callback():
    text = COMM_NODE.read_text(encoding="utf-8")
    callback = text[text.index("void MantisCommNode::handle_set_ctrl_src"):]

    assert "std::lock_guard<std::mutex> cmd_lock(this->command_mutex_);" in callback
    assert callback.index("std::lock_guard<std::mutex>") < callback.index(
        "this->stage_v3_control_enable(request->value);"
    )


def test_v3_arm_enable_waits_for_fresh_joint_command_and_watchdog_is_not_undone():
    text = COMM_NODE.read_text(encoding="utf-8")

    assert "constexpr uint8_t V3_ARM_CONTROL_MASK = 0x18U;" in text
    assert "void MantisCommNode::stage_v3_control_enable" in text
    assert "if (requested_arm_bits == 0U)" in text
    assert "this->has_joint_command_ = false;" in text
    assert "v3_arm_enable_awaiting_fresh_command_" in text
    assert "command_stamp <= this->v3_arm_enable_request_time_" in text
    assert "V3_ARM_ENABLE_PRECHARGE_DURATION" in text
    assert "starting 10 ms V3 arm precharge" in text
    assert "V3 arm precharge completed" in text
    assert "!this->v3_arm_enable_precharge_active_" in text
    assert "this->protocol_version_ != MantisProtocolVersion::V3 || prev_state == 3" in text
    assert "this->mantis_command_v3_.control_flag = this->requested_v3_control_flag_;" in text


def test_v3_runtime_logs_effective_control_and_lower_controller_status():
    text = COMM_NODE.read_text(encoding="utf-8")

    assert "V3 effective control flag changed" in text
    assert "V3 feedback status changed" in text
    assert "left_motors=0x%02X right_motors=0x%02X" in text


def test_v3_non_arm_permission_change_preserves_enabled_arms():
    text = COMM_NODE.read_text(encoding="utf-8")
    stage = text[
        text.index("void MantisCommNode::stage_v3_control_enable") :
        text.index("void MantisCommNode::start_pelvis_startup_initialization")
    ]

    assert "requested_arm_bits == active_arm_bits" in stage
    assert "!this->arm_watchdog_tripped_" in stage
    assert "requested_flag & static_cast<uint8_t>(~V3_ARM_CONTROL_MASK)" not in stage
    assert "previous lower-controller state" in stage


def test_v3_staged_enable_still_prepares_pelvis_initialization():
    text = COMM_NODE.read_text(encoding="utf-8")
    callback = text[text.index("void MantisCommNode::handle_set_ctrl_src") :]

    assert "const bool lower_body_enable_requested" in callback
    assert "request->value != 0U" in callback
    assert "if (!was_lower_body_powered && lower_body_enable_requested)" in callback


def test_explicit_emergency_still_forces_v3_power_off():
    text = COMM_NODE.read_text(encoding="utf-8")
    callback = text[
        text.index("void MantisCommNode::system_status_callback") :
        text.index("void MantisCommNode::async_receive_message")
    ]

    emergency = callback[callback.index("msg->system_state == 3") :]
    assert "this->mantis_command_v3_.control_flag = 0;" in emergency


def test_v3_test_publisher_defaults_to_software_power_bit_only():
    script_text = V3_TEST_PUBLISHER.read_text(encoding="utf-8")
    launch_text = V3_TEST_LAUNCH.read_text(encoding="utf-8")

    assert 'self.declare_parameter("ctrl_src_value", 1)' in script_text
    assert '"ctrl_src_value": 1' in launch_text
    assert 'self.declare_parameter("ctrl_src_value", 31)' not in script_text
    assert '"ctrl_src_value": 31' not in launch_text
