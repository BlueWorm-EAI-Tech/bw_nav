from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
COMM_NODE = PKG / "src" / "mantis_comm_node.cpp"
COMM_HEADER = PKG / "include" / "bw_serial" / "mantis_comm_node.hpp"


def test_standard_v3_auto_detection_has_explicit_hardware_shape_state():
    header = COMM_HEADER.read_text(encoding="utf-8")
    source = COMM_NODE.read_text(encoding="utf-8")

    assert "auto_detect_hardware_shape" in header
    assert "V3HardwareShape::UNKNOWN" in header
    assert "FULL_BODY = 1" in header
    assert "CHASSIS_ONLY = 2" in header
    assert "is_v3_auto_detection_pending" in source


def test_auto_detection_accepts_full_body_and_chassis_feedback_frames():
    source = COMM_NODE.read_text(encoding="utf-8")

    assert "MANTIS_FEEDBACK_V3_TYPE" in source
    assert "MANTIS_FEEDBACK_V3_PAYLOAD_LENGTH" in source
    assert "MANTIS_FEEDBACK_V3_CHASSIS_TYPE" in source
    assert "MANTIS_FEEDBACK_V3_CHASSIS_PAYLOAD_LENGTH" in source
    assert "decode_v3_chassis_feedback_frame" in source
    assert "switch_to_v3_chassis_mode" in source
    assert "V3 hardware shape detected: full-body" in source
    assert "V3 hardware shape detected: chassis-only" in source


def test_chassis_probe_is_stationary_and_stays_chassis_after_switch():
    source = COMM_NODE.read_text(encoding="utf-8")

    assert "chassis_probe.vx = 0.0F" in source
    assert "chassis_probe.vy = 0.0F" in source
    assert "chassis_probe.omega = 0.0F" in source
    assert "chassis_probe.pelvis_height = 1200.0F" in source
    assert "chassis_probe.pelvis_max_velocity = 200.0F" in source
    assert "chassis_probe.pelvis_acceleration" in source
    assert "transmit_data_buffer = encode_v3_chassis_command_frame(chassis_probe)" in source
    assert "this->protocol_version_ = MantisProtocolVersion::V3_CHASSIS;" in source
    assert "this->pelvis_jog_config_.min_height_mm = 900.0F;" in source
    assert "this->pelvis_jog_config_.max_height_mm = 1500.0F;" in source


def test_auto_detection_probes_chassis_in_a_consecutive_phase_first():
    source = COMM_NODE.read_text(encoding="utf-8")

    assert "V3_AUTO_PROBE_PHASE_FRAMES" in source
    assert "const uint64_t probe_phase" in source
    assert "if ((probe_phase & 0x01U) != 0U)" in source


def test_auto_detection_falls_back_without_feedback_and_publishes_chassis_shape():
    source = COMM_NODE.read_text(encoding="utf-8")
    header = COMM_HEADER.read_text(encoding="utf-8")

    assert "V3_AUTO_PROBE_TOTAL_FRAMES" in source
    assert "fallback_to_chassis" in source
    assert "assuming chassis-only and switching to 0x21 payloads" in source
    assert "publish_available_arms(0U)" in source
    assert "available_arms_pub_" in header
    assert "available_arms_options.use_intra_process_comm" in source
    assert "IntraProcessSetting::Disable" in source
