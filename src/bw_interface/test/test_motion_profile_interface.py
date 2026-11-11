from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
MSG = PKG / "msg" / "MotionProfile.msg"
TAU_CMD_MSG = PKG / "msg" / "ForceHandleTauCmd.msg"
CMAKE_TEXT = (PKG / "CMakeLists.txt").read_text(encoding="utf-8")


def test_motion_profile_message_contract_is_registered() -> None:
    assert MSG.exists()

    text = MSG.read_text(encoding="utf-8")
    assert "std_msgs/Header header" in text
    assert "string mode" in text
    assert "float64 max_velocity" in text
    assert "float64 max_acceleration" in text
    assert "float64 max_jerk" in text
    assert "float64 duration_sec" in text
    assert '"msg/MotionProfile.msg"' in CMAKE_TEXT


def test_force_handle_tau_cmd_message_contract_is_registered() -> None:
    assert TAU_CMD_MSG.exists()

    text = TAU_CMD_MSG.read_text(encoding="utf-8")
    assert "std_msgs/Header header" in text
    assert "float32 left_tau_cmd" in text
    assert "float32 right_tau_cmd" in text
    assert "bool left_valid" in text
    assert "bool right_valid" in text
    assert '"msg/ForceHandleTauCmd.msg"' in CMAKE_TEXT
