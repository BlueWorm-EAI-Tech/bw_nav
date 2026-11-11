from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
COMM_NODE = PKG / "src" / "mantis_comm_node.cpp"


def test_v3_chassis_acceleration_defaults_to_protocol_midrange():
    text = COMM_NODE.read_text(encoding="utf-8")

    assert "mantis_command_v3_.max_acc_x = 1.5f;" in text
    assert "mantis_command_v3_.max_acc_y = 1.5f;" in text
    assert "mantis_command_v3_.max_acc_omega = 1.5f;" in text
    assert "mantis_command_v3_.max_acc_x = 1.0f;" not in text
    assert "mantis_command_v3_.max_acc_y = 1.0f;" not in text
    assert "mantis_command_v3_.max_acc_omega = 1.0f;" not in text
