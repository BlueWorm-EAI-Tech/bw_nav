from pathlib import Path


PKG = Path(__file__).resolve().parents[1]
BRINGUP = PKG.parents[0] / "bw_bringup" / "launch" / "system_composition.launch.py"
CPP_TEXT = (PKG / "src" / "mantis_comm_node.cpp").read_text(encoding="utf-8")
HPP_TEXT = (PKG / "include" / "bw_serial" / "mantis_comm_node.hpp").read_text(encoding="utf-8")


def test_v3_decodes_every_frame_but_throttles_ros_feedback_publication() -> None:
    assert 'declare_parameter("v3_feedback_publish_frequency_hz", 250.0)' in CPP_TEXT
    assert "v3_feedback_publish_frequency_hz_" in HPP_TEXT
    assert "should_publish_v3_ros_feedback" in CPP_TEXT

    receive_start = CPP_TEXT.index("void MantisCommNode::async_receive_message")
    v3_start = CPP_TEXT.index(
        "if (this->protocol_version_ == MantisProtocolVersion::V3 ||",
        receive_start,
    )
    receive_end = CPP_TEXT.index(
        "bool MantisCommNode::should_publish_v3_ros_feedback",
        receive_start,
    )
    v3_receive = CPP_TEXT[v3_start:receive_end]
    assert v3_receive.index("decode_mantis_feedback") < v3_receive.index(
        "should_publish_v3_ros_feedback"
    )
    assert "update_latest_pelvis_feedback();" in v3_receive
    assert '"v3_feedback_publish_frequency_hz": (' in BRINGUP.read_text(encoding="utf-8")
