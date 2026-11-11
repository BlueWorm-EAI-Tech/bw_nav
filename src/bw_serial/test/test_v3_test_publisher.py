import importlib.util
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "scripts" / "v3_test_publisher.py"
SPEC = importlib.util.spec_from_file_location("v3_test_publisher", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_build_arm_profile_defaults_to_zero_positions():
    positions, velocities, efforts = MODULE.build_arm_profile(
        now_sec=0.25,
        active_joint=-1,
        amplitude=0.4,
        period_sec=2.0,
        velocity_limit=0.3,
    )

    assert positions == [0.0] * 14
    assert velocities == [0.3] * 14
    assert efforts == [0.0] * 14


def test_build_arm_profile_generates_single_joint_sine_motion():
    positions, velocities, efforts = MODULE.build_arm_profile(
        now_sec=0.5,
        active_joint=3,
        amplitude=0.2,
        period_sec=2.0,
        velocity_limit=0.4,
    )

    assert positions[3] == 0.2
    assert sum(abs(value) for value in positions) == 0.2
    assert velocities == [0.4] * 14
    assert efforts == [0.0] * 14
