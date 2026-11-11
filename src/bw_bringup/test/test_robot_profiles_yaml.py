from pathlib import Path

import yaml


PROFILES_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "robot_profiles.yaml"
)


def test_robot_profiles_yaml_has_all_supported_versions():
    document = yaml.safe_load(PROFILES_PATH.read_text(encoding="utf-8"))
    profiles = document["robot_profiles"]

    for version in ("1.0", "2.0", "3.0"):
        profile = profiles[version]
        assert "pelvis_default_height_mm" in profile
        assert "ik_urdf_relative_path" in profile
        assert "mantis_model" in profile

    assert profiles["2.0"]["pelvis_default_height_mm"] == 1350.0
    assert profiles["3.0"]["pelvis_default_height_mm"] == 900.0
    assert profiles["3.0"]["supports_waist_bend"] is True
