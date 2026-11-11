import os
import sys
from pathlib import Path

import pytest


BW_UTILS_SRC = Path(__file__).resolve().parents[1]
if str(BW_UTILS_SRC) not in sys.path:
    sys.path.insert(0, str(BW_UTILS_SRC))

from bw_utils.robot_profile import (  # noqa: E402
    load_robot_profile,
    normalize_robot_version,
    resolve_mantis_model,
    robot_profile_to_pelvis_params,
)


PROFILES_PATH = (
    Path(__file__).resolve().parents[2]
    / "bw_bringup"
    / "config"
    / "robot_profiles.yaml"
)
BW_CORE_SHARE = Path(__file__).resolve().parents[2] / "bw_core"


def test_normalize_robot_version_accepts_short_aliases_and_falls_back():
    assert normalize_robot_version("3") == "3.0"
    assert normalize_robot_version("2.0") == "2.0"
    assert normalize_robot_version("unknown") == "2.0"
    assert normalize_robot_version(None) == "2.0"


def test_load_robot_profile_reads_yaml_and_resolves_fields():
    profile = load_robot_profile(
        "3",
        profiles_path=str(PROFILES_PATH),
        bw_core_share=str(BW_CORE_SHARE),
    )

    assert profile.robot_version == "3.0"
    assert profile.mantis_model == "mantis_3_0"
    assert profile.pelvis_default_height_mm == pytest.approx(900.0)
    assert profile.pelvis_min_height_mm == pytest.approx(600.0)
    assert profile.pelvis_max_height_mm == pytest.approx(1000.0)
    assert profile.supports_waist_bend is True
    assert profile.ik_urdf_relative_path.endswith("mantis_2_0_ik/mantis_2_0_ik.urdf")


def test_resolve_mantis_model_uses_profile_model_name():
    assert resolve_mantis_model("2", profiles_path=str(PROFILES_PATH)) == "mantis_2_0"
    assert resolve_mantis_model("3.0", profiles_path=str(PROFILES_PATH)) == "mantis_3_0"


def test_robot_profile_to_pelvis_params_matches_launch_parameter_names():
    profile = load_robot_profile("2.0", profiles_path=str(PROFILES_PATH))

    assert robot_profile_to_pelvis_params(profile) == {
        "robot_version": "2.0",
        "pelvis_default_height_mm": 1350.0,
        "pelvis_min_height_mm": 900.0,
        "pelvis_max_height_mm": 1500.0,
    }
