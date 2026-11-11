"""机器人型号 profile 读取工具。

本模块集中读取 `bw_bringup/config/robot_profiles.yaml`，避免 launch、仿真后端
和 RViz 各自维护一份 robot_version 归一化与 profile 字段解析逻辑。
"""

from __future__ import annotations

from dataclasses import dataclass
import os
from typing import Any

import yaml
from ament_index_python.packages import get_package_share_directory


ROBOT_VERSION_ALIASES = {"1": "1.0", "2": "2.0", "3": "3.0"}
SUPPORTED_ROBOT_VERSIONS = ("1.0", "2.0", "3.0")
DEFAULT_ROBOT_VERSION = "2.0"


@dataclass(frozen=True)
class RobotProfile:
    """机器人型号机械参数集合。"""

    robot_version: str
    description: str
    mantis_model: str
    ik_urdf_relative_path: str
    ik_urdf_fallback_relative_path: str
    pelvis_default_height_mm: float
    pelvis_min_height_mm: float
    pelvis_max_height_mm: float
    supports_waist_bend: bool


def normalize_robot_version(
    version: str | None,
    default: str = DEFAULT_ROBOT_VERSION,
) -> str:
    """将机器人型号别名统一为 `1.0` / `2.0` / `3.0`。"""
    fallback = ROBOT_VERSION_ALIASES.get(str(default or DEFAULT_ROBOT_VERSION).strip(), default)
    if fallback not in SUPPORTED_ROBOT_VERSIONS:
        fallback = DEFAULT_ROBOT_VERSION

    raw = str(version or fallback).strip()
    normalized = ROBOT_VERSION_ALIASES.get(raw, raw)
    if normalized in SUPPORTED_ROBOT_VERSIONS:
        return normalized
    return fallback


# 兼容历史调用名：部分节点显式 import 这个私有名字。
_normalize_robot_version = normalize_robot_version


def robot_profiles_yaml_path() -> str:
    """返回已安装 `bw_bringup` 中 robot_profiles.yaml 的路径。"""
    installed_path = ""
    try:
        bringup_share = get_package_share_directory("bw_bringup")
        installed_path = os.path.join(bringup_share, "config", "robot_profiles.yaml")
        if os.path.exists(installed_path):
            return installed_path
    except Exception:
        pass

    # 本地未重新 colcon build/install 时，回退到同一工作区源码树的配置文件。
    current_path = os.path.realpath(__file__)
    for parent in _walk_parent_dirs(current_path):
        source_path = os.path.join(
            parent,
            "bw_bringup",
            "config",
            "robot_profiles.yaml",
        )
        if os.path.exists(source_path):
            return source_path

    raise RuntimeError("无法定位 bw_bringup/config/robot_profiles.yaml")


def _walk_parent_dirs(path: str) -> list[str]:
    parents: list[str] = []
    current = os.path.dirname(path)
    while current and current != os.path.dirname(current):
        parents.append(current)
        current = os.path.dirname(current)
    return parents


def _load_profiles_document(profiles_path: str | None = None) -> dict[str, Any]:
    path = profiles_path or robot_profiles_yaml_path()
    with open(path, "r", encoding="utf-8") as handle:
        document = yaml.safe_load(handle) or {}

    profiles = document.get("robot_profiles")
    if not isinstance(profiles, dict):
        raise RuntimeError(f"{path} 缺少 robot_profiles 根键")
    return profiles


def load_robot_profile(
    robot_version: str | None,
    *,
    profiles_path: str | None = None,
    bw_core_share: str | None = None,
) -> RobotProfile:
    """读取指定机器人型号的 profile。

    Args:
        robot_version: 机器人型号，支持 `1/2/3` 和 `1.0/2.0/3.0`。
        profiles_path: 可选 YAML 路径，测试或离线工具可传入源码路径。
        bw_core_share: 可选 `bw_core` share/source 路径；若 IK URDF 不存在则回退。

    Returns:
        归一化后的 `RobotProfile`。
    """
    profiles = _load_profiles_document(profiles_path)
    key = normalize_robot_version(robot_version)
    raw_profile = profiles.get(key)
    if not isinstance(raw_profile, dict):
        raise RuntimeError(f"robot_profiles.yaml 未定义型号: {key}")

    ik_relative = str(raw_profile.get("ik_urdf_relative_path", "")).strip()
    fallback_relative = str(
        raw_profile.get("ik_urdf_fallback_relative_path", ik_relative)
    ).strip()
    if bw_core_share and ik_relative:
        candidate = os.path.join(bw_core_share, ik_relative)
        if not os.path.exists(candidate):
            print(
                f"[robot_profiles] IK URDF 不存在: {candidate}，"
                f"robot_version={key} 回退到 {fallback_relative}"
            )
            ik_relative = fallback_relative

    return RobotProfile(
        robot_version=key,
        description=str(raw_profile.get("description", "")),
        mantis_model=str(raw_profile.get("mantis_model", f"mantis_{key.replace('.', '_')}")),
        ik_urdf_relative_path=ik_relative,
        ik_urdf_fallback_relative_path=fallback_relative,
        pelvis_default_height_mm=float(raw_profile["pelvis_default_height_mm"]),
        pelvis_min_height_mm=float(raw_profile["pelvis_min_height_mm"]),
        pelvis_max_height_mm=float(raw_profile["pelvis_max_height_mm"]),
        supports_waist_bend=bool(raw_profile.get("supports_waist_bend", False)),
    )


def robot_profile_to_pelvis_params(profile: RobotProfile) -> dict[str, float | str]:
    """转换为串口/仿真后端复用的 launch 参数名。"""
    return {
        "robot_version": profile.robot_version,
        "pelvis_default_height_mm": profile.pelvis_default_height_mm,
        "pelvis_min_height_mm": profile.pelvis_min_height_mm,
        "pelvis_max_height_mm": profile.pelvis_max_height_mm,
    }


def resolve_mantis_model(
    robot_version: str | None,
    *,
    profiles_path: str | None = None,
) -> str:
    """根据机器人型号返回 `mantis_description` 中的模型名前缀。"""
    return load_robot_profile(robot_version, profiles_path=profiles_path).mantis_model
