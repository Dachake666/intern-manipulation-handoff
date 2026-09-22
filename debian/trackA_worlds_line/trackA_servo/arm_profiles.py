#!/usr/bin/env python3
"""双臂运动配置的唯一权威入口。

本模块只依赖标准库，可被规划机、ROS 2 节点和真机部署包共同使用。右臂的
真实限位/TCP/SDK 映射在现场确认前保持 unresolved；调用 require_hardware_ready
会强制阻断，不会静默退回左臂镜像值。
"""
from __future__ import annotations

import json
import os
from copy import deepcopy
from pathlib import Path

SCHEMA_VERSION = "arm_profiles.v1"


def _config_path() -> Path:
    override = os.environ.get("XIFENG_ARM_PROFILES")
    if override:
        return Path(override).expanduser().resolve()
    here = Path(__file__).resolve().parent
    for candidate in (here / "arm_profiles.v1.json",
                      here.parent / "arm_profiles.v1.json"):
        if candidate.is_file():
            return candidate
    raise FileNotFoundError("找不到 arm_profiles.v1.json; 可用 XIFENG_ARM_PROFILES 指定")


def load_profiles(path: str | os.PathLike | None = None) -> dict:
    source = Path(path).expanduser().resolve() if path else _config_path()
    data = json.loads(source.read_text(encoding="utf-8"))
    if data.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"arm profile schema 应为 {SCHEMA_VERSION}")
    for arm in ("left", "right"):
        p = data.get("arms", {}).get(arm)
        if not isinstance(p, dict):
            raise ValueError(f"缺少 arms.{arm}")
        if len(p.get("joint_ids", [])) != 7 or len(p.get("home_deg", [])) != 7:
            raise ValueError(f"arms.{arm} 必须有 7 个关节和 7 个 HOME 角")
        if p.get("workspace_m", {}).get("z") != data["shared"]["workspace_z_m"]:
            raise ValueError(f"arms.{arm}.workspace_m.z 与 shared.workspace_z_m 漂移")
    return data


def arm_profile(arm: str, path: str | os.PathLike | None = None) -> dict:
    data = load_profiles(path)
    try:
        profile = deepcopy(data["arms"][arm])
    except KeyError as exc:
        raise ValueError(f"arm 只能是 left/right, 收到 {arm!r}") from exc
    profile["arm"] = arm
    profile["shared"] = deepcopy(data["shared"])
    return profile


def profile_for_arm_id(arm_id: int, path: str | os.PathLike | None = None) -> dict:
    for name in ("left", "right"):
        p = arm_profile(name, path)
        if p["arm_id"] == int(arm_id):
            return p
    raise ValueError(f"未知 arm_id={arm_id}")


def controller_limit_profile(profile_id: str,
                             path: str | os.PathLike | None = None) -> dict:
    """读取一次现场回读形成的候选限位证据；不把动态 IP 固化成永久真值。"""
    data = load_profiles(path)
    try:
        profile = deepcopy(data["controller_limit_profiles"][profile_id])
    except KeyError as exc:
        raise ValueError(f"未知 controller_limit_profile={profile_id!r}") from exc
    reported = profile.get("reported_limits_deg")
    planning = profile.get("planning_limits_deg")
    for label, limits in (("reported_limits_deg", reported),
                          ("planning_limits_deg", planning)):
        if not isinstance(limits, list) or len(limits) != 7:
            raise ValueError(f"{profile_id}.{label} 必须包含 7 组限位")
        for index, pair in enumerate(limits, 1):
            if (not isinstance(pair, list) or len(pair) != 2 or
                    float(pair[0]) >= float(pair[1])):
                raise ValueError(f"{profile_id}.{label}[{index}] 无效")
    if profile.get("arm_id") != {"left": 1, "right": 2}.get(profile.get("arm")):
        raise ValueError(f"{profile_id} 的 arm/arm_id 不一致")
    profile["profile_id"] = profile_id
    return profile


def require_hardware_ready(profile: dict) -> None:
    missing = []
    if profile.get("controller_limits_deg") is None:
        missing.append("controller_limits_deg")
    if "CANDIDATE" in profile.get("grip_center_status", ""):
        missing.append("measured grip_center_link_mm/TCP")
    if "BLOCKED" in profile.get("status", ""):
        missing.append("hardware qualification status")
    if missing:
        raise RuntimeError(
            f"{profile.get('arm')} arm hardware gate BLOCKED: " + ", ".join(missing))


def sdk_to_urdf_deg(profile: dict, values: list[float]) -> list[float]:
    if len(values) != 7:
        raise ValueError("关节向量必须恰好 7 个值")
    return [float(v) * int(s) for v, s in zip(values, profile["sdk_from_urdf_sign"])]


def urdf_to_sdk_deg(profile: dict, values: list[float]) -> list[float]:
    return sdk_to_urdf_deg(profile, values)  # 符号向量是对合变换
