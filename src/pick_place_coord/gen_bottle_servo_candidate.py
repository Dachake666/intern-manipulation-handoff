#!/usr/bin/env python3
"""从已知 SDK 世界位姿/关节角生成左臂瓶子 Servo 抓放候选。

复用 Track C 已验收 HOME/READY 路径和 Pulse Servo 参数；绝对 PB<->SDK 平移 T
当前失效，所以只用已知关节做 FK 几何锚点，放置采用相对平移。输出始终是
CANDIDATE；控制器真实限位、瓶子实测尺寸、首点和密集碰撞仍是硬门。
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
import pybullet as p
import pybullet_planning as pp

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WORK))
sys.path.insert(0, str(WORK / "frame_calibration" / "analysis"))

import arm_profiles
import calib_common as cc
import xifeng_pb as xf

# 仓库同时有 pick_place_coord/ 目录和 pick_place_coord.py 规划源。
# unittest 从仓库根导入时，普通 ``import pick_place_coord`` 会命中目录
# 命名空间而不是规划源；显式按真源文件加载，保证生成与测试一致。
_PARENT_SPEC = importlib.util.spec_from_file_location(
    "pick_place_coord_legacy_planner", HERE / "pick_place_coord.py")
if _PARENT_SPEC is None or _PARENT_SPEC.loader is None:
    raise ImportError("无法加载 pick_place_coord.py 规划真源")
parent_planner = importlib.util.module_from_spec(_PARENT_SPEC)
_PARENT_SPEC.loader.exec_module(parent_planner)

SCHEMA_V1 = "bottle_relative_pick_place.v1"
SCHEMA_V2 = "bottle_relative_pick_place.v2"
SCHEMA_V3 = "bottle_relative_pick_place.v3"
SUPPORTED_SCHEMAS = {SCHEMA_V1, SCHEMA_V2, SCHEMA_V3}
ARM = "left"
FREE_ROLL_MODE = "FREE_ROLL_ABOUT_APPROACH_AXIS"
VERTICAL_PARENT_MODE = "NEAR_VERTICAL_FROM_VERIFIED_TRACK_C"
FIXED_POSE_ANCHOR_MODE = "FIXED_POSE_ANCHOR"
PATH_STYLE_CARTESIAN_HIGH_VERTICAL = "CARTESIAN_HIGH_HORIZONTAL_VERTICAL"


def _merge_dict(base: dict, overrides: dict) -> dict:
    """递归合并一个任务 variant；公共场景/安全契约只保留一份。"""
    out = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _merge_dict(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def resolve_task_variant(raw_task: dict, variant: str | None) -> tuple[dict, str | None]:
    schema = raw_task.get("schema_version")
    if schema != SCHEMA_V3:
        if variant is not None:
            raise ValueError("--variant 只适用于 bottle_relative_pick_place.v3")
        return raw_task, None
    variants = raw_task.get("variants")
    if not isinstance(variants, dict) or not variants:
        raise ValueError("v3 task 必须提供非空 variants")
    if variant is None:
        raise ValueError(f"v3 task 必须用 --variant 选择 {sorted(variants)}")
    if variant not in variants:
        raise ValueError(f"未知 variant {variant!r}; 可选 {sorted(variants)}")
    base = {key: value for key, value in raw_task.items() if key != "variants"}
    task = _merge_dict(base, variants[variant])
    task["selected_variant"] = variant
    return task, variant


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _workspace_path(path: Path) -> str:
    """工作区内记录相对路径；离线搜索的临时任务保留绝对路径。"""
    try:
        return str(path.resolve().relative_to(WORK))
    except ValueError:
        return str(path.resolve())


def _rotz(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _roty(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rotation_vector(R: np.ndarray) -> np.ndarray:
    cos_a = float(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cos_a)
    if angle < 1e-9:
        return np.zeros(3)
    if abs(math.sin(angle)) < 1e-7:
        vals, vecs = np.linalg.eig(R)
        axis = np.real(vecs[:, int(np.argmin(np.abs(vals - 1.0)))])
        axis /= np.linalg.norm(axis) + 1e-12
        return axis * angle
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0],
                     R[1, 0] - R[0, 1]]) / (2.0 * math.sin(angle))
    return axis * angle


def _interpolate_R(R0: np.ndarray, R1: np.ndarray, t: float) -> np.ndarray:
    """沿相对旋转向量插值姿态；t=0 为 R0，t=1 为 R1。"""
    t = float(t)
    R0 = np.asarray(R0, float)
    R1 = np.asarray(R1, float)
    if t <= 0.0:
        return R0
    if t >= 1.0:
        return R1
    rv = _rotation_vector(R0.T @ R1)
    angle = float(np.linalg.norm(rv))
    if angle < 1e-12:
        return R0
    axis = rv / angle
    a = angle * t
    kx, ky, kz = axis
    K = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return R0 @ (np.eye(3) + math.sin(a) * K + (1.0 - math.cos(a)) * (K @ K))


def _matrix_to_uvw(R: np.ndarray) -> list[float]:
    """R = Rz(W)·Ry(V)·Rx(U) 的标准 SDK UVW 分解。"""
    v = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    w = math.atan2(R[1, 0], R[0, 0])
    u = math.atan2(R[2, 1], R[2, 2])
    return [_normalize_angle_deg(math.degrees(x)) for x in (u, v, w)]


def _tool_axis_from_link_rotation(R_link: np.ndarray) -> np.ndarray:
    R_sdk = np.asarray(R_link, float) @ _rotz(90.0)
    axis = R_sdk[:, 0]
    return axis / np.linalg.norm(axis)


def _sdk_deg_to_urdf_rad(q_sdk_deg) -> np.ndarray:
    return np.radians(cc.sdk_q_to_urdf_q(q_sdk_deg))


def _urdf_rad_to_sdk_deg(q_urdf_rad) -> list[float]:
    return [float(v) for v in cc.urdf_q_to_sdk_q(np.degrees(q_urdf_rad))]


def fk_grip(robot: int, q_sdk_deg) -> tuple[np.ndarray, np.ndarray]:
    q = _sdk_deg_to_urdf_rad(q_sdk_deg)
    profile = arm_profiles.arm_profile(ARM)
    for jid, value in zip(profile["joint_ids"], q):
        p.resetJointState(robot, jid, float(value))
    state = p.getLinkState(robot, profile["ee_link_id"],
                           computeForwardKinematics=True)
    R_link = np.asarray(p.getMatrixFromQuaternion(state[5]), float).reshape(3, 3)
    grip_offset = np.asarray(profile["grip_center_link_mm"], float) / 1000.0
    grip = np.asarray(state[4], float) + R_link @ grip_offset
    return grip, R_link


def _solver_bounds_sdk(profile: dict, q_pick_sdk_deg: list[float]) -> tuple[list, list]:
    """只扩到包含观测目标的几何求解盒；不是硬件限位，扩展即保持 BLOCKED。"""
    limits = [list(map(float, pair)) for pair in profile["controller_limits_deg"]]
    expanded = []
    for index, (value, pair) in enumerate(zip(q_pick_sdk_deg, limits), 1):
        lo0, hi0 = pair
        lo, hi = min(lo0, value), max(hi0, value)
        if lo != lo0 or hi != hi0:
            expanded.append({"joint": index, "baseline_deg": [lo0, hi0],
                             "geometric_solver_deg": [lo, hi],
                             "target_deg": value})
        pair[:] = [lo, hi]
    urdf = []
    for index, (lo, hi) in enumerate(limits):
        urdf.append((math.radians(-hi), math.radians(-lo)) if
                    index == cc.J6_FLIP_INDEX else
                    (math.radians(lo), math.radians(hi)))
    return urdf, expanded


def _solver_bounds_from_limits(limits_sdk_deg, margin_deg: float) -> list[tuple[float, float]]:
    """把共享控制器证据收紧安全余量后转换到 URDF 关节顺序/符号。"""
    urdf = []
    for index, pair in enumerate(limits_sdk_deg):
        lo, hi = float(pair[0]) + margin_deg, float(pair[1]) - margin_deg
        if lo >= hi:
            raise ValueError(f"joint {index + 1} 限位收 margin 后为空")
        urdf.append((math.radians(-hi), math.radians(-lo)) if
                    index == cc.J6_FLIP_INDEX else
                    (math.radians(lo), math.radians(hi)))
    return urdf


def _raw_joint_margin(q_sdk_deg, limits_sdk_deg) -> float:
    return min(min(float(value) - float(pair[0]),
                   float(pair[1]) - float(value))
               for value, pair in zip(q_sdk_deg, limits_sdk_deg))


def _normalize_angle_deg(value: float) -> float:
    return (float(value) + 180.0) % 360.0 - 180.0


def _parent_pick_anchors(parent: dict) -> list[list[float]]:
    anchors, previous = [], None
    for waypoint in parent["waypoints"]:
        if "q_sdk_deg" in waypoint:
            previous = waypoint["q_sdk_deg"]
        elif waypoint.get("gripper") == "close" and previous is not None:
            anchors.append([float(v) for v in previous])
    return anchors


def _orientation_roll_values(policy: dict) -> list[float]:
    spec = policy["roll_search_deg"]
    start, stop, step = (float(spec[k]) for k in ("start", "stop", "step"))
    if step <= 0.0 or stop < start:
        raise ValueError("roll_search_deg 必须 start<=stop 且 step>0")
    count = int(math.floor((stop - start) / step + 1e-9))
    values = [start + index * step for index in range(count + 1)]
    if values[-1] < stop - 1e-8:
        values.append(stop)
    return values


def _axis_angle_deg(a, b) -> float:
    a = np.asarray(a, float)
    b = np.asarray(b, float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    return math.degrees(math.acos(float(np.clip(np.dot(a, b), -1.0, 1.0))))


def select_vertical_parent_orientation(robot: int, geometry: dict, task: dict,
                                       parent: dict, limits_sdk_deg) -> dict:
    """验证由真机双抓基线向下轴导出的近竖直抓取锚点。

    新目标比双抓基线更远，严格复制旧完整姿态在安全限位内无 IK 解。因此任务文件
    保存离线约束优化得到的关节锚点；本函数不盲信它，而是重新 FK 验证抓取中心、
    工具向下倾角、限位余量以及父基线两次抓取确实近似竖直。
    """
    policy = task["orientation_policy"]
    q = [float(v) for v in policy["orientation_anchor_q_sdk_deg"]]
    if len(q) != 7 or not all(math.isfinite(v) for v in q):
        raise ValueError("orientation_anchor_q_sdk_deg 必须是有限 7 维向量")

    grip, R_link = fk_grip(robot, q)
    position_error_mm = float(np.linalg.norm(grip - geometry["grip_pick"]) * 1000.0)
    maximum_position_error_mm = float(policy["maximum_anchor_position_error_mm"])
    if position_error_mm > maximum_position_error_mm:
        raise ValueError(
            f"near-vertical anchor position error {position_error_mm:.4f}mm > "
            f"{maximum_position_error_mm:.4f}mm")

    target_axis = np.asarray(policy["target_tool_axis_sdk"], float)
    if target_axis.shape != (3,) or not np.all(np.isfinite(target_axis)):
        raise ValueError("target_tool_axis_sdk 必须是有限 3 维向量")
    norm = float(np.linalg.norm(target_axis))
    if norm < 1e-9:
        raise ValueError("target_tool_axis_sdk 不能是零向量")
    target_axis /= norm
    tool_axis = _tool_axis_from_link_rotation(R_link)
    tilt_deg = _axis_angle_deg(tool_axis, target_axis)
    maximum_tilt_deg = float(policy["maximum_tool_tilt_from_down_deg"])
    if tilt_deg > maximum_tilt_deg:
        raise ValueError(
            f"near-vertical anchor tilt {tilt_deg:.4f}deg > {maximum_tilt_deg:.4f}deg")

    required_margin = float(policy["minimum_pick_raw_limit_margin_deg"])
    raw_margin = _raw_joint_margin(q, limits_sdk_deg)
    if raw_margin < required_margin:
        raise ValueError(
            f"near-vertical anchor raw margin {raw_margin:.4f}deg < "
            f"{required_margin:.4f}deg")

    parent_axes = []
    for index, parent_q in enumerate(_parent_pick_anchors(parent), 1):
        _grip, parent_R = fk_grip(robot, parent_q)
        axis = _tool_axis_from_link_rotation(parent_R)
        parent_axes.append({
            "close_index": index,
            "tool_axis_sdk": axis.tolist(),
            "tilt_from_target_deg": _axis_angle_deg(axis, target_axis),
        })
    if len(parent_axes) < 2 or max(x["tilt_from_target_deg"] for x in parent_axes) > 1.0:
        raise ValueError("verified parent close anchors do not establish a downward tool axis")

    selected_uvw = _matrix_to_uvw(R_link @ _rotz(90.0))
    selected_solution = {
        "seed": "OFFLINE_AXIS_CONSTRAINED_IK_ANCHOR",
        "q_sdk_deg": q,
        "raw_limit_margin_deg": raw_margin,
        "position_error_mm": position_error_mm,
        "tool_tilt_from_target_deg": tilt_deg,
    }
    return {
        "mode": VERTICAL_PARENT_MODE,
        "selection": policy["selection"],
        "minimum_pick_raw_limit_margin_deg": required_margin,
        "maximum_tool_tilt_from_down_deg": maximum_tilt_deg,
        "target_tool_axis_sdk": target_axis.tolist(),
        "selected_roll_deg": 0.0,
        "selected_uvw_deg": selected_uvw,
        "selected_q_sdk_deg": q,
        "selected_R_link": R_link,
        "selected_solution": selected_solution,
        "verified_parent_close_axes": parent_axes,
        "seed_names": ["TRACK_C_REALVERIFIED_2GRASP_DOWN_AXIS",
                       "OFFLINE_AXIS_CONSTRAINED_IK_ANCHOR"],
        "per_roll": [],
    }


def select_fixed_pose_anchor(robot: int, geometry: dict, task: dict,
                             parent: dict, limits_sdk_deg) -> dict:
    """复验离线搜索得到的全姿态锚点，不在生成时复制另一套搜索器。"""
    policy = task["orientation_policy"]
    q = [float(v) for v in policy["orientation_anchor_q_sdk_deg"]]
    if len(q) != 7 or not all(math.isfinite(v) for v in q):
        raise ValueError("orientation_anchor_q_sdk_deg 必须是有限 7 维向量")

    grip, R_link = fk_grip(robot, q)
    position_error_mm = float(np.linalg.norm(grip - geometry["grip_pick"]) * 1000.0)
    maximum_position_error_mm = float(policy["maximum_anchor_position_error_mm"])
    if position_error_mm > maximum_position_error_mm:
        raise ValueError(
            f"fixed anchor position error {position_error_mm:.4f}mm > "
            f"{maximum_position_error_mm:.4f}mm")

    target_axis = np.asarray(policy["target_tool_axis_sdk"], float)
    if target_axis.shape != (3,) or not np.all(np.isfinite(target_axis)):
        raise ValueError("target_tool_axis_sdk 必须是有限 3 维向量")
    norm = float(np.linalg.norm(target_axis))
    if norm < 1e-9:
        raise ValueError("target_tool_axis_sdk 不能是零向量")
    target_axis /= norm
    tool_axis = _tool_axis_from_link_rotation(R_link)
    axis_error_deg = _axis_angle_deg(tool_axis, target_axis)
    maximum_axis_error_deg = float(policy["maximum_tool_axis_error_deg"])
    if axis_error_deg > maximum_axis_error_deg:
        raise ValueError(
            f"fixed anchor tool-axis error {axis_error_deg:.4f}deg > "
            f"{maximum_axis_error_deg:.4f}deg")

    required_margin = float(policy["minimum_pick_raw_limit_margin_deg"])
    raw_margin = _raw_joint_margin(q, limits_sdk_deg)
    if raw_margin < required_margin:
        raise ValueError(
            f"fixed anchor raw margin {raw_margin:.4f}deg < {required_margin:.4f}deg")

    parent_axes = []
    if policy.get("verify_parent_down_axes"):
        for index, parent_q in enumerate(_parent_pick_anchors(parent), 1):
            _grip, parent_R = fk_grip(robot, parent_q)
            axis = _tool_axis_from_link_rotation(parent_R)
            parent_axes.append({
                "close_index": index,
                "tool_axis_sdk": axis.tolist(),
                "tilt_from_down_deg": _axis_angle_deg(axis, [0.0, 0.0, -1.0]),
            })
        if len(parent_axes) < 2 or max(
                row["tilt_from_down_deg"] for row in parent_axes) > 1.0:
            raise ValueError("verified parent close anchors do not establish a downward tool axis")

    selected_uvw = _matrix_to_uvw(R_link @ _rotz(90.0))
    selected_solution = {
        "seed": policy.get("anchor_source", "OFFLINE_BOUNDED_FULL_POSE_SEARCH"),
        "q_sdk_deg": q,
        "raw_limit_margin_deg": raw_margin,
        "position_error_mm": position_error_mm,
        "tool_axis_error_deg": axis_error_deg,
    }
    return {
        "mode": FIXED_POSE_ANCHOR_MODE,
        "grasp_style": policy["grasp_style"],
        "selection": policy["selection"],
        "minimum_pick_raw_limit_margin_deg": required_margin,
        "target_tool_axis_sdk": target_axis.tolist(),
        "maximum_tool_axis_error_deg": maximum_axis_error_deg,
        "selected_roll_deg": float(policy["selected_roll_deg"]),
        "selected_uvw_deg": selected_uvw,
        "selected_q_sdk_deg": q,
        "selected_R_link": R_link,
        "selected_solution": selected_solution,
        "verified_parent_close_axes": parent_axes,
        "seed_names": [selected_solution["seed"]],
        "per_roll": [],
    }


def search_pick_orientation(robot: int, geometry: dict, task: dict, parent: dict,
                            ready_q_sdk_deg, limits_sdk_deg, bounds_urdf) -> dict:
    """固定抓取中心和接近轴，只搜索圆柱瓶允许的绕接近轴转角与 IK 分支。"""
    policy = task["orientation_policy"]
    mode = policy.get("mode")
    if mode == VERTICAL_PARENT_MODE:
        return select_vertical_parent_orientation(
            robot, geometry, task, parent, limits_sdk_deg)
    if mode == FIXED_POSE_ANCHOR_MODE:
        return select_fixed_pose_anchor(
            robot, geometry, task, parent, limits_sdk_deg)
    if mode != FREE_ROLL_MODE:
        raise ValueError(
            f"v2 orientation_policy.mode 不支持 {mode!r}; "
            f"期望 {FREE_ROLL_MODE!r}、{VERTICAL_PARENT_MODE!r} 或 "
            f"{FIXED_POSE_ANCHOR_MODE!r}")
    profile = arm_profiles.arm_profile(ARM)
    seeds = [("POSITION_ANCHOR_Q", geometry["q_pick"]),
             ("TRACK_C_READY", ready_q_sdk_deg)]
    seeds += [(f"TRACK_C_PICK_{index}", q) for index, q in
              enumerate(_parent_pick_anchors(parent), 1)]
    seeds.append(("ARM_PROFILE_IK_SEED", profile["ik_seed_deg"]))
    unique_seeds, seen = [], set()
    for name, values in seeds:
        key = tuple(round(float(v), 6) for v in values)
        if key not in seen:
            seen.add(key)
            unique_seeds.append((name, [float(v) for v in values]))

    required = float(policy["minimum_pick_raw_limit_margin_deg"])
    preferred_uvw = [float(v) for v in geometry["preferred_uvw_deg"]]
    per_roll, feasible = [], []
    base_R = geometry["R_link"]
    for roll in _orientation_roll_values(policy):
        target_R = base_R @ _roty(roll)
        solutions = []
        for seed_name, seed in unique_seeds:
            q, residual = solve_full_pose(
                robot, geometry["grip_pick"], target_R, seed, bounds_urdf,
                iterations=700, position_tolerance_mm=0.15,
                rotation_tolerance_deg=0.1)
            if q is None:
                continue
            margin = _raw_joint_margin(q, limits_sdk_deg)
            ready_distance = math.sqrt(sum((float(a) - float(b)) ** 2
                                           for a, b in zip(q, ready_q_sdk_deg)))
            solutions.append({"seed": seed_name, "q_sdk_deg": q,
                              "raw_limit_margin_deg": margin,
                              "ready_l2_distance_deg": ready_distance,
                              **residual})
        if not solutions:
            per_roll.append({"roll_deg": roll, "status": "IK_FAILED"})
            continue
        best = min(solutions, key=lambda item: (-item["raw_limit_margin_deg"],
                                                item["ready_l2_distance_deg"]))
        row = {"roll_deg": roll,
               "selected_uvw_deg": [_normalize_angle_deg(preferred_uvw[0] + roll),
                                     preferred_uvw[1], preferred_uvw[2]],
               "status": "PASS_MARGIN" if
                         best["raw_limit_margin_deg"] >= required else "BELOW_MARGIN",
               "best_solution": best, "solution_count": len(solutions)}
        per_roll.append(row)
        if row["status"] == "PASS_MARGIN":
            feasible.append((row, target_R))
    if not feasible:
        best_seen = max((row for row in per_roll if "best_solution" in row),
                        key=lambda row: row["best_solution"]["raw_limit_margin_deg"],
                        default=None)
        raise RuntimeError(f"姿态搜索没有满足 {required}deg 余量的候选；best={best_seen}")
    selected, selected_R = min(
        feasible,
        key=lambda item: (abs(item[0]["roll_deg"]),
                          -item[0]["best_solution"]["raw_limit_margin_deg"],
                          item[0]["best_solution"]["ready_l2_distance_deg"]))
    return {"mode": policy["mode"], "selection": policy["selection"],
            "minimum_pick_raw_limit_margin_deg": required,
            "selected_roll_deg": selected["roll_deg"],
            "selected_uvw_deg": selected["selected_uvw_deg"],
            "selected_q_sdk_deg": selected["best_solution"]["q_sdk_deg"],
            "selected_R_link": selected_R,
            "selected_solution": selected["best_solution"],
            "per_roll": per_roll,
            "seed_names": [name for name, _ in unique_seeds]}


def _pose_error(robot: int, q_urdf: np.ndarray, target_grip: np.ndarray,
                target_R_link: np.ndarray) -> np.ndarray:
    profile = arm_profiles.arm_profile(ARM)
    for jid, value in zip(profile["joint_ids"], q_urdf):
        p.resetJointState(robot, jid, float(value))
    state = p.getLinkState(robot, profile["ee_link_id"],
                           computeForwardKinematics=True)
    R = np.asarray(p.getMatrixFromQuaternion(state[5]), float).reshape(3, 3)
    offset = np.asarray(profile["grip_center_link_mm"], float) / 1000.0
    grip = np.asarray(state[4], float) + R @ offset
    return np.concatenate((target_grip - grip,
                           _rotation_vector(target_R_link @ R.T)))


def solve_full_pose(robot: int, target_grip: np.ndarray, target_R_link: np.ndarray,
                    seed_sdk_deg, bounds_urdf, iterations: int = 220,
                    position_tolerance_mm: float = 0.8,
                    rotation_tolerance_deg: float = 0.5):
    """局部全姿态 DLS；用于相对移动，不建立或更新会话 T。"""
    q = _sdk_deg_to_urdf_rad(seed_sdk_deg)
    lo = np.asarray([x[0] for x in bounds_urdf], float)
    hi = np.asarray([x[1] for x in bounds_urdf], float)
    q = np.clip(q, lo, hi)
    damping, numerical_step = 0.03, 1e-5
    step_cap = math.radians(5.0)
    for _ in range(iterations):
        error = _pose_error(robot, q, target_grip, target_R_link)
        pos_mm = float(np.linalg.norm(error[:3]) * 1000.0)
        rot_deg = float(np.linalg.norm(error[3:]) * 180.0 / math.pi)
        if pos_mm <= position_tolerance_mm and rot_deg <= rotation_tolerance_deg:
            return _urdf_rad_to_sdk_deg(q), {
                "position_error_mm": pos_mm, "rotation_error_deg": rot_deg}
        J_error = np.zeros((6, 7))
        for joint in range(7):
            q2 = q.copy()
            q2[joint] += numerical_step
            J_error[:, joint] = (
                _pose_error(robot, q2, target_grip, target_R_link) - error
            ) / numerical_step
        J = -J_error
        dq = J.T @ np.linalg.solve(J @ J.T + damping ** 2 * np.eye(6), error)
        q = np.clip(q + np.clip(dq, -step_cap, step_cap), lo, hi)
    error = _pose_error(robot, q, target_grip, target_R_link)
    return None, {"position_error_mm": float(np.linalg.norm(error[:3]) * 1000.0),
                  "rotation_error_deg": float(np.linalg.norm(error[3:]) * 180.0 / math.pi)}


def solve_cartesian_chain(robot: int, start_xyz: np.ndarray, end_xyz: np.ndarray,
                          R_link: np.ndarray, seed_sdk_deg, bounds_urdf,
                          sample_mm: float, max_anchor_step_deg: float,
                          position_tolerance_mm: float = 0.8,
                          rotation_tolerance_deg: float = 0.5,
                          R_end=None):
    R_start = np.asarray(R_link, float)
    R_goal = R_start if R_end is None else np.asarray(R_end, float)
    distance_mm = float(np.linalg.norm(end_xyz - start_xyz) * 1000.0)
    count = max(1, int(math.ceil(distance_mm / sample_mm)))
    rows, seed, residuals = [], list(map(float, seed_sdk_deg)), []
    worst_step = 0.0
    for index in range(count + 1):
        frac = index / count
        xyz = start_xyz + (end_xyz - start_xyz) * frac
        R_target = _interpolate_R(R_start, R_goal, frac)
        if index == 0:
            q = seed
            grip, R_now = fk_grip(robot, q)
            info = {"position_error_mm": float(np.linalg.norm(grip - xyz) * 1000.0),
                    "rotation_error_deg": cc.rotation_angle_deg(R_target @ R_now.T)}
        else:
            # 近竖直姿态在工作空间外缘更接近腕部限位，220 次迭代偶尔会在
            # 0.6deg 左右提前耗尽；提高迭代预算，不放宽位置/姿态容差。
            q, info = solve_full_pose(
                robot, xyz, R_target, seed, bounds_urdf, iterations=500,
                position_tolerance_mm=position_tolerance_mm,
                rotation_tolerance_deg=rotation_tolerance_deg)
            if q is None:
                raise RuntimeError(f"Cartesian IK failed at {index}/{count}: {info}")
        if rows:
            step = max(abs(a - b) for a, b in zip(rows[-1], q))
            worst_step = max(worst_step, step)
            if step > max_anchor_step_deg:
                raise RuntimeError(f"IK branch jump {step:.3f}deg > "
                                   f"{max_anchor_step_deg:.3f}deg")
        rows.append([float(v) for v in q])
        residuals.append(info)
        seed = q
    return rows, {"samples": len(rows), "distance_mm": distance_mm,
                  "maximum_anchor_step_deg": worst_step,
                  "maximum_position_error_mm": max(x["position_error_mm"] for x in residuals),
                  "maximum_rotation_error_deg": max(x["rotation_error_deg"] for x in residuals)}


def _empty_approach_lift_m(task: dict, grip_pick: np.ndarray) -> float:
    """空载水平接近必须高过直立瓶顶加夹爪保守代理下伸，避免从侧面扫瓶。"""
    profile = arm_profiles.arm_profile(ARM)
    transfer = float(task["motion"]["vertical_lift_mm"]) / 1000.0
    specified = task["motion"].get("empty_approach_lift_mm")
    if specified is not None:
        return float(specified) / 1000.0
    scene = task["scene"]
    bottle_top = float(scene["table_top_pb_m"]) + float(scene["bottle_height_m"])
    grip_off = np.asarray(profile["grip_center_link_mm"], float)
    proxy_c = np.asarray(profile["gripper_collision_center_link_mm"], float)
    box_mm = np.asarray(profile["gripper_collision_box_m"], float) * 1000.0
    axis = int(np.argmax(np.abs(grip_off)))
    beyond_m = (proxy_c[axis] + box_mm[axis] / 2.0 - grip_off[axis]) / 1000.0
    clearance = float(scene.get("minimum_gripper_table_clearance_m", 0.02))
    needed = bottle_top + max(beyond_m, 0.0) + clearance - float(grip_pick[2])
    return max(transfer, needed)


def derive_geometry(robot: int, task: dict):
    pick = task["pick"]
    if task["schema_version"] in (SCHEMA_V2, SCHEMA_V3):
        q_pick = list(map(float, pick["position_anchor_q_sdk_deg"]))
        preferred_uvw = list(map(float, pick["preferred_sdk_world_uvw_deg"]))
    else:
        q_pick = list(map(float, pick["q_sdk_deg"]))
        preferred_uvw = list(map(float, pick["sdk_world_uvw_deg"]))
    position_anchor_grip, R_link = fk_grip(robot, q_pick)
    R_sdk_given = cc.uvw_to_matrix(cc.CONFIRMED_UVW_CANDIDATE,
                                   preferred_uvw)
    orientation_delta = cc.rotation_angle_deg(R_sdk_given @ (R_link @ _rotz(90.0)).T)
    if orientation_delta > 1.0:
        raise ValueError(f"given q and UVW disagree by {orientation_delta:.3f}deg")
    xyz_sdk = np.asarray(pick["sdk_world_xyz_mm"], float)
    tool_axis = R_sdk_given[:, 0]
    tool_axis /= np.linalg.norm(tool_axis)
    offset = np.asarray(task["place_offset_sdk_mm"], float) / 1000.0
    if abs(float(np.linalg.norm(offset[:2])) - 0.1) > 1e-9 or abs(offset[2]) > 1e-12:
        raise ValueError("place offset must be exactly 100mm in the XY plane")
    path_style = task["motion"].get("path_style")
    lift = float(task["motion"]["vertical_lift_mm"]) / 1000.0
    target_offset = np.asarray(
        task.get("grip_target_world_offset_mm", [0.0, 0.0, 0.0]), float) / 1000.0
    if target_offset.shape != (3,) or not np.all(np.isfinite(target_offset)):
        raise ValueError("grip_target_world_offset_mm 必须是有限 3 维向量")
    if path_style == PATH_STYLE_CARTESIAN_HIGH_VERTICAL:
        retreat_mm = float(task["motion"].get("tool_axis_retreat_mm", 0.0))
        if abs(retreat_mm) > 1e-12:
            raise ValueError("CARTESIAN_HIGH_HORIZONTAL_VERTICAL 禁止 tool-axis retreat")
        retreat = 0.0
    else:
        retreat = float(task["motion"]["tool_axis_retreat_mm"]) / 1000.0
    grip_pick = position_anchor_grip + target_offset
    place = grip_pick + offset
    pre_pick = grip_pick - tool_axis * retreat
    pre_place = place - tool_axis * retreat
    if path_style == PATH_STYLE_CARTESIAN_HIGH_VERTICAL:
        approach = _empty_approach_lift_m(task, grip_pick)
        high_pick = grip_pick + np.array([0.0, 0.0, approach])
        high_place = place + np.array([0.0, 0.0, approach])
        transfer_high_pick = grip_pick + np.array([0.0, 0.0, lift])
        transfer_high_place = place + np.array([0.0, 0.0, lift])
        pre_pick = np.asarray(high_pick, float)
        pre_place = np.asarray(high_place, float)
    else:
        high_pick = pre_pick + np.array([0.0, 0.0, lift])
        high_place = pre_place + np.array([0.0, 0.0, lift])
        transfer_high_pick = high_pick
        transfer_high_place = high_place
    return {"q_pick": q_pick, "position_anchor_grip": position_anchor_grip,
            "grip_target_world_offset_m": target_offset,
            "grip_pick": grip_pick, "R_link": R_link,
            "preferred_uvw_deg": preferred_uvw,
            "tool_axis_sdk": tool_axis,
            "observed_single_point_t_mm": xyz_sdk - position_anchor_grip * 1000.0,
            "orientation_delta_deg": orientation_delta, "place": place,
            "pre_pick": pre_pick, "high_pick": high_pick,
            "pre_place": pre_place, "high_place": high_place,
            "transfer_high_pick": transfer_high_pick,
            "transfer_high_place": transfer_high_place,
            "path_style": path_style}


def apply_orientation_search(geometry: dict, result: dict, task: dict) -> dict:
    selected = dict(geometry)
    selected["q_pick"] = [float(v) for v in result["selected_q_sdk_deg"]]
    selected["R_link"] = np.asarray(result["selected_R_link"], float)
    selected["selected_uvw_deg"] = [float(v) for v in result["selected_uvw_deg"]]
    R_sdk = selected["R_link"] @ _rotz(90.0)
    selected["tool_axis_sdk"] = R_sdk[:, 0] / np.linalg.norm(R_sdk[:, 0])
    path_style = task["motion"].get("path_style")
    lift = float(task["motion"]["vertical_lift_mm"]) / 1000.0
    if path_style == PATH_STYLE_CARTESIAN_HIGH_VERTICAL:
        approach = _empty_approach_lift_m(task, selected["grip_pick"])
        selected["high_pick"] = selected["grip_pick"] + np.array([0.0, 0.0, approach])
        selected["high_place"] = selected["place"] + np.array([0.0, 0.0, approach])
        selected["transfer_high_pick"] = selected["grip_pick"] + np.array([0.0, 0.0, lift])
        selected["transfer_high_place"] = selected["place"] + np.array([0.0, 0.0, lift])
        selected["pre_pick"] = np.asarray(selected["high_pick"], float)
        selected["pre_place"] = np.asarray(selected["high_place"], float)
    else:
        retreat = float(task["motion"]["tool_axis_retreat_mm"]) / 1000.0
        selected["pre_pick"] = selected["grip_pick"] - selected["tool_axis_sdk"] * retreat
        selected["pre_place"] = selected["place"] - selected["tool_axis_sdk"] * retreat
        selected["high_pick"] = selected["pre_pick"] + np.array([0.0, 0.0, lift])
        selected["high_place"] = selected["pre_place"] + np.array([0.0, 0.0, lift])
        selected["transfer_high_pick"] = selected["high_pick"]
        selected["transfer_high_place"] = selected["high_place"]
    selected["path_style"] = path_style
    return selected


def _bottle_geometry(robot: int, task: dict, geometry: dict, place_q_sdk_deg) -> dict:
    scene = task["scene"]
    actual_pick_grip, _ = fk_grip(robot, geometry["q_pick"])
    actual_place_grip, _ = fk_grip(robot, place_q_sdk_deg)
    fixed_object_target = bool(task.get("coordinate_contract", {}).get(
        "object_target_from_position_anchor"))
    object_anchor = (np.asarray(geometry["position_anchor_grip"], float)
                     if fixed_object_target else np.asarray(actual_pick_grip, float))
    if scene.get("bottle_base_on_table"):
        pick_center = np.array([object_anchor[0], object_anchor[1],
                                float(scene["table_top_pb_m"]) +
                                float(scene["bottle_height_m"]) / 2.0])
    else:
        pick_center = object_anchor
    if fixed_object_target:
        place_center = pick_center + np.asarray(task["place_offset_sdk_mm"], float) / 1000.0
    else:
        place_center = np.asarray(actual_place_grip, float) + (
            pick_center - np.asarray(actual_pick_grip, float))
    offset = pick_center - np.asarray(actual_pick_grip, float)
    return {"pick_center": pick_center,
            "place_center": place_center,
            "actual_pick_grip": np.asarray(actual_pick_grip, float),
            "actual_place_grip": np.asarray(actual_place_grip, float),
            "center_offset_from_grip_world_m": offset}


def _grasp_capture_report(profile: dict, geometry: dict,
                          bottle_geometry: dict) -> dict:
    """确认 close 时瓶心确实位于当前夹爪的可捕获区域，而不是事后强制绑定。"""
    delta = (np.asarray(bottle_geometry["pick_center"], float) -
             np.asarray(bottle_geometry["actual_pick_grip"], float))
    axis = np.asarray(geometry["tool_axis_sdk"], float)
    axis /= np.linalg.norm(axis)
    axial = float(np.dot(delta, axis))
    lateral = float(np.linalg.norm(delta - axial * axis))
    axial_range = [float(v) for v in profile["grasp_capture_axis_range_m"]]
    lateral_limit = float(profile["grasp_capture_lateral_tolerance_m"])
    passed = (axial_range[0] <= axial <= axial_range[1] and
              lateral <= lateral_limit)
    report = {
        "status": "PASS" if passed else "BLOCKED",
        "axis_offset_m": axial,
        "axis_range_m": axial_range,
        "lateral_error_m": lateral,
        "lateral_tolerance_m": lateral_limit,
        "tool_axis_world": axis.tolist(),
    }
    if not passed:
        raise RuntimeError(f"grasp capture failed: {report}")
    return report


def _task_corridor_report(robot: int, rows, target_R_link: np.ndarray,
                          motion: dict, name: str) -> dict:
    """RRT 只负责高位过渡；禁止 READY 后再次大幅翻腕。"""
    maximum_tilt = float(motion["maximum_task_corridor_tool_tilt_deg"])
    maximum_delta = float(motion["maximum_task_corridor_orientation_delta_deg"])
    worst_tilt = (0.0, -1)
    worst_delta = (0.0, -1)
    for index, q in enumerate(rows):
        _grip, R_link = fk_grip(robot, q)
        tilt = _axis_angle_deg(_tool_axis_from_link_rotation(R_link), [0.0, 0.0, -1.0])
        delta = cc.rotation_angle_deg(np.asarray(R_link) @ np.asarray(target_R_link).T)
        if tilt > worst_tilt[0]:
            worst_tilt = (tilt, index)
        if delta > worst_delta[0]:
            worst_delta = (delta, index)
    report = {
        "name": name,
        "maximum_tool_tilt_deg": worst_tilt[0],
        "maximum_tool_tilt_at": worst_tilt[1],
        "allowed_tool_tilt_deg": maximum_tilt,
        "maximum_orientation_delta_deg": worst_delta[0],
        "maximum_orientation_delta_at": worst_delta[1],
        "allowed_orientation_delta_deg": maximum_delta,
    }
    if worst_tilt[0] > maximum_tilt or worst_delta[0] > maximum_delta:
        raise RuntimeError(f"task corridor orientation gate failed: {report}")
    report["status"] = "PASS"
    return report


def _table_box_spec(task: dict, geometry: dict) -> dict:
    scene = task["scene"]
    padding = float(scene["table_xy_padding_m"])
    thickness = float(scene["table_thickness_m"])
    pick, place = geometry["grip_pick"], geometry["place"]
    half = [abs(float(pick[0] - place[0])) / 2.0 + padding,
            abs(float(pick[1] - place[1])) / 2.0 + padding,
            thickness / 2.0]
    top = float(scene["table_top_pb_m"])
    cx, cy = (float(pick[0] + place[0]) / 2.0,
              float(pick[1] + place[1]) / 2.0)
    return {"source_obstacle_id": "physical_table_local_patch",
            "center": [cx, cy, top - half[2]], "half": half,
            "top_pb_m": top,
            "support_contact_tolerance_m": float(
                scene.get("support_contact_tolerance_m", 0.0)),
            "minimum_robot_clearance_m": float(
                scene.get("minimum_robot_table_clearance_m", 0.0)),
            "minimum_gripper_clearance_m": float(
                scene.get("minimum_gripper_table_clearance_m", 0.0)),
            "comment": "与候选 RRT 相同的 72cm 物理桌面局部障碍体"}


def _create_table(task: dict, geometry: dict) -> int:
    spec = _table_box_spec(task, geometry)
    return p.createMultiBody(0, p.createCollisionShape(
        p.GEOM_BOX, halfExtents=spec["half"]), basePosition=spec["center"])


def _create_bottle(task: dict, center: np.ndarray) -> int:
    scene = task["scene"]
    return p.createMultiBody(0, p.createCollisionShape(
        p.GEOM_CYLINDER, radius=float(scene["bottle_radius_m"]),
        height=float(scene["bottle_height_m"])), basePosition=center.tolist())


def _set_q(robot: int, q_sdk_deg):
    profile = arm_profiles.arm_profile(ARM)
    for jid, value in zip(profile["joint_ids"], _sdk_deg_to_urdf_rad(q_sdk_deg)):
        p.resetJointState(robot, jid, float(value))


def _plan_joint_path(robot: int, start_sdk, goal_sdk, obstacles, bounds_urdf,
                     seed: int):
    profile = arm_profiles.arm_profile(ARM)
    _set_q(robot, start_sdk)
    random.seed(seed)
    np.random.seed(seed)
    custom = {jid: pair for jid, pair in zip(profile["joint_ids"], bounds_urdf)}
    path = pp.plan_joint_motion(
        robot, profile["joint_ids"], _sdk_deg_to_urdf_rad(goal_sdk).tolist(),
        obstacles=list(obstacles), self_collisions=True,
        disabled_collisions=parent_planner.DISABLED, custom_limits=custom,
        resolutions=[parent_planner.RESOLUTION] * 7,
        smooth=parent_planner.SMOOTH_ITERS)
    if path is None:
        raise RuntimeError("RRT failed")
    dense = parent_planner.densify(path)
    if not parent_planner.verify_path_free(robot, dense, list(obstacles)):
        raise RuntimeError("dense RRT verification failed")
    return [_urdf_rad_to_sdk_deg(q) for q in dense]


def _parent_parts(parent: dict):
    prefix = [w for w in parent["waypoints"]
              if w.get("seg") in ("HOME", "HOME->READY")]
    tail = [w for w in parent["waypoints"] if w.get("seg") == "READY->HOME"]
    if not prefix or not tail:
        raise ValueError("verified parent lacks HOME/READY segments")
    return prefix, tail


def _append_motion(out: list[dict], seg: str, rows):
    for q in rows:
        if out and out[-1].get("q_sdk_deg") and max(
                abs(a - b) for a, b in zip(out[-1]["q_sdk_deg"], q)) < 1e-8:
            continue
        out.append({"seg": seg, "q_sdk_deg": [round(float(v), 4) for v in q]})


def _plan_cartesian_high_vertical(robot, geometry, ready_q_sdk, bounds,
                                  sample_mm, max_step, position_tolerance_mm,
                                  rotation_tolerance_deg):
    """READY 高位水平接近 + 纯世界 Z 升降/放置；姿态在入口/出口 slerp，任务段锁定。"""
    ready_xyz, ready_R = fk_grip(robot, ready_q_sdk)
    pick_R = geometry["R_link"]
    entry, rep_entry = solve_cartesian_chain(
        robot, ready_xyz, geometry["high_pick"], ready_R, ready_q_sdk, bounds,
        sample_mm, max_step, position_tolerance_mm, rotation_tolerance_deg,
        R_end=pick_R)
    descend, rep_descend = solve_cartesian_chain(
        robot, geometry["high_pick"], geometry["grip_pick"], pick_R, entry[-1],
        bounds, sample_mm, max_step, position_tolerance_mm, rotation_tolerance_deg)
    lift_rows, rep_lift = solve_cartesian_chain(
        robot, geometry["grip_pick"], geometry["transfer_high_pick"], pick_R, descend[-1],
        bounds, sample_mm, max_step, position_tolerance_mm, rotation_tolerance_deg)
    transfer, rep_transfer = solve_cartesian_chain(
        robot, geometry["transfer_high_pick"], geometry["transfer_high_place"], pick_R,
        lift_rows[-1], bounds, sample_mm, max_step, position_tolerance_mm,
        rotation_tolerance_deg)
    place_descend, rep_place_descend = solve_cartesian_chain(
        robot, geometry["transfer_high_place"], geometry["place"], pick_R, transfer[-1],
        bounds, sample_mm, max_step, position_tolerance_mm, rotation_tolerance_deg)
    # 放置点比抓取点更靠外，空载竖直抬升也到不了入口那档 150mm；松开后只升到
    # 已验证的连续 IK 高度，再 slerp 回 READY。
    retreat_high = geometry["transfer_high_place"]
    place_lift, rep_place_lift = solve_cartesian_chain(
        robot, geometry["place"], retreat_high, pick_R,
        place_descend[-1], bounds, sample_mm, max_step, position_tolerance_mm,
        rotation_tolerance_deg)
    exit_rows, rep_exit = solve_cartesian_chain(
        robot, retreat_high, ready_xyz, pick_R, place_lift[-1], bounds,
        sample_mm, max_step, position_tolerance_mm, rotation_tolerance_deg,
        R_end=ready_R)
    ready_gap = max(abs(a - b) for a, b in zip(exit_rows[-1], ready_q_sdk))
    # 笛卡尔退出有时会在 READY 的同一末端位姿落到相邻 IK 支路。
    # 不放宽 max_step；只允许一次可被拆成两个合法步长的小支路收敛，
    # 且该高位关节桥最终必须通过 Servo 密集碰撞资格门。
    if ready_gap > 2.0 * max_step:
        raise RuntimeError(
            f"cartesian exit missed READY by {ready_gap:.3f}deg > "
            f"two allowed {max_step:.3f}deg bridge steps")
    if ready_gap > 1e-8:
        start_q = np.asarray(exit_rows[-1], float)
        target_q = np.asarray(ready_q_sdk, float)
        bridge_count = max(1, int(math.ceil(ready_gap / max_step)))
        for index in range(1, bridge_count + 1):
            exit_rows.append(
                (start_q + (target_q - start_q) * index / bridge_count).tolist())
    else:
        bridge_count = 0
    return {
        "entry": entry,
        "descend": descend,
        "lift": lift_rows,
        "transfer": transfer,
        "place_descend": place_descend,
        "place_lift": place_lift,
        "exit": exit_rows,
        "pick_q": descend[-1],
        "place_q": place_descend[-1],
        "ready_return_gap_deg": ready_gap,
        "ready_joint_bridge_points": bridge_count,
        "reports": {
            "entry_slerp_ready_to_high": rep_entry,
            "pick_descend": rep_descend,
            "pick_lift": rep_lift,
            "transfer": rep_transfer,
            "place_descend": rep_place_descend,
            "place_lift": rep_place_lift,
            "exit_slerp_high_to_ready": rep_exit,
        },
    }


def _limit_report(profile: dict, rows: list[dict], limits=None):
    limits = profile["controller_limits_deg"] if limits is None else limits
    margin = float(profile["shared"]["minimum_joint_margin_deg"])
    violations, minimum = [], float("inf")
    for index, row in enumerate(rows):
        q = row.get("q_sdk_deg")
        if q is None:
            continue
        for joint, (value, pair) in enumerate(zip(q, limits), 1):
            lo, hi = map(float, pair)
            minimum = min(minimum, value - lo, hi - value)
            if not lo + margin <= value <= hi - margin:
                violations.append({"waypoint": index, "seg": row["seg"],
                                   "joint": joint, "value_deg": value,
                                   "required_deg": [lo + margin, hi - margin]})
    return {"status": "PASS" if not violations else "BLOCKED_LIVE_LIMITS_REQUIRED",
            "minimum_raw_margin_deg": minimum, "violations": violations[:40]}


def build(task_path: Path, out_path: Path, qualification_task_out: Path,
          seed: int = 26, variant: str | None = None):
    raw_task = json.loads(task_path.read_text(encoding="utf-8"))
    task, selected_variant = resolve_task_variant(raw_task, variant)
    schema = task.get("schema_version")
    if schema not in SUPPORTED_SCHEMAS:
        raise ValueError(f"expected one of {sorted(SUPPORTED_SCHEMAS)}")
    optimized = schema in (SCHEMA_V2, SCHEMA_V3)
    profile = arm_profiles.arm_profile(ARM)
    if (task["arm"], task["arm_id"], task["gripper_id"]) != (
            ARM, profile["arm_id"], profile["gripper_id"]):
        raise ValueError("arm/gripper contract mismatch")
    parent_path = WORK / task["verified_parent"]["path"]
    parent_hash = sha256_file(parent_path)
    if parent_hash != task["verified_parent"]["sha256"]:
        raise ValueError("verified parent SHA-256 mismatch")
    parent = json.loads(parent_path.read_text(encoding="utf-8"))
    prefix, tail = _parent_parts(parent)

    parent_planner.set_arm(ARM)
    robot = xf.load_xifeng(gui=False)
    try:
        parent_planner.DISABLED = parent_planner.compute_disabled_pairs(robot)
        geometry = derive_geometry(robot, task)
        orientation_search = None
        limit_evidence = None
        if optimized:
            limit_evidence = arm_profiles.controller_limit_profile(
                task["controller_limit_profile"])
            if (limit_evidence["arm"], limit_evidence["arm_id"]) != (ARM, profile["arm_id"]):
                raise ValueError("controller limit profile arm mismatch")
            planning_limits = limit_evidence["planning_limits_deg"]
            bounds = _solver_bounds_from_limits(
                planning_limits, float(profile["shared"]["minimum_joint_margin_deg"]))
            expansions = []
            orientation_search = search_pick_orientation(
                robot, geometry, task, parent, prefix[-1]["q_sdk_deg"],
                planning_limits, bounds)
            geometry = apply_orientation_search(geometry, orientation_search, task)
        else:
            planning_limits = profile["controller_limits_deg"]
            bounds, expansions = _solver_bounds_sdk(profile, geometry["q_pick"])
        sample_mm = float(task["motion"]["cartesian_sample_mm"])
        max_step = float(task["motion"]["maximum_anchor_joint_step_deg"])
        position_tolerance_mm = float(
            task["motion"].get("cartesian_position_tolerance_mm", 0.8))
        rotation_tolerance_deg = float(
            task["motion"].get("cartesian_rotation_tolerance_deg", 0.5))

        path_style = task["motion"].get("path_style")
        cartesian_only = path_style == PATH_STYLE_CARTESIAN_HIGH_VERTICAL
        if cartesian_only and task["motion"].get("carried_clearance_sequence") == (
                "LIFT_THEN_HORIZONTAL"):
            raise ValueError("CARTESIAN_HIGH_HORIZONTAL_VERTICAL 与 LIFT_THEN_HORIZONTAL 不能同时使用")
        if cartesian_only and task["motion"].get("exit_to_home_avoids_placed_object"):
            raise ValueError("CARTESIAN_HIGH_HORIZONTAL_VERTICAL 必须先升到高位再回 READY，禁止 RRT 绕物")

        def solve_chain(start, end, seed_q):
            return solve_cartesian_chain(
                robot, start, end, geometry["R_link"], seed_q, bounds,
                sample_mm, max_step, position_tolerance_mm, rotation_tolerance_deg)

        cartesian_plan = None
        lift_first = False
        if cartesian_only:
            cartesian_plan = _plan_cartesian_high_vertical(
                robot, geometry, prefix[-1]["q_sdk_deg"], bounds, sample_mm,
                max_step, position_tolerance_mm, rotation_tolerance_deg)
            geometry["q_pick"] = [float(v) for v in cartesian_plan["pick_q"]]
            place_q = cartesian_plan["place_q"]
            chain_reports = cartesian_plan["reports"]
            if optimized:
                required_margin = float(
                    task["orientation_policy"]["minimum_pick_raw_limit_margin_deg"])
                pick_margin = _raw_joint_margin(geometry["q_pick"], planning_limits)
                if pick_margin < required_margin:
                    raise RuntimeError(
                        f"cartesian pick raw margin {pick_margin:.4f}deg < "
                        f"{required_margin:.4f}deg")
            bottle_geometry = _bottle_geometry(robot, task, geometry, place_q)
            grasp_capture = _grasp_capture_report(profile, geometry, bottle_geometry)
            task_rows = (cartesian_plan["entry"] + cartesian_plan["descend"] +
                         cartesian_plan["lift"] + cartesian_plan["transfer"] +
                         cartesian_plan["place_descend"] + cartesian_plan["place_lift"] +
                         cartesian_plan["exit"])
            corridor_reports = {
                "task": _task_corridor_report(
                    robot, task_rows, geometry["R_link"], task["motion"],
                    "CARTESIAN_HIGH_HORIZONTAL_VERTICAL"),
            }
        else:
            pick_to_pre, rep_retreat = solve_chain(
                geometry["grip_pick"], geometry["pre_pick"], geometry["q_pick"])
            lift_first = task["motion"].get("carried_clearance_sequence") == (
                "LIFT_THEN_HORIZONTAL")
            if lift_first:
                lift = float(task["motion"]["vertical_lift_mm"]) / 1000.0
                geometry["lifted_pick"] = geometry["grip_pick"] + [0.0, 0.0, lift]
                geometry["lifted_place"] = geometry["place"] + [0.0, 0.0, lift]
                pick_to_lifted, rep_pick_lift = solve_chain(
                    geometry["grip_pick"], geometry["lifted_pick"], geometry["q_pick"])
                lifted_to_high, rep_high_retreat = solve_chain(
                    geometry["lifted_pick"], geometry["high_pick"], pick_to_lifted[-1])
                high_to_place, rep_transfer = solve_chain(
                    geometry["high_pick"], geometry["high_place"], lifted_to_high[-1])
                high_to_lifted_place, rep_high_approach = solve_chain(
                    geometry["high_place"], geometry["lifted_place"], high_to_place[-1])
                lifted_to_place, rep_place_descend = solve_chain(
                    geometry["lifted_place"], geometry["place"], high_to_lifted_place[-1])
                place_q = lifted_to_place[-1]
                rrt_out_start = high_to_lifted_place[0]
                chain_reports = {
                    "empty_gripper_low_approach": rep_retreat,
                    "carried_pick_lift": rep_pick_lift,
                    "carried_high_retreat": rep_high_retreat,
                    "transfer": rep_transfer,
                    "carried_high_place_approach": rep_high_approach,
                    "carried_place_descend": rep_place_descend,
                }
            else:
                pre_to_high, rep_lift = solve_chain(
                    geometry["pre_pick"], geometry["high_pick"], pick_to_pre[-1])
                high_to_place, rep_transfer = solve_chain(
                    geometry["high_pick"], geometry["high_place"], pre_to_high[-1])
                high_to_pre_place, rep_descend = solve_chain(
                    geometry["high_place"], geometry["pre_place"], high_to_place[-1])
                pre_to_place, rep_approach = solve_chain(
                    geometry["pre_place"], geometry["place"], high_to_pre_place[-1])
                place_q = pre_to_place[-1]
                rrt_out_start = high_to_pre_place[0]
                chain_reports = {"pick_retreat": rep_retreat, "lift": rep_lift,
                                 "transfer": rep_transfer, "place_descend": rep_descend,
                                 "place_approach": rep_approach}

            table = _create_table(task, geometry)
            bottle_geometry = _bottle_geometry(robot, task, geometry, place_q)
            grasp_capture = _grasp_capture_report(profile, geometry, bottle_geometry)
            bottle = _create_bottle(task, bottle_geometry["pick_center"])
            rrt_in = _plan_joint_path(robot, prefix[-1]["q_sdk_deg"], pick_to_pre[-1],
                                      [table], bounds, seed)
            p.resetBasePositionAndOrientation(bottle, bottle_geometry["place_center"].tolist(),
                                              [0, 0, 0, 1])
            avoid_placed_on_exit = bool(
                task["motion"].get("exit_to_home_avoids_placed_object"))
            rrt_out = _plan_joint_path(
                robot, rrt_out_start,
                tail[-1]["q_sdk_deg"] if avoid_placed_on_exit else tail[0]["q_sdk_deg"],
                [table, bottle] if avoid_placed_on_exit else [table], bounds, seed + 1)
            corridor_reports = {
                "entry": _task_corridor_report(
                    robot, rrt_in, geometry["R_link"], task["motion"], "READY_TO_PREGRASP"),
                "exit": _task_corridor_report(
                    robot, rrt_out, geometry["R_link"], task["motion"], "HIGH_TO_READY_OR_HOME"),
            }

        waypoints = [{"seg": w["seg"],
                      "q_sdk_deg": [float(v) for v in w["q_sdk_deg"]]}
                     for w in prefix]
        if cartesian_only:
            _append_motion(waypoints, "READY->BOTTLE_HIGH", cartesian_plan["entry"])
            _append_motion(waypoints, "BOTTLE_DESCEND", cartesian_plan["descend"])
            waypoints.append({"seg": "BOTTLE_PICK", "gripper": "close"})
            _append_motion(waypoints, "BOTTLE_LIFT", cartesian_plan["lift"])
            _append_motion(waypoints, "BOTTLE_TRANSFER_100MM", cartesian_plan["transfer"])
            _append_motion(waypoints, "BOTTLE_PLACE_DESCEND", cartesian_plan["place_descend"])
            waypoints.append({"seg": "BOTTLE_PLACE", "gripper": "open"})
            _append_motion(waypoints, "BOTTLE_PLACE_LIFT", cartesian_plan["place_lift"])
            _append_motion(waypoints, "BOTTLE_HIGH->READY", cartesian_plan["exit"])
            # 是否复用父轨迹 READY→HOME 由任务显式声明；不能因为父轨迹
            # 真机跑过就默认它不会扫过新的放置瓶。最终仍由密集资格检查放行。
            if task["motion"].get("append_verified_ready_to_home"):
                _append_motion(waypoints, "READY->HOME",
                               [w["q_sdk_deg"] for w in tail])
        else:
            _append_motion(waypoints, "READY->BOTTLE_PREGRASP", rrt_in)
            _append_motion(waypoints, "BOTTLE_APPROACH", reversed(pick_to_pre))
            waypoints.append({"seg": "BOTTLE_PICK", "gripper": "close"})
            if lift_first:
                _append_motion(waypoints, "BOTTLE_LIFT_AFTER_PICK", pick_to_lifted)
                _append_motion(waypoints, "BOTTLE_RETREAT_HIGH", lifted_to_high)
                _append_motion(waypoints, "BOTTLE_TRANSFER_100MM", high_to_place)
                _append_motion(waypoints, "BOTTLE_PLACE_APPROACH_HIGH", high_to_lifted_place)
                _append_motion(waypoints, "BOTTLE_PLACE_DESCEND", lifted_to_place)
                waypoints.append({"seg": "BOTTLE_PLACE", "gripper": "open"})
                _append_motion(waypoints, "BOTTLE_PLACE_ASCEND", reversed(lifted_to_place))
                _append_motion(waypoints, "BOTTLE_PLACE_RETREAT_HIGH",
                               reversed(high_to_lifted_place))
            else:
                _append_motion(waypoints, "BOTTLE_RETREAT", pick_to_pre)
                _append_motion(waypoints, "BOTTLE_LIFT", pre_to_high)
                _append_motion(waypoints, "BOTTLE_TRANSFER_100MM", high_to_place)
                _append_motion(waypoints, "BOTTLE_PLACE_DESCEND", high_to_pre_place)
                _append_motion(waypoints, "BOTTLE_PLACE_APPROACH", pre_to_place)
                waypoints.append({"seg": "BOTTLE_PLACE", "gripper": "open"})
                _append_motion(waypoints, "BOTTLE_PLACE_RETREAT", reversed(pre_to_place))
                _append_motion(waypoints, "BOTTLE_PLACE_LIFT", reversed(high_to_pre_place))
            _append_motion(waypoints,
                           ("BOTTLE_HIGH->HOME_AVOID_PLACED_OBJECT" if avoid_placed_on_exit
                            else "BOTTLE_HIGH->READY"),
                           rrt_out)
            if not avoid_placed_on_exit:
                _append_motion(waypoints, "READY->HOME", [w["q_sdk_deg"] for w in tail])

        limit_report = _limit_report(profile, waypoints, planning_limits)
        motion_rows = [w for w in waypoints if "q_sdk_deg" in w]
        max_adjacent = max(max(abs(a - b) for a, b in zip(x["q_sdk_deg"], y["q_sdk_deg"]))
                           for x, y in zip(motion_rows, motion_rows[1:]))
        qualification = ("CANDIDATE_GUI_REVIEW_REQUIRED" if optimized else
                         "BLOCKED_LIVE_CONTROLLER_LIMITS" if expansions else "CANDIDATE")
        source_pick = task["pick"]
        target_offset_mm = np.asarray(
            task.get("grip_target_world_offset_mm", [0.0, 0.0, 0.0]), float)
        source_xyz_mm = np.asarray(source_pick["sdk_world_xyz_mm"], float)
        target_xyz_mm = source_xyz_mm + target_offset_mm
        task_name = task.get(
            "task_name",
            "bottle_relative_pick_place_orientation_optimized_100mm" if optimized else
            "bottle_relative_pick_place_100mm")
        meta = {
            "arm_id": profile["arm_id"], "arm": ARM,
            "gripper_id": profile["gripper_id"], "j6_flipped": True,
            "task": task_name,
            "source_task": _workspace_path(task_path),
            "source_task_sha256": sha256_file(task_path),
            "source_task_variant": selected_variant,
            "generator": str(Path(__file__).resolve().relative_to(WORK)),
            "generator_sha256": sha256_file(Path(__file__).resolve()),
            "qualification": qualification,
            "requires_live_controller_limits": bool(expansions) or optimized,
            "requires_live_controller_recheck": optimized,
            "real_motion_authorized": False,
            "verified_parent_sha256": parent_hash,
            "verified_parent_reused_segments": (
                (["HOME", "HOME->READY", "READY->HOME"]
                 if task["motion"].get("append_verified_ready_to_home") else
                 ["HOME", "HOME->READY"]) if cartesian_only else
                ["HOME", "HOME->READY", "READY->HOME"]),
            "pick_sdk_world": ({"sdk_world_xyz_mm": target_xyz_mm.tolist(),
                                "source_sdk_world_xyz_mm": source_xyz_mm.tolist(),
                                "grip_target_world_offset_mm": target_offset_mm.tolist(),
                                "preferred_sdk_world_uvw_deg":
                                    source_pick["preferred_sdk_world_uvw_deg"],
                                "selected_sdk_world_uvw_deg":
                                    orientation_search["selected_uvw_deg"],
                                "position_anchor_q_sdk_deg":
                                    source_pick["position_anchor_q_sdk_deg"],
                                "selected_q_sdk_deg": geometry["q_pick"]}
                               if optimized else task["pick"]),
            "place_sdk_world": {
                "xyz_mm": (target_xyz_mm +
                           np.asarray(task["place_offset_sdk_mm"], float)).tolist(),
                "uvw_deg": (orientation_search["selected_uvw_deg"] if optimized else
                            source_pick["sdk_world_uvw_deg"])},
            "place_offset_sdk_mm": task["place_offset_sdk_mm"],
            "derived_pb_geometry_m": {k: value.tolist() for k, value in geometry.items()
                                      if k in {
                                          "position_anchor_grip",
                                          "grip_target_world_offset_m", "grip_pick",
                                          "place", "pre_pick", "high_pick", "pre_place",
                                          "high_place", "transfer_high_pick",
                                          "transfer_high_place",
                                          "lifted_pick", "lifted_place"}},
            "single_point_t_observation_mm": geometry["observed_single_point_t_mm"].tolist(),
            "single_point_t_is_calibration": False,
            "q_uvw_orientation_delta_deg": geometry["orientation_delta_deg"],
            "tool_axis_sdk": geometry["tool_axis_sdk"].tolist(),
            "solver_bound_expansions": expansions,
            "controller_limit_profile": (task.get("controller_limit_profile") if
                                           optimized else None),
            "controller_limit_evidence": ({k: limit_evidence[k] for k in
                ("profile_id", "status", "source_log_sha256", "reported_limits_deg",
                 "planning_limits_deg", "planning_note")} if optimized else None),
            "orientation_search": ({k: v for k, v in orientation_search.items()
                                     if k != "selected_R_link"} if optimized else None),
            "local_limit_report": limit_report,
            "scene": task["scene"], "motion": task["motion"],
            "coordinate_contract": task.get("coordinate_contract"),
            "tool_geometry_evidence": task.get("tool_geometry_evidence"),
            "bottle_geometry_pb_m": {k: v.tolist() for k, v in bottle_geometry.items()},
            "grasp_capture_report": grasp_capture,
            "task_corridor_orientation_reports": corridor_reports,
            "path_style": path_style,
            "cartesian_ready_return_gap_deg": (cartesian_plan["ready_return_gap_deg"]
                                               if cartesian_only else None),
            "gui_review": ({"required": True, "status": "PENDING_HUMAN_REVIEW",
                            "dense_step_deg": task["gui_review"]["dense_step_deg"],
                            "demo_script": "pick_place_coord/demo_bottle_trajectory.py"}
                           if optimized else {"required": False}),
            "robot_object_collision_exemption": ({
                "scope": "NONE_CARTESIAN_ONLY",
                "reason": "本候选无关节 RRT；桌面/夹爪代理/携带物碰撞全部交给资格检查。",
                "does_not_exempt": [
                    "dense robot self collision",
                    "dense robot-table collision",
                    "gripper proxy collision",
                    "carried-object collision after close"
                ],
                "qualification_owner": "robot_mission.qualify_candidate"
            } if cartesian_only else {
                "scope": "RRT_STATIC_OBSTACLES_ONLY",
                "reason": "已有 URDF link11 包含 278mm 虚拟手部，与抓取中心的瓶体必然重叠；故 RRT 只对桌面做机器人碰撞。",
                "does_not_exempt": [
                    "dense robot self collision",
                    "dense robot-table collision",
                    "gripper proxy collision",
                    "carried-object collision after close"
                ],
                "qualification_owner": "robot_mission.qualify_candidate"
            }),
            "pulse": {"step_deg": profile["shared"]["pulse_step_deg"],
                      "period_ms": profile["shared"]["pulse_period_ms"]},
            "maximum_exported_adjacent_step_deg": max_adjacent,
            "cartesian_chain_reports": chain_reports,
            "kinematic_note": (
                "锁定父轨迹 close 的 ~1deg 竖直姿态时，该瓶子 XY 在 j7=76deg 内仍差约 40mm；"
                "本候选使用已有 Track C 向下轴 9.85deg 姿态，笛卡尔高位接近+"
                "纯世界 Z 下降/抬起，禁止 RRT，且不再叠加世界 Z 补偿。"
                "空载在瓶顶之上对准，闭爪前不做倾斜横向扫入；回程含父轨迹 READY→HOME，"
                "必须由放置瓶密集碰撞门复核。"
                if cartesian_only else None),
            "note": ("CANDIDATE only; GUI review is mandatory before staged testing. "
                     "Require live controller recheck, measured bottle dimensions, dense "
                     "qualification and guarded partial runs." if optimized else
                     "CANDIDATE only; require valid live controller limits, current first-point "
                     "check, measured bottle dimensions, dense collision qualification and "
                     "staged guarded run.")
        }
        payload = {"meta": meta, "waypoints": waypoints}
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
        qtask = {"comment": "由瓶子 SDK 位姿+关节锚点推导；只用于当前候选 PB 资格。",
                 "source_task_variant": selected_variant,
                 "pairs": [{"pick": geometry["grip_pick"].tolist(),
                            "place": geometry["place"].tolist(),
                            "actual_pick_grip":
                                bottle_geometry["actual_pick_grip"].tolist(),
                            "actual_place_grip":
                                bottle_geometry["actual_place_grip"].tolist(),
                            "object_pick_center":
                                bottle_geometry["pick_center"].tolist(),
                            "object_place_center":
                                bottle_geometry["place_center"].tolist(),
                            "grasp_capture": grasp_capture,
                            "dimensions_m": [task["scene"]["bottle_radius_m"] * 2,
                                             task["scene"]["bottle_radius_m"] * 2,
                                             task["scene"]["bottle_height_m"]],
                            "carried_center_offset_grip_m":
                                bottle_geometry["center_offset_from_grip_world_m"].tolist()}],
                 "obstacles": [_table_box_spec(task, geometry)]}
        qualification_task_out.parent.mkdir(parents=True, exist_ok=True)
        qualification_task_out.write_text(json.dumps(qtask, ensure_ascii=False, indent=2) + "\n",
                                           encoding="utf-8")
        return {"trajectory": str(out_path), "trajectory_sha256": sha256_file(out_path),
                "qualification_task": str(qualification_task_out),
                "motion_points": len(motion_rows), "gripper_events": 2,
                "maximum_adjacent_step_deg": max_adjacent,
                "orientation_delta_deg": geometry["orientation_delta_deg"],
                "selected_roll_deg": (orientation_search["selected_roll_deg"] if
                                      optimized else 0.0),
                "selected_uvw_deg": (orientation_search["selected_uvw_deg"] if
                                     optimized else geometry["preferred_uvw_deg"]),
                "selected_pick_raw_limit_margin_deg":
                    (orientation_search["selected_solution"]["raw_limit_margin_deg"] if
                     optimized else _raw_joint_margin(geometry["q_pick"], planning_limits)),
                "limit_status": limit_report["status"],
                "solver_bound_expansions": expansions,
                "qualification": qualification}
    finally:
        if p.isConnected():
            p.disconnect()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--task", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--qualification-task-out", type=Path, required=True)
    ap.add_argument("--variant", help="v3 任务中的轨迹 variant 名称")
    ap.add_argument("--seed", type=int, default=26)
    args = ap.parse_args(argv)
    result = build(args.task.resolve(), args.out.resolve(),
                   args.qualification_task_out.resolve(), args.seed, args.variant)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
