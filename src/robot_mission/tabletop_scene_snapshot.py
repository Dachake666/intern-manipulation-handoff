#!/usr/bin/env python3
"""桌上视觉现场快照的可追溯数据处理。

本模块只整理原始观测、处理后的 feature pose 与完整性门禁；
不会把视觉 feature 冒充为抓取点，不会生成轨迹，也不会授权真机运动。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

from .grasp_pose import matrix_to_quaternion_xyzw, sdk_uvw_deg_to_matrix


SCHEMA_VERSION = "tabletop_scene_snapshot.v1"
_REQUIRED_MISSING_IDS = {
    "bottle_feature_to_grasp_transform",
    "bottle_dimensions_and_local_axes",
    "material_box_datum_role",
    "material_box_inner_geometry_floor_and_rim",
    "table_surface_z_mm",
    "safe_transfer_corridor_height_mm",
    "live_arm_try_worlds_joint_path",
    "frame_calibration_gate_pass",
}


def _finite_vector(values, size, label):
    out = np.asarray(values, dtype=float)
    if out.shape != (size,) or not np.all(np.isfinite(out)):
        raise ValueError(f"{label} 必须是 {size} 个有限数值")
    return out


def _unit_quaternion(values, label="quaternion_xyzw"):
    q = _finite_vector(values, 4, label)
    norm = float(np.linalg.norm(q))
    if norm <= 1e-12:
        raise ValueError(f"{label} 模长不能为 0")
    if abs(norm - 1.0) > 1e-3:
        raise ValueError(f"{label} 模长={norm:.9f}, 必须是已归一化四元数")
    return q / norm


def quaternion_shortest_arc_midpoint(q_a, q_b):
    """返回 q_a 与 q_b 的等权最短弧中间姿态（xyzw）。

    四元数 q 和 -q 表示同一姿态。先对齐符号再归一化求和，
    可避免直接分量平均走长弧或接近零四元数。
    """
    a = _unit_quaternion(q_a, "q_a")
    b = _unit_quaternion(q_b, "q_b")
    if float(np.dot(a, b)) < 0.0:
        b = -b
    midpoint = a + b
    norm = float(np.linalg.norm(midpoint))
    if norm <= 1e-12:
        raise ValueError("四元数最短弧中值不唯一")
    return [float(v) for v in midpoint / norm]


def merge_ab_pose(pose_a, pose_b):
    """位置取算术平均，姿态取最短弧等权中值。"""
    p_a = _finite_vector(pose_a["position_mm"], 3,
                         "pose_a.position_mm")
    p_b = _finite_vector(pose_b["position_mm"], 3,
                         "pose_b.position_mm")
    return {
        "position_mm": [float(v) for v in (p_a + p_b) / 2.0],
        "quaternion_xyzw": quaternion_shortest_arc_midpoint(
            pose_a["quaternion_xyzw"], pose_b["quaternion_xyzw"]),
    }


def _same_orientation(q_a, q_b, atol=2e-9):
    a = _unit_quaternion(q_a, "q_a")
    b = _unit_quaternion(q_b, "q_b")
    return 1.0 - abs(float(np.dot(a, b))) <= atol


def validate_snapshot_semantics(snapshot):
    """执行 schema 之外的数值和语义门禁。"""
    if snapshot.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"schema_version 必须为 {SCHEMA_VERSION}")
    if snapshot.get("real_motion_authorized") is not False:
        raise ValueError("现场快照不是轨迹，real_motion_authorized 必须为 false")

    bottle = snapshot["bottle"]
    if (bottle.get("processed_pose_role") != "VISUAL_FEATURE_POSE" or
            bottle.get("visual_feature_location") != "CENTER_UPPER_RIGHT" or
            bottle.get("may_be_used_as_grasp_pose") is not False):
        raise ValueError("瓶子中心偏右上 feature 不是抓取点，禁止直接用于运动")
    observations = bottle["raw_observations"]
    ids = [item["observation_id"] for item in observations]
    if ids != ["A1", "B1"]:
        raise ValueError("bottle.raw_observations 必须按 A1、B1 保留原始观测")
    expected = merge_ab_pose(observations[0]["pose"], observations[1]["pose"])
    actual = bottle["processed_pose"]
    if not np.allclose(expected["position_mm"], actual["position_mm"],
                       atol=1e-9, rtol=0):
        raise ValueError("bottle.processed_pose 位置不是 A1/B1 算术均值")
    if not _same_orientation(expected["quaternion_xyzw"],
                             actual["quaternion_xyzw"]):
        raise ValueError("bottle.processed_pose 姿态不是 A1/B1 最短弧中值")

    _unit_quaternion(snapshot["material_box"]["processed_pose"]
                     ["quaternion_xyzw"], "material_box.processed_pose")
    safe = snapshot["safe_candidate"]
    if safe.get("may_be_used_as_transfer_corridor") is not False:
        raise ValueError("当前 safe 只是起终点候选，未证明可作持物转运高度")
    safe_q = _unit_quaternion(safe["pose"]["quaternion_xyzw"],
                              "safe_candidate.pose.quaternion_xyzw")
    uvw_q = _unit_quaternion(matrix_to_quaternion_xyzw(
        sdk_uvw_deg_to_matrix(safe["pose"]["sdk_world_uvw_deg"])),
        "safe_candidate.pose.sdk_world_uvw_deg")
    if not _same_orientation(safe_q, uvw_q, atol=1e-10):
        raise ValueError("safe_candidate 的 quaternion 与 SDK UVW 不一致")

    missing = [item["requirement_id"] for item in snapshot["missing_requirements"]]
    if len(missing) != len(set(missing)):
        raise ValueError("missing_requirements.requirement_id 不能重复")
    absent = sorted(_REQUIRED_MISSING_IDS - set(missing))
    if absent:
        raise ValueError(f"现场快照缺少必要门禁项: {absent}")

    policy = snapshot["gripper_policy"]
    if any(value is not None for value in (
            policy["open"]["position"], policy["close"]["speed"],
            policy["close"]["force"])):
        raise ValueError("本快照的夹爪参数必须保留为现场可调空接口")


def _delta_metrics(origin, target):
    delta = (_finite_vector(target, 3, "target") -
             _finite_vector(origin, 3, "origin"))
    return {
        "delta_xyz_mm": [float(v) for v in delta],
        "distance_xy_mm": float(np.linalg.norm(delta[:2])),
        "distance_3d_mm": float(np.linalg.norm(delta)),
    }


def analyze_scene_snapshot(snapshot):
    """返回可用于 GUI/审核的非运动分析摘要。"""
    validate_snapshot_semantics(snapshot)
    safe = snapshot["safe_candidate"]["pose"]["position_mm"]
    bottle = snapshot["bottle"]["processed_pose"]["position_mm"]
    box = snapshot["material_box"]["processed_pose"]["position_mm"]
    return {
        "schema_version": "tabletop_scene_snapshot_analysis.v1",
        "snapshot_id": snapshot["snapshot_id"],
        "status": "BLOCKED_FOR_TRAJECTORY",
        "real_motion_authorized": False,
        "bottle_pose_interpretation": "VISUAL_FEATURE_CENTER_UPPER_RIGHT_ONLY",
        "material_box_pose_interpretation": "MERGED_CONTAINER_DATUM_ONLY",
        "safe_candidate_interpretation": "START_END_ONLY_NOT_TRANSFER_CORRIDOR",
        "safe_to_bottle": _delta_metrics(safe, bottle),
        "safe_to_material_box": _delta_metrics(safe, box),
        "bottle_to_material_box": _delta_metrics(bottle, box),
        "blockers": [item["requirement_id"]
                     for item in snapshot["missing_requirements"]],
    }


def load_and_analyze(path):
    """通过版本化 schema 和语义门禁后生成分析。"""
    from .contracts import load_and_validate
    snapshot = load_and_validate(path, SCHEMA_VERSION)
    return snapshot, analyze_scene_snapshot(snapshot)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    args = parser.parse_args(argv)
    _snapshot, analysis = load_and_analyze(args.snapshot)
    print(json.dumps(analysis, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
