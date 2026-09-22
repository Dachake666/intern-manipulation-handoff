#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import datetime as dt
import json
import math
import os
import sys
import uuid
from pathlib import Path

import numpy as np

WORK = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORK))
sys.path.insert(0, str(WORK / "frame_calibration" / "analysis"))
import arm_profiles
import calib_common as cc
from robot_mission.contracts import (canonical_bytes, load_and_validate,
                                     sha256_bytes, sha256_file, validate_document)
from robot_mission.grasp_pose import normalize_quaternion_xyzw


def densify(a, b, step_deg=0.4):
    span = max(abs(float(x) - float(y)) for x, y in zip(a, b))
    count = max(1, int(math.ceil(span / step_deg)))
    return [[float(x) + (float(y) - float(x)) * k / count
             for x, y in zip(a, b)] for k in range(1, count + 1)]


def load_trajectory(path: str | Path) -> tuple[dict, list[dict], str]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    if raw.get("schema_version") == "trajectory.v2":
        validate_document(raw, "trajectory.v2")
        c = raw["contract"]
        meta = {"arm": c["arm"], "arm_id": c["arm_id"],
                "gripper_id": c["gripper_id"], "j6_flipped": True,
                "robot_config_identity": c.get("robot_config_identity"),
                "tcp_id": c.get("tcp_id"),
                "provenance": raw["provenance"]}
        return meta, raw["waypoints"], "trajectory.v2"
    meta, waypoints = raw.get("meta", {}), raw.get("waypoints")
    if not isinstance(waypoints, list) or not waypoints:
        raise ValueError("轨迹缺 waypoints")
    if meta.get("arm_id") not in (1, 2) or meta.get("j6_flipped") is not True:
        raise ValueError("旧轨迹必须声明 meta.arm_id 与 j6_flipped=true")
    meta = dict(meta)
    meta.setdefault("arm", "left" if meta["arm_id"] == 1 else "right")
    meta.setdefault("gripper_id", {1: 2, 2: 1}[meta["arm_id"]])
    return meta, waypoints, "legacy_trajectory.v1"


def expanded_frames(waypoints, current_q, step_deg=0.4):
    moves = [w for w in waypoints if "q_sdk_deg" in w]
    if not moves:
        raise ValueError("轨迹没有关节路点")
    first_path = densify(current_q, moves[0]["q_sdk_deg"], step_deg)
    frames, prev, carrying, carried_index = [], None, False, -1
    for w in waypoints:
        if "gripper" in w:
            carrying = w["gripper"] == "close"
            if carrying:
                carried_index += 1
            continue
        if "vision_check" in w:
            continue
        q = w["q_sdk_deg"]
        if not isinstance(q, list) or len(q) != 7:
            raise ValueError("q_sdk_deg 必须为 7 元数组")
        if prev is None:
            prev = q
            continue
        for dense in densify(prev, q, step_deg):
            frames.append({"q_sdk_deg": dense, "seg": w.get("seg", ""),
                           "carrying": carrying,
                           "carried_index": carried_index if carrying else None})
        prev = q
    return first_path, frames


def grasp_event_qs(waypoints):
    """返回每次 close 对应的最后一个运动姿态，供捕获区门复核。"""
    last_q, result = None, []
    for waypoint in waypoints:
        if "q_sdk_deg" in waypoint:
            last_q = waypoint["q_sdk_deg"]
        elif waypoint.get("gripper") == "close" and last_q is not None:
            result.append(last_q)
    return result


def grasp_capture_geometry(profile, grip, rotation, pair):
    """抓握点与瓶体质心分开；有示教抓握点时必须首先证明它位于瓶体内。

    不改变物体位置，也不放宽 profile 中的捕获范围。旧任务无抓握点时保留质心契约。
    """
    spec = pair.get("grasp_capture", {})
    center = np.asarray(pair["object_pick_center"], float)
    point = np.asarray(spec.get("object_grasp_point_world_m", center), float)
    dimensions = np.asarray(pair["dimensions_m"], float)
    if (center.shape != (3,) or point.shape != (3,) or dimensions.shape != (3,) or
            not np.isfinite(np.concatenate((center, point, dimensions))).all() or
            np.any(dimensions <= 0)):
        raise ValueError("瓶体中心、抓握点和尺寸必须是有限三元组，尺寸必须大于零")
    half = dimensions / 2.0
    in_body = bool(np.all(np.isfinite(point)) and np.all(np.abs(point-center) <= half + 1e-9))
    axis = np.asarray(rotation, float)[:, 1]
    if not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-9:
        raise ValueError("抓握方向必须是有效旋转轴")
    axis = axis / np.linalg.norm(axis)
    delta = point - np.asarray(grip, float)
    axial = float(np.dot(delta, axis))
    lateral = float(np.linalg.norm(delta - axial * axis))
    axial_range = [float(v) for v in spec.get("axis_range_m", profile["grasp_capture_axis_range_m"])]
    lateral_limit = float(spec.get("lateral_tolerance_m", profile["grasp_capture_lateral_tolerance_m"]))
    passed = in_body and axial_range[0] <= axial <= axial_range[1] and lateral <= lateral_limit
    return {"status": "PASS" if passed else "BLOCKED", "axis_offset_m": axial,
            "axis_range_m": axial_range, "lateral_error_m": lateral,
            "lateral_tolerance_m": lateral_limit, "grasp_point_inside_object": in_body,
            "grasp_point_world_m": point.tolist(), "object_center_world_m": center.tolist(),
            "grasp_point_reference": spec.get("reference", "OBJECT_CENTER_LEGACY")}


def limit_check(profile, first_path, frames, minimum_margin=5.0):
    limits = profile.get("controller_limits_deg")
    if limits is None:
        return {"status": "UNRESOLVED", "details": {
            "reason": profile.get("controller_limits_status")}}
    worst = (float("inf"), None, None)
    violations = []
    for source, seq in (("first_point", first_path), ("trajectory", [x["q_sdk_deg"] for x in frames])):
        for i, q in enumerate(seq):
            for j, (value, (lo, hi)) in enumerate(zip(q, limits), 1):
                margin = min(float(value) - lo, hi - float(value))
                if margin < worst[0]:
                    worst = (margin, f"{source}[{i}]", j)
                if margin < minimum_margin - 1e-6:
                    violations.append({"where": f"{source}[{i}]", "joint": j,
                                       "value_deg": value, "limit_deg": [lo, hi],
                                       "margin_deg": margin})
                    if len(violations) >= 20:
                        break
    return {"status": "PASS" if not violations else "BLOCKED",
            "details": {"minimum_margin_deg": worst[0], "minimum_at": worst[1],
                        "minimum_joint": worst[2], "required_margin_deg": minimum_margin,
                        "violations": violations}}


def _set_robot(p, robot, active_profile, active_q_sdk, inactive_profile, inactive_q_sdk):
    for profile, q_sdk in ((active_profile, active_q_sdk),
                           (inactive_profile, inactive_q_sdk)):
        q_urdf = arm_profiles.sdk_to_urdf_deg(profile, q_sdk)
        for jid, value in zip(profile["joint_ids"], q_urdf):
            p.resetJointState(robot, jid, math.radians(value))
    occupied = set(active_profile["joint_ids"] + inactive_profile["joint_ids"])
    for jid in range(p.getNumJoints(robot)):
        if jid not in occupied and p.getJointInfo(robot, jid)[2] != p.JOINT_FIXED:
            p.resetJointState(robot, jid, 0.0)


def pybullet_planning_custom_limits(profile: dict) -> dict[int, tuple[float, float]]:
    """把资格检查实际采用的 SDK 限位映射为 pybullet_planning 的 URDF 弧度限位。

    ``get_collision_fn`` 默认把 URDF 限位检查混进自碰撞判断。控制器实测范围与
    URDF 不完全相同（左臂 j2 就允许小于 0°），若不显式传入会把合法姿态误报
    成 ``robot_self_or_dual_arm``，且报告里没有任何碰撞 link pair。
    """
    limits = profile.get("controller_limits_deg")
    if limits is None:
        return {}
    lower_sdk = [float(pair[0]) for pair in limits]
    upper_sdk = [float(pair[1]) for pair in limits]
    lower_urdf = arm_profiles.sdk_to_urdf_deg(profile, lower_sdk)
    upper_urdf = arm_profiles.sdk_to_urdf_deg(profile, upper_sdk)
    return {
        int(jid): (math.radians(min(float(lo), float(hi))),
                   math.radians(max(float(lo), float(hi))))
        for jid, lo, hi in zip(profile["joint_ids"], lower_urdf, upper_urdf)
    }


def collision_check(profile, inactive_profile, inactive_q_sdk, first_path, frames,
                    task_data, object_dimensions_m=(0.05, 0.05, 0.05),
                    grasp_event_qs=()):
    try:
        import pybullet as p
    except ImportError:
        return {"status": "UNRESOLVED", "details": {"reason": "pybullet_missing"}}
    client = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(cc.DEFAULT_URDF, useFixedBase=True)
        obstacles = []
        for item in task_data.get("obstacles", []):
            orientation = normalize_quaternion_xyzw(
                item.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]))
            body = p.createMultiBody(0, p.createCollisionShape(
                p.GEOM_BOX, halfExtents=item["half"]),
                basePosition=item["center"], baseOrientation=orientation.tolist())
            obstacles.append((body, item))
        proxy = p.createMultiBody(0, p.createCollisionShape(
            p.GEOM_BOX, halfExtents=(np.asarray(profile["gripper_collision_box_m"]) / 2).tolist()))
        dimensions = object_dimensions_m
        if not dimensions or not isinstance(dimensions[0], (list, tuple)):
            dimensions = [dimensions]
        pairs = task_data.get("pairs", [])
        if len(pairs) > len(dimensions):
            raise ValueError("每个抓放对象都必须有独立的尺寸")
        rigid = [index < len(pairs) and pairs[index].get("object_attachment_mode") == "RIGID_FROM_CLOSE"
                 for index in range(len(dimensions))]
        grasp_rotations = [None] * len(dimensions)
        local_offsets = [None] * len(dimensions)
        released_poses = [None] * len(dimensions)
        carried_offsets = [pair.get("carried_center_offset_grip_m", [0.0, 0.0, 0.0])
                           for pair in pairs]
        while len(carried_offsets) < len(dimensions):
            carried_offsets.append([0.0, 0.0, 0.0])
        carried_bodies = [p.createMultiBody(0, p.createCollisionShape(
            p.GEOM_BOX, halfExtents=(np.asarray(item, float) / 2).tolist()))
            for item in dimensions]
        object_pick_centers, object_place_centers = [], []
        for index, item in enumerate(dimensions):
            pair = pairs[index] if index < len(pairs) else {}
            offset = np.asarray(carried_offsets[index], float)
            pick_grip = np.asarray(pair.get("actual_pick_grip", pair.get("pick", [0, 0, 0])),
                                   float)
            place_grip = np.asarray(
                pair.get("actual_place_grip", pair.get("place", pick_grip)), float)
            object_pick_centers.append(np.asarray(
                pair.get("object_pick_center", pick_grip + offset), float))
            object_place_centers.append(np.asarray(
                pair.get("object_place_center", place_grip + offset), float))
        was_carried = [False] * len(carried_bodies)
        violations, min_clearance = [], float("inf")
        minimum_by_kind = {
            "robot_environment": float("inf"),
            "gripper_proxy_environment": float("inf"),
            "carried_object_environment": float("inf"),
            "released_or_stationary_object_environment": float("inf"),
            "robot_object": float("inf"),
        }
        min_gripper_object_clearance = float("inf")
        capture_checks = []
        self_collision = None
        disabled_pairs = task_data.get("self_collision_disabled_link_pairs")
        if disabled_pairs is not None:
            # getContactPoints 在未启用 URDF 自碰撞时可能永远为空；显式最近点逐连杆检查。
            import pybullet_planning as pp
            self_collision = pp.get_collision_fn(
                robot, profile["joint_ids"], obstacles=[], self_collisions=True,
                disabled_collisions={tuple(pair) for pair in disabled_pairs},
                custom_limits=pybullet_planning_custom_limits(profile))

        for index, q_sdk in enumerate(grasp_event_qs):
            if index >= len(pairs):
                violations.append({"where": f"grasp[{index}]",
                                   "kind": "grasp_capture_contract",
                                   "reason": "missing_pair"})
                continue
            spec = pairs[index].get("grasp_capture")
            if not isinstance(spec, dict):
                violations.append({"where": f"grasp[{index}]",
                                   "kind": "grasp_capture_contract",
                                   "reason": "missing_grasp_capture"})
                continue
            _set_robot(p, robot, profile, q_sdk, inactive_profile, inactive_q_sdk)
            st = p.getLinkState(robot, profile["ee_link_id"],
                                computeForwardKinematics=True)
            R = np.asarray(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
            grip = np.asarray(st[4]) + R @ (
                np.asarray(profile["grip_center_link_mm"]) / 1000.0)
            grasp_rotations[index] = st[5]
            local_offsets[index] = R.T @ (object_pick_centers[index] - grip)
            proxy_center = np.asarray(st[4]) + R @ (
                np.asarray(profile["gripper_collision_center_link_mm"]) / 1000.0)
            p.resetBasePositionAndOrientation(proxy, proxy_center, st[5])
            p.performCollisionDetection()

            # 闭爪一刻单独量桌面/环境间隙，用来区分“已到瓶子但末端
            # 过低”和“末端根本没到抓取区”。供应商 URDF 的幽灵手继续由
            # 实测保守 proxy 代替，但除末端幽灵网格外的连杆都仍参与。
            close_environment = {}
            excluded_ghost_links = {profile["ee_link_id"],
                                    inactive_profile["ee_link_id"]}
            for kind, candidate, required_key in (
                    ("robot", robot, "minimum_robot_clearance_m"),
                    ("gripper", proxy, "minimum_gripper_clearance_m")):
                worst = None
                for body, obstacle_spec in obstacles:
                    near = p.getClosestPoints(candidate, body, 1.0)
                    if kind == "robot":
                        near = [row for row in near
                                if row[3] not in excluded_ghost_links]
                    if not near:
                        continue
                    distance = min(float(row[8]) for row in near)
                    required = float(obstacle_spec.get(required_key, 0.0))
                    row = {
                        "obstacle": obstacle_spec.get("source_obstacle_id", "legacy"),
                        "clearance_m": distance,
                        "required_clearance_m": required,
                        "margin_m": distance - required,
                    }
                    if worst is None or row["margin_m"] < worst["margin_m"]:
                        worst = row
                close_environment[kind] = worst
            if not obstacles:
                close_environment_status = "UNRESOLVED"
            else:
                close_environment_status = (
                    "PASS" if all(row is None or row["margin_m"] >= 0.0
                                  for row in close_environment.values())
                    else "BLOCKED")
            close_environment["status"] = close_environment_status

            capture_pair = dict(pairs[index], object_pick_center=object_pick_centers[index].tolist(),
                                dimensions_m=dimensions[index])
            result = {"where": f"grasp[{index}]",
                      **grasp_capture_geometry(profile, grip, R, capture_pair),
                      "environment_clearance_at_close": close_environment}
            capture_checks.append(result)
            if result["status"] != "PASS":
                violations.append({**result, "kind": "grasp_capture"})

        def inspect(q_sdk, where, carrying, carried_index=None):
            nonlocal min_clearance, min_gripper_object_clearance
            _set_robot(p, robot, profile, q_sdk, inactive_profile, inactive_q_sdk)
            st = p.getLinkState(robot, profile["ee_link_id"], computeForwardKinematics=True)
            R = np.asarray(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
            grip = np.asarray(st[4]) + R @ (np.asarray(profile["grip_center_link_mm"]) / 1000.0)
            proxy_center = np.asarray(st[4]) + R @ (np.asarray(
                profile["gripper_collision_center_link_mm"]) / 1000.0)
            p.resetBasePositionAndOrientation(proxy, proxy_center, st[5])
            selected_carried = None
            if carrying:
                if carried_index is None or carried_index >= len(carried_bodies):
                    violations.append({"where": where, "kind": "carried_object_contract",
                                       "reason": "missing_dimensions_for_action",
                                       "carried_index": carried_index})
                else:
                    selected_carried = carried_bodies[carried_index]
                    was_carried[carried_index] = True
                    if rigid[carried_index] and local_offsets[carried_index] is None:
                        raise ValueError("刚性携带缺少闭爪参考姿态，不能回退到平移模型")
                    if rigid[carried_index]:
                        object_center = grip + R @ local_offsets[carried_index]
                        _zero, inverse = p.invertTransform([0, 0, 0], grasp_rotations[carried_index])
                        _zero, orientation = p.multiplyTransforms([0, 0, 0], st[5], [0, 0, 0], inverse)
                    else:
                        object_center = grip + np.asarray(carried_offsets[carried_index], float)
                        orientation = [0, 0, 0, 1]
                    p.resetBasePositionAndOrientation(selected_carried, object_center, orientation)
                    released_poses[carried_index] = (object_center.tolist(), orientation)
            for index, body in enumerate(carried_bodies):
                if body == selected_carried:
                    continue
                if rigid[index] and was_carried[index] and released_poses[index] is not None:
                    # 松开后保持实际释放位置，不能瞬移到任务声明的放置点来掩盖落差。
                    center, orientation = released_poses[index]
                else:
                    center = (object_place_centers[index] if was_carried[index]
                              else object_pick_centers[index])
                    orientation = [0, 0, 0, 1]
                p.resetBasePositionAndOrientation(body, center, orientation)
            p.performCollisionDetection()
            self_hits = {(min(c[3], c[4]), max(c[3], c[4]))
                         for c in p.getContactPoints(robot, robot)
                         if c[3] != c[4]}
            self_failed = (self_collision([math.radians(v) for v in
                           arm_profiles.sdk_to_urdf_deg(profile, q_sdk)])
                           if self_collision is not None else bool(self_hits))
            if self_failed:
                violations.append({"where": where, "kind": "robot_self_or_dual_arm",
                                   "link_pairs": sorted(self_hits)[:10]})
            for body, spec in obstacles:
                candidates = [("robot_environment", robot), ("gripper_proxy_environment", proxy)]
                candidates.extend(("carried_object_environment" if candidate == selected_carried else
                                   "released_or_stationary_object_environment", candidate)
                                  for candidate in carried_bodies)
                for kind, candidate in candidates:
                    if candidate is None:
                        continue
                    near = p.getClosestPoints(candidate, body, 1.0)
                    if kind == "robot_environment":
                        # 供应商 URDF 两个腕末端含历史“幽灵手”碰撞网格；视觉线用
                        # 上面的实测保守 proxy 取代它们，但不豁免其余任何连杆。
                        excluded_ghost_links = {profile["ee_link_id"],
                                                inactive_profile["ee_link_id"]}
                        near = [x for x in near if x[3] not in excluded_ghost_links]
                    if near:
                        distance = min(float(x[8]) for x in near)
                        min_clearance = min(min_clearance, distance)
                        if kind in minimum_by_kind:
                            minimum_by_kind[kind] = min(minimum_by_kind[kind], distance)
                        # 瓶底放到桌面时理论距离就是 0，PyBullet 的盒体接触会出现
                        # 微米级负值。只允许资格任务显式声明的支撑面数值容差；机器人、
                        # 夹爪代理和非支撑障碍仍保持严格零穿透。
                        tolerance = (float(spec.get("support_contact_tolerance_m", 0.0))
                                     if kind in ("carried_object_environment",
                                                 "released_or_stationary_object_environment") else 0.0)
                        required_clearance = (
                            float(spec.get("minimum_robot_clearance_m", 0.0))
                            if kind == "robot_environment" else
                            float(spec.get("minimum_gripper_clearance_m", 0.0))
                            if kind == "gripper_proxy_environment" else -tolerance)
                        if distance < required_clearance:
                            violations.append({"where": where, "kind": kind,
                                               "obstacle": spec.get("source_obstacle_id", "legacy"),
                                               "distance_m": distance,
                                               "required_clearance_m": required_clearance,
                                               "allowed_support_contact_tolerance_m": tolerance,
                                               "link_ids": sorted({x[3] for x in near})})
            # 机械臂实体对瓶子不享受手指接触豁免；端部幽灵网格由实测夹爪代理替代。
            for index, body in enumerate(carried_bodies):
                near = [row for row in p.getClosestPoints(robot, body, 1.0)
                        if row[3] not in {profile["ee_link_id"], inactive_profile["ee_link_id"]}]
                if near:
                    distance = min(float(row[8]) for row in near)
                    minimum_by_kind["robot_object"] = min(minimum_by_kind["robot_object"], distance)
                    if distance < 0:
                        violations.append({"where": where, "kind": "robot_object",
                                           "object_index": index, "distance_m": distance})
            # 该代理包住整只夹爪，并非只含掌部。包络与目标重叠不能区分手指内侧
            # 的预期接触；以下豁免范围必须公开，不能据此声称动力学抓取一定成功。
            for index, body in enumerate(carried_bodies):
                if body == selected_carried:
                    continue
                # 张开的手指在最终接近段包围目标瓶体是预期行为；close 瞬间由上面的
                # 独立捕获区门复核。放置后的瓶体不享受此豁免。
                if not was_carried[index] and any(
                        name in where for name in (
                            "BOTTLE_APPROACH", "BOTTLE_DESCEND", "PICK_DESCEND")):
                    continue
                if was_carried[index] and any(name in where for name in (
                        "BOTTLE_PLACE_RETREAT", "BOTTLE_PLACE_LIFT",
                        "BOTTLE_PLACE_ASCEND", "PLACE_ASCEND")):
                    continue
                near = p.getClosestPoints(proxy, body, 1.0)
                if not near:
                    continue
                distance = min(float(x[8]) for x in near)
                min_clearance = min(min_clearance, distance)
                min_gripper_object_clearance = min(
                    min_gripper_object_clearance, distance)
                if distance < 0.0:
                    violations.append({"where": where,
                                       "kind": "gripper_proxy_object",
                                       "object_index": index,
                                       "distance_m": distance})
            return len(violations) < 20

        checked_first = checked_motion = 0
        for i, q in enumerate(first_path):
            checked_first += 1
            if not inspect(q, f"first_point[{i}]", False):
                break
        if len(violations) < 20:
            for i, frame in enumerate(frames):
                checked_motion += 1
                if not inspect(frame["q_sdk_deg"], f"dense[{i}]/{frame['seg']}",
                               frame["carrying"], frame.get("carried_index")):
                    break
        return {"status": "PASS" if not violations else "BLOCKED",
                "details": {"checked_first_point_frames": checked_first,
                            "checked_trajectory_frames": checked_motion,
                            "planned_trajectory_frames": len(frames),
                            "completed_dense_check": checked_motion == len(frames),
                            "self_collision_check_mode": ("EXPLICIT_LINK_CLOSEST_POINTS" if self_collision
                                                          else "LEGACY_CONTACT_POINTS_ONLY"),
                            "self_collision_joint_limits": (
                                "ACTIVE_CONTROLLER_PROFILE_MAPPED_TO_URDF"
                                if self_collision and profile.get("controller_limits_deg") is not None
                                else "URDF_DEFAULTS"),
                            "self_collision_disabled_link_pairs": disabled_pairs,
                            "minimum_clearance_m": None if min_clearance == float("inf") else min_clearance,
                            "gripper_proxy_m": profile["gripper_collision_box_m"],
                            "gripper_proxy_center_link_mm": profile["gripper_collision_center_link_mm"],
                            "minimum_gripper_object_clearance_m": (
                                None if min_gripper_object_clearance == float("inf")
                                else min_gripper_object_clearance),
                            "minimum_clearances_by_kind_m": {
                                key: (None if value == float("inf") else value)
                                for key, value in minimum_by_kind.items()
                            },
                            "grasp_capture_checks": capture_checks,
                            "carried_object_dimensions_m": dimensions,
                            "carried_center_offsets_grip_m": carried_offsets,
                            "object_attachment_modes": [pair.get("object_attachment_mode", "LEGACY_WORLD_OFFSET") for pair in pairs],
                            "actual_release_poses": released_poses,
                            "intentional_contact_exemptions": [
                                "target bottle vs full gripper envelope during final pick descend",
                                "held bottle vs gripper envelope while attached",
                                "released bottle vs gripper envelope during place ascend only; subsequent return is checked"],
                            "support_contact_tolerances_m": {
                                spec.get("source_obstacle_id", "legacy"): float(
                                    spec.get("support_contact_tolerance_m", 0.0))
                                for _body, spec in obstacles
                                if float(spec.get("support_contact_tolerance_m", 0.0)) > 0.0
                            },
                            "violations": violations}}
    finally:
        p.disconnect(client)


def report_hash(report: dict) -> str:
    normalized = copy.deepcopy(report)
    normalized["approval"]["report_sha256"] = "0" * 64
    normalized["approval"]["token"] = None
    return sha256_bytes(canonical_bytes(normalized))


def _check(name, status, **details):
    return {"name": name, "status": status, "details": details}


def build_report(args) -> dict:
    trajectory_path, task_path = Path(args.trajectory), Path(args.task)
    meta, waypoints, trajectory_schema = load_trajectory(trajectory_path)
    profile = arm_profiles.arm_profile(meta["arm"])
    inactive = arm_profiles.arm_profile("right" if meta["arm"] == "left" else "left")
    current = {"left": list(map(float, args.left_q)),
               "right": list(map(float, args.right_q))}
    first_path, frames = expanded_frames(waypoints, current[meta["arm"]], args.step_deg)
    task_data = json.loads(task_path.read_text(encoding="utf-8"))
    if "pairs" not in task_data:
        raise ValueError("preflight 当前要求 --task 为适配后的 pairs+obstacles JSON")
    dimensions = [pair.get("dimensions_m", [0.05, 0.05, 0.05])
                  for pair in task_data["pairs"]]
    checks = []
    identity_ok = (meta["arm_id"] == profile["arm_id"] and
                   meta["gripper_id"] == profile["gripper_id"] and
                   (not meta.get("robot_config_identity") or
                    meta["robot_config_identity"] == args.robot_config_identity))
    checks.append(_check("arm_gripper_contract", "PASS" if identity_ok else "BLOCKED",
                         trajectory_schema=trajectory_schema, arm=meta["arm"],
                         arm_id=meta["arm_id"], gripper_id=meta["gripper_id"],
                         expected=[profile["arm_id"], profile["gripper_id"]]))
    if trajectory_schema == "trajectory.v2":
        closes = sum(w.get("gripper") == "close" for w in waypoints)
        opens = sum(w.get("gripper") == "open" for w in waypoints)
        grasp_checks = sum(w.get("vision_check", {}).get("kind") == "grasp_follow"
                           for w in waypoints)
        place_checks = sum(w.get("vision_check", {}).get("kind") == "placement"
                           for w in waypoints)
        event_ok = closes == grasp_checks and opens == place_checks and closes == opens
        checks.append(_check("vision_acceptance_events", "PASS" if event_ok else "BLOCKED",
                             closes=closes, opens=opens, grasp_checks=grasp_checks,
                             placement_checks=place_checks))
    dual_ready = all(arm_profiles.arm_profile(a).get("controller_limits_deg") is not None
                     for a in ("left", "right"))
    checks.append(_check("dual_arm_profiles", "PASS" if dual_ready else "UNRESOLVED",
                         left_status=arm_profiles.arm_profile("left")["controller_limits_status"],
                         right_status=arm_profiles.arm_profile("right")["controller_limits_status"]))
    checks.append({"name": "joint_limits_and_margin", **limit_check(
        profile, first_path, frames, args.minimum_joint_margin_deg)})
    checks.append({"name": "dense_collision_and_carried_object", **collision_check(
        profile, inactive, current[inactive["arm"]], first_path, frames, task_data,
        dimensions, grasp_event_qs(waypoints))})

    frame_sha = calibration_sha = "0" * 64
    if args.frame_gate:
        frame = json.loads(Path(args.frame_gate).read_text(encoding="utf-8"))
        frame_sha = sha256_file(args.frame_gate)
        checks.append(_check("pb_sdk_frame_gate", "PASS" if frame.get("verdict") == "PASS" else "BLOCKED",
                             verdict=frame.get("verdict"), age_days=frame.get("age_days"),
                             drift_xyz_mm=frame.get("drift_xyz_mm")))
    else:
        checks.append(_check("pb_sdk_frame_gate", "UNRESOLVED", reason="report_missing"))
    if args.calibration:
        try:
            calib = load_and_validate(args.calibration, "eye_to_hand_calibration.v1")
            calibration_sha = sha256_file(args.calibration)
            status = "PASS" if calib["arm"] == meta["arm"] else "BLOCKED"
            checks.append(_check("eye_to_hand_calibration", status,
                                 calibration_id=calib["calibration_id"], arm=calib["arm"]))
        except Exception as exc:
            checks.append(_check("eye_to_hand_calibration", "BLOCKED", error=str(exc)))
    else:
        checks.append(_check("eye_to_hand_calibration", "UNRESOLVED", reason="validated_calibration_missing"))

    observation_sha = "0" * 64
    if args.observation:
        try:
            obs = load_and_validate(args.observation, "vision_observation.v1")
            observation_sha = sha256_file(args.observation)
            stamp = dt.datetime.fromtimestamp(obs["camera"]["ros_timestamp_ns"] / 1e9,
                                              tz=dt.timezone.utc)
            age = (dt.datetime.now(dt.timezone.utc) - stamp).total_seconds()
            checks.append(_check("scene_freshness", "PASS" if 0 <= age <= args.max_scene_age_s else "BLOCKED",
                                 age_s=age, max_age_s=args.max_scene_age_s))
        except Exception as exc:
            checks.append(_check("scene_freshness", "BLOCKED", error=str(exc)))
    else:
        checks.append(_check("scene_freshness", "UNRESOLVED", reason="observation_missing"))

    adapted_task_sha = sha256_file(task_path)
    task_sha = adapted_task_sha
    task_request = None
    if args.task_request:
        try:
            task_request = load_and_validate(args.task_request, "task_request.v1")
            task_sha = sha256_file(args.task_request)
            if task_request["observation"]["sha256"] != observation_sha:
                raise ValueError("task_request 未绑定当前 observation")
            checks.append(_check("task_request_contract", "PASS",
                                 task_id=task_request["task_id"],
                                 action_count=len(task_request["actions"])))
        except Exception as exc:
            task_request = None
            checks.append(_check("task_request_contract", "BLOCKED", error=str(exc)))
    elif trajectory_schema == "trajectory.v2":
        checks.append(_check("task_request_contract", "UNRESOLVED",
                             reason="trajectory.v2 必须提供原始 --task-request"))
    if trajectory_schema == "trajectory.v2" and task_request is not None:
        provenance = meta["provenance"]
        expected = {"observation_sha256": observation_sha,
                    "task_sha256": task_sha,
                    "adapted_task_sha256": adapted_task_sha,
                    "calibration_id": calib["calibration_id"] if args.calibration and 'calib' in locals() else None}
        mismatches = {key: [provenance.get(key), value] for key, value in expected.items()
                      if value is None or provenance.get(key) != value}
        checks.append(_check("trajectory_provenance", "PASS" if not mismatches else "BLOCKED",
                             mismatches=mismatches))
    verdict = "PASS" if all(x["status"] == "PASS" for x in checks) else "BLOCKED"
    now = dt.datetime.now(dt.timezone.utc).isoformat()
    report = {
        "schema_version": "preflight_report.v1", "report_id": str(uuid.uuid4()),
        "created_at": now, "mode": "no_motion_read_only",
        "inputs": {"observation_sha256": observation_sha, "task_sha256": task_sha,
                   "adapted_task_sha256": adapted_task_sha,
                   "trajectory_sha256": sha256_file(trajectory_path),
                   "calibration_sha256": calibration_sha,
                   "arm_profiles_sha256": sha256_file(WORK / "arm_profiles.v1.json"),
                   "robot_config_identity": args.robot_config_identity},
        "robot_state": {"source": "explicit_snapshot", "left_q_sdk_deg": current["left"],
                        "right_q_sdk_deg": current["right"]},
        "checks": checks, "verdict": verdict,
        "approval": {"required": True, "report_sha256": "0" * 64,
                     "token": None, "expires_on_change": True},
    }
    report["approval"]["report_sha256"] = report_hash(report)
    validate_document(report, "preflight_report.v1")
    return report


def render_text(report):
    lines = [f"NO-MOTION PREFLIGHT: {report['verdict']}",
             f"report_sha256: {report['approval']['report_sha256']}"]
    for c in report["checks"]:
        lines.append(f"[{c['status']:10s}] {c['name']}: " +
                     json.dumps(c["details"], ensure_ascii=False, separators=(",", ":")))
    lines.append("PASS 后仍需操作员批准；任何输入/姿态/标定/场景变化都会使批准失效。")
    return "\n".join(lines) + "\n"


def read_live_snapshot(robot_ip, local_ip, arm_ip, arm_port):
    """唯一允许的现场路径：sdk_session enable=False，不清报警/不使能/不设速度。"""
    robot_side = WORK / "frame_calibration" / "robot_side"
    sys.path.insert(0, str(robot_side))
    import robot_lock
    import sdk_session as ss
    sdk = None
    with robot_lock.acquire_robot(robot_ip, "preflight_read_only.py"):
        try:
            sdk = ss.open_session(robot_ip, local_ip, arm_ip, arm_port, 1.0,
                                  (1, 2), enable=False)
            snapshot = {"left": ss.read_joints(sdk, 1),
                        "right": ss.read_joints(sdk, 2)}
            limits = {}
            for arm_id, name in ((1, "left"), (2, "right")):
                try:
                    value = ss.read_axis_limits(sdk, arm_id)
                    valid, details = ss.axis_limits_look_valid(value)
                    limits[name] = {"valid": valid, "details": details,
                                    "limits_deg": value}
                except Exception as exc:
                    limits[name] = {"valid": False, "error": str(exc)}
            return snapshot, limits
        finally:
            if sdk is not None:
                ss.close_session(sdk)


def main(argv=None):
    ap = argparse.ArgumentParser(description="一键无运动预检；不导入 pypilot，不连接机器人")
    ap.add_argument("--no-motion", action="store_true", required=True)
    ap.add_argument("--trajectory", required=True); ap.add_argument("--task", required=True)
    ap.add_argument("--task-request", help="原始 task_request.v1；trajectory.v2 必填")
    ap.add_argument("--observation"); ap.add_argument("--calibration"); ap.add_argument("--frame-gate")
    ap.add_argument("--left-q", type=float, nargs=7)
    ap.add_argument("--right-q", type=float, nargs=7)
    ap.add_argument("--live-read-only", action="store_true")
    ap.add_argument("--robot-ip"); ap.add_argument("--local-ip")
    ap.add_argument("--arm-ip"); ap.add_argument("--arm-port", type=int, default=8080)
    ap.add_argument("--robot-config-identity", required=True)
    ap.add_argument("--step-deg", type=float, default=0.4)
    ap.add_argument("--minimum-joint-margin-deg", type=float, default=5.0)
    ap.add_argument("--max-scene-age-s", type=float, default=2.0)
    ap.add_argument("--json-out", required=True); ap.add_argument("--text-out", required=True)
    args = ap.parse_args(argv)
    if abs(args.step_deg - 0.4) > 1e-12:
        ap.error("视觉发布线固定使用 0.4° 密集插值")
    if args.live_read_only:
        if not args.robot_ip or not args.local_ip:
            ap.error("--live-read-only 必须提供 robot-ip/local-ip（arm-ip 默认跟 robot）")
        snapshot, live_limits = read_live_snapshot(
            args.robot_ip, args.local_ip, args.arm_ip or args.robot_ip, args.arm_port)
        args.left_q, args.right_q = snapshot["left"], snapshot["right"]
    elif args.left_q is None or args.right_q is None:
        ap.error("离线模式必须显式提供 --left-q 和 --right-q")
    report = build_report(args)
    if args.live_read_only:
        report["robot_state"]["source"] = "live_read_only"
        report["checks"].append(_check("live_controller_limits", "PASS" if
            all(x.get("valid") for x in live_limits.values()) else "BLOCKED", **live_limits))
        report["verdict"] = "PASS" if all(x["status"] == "PASS" for x in report["checks"]) else "BLOCKED"
        report["approval"]["report_sha256"] = report_hash(report)
        validate_document(report, "preflight_report.v1")
    Path(args.json_out).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    Path(args.text_out).write_text(render_text(report), encoding="utf-8")
    print(render_text(report), end="")
    return 0 if report["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
