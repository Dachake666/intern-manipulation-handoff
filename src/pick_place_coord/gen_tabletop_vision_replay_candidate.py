#!/usr/bin/env python3
"""由桌上 AB 视觉快照生成仅供 PyBullet 回放的关节候选。

这是一个刻意受限的离线假设候选：缺失的视觉 feature->抓取中心、料框 datum、
本次瓶子尺寸和 PB<->SDK 标定均不会被冒充为实测结论。脚本只在显式给出
``--accept-simulation-assumptions`` 时生成，并始终写入
``real_motion_authorized=false`` / ``debian_execution_allowed=false``。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WORK))

import arm_profiles
import gen_bottle_servo_candidate as gen
import xifeng_pb as xf
from frame_calibration.analysis import calib_common as cc
from robot_mission.grasp_pose import quaternion_xyzw_to_matrix


SCHEMA = "tabletop_scene_snapshot.v1"
ARM = "left"
BOTTLE_DIMENSIONS_M = np.array([0.070, 0.070, 0.200])
BOX_WALL_M = 0.010
BOX_FLOOR_PROXY_M = 0.004
TRANSFER_CLEARANCE_M = 0.020
BOTTLE_INTERIOR_CLEARANCE_M = 0.020
GRIPPER_WALL_CLEARANCE_M = 0.0025
SUPPORT_PLANE_CLEARANCE_M = 0.002
TABLE_THICKNESS_M = 0.050
TABLE_XY_PADDING_M = 0.300
# [待核] 仅供本次 GUI 仿真的稳定抓取姿态。由固定 seed=42 的有界离线搜索
# 得到：同一姿态覆盖 pick/high/place，且 proxy 投影可在箱内留出余量。
TASK_R_LINK_ANCHOR = np.array([
    [0.0196799701322955, 0.7650910846447097, 0.6436212636114298],
    [-0.9501489608156220, -0.1860443003458956, 0.2502088538996966],
    [0.3111746310941582, -0.6164601775508194, 0.7232891527303771],
])
ANCHOR_CONTRACT_STATUS = (
    "MODEL_A_EE_LINK_ORIGIN_SINGLE_POSE_ASSUMPTION_CONFLICTS_WITH_SNAPSHOT_TCP_LABEL")
CANDIDATE_POLICY = {
    "qualification": "SIMULATION_ASSUMPTION_CANDIDATE",
    "scene_geometry_status": "SIMULATION_ASSUMPTIONS_UNCONFIRMED",
    "controller_limit_profile": "left_preview102_20260827",
    "real_motion_authorized": False,
    "debian_execution_allowed": False,
}


class ReplayCandidateBlocked(RuntimeError):
    """离线坐标假设无法形成连续关节轨迹；禁止写出候选。"""

    def __init__(self, stage: str, cause: Exception, start, end):
        super().__init__(
            f"{stage} IK 不可达或不连续: {cause}; "
            "禁止生成假轨迹。需要有效 PB-SDK 标定，或 Debian armTryWorlds "
            "返回 PICK_HIGH/PICK/PLACE_HIGH/PLACE 的真实 q_sdk_deg。")
        self.stage = stage
        self.cause = str(cause)
        self.start_pb_m = np.asarray(start, float).tolist()
        self.end_pb_m = np.asarray(end, float).tolist()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _q_xyzw(rotation: np.ndarray) -> list[float]:
    # PyBullet 没有 matrix->quaternion API；复用共享变换实现，避免第二套约定。
    from robot_mission.grasp_pose import matrix_to_quaternion_xyzw
    return [float(v) for v in matrix_to_quaternion_xyzw(rotation)]


def _append_motion(out: list[dict], segment: str, rows) -> None:
    for values in rows:
        q = [round(float(v), 6) for v in values]
        if out and "q_sdk_deg" in out[-1] and max(
                abs(a - b) for a, b in zip(out[-1]["q_sdk_deg"], q)) < 1e-9:
            continue
        out.append({"seg": segment, "q_sdk_deg": q})


def _wall_obstacles(box_center: np.ndarray, box_dimensions: np.ndarray,
                    box_rotation: np.ndarray) -> list[dict]:
    dx, dy, dz = (float(v) for v in box_dimensions)
    wall = BOX_WALL_M
    specs = (
        ([wall / 2, dy / 2, dz / 2], [-dx / 2 + wall / 2, 0, 0], "bin_wall_x_minus"),
        ([wall / 2, dy / 2, dz / 2], [ dx / 2 - wall / 2, 0, 0], "bin_wall_x_plus"),
        ([dx / 2, wall / 2, dz / 2], [0, -dy / 2 + wall / 2, 0], "bin_wall_y_minus"),
        ([dx / 2, wall / 2, dz / 2], [0,  dy / 2 - wall / 2, 0], "bin_wall_y_plus"),
    )
    orientation = _q_xyzw(box_rotation)
    rows = []
    for half, local, name in specs:
        rows.append({
            "source_obstacle_id": name,
            "center": (box_center + box_rotation @ np.asarray(local, float)).tolist(),
            "half": [float(v) for v in half],
            "orientation_xyzw": orientation,
            "minimum_robot_clearance_m": 0.0,
            "minimum_gripper_clearance_m": 0.0,
            "support_contact_tolerance_m": 0.0,
        })
    # 放置定义遵从本次临时假设“外形底面 + 半瓶高”；薄代理的顶面正好是底面。
    bottom_local = np.array([0.0, 0.0, -dz / 2.0])
    floor_center = box_center + box_rotation @ (
        bottom_local - np.array([0.0, 0.0, BOX_FLOOR_PROXY_M / 2.0]))
    rows.append({
        "source_obstacle_id": "bin_assumed_bottom_plane",
        "center": floor_center.tolist(),
        "half": [dx / 2, dy / 2, BOX_FLOOR_PROXY_M / 2.0],
        "orientation_xyzw": orientation,
        "minimum_robot_clearance_m": 0.0,
        "minimum_gripper_clearance_m": 0.0,
        "support_contact_tolerance_m": 0.0001,
    })
    return rows


def _derive_place_local(box_dimensions: np.ndarray, box_rotation: np.ndarray,
                        profile: dict) -> tuple[np.ndarray, dict]:
    """从瓶/夹爪 OBB 与箱壁求靠机器人侧落点，并用 PB OBB 求最小修正。"""
    proxy_half = np.asarray(profile["gripper_collision_box_m"], float) / 2.0
    proxy_extent_box = np.abs(box_rotation.T @ TASK_R_LINK_ANCHOR) @ proxy_half
    bottle_half = BOTTLE_DIMENSIONS_M / 2.0
    required_half_xy = np.maximum(bottle_half[:2], proxy_extent_box[:2])
    base_inset = (box_dimensions[:2] / 2.0 - BOX_WALL_M
                  - required_half_xy - BOTTLE_INTERIOR_CLEARANCE_M)
    if np.any(base_inset <= 0.0):
        raise ValueError("箱体内尺寸不足以容纳瓶子、夹爪和内壁余量")

    # 箱底略有倾斜；资格碰撞体把瓶子按世界竖直盒体检查。用箱局部 Z 上的
    # 投影再加数值净空，推导底面中心高度，避免把 +3mm 写成目标魔数。
    upright_bottle_half_extent_box_z = float(
        np.abs(box_rotation.T[2]) @ bottle_half)
    local_z = (-box_dimensions[2] / 2.0
               + upright_bottle_half_extent_box_z
               + SUPPORT_PLANE_CLEARANCE_M)
    base_local = np.array([-base_inset[0], +base_inset[1], local_z])

    proxy_delta_link = (
        np.asarray(profile["gripper_collision_center_link_mm"], float)
        - np.asarray(profile["grip_center_link_mm"], float)) / 1000.0
    proxy_orientation = _q_xyzw(TASK_R_LINK_ANCHOR)
    bodies = []
    proxy = -1
    try:
        for spec in _wall_obstacles(np.zeros(3), box_dimensions, box_rotation):
            if not spec["source_obstacle_id"].startswith("bin_wall_"):
                continue
            bodies.append(p.createMultiBody(
                0, p.createCollisionShape(p.GEOM_BOX, halfExtents=spec["half"]),
                basePosition=spec["center"], baseOrientation=spec["orientation_xyzw"]))
        proxy = p.createMultiBody(
            0, p.createCollisionShape(p.GEOM_BOX, halfExtents=proxy_half.tolist()))
        selected = None
        samples = []
        for millimetres in range(0, 51):
            local = base_local + np.array([millimetres / 1000.0, 0.0, 0.0])
            grip = box_rotation @ local
            proxy_center = grip + TASK_R_LINK_ANCHOR @ proxy_delta_link
            p.resetBasePositionAndOrientation(proxy, proxy_center.tolist(), proxy_orientation)
            p.performCollisionDetection()
            clearance = min(
                min(float(row[8]) for row in p.getClosestPoints(proxy, wall, 1.0))
                for wall in bodies)
            samples.append({"local_x_correction_mm": millimetres,
                            "minimum_proxy_wall_clearance_mm": clearance * 1000.0})
            if clearance >= GRIPPER_WALL_CLEARANCE_M:
                selected = (local, millimetres, clearance)
                break
        if selected is None:
            raise ValueError("50mm 箱内修正范围仍无法给夹爪 proxy 留出墙面净空")
        local, correction_mm, clearance = selected
        return local, {
            "base_local_offset_mm": (base_local * 1000.0).tolist(),
            "selected_local_x_correction_mm": correction_mm,
            "minimum_static_proxy_wall_clearance_mm": clearance * 1000.0,
            "required_static_proxy_wall_clearance_mm": GRIPPER_WALL_CLEARANCE_M * 1000.0,
            "upright_bottle_half_extent_along_box_z_mm": (
                upright_bottle_half_extent_box_z * 1000.0),
            "support_plane_clearance_mm": SUPPORT_PLANE_CLEARANCE_M * 1000.0,
            "proxy_half_extent_in_box_axes_mm": (proxy_extent_box * 1000.0).tolist(),
            "search_samples": samples,
        }
    finally:
        if proxy >= 0:
            p.removeBody(proxy)
        for body in bodies:
            p.removeBody(body)


def build(snapshot_path: Path, trajectory_out: Path, qualification_out: Path) -> dict:
    snapshot = json.loads(snapshot_path.read_text(encoding="utf-8"))
    if snapshot.get("schema_version") != SCHEMA:
        raise ValueError(f"expected {SCHEMA}")
    if snapshot.get("real_motion_authorized") is not False:
        raise ValueError("输入快照不得授权真机")
    profile = arm_profiles.arm_profile(ARM)
    if (snapshot["contract"]["arm"], snapshot["contract"]["arm_id"],
            snapshot["contract"]["gripper_id"]) != (
            ARM, profile["arm_id"], profile["gripper_id"]):
        raise ValueError("arm/gripper contract mismatch")

    safe_q = [float(v) for v in snapshot["safe_candidate"]["joints_sdk_deg"]]
    safe_sdk_m = np.asarray(
        snapshot["safe_candidate"]["pose"]["position_mm"], float) / 1000.0
    bottle_sdk_m = np.asarray(
        snapshot["bottle"]["processed_pose"]["position_mm"], float) / 1000.0
    box_sdk_m = np.asarray(
        snapshot["material_box"]["processed_pose"]["position_mm"], float) / 1000.0
    box_dimensions = np.asarray(
        snapshot["material_box"]["dimensions_mm"], float) / 1000.0
    box_rotation = quaternion_xyzw_to_matrix(
        snapshot["material_box"]["processed_pose"]["quaternion_xyzw"])

    robot = xf.load_xifeng(gui=False)
    try:
        safe_grip_pb, safe_R_link = gen.fk_grip(robot, safe_q)
        state = p.getLinkState(robot, profile["ee_link_id"], computeForwardKinematics=True)
        safe_link_pb = np.asarray(state[4], float)
        safe_tcp_pb = safe_link_pb + safe_R_link @ (
            np.asarray(cc.CONFIRMED_R_TCP_LINK11_MM, float) / 1000.0)
        # 当前工程 Model A 明确使用 SDK Worlds = PB EE_LINK_ORIGIN + t。
        # 这里只用这一对同步 Worlds/Joints 更新平移；它是仿真显示锚，不是
        # 标定，也不会回写 CONFIRMED_T。TCP/夹爪补偿仍在目标求解层单独处理。
        t_sdk_minus_pb = safe_sdk_m - safe_link_pb

        def sdk_to_pb(xyz_sdk_m) -> np.ndarray:
            return np.asarray(xyz_sdk_m, float) - t_sdk_minus_pb

        bottle_pick_pb = sdk_to_pb(bottle_sdk_m)
        box_center_pb = sdk_to_pb(box_sdk_m)
        # 不能把瓶放到箱中心后还要求左臂跨到不可达区。落点同时考虑瓶半径和
        # 稳定姿态下完整夹爪 proxy 的箱轴投影，再由几何推导，不写死 XYZ。
        proxy_half_m = np.asarray(profile["gripper_collision_box_m"], float) / 2.0
        proxy_half_extent_box_m = (
            np.abs(box_rotation.T @ TASK_R_LINK_ANCHOR) @ proxy_half_m)
        proxy_center_from_grip_box_m = box_rotation.T @ TASK_R_LINK_ANCHOR @ (
            (np.asarray(profile["gripper_collision_center_link_mm"], float)
             - np.asarray(profile["grip_center_link_mm"], float)) / 1000.0)
        bottle_half_xy_m = BOTTLE_DIMENSIONS_M[:2] / 2.0
        occupied_low_xy_m = np.minimum(
            -bottle_half_xy_m,
            proxy_center_from_grip_box_m[:2] - proxy_half_extent_box_m[:2])
        occupied_high_xy_m = np.maximum(
            bottle_half_xy_m,
            proxy_center_from_grip_box_m[:2] + proxy_half_extent_box_m[:2])
        inner_half_xy_m = box_dimensions[:2] / 2.0 - BOX_WALL_M
        bottle_radius_xy_m = BOTTLE_DIMENSIONS_M[:2] / 2.0
        if np.any(2.0 * (bottle_radius_xy_m + BOTTLE_INTERIOR_CLEARANCE_M)
                  > 2.0 * inner_half_xy_m):
            raise ValueError("箱体内尺寸不足以容纳瓶子、墙厚和内壁余量")
        # 靠机器人侧选 local X 较小、local Y 较大的一角；proxy 中心并不与
        # grip 重合，因此必须用非对称占用区间，而不是只比较半宽。
        inset_xy = np.array([
            max(-inner_half_xy_m[0] + BOTTLE_INTERIOR_CLEARANCE_M
                + bottle_radius_xy_m[0],
                -inner_half_xy_m[0] + GRIPPER_WALL_CLEARANCE_M
                - (proxy_center_from_grip_box_m[0] - proxy_half_extent_box_m[0])),
            min(+inner_half_xy_m[1] - BOTTLE_INTERIOR_CLEARANCE_M
                - bottle_radius_xy_m[1],
                +inner_half_xy_m[1] - GRIPPER_WALL_CLEARANCE_M
                - (proxy_center_from_grip_box_m[1] + proxy_half_extent_box_m[1])),
        ])
        place_local_m = np.array([
            -inset_xy[0],
            +inset_xy[1],
            -box_dimensions[2] / 2.0 + BOTTLE_DIMENSIONS_M[2] / 2.0,
        ])
        place_center_sdk = box_sdk_m + box_rotation @ place_local_m
        bottle_place_pb = sdk_to_pb(place_center_sdk)

        half_box = box_dimensions / 2.0
        box_rim_sdk_z = float(box_sdk_m[2] + np.abs(box_rotation[2]) @ half_box)
        carried_transfer_center_sdk_z = (
            box_rim_sdk_z + TRANSFER_CLEARANCE_M + BOTTLE_DIMENSIONS_M[2] / 2.0)
        proxy_delta_from_grip_link_m = (
            np.asarray(profile["gripper_collision_center_link_mm"], float)
            - np.asarray(profile["grip_center_link_mm"], float)) / 1000.0
        proxy_bottom_from_grip_m = float(
            (TASK_R_LINK_ANCHOR @ proxy_delta_from_grip_link_m)[2]
            - np.abs(TASK_R_LINK_ANCHOR[2]) @ proxy_half_m)
        bottle_top_sdk_z = float(bottle_sdk_m[2] + BOTTLE_DIMENSIONS_M[2] / 2.0)
        empty_approach_center_sdk_z = (
            bottle_top_sdk_z + float(profile["minimum_table_clearance_m"])
            - proxy_bottom_from_grip_m)
        transfer_center_sdk_z = max(
            carried_transfer_center_sdk_z, empty_approach_center_sdk_z)
        pick_high_sdk = bottle_sdk_m.copy()
        pick_high_sdk[2] = transfer_center_sdk_z
        place_high_sdk = place_center_sdk.copy()
        place_high_sdk[2] = transfer_center_sdk_z
        pick_high_pb = sdk_to_pb(pick_high_sdk)
        place_high_pb = sdk_to_pb(place_high_sdk)
        safe_high_pb = np.asarray(safe_grip_pb, float).copy()
        safe_high_pb[2] = float(pick_high_pb[2])

        limit_evidence = arm_profiles.controller_limit_profile(
            "left_preview102_20260827")
        planning_limits = limit_evidence["planning_limits_deg"]
        required_margin = float(profile["shared"]["minimum_joint_margin_deg"])
        bounds = gen._solver_bounds_from_limits(planning_limits, required_margin)
        sample_mm = 2.0
        maximum_anchor_step_deg = 8.0

        def chain(stage, start, end, seed,
                  R_start=TASK_R_LINK_ANCHOR, R_end=None):
            try:
                return gen.solve_cartesian_chain(
                    robot, np.asarray(start), np.asarray(end), R_start, seed, bounds,
                    sample_mm, maximum_anchor_step_deg,
                    position_tolerance_mm=0.8, rotation_tolerance_deg=0.5,
                    R_end=R_end)
            except RuntimeError as exc:
                raise ReplayCandidateBlocked(stage, exc, start, end) from exc

        safe_raise, safe_raise_report = chain(
            "SAFE_RAISE", safe_grip_pb, safe_high_pb, safe_q,
            R_start=safe_R_link)
        entry, entry_report = chain(
            "SAFE_HIGH_TO_PICK_HIGH", safe_high_pb, pick_high_pb, safe_raise[-1],
            R_start=safe_R_link, R_end=TASK_R_LINK_ANCHOR)
        descend, descend_report = chain(
            "PICK_DESCEND", pick_high_pb, bottle_pick_pb, entry[-1])
        lift, lift_report = chain(
            "PICK_LIFT", bottle_pick_pb, pick_high_pb, descend[-1])
        transfer, transfer_report = chain(
            "TRANSFER_ABOVE_BIN", pick_high_pb, place_high_pb, lift[-1])
        place_down, place_down_report = chain(
            "PLACE_DESCEND", place_high_pb, bottle_place_pb, transfer[-1])
        place_up, place_up_report = chain(
            "PLACE_ASCEND", bottle_place_pb, place_high_pb, place_down[-1])
        # 7 轴同一末端位姿可落到不同支路；空爪回程精确反向复用已解高位链，
        # 避免为了回 safe 硬接一条大关节跳变。
        exit_high = list(reversed(transfer))
        exit_entry = list(reversed(entry))
        exit_rows = list(reversed(safe_raise))
        safe_gap = max(abs(a - b) for a, b in zip(exit_rows[-1], safe_q))
        if safe_gap > 1e-6:
            raise RuntimeError(f"reversed return missed safe by {safe_gap:.6f}deg")

        waypoints: list[dict] = []
        _append_motion(waypoints, "SAFE_START", [safe_q])
        _append_motion(waypoints, "SAFE_RAISE", safe_raise)
        _append_motion(waypoints, "SAFE_HIGH_TO_PICK_HIGH", entry)
        _append_motion(waypoints, "PICK_DESCEND", descend)
        waypoints.append({"seg": "PICK", "gripper": "close"})
        _append_motion(waypoints, "PICK_LIFT", lift)
        _append_motion(waypoints, "TRANSFER_ABOVE_BIN", transfer)
        _append_motion(waypoints, "PLACE_DESCEND", place_down)
        waypoints.append({"seg": "PLACE", "gripper": "open"})
        _append_motion(waypoints, "PLACE_ASCEND", place_up)
        _append_motion(waypoints, "EMPTY_RETURN_TRANSFER", exit_high)
        _append_motion(waypoints, "PICK_HIGH_TO_SAFE_HIGH", exit_entry)
        _append_motion(waypoints, "SAFE_LOWER", exit_rows)

        limit_report = gen._limit_report(profile, waypoints, planning_limits)
        if limit_report["status"] != "PASS":
            raise RuntimeError(f"joint limit gate failed: {limit_report}")
        motion_rows = [row for row in waypoints if "q_sdk_deg" in row]
        maximum_exported_step = max(
            max(abs(a - b) for a, b in zip(left["q_sdk_deg"], right["q_sdk_deg"]))
            for left, right in zip(motion_rows, motion_rows[1:]))
        actual_pick_grip, _ = gen.fk_grip(robot, descend[-1])
        actual_place_grip, _ = gen.fk_grip(robot, place_down[-1])

        table_top_sdk_z = float(bottle_sdk_m[2] - BOTTLE_DIMENSIONS_M[2] / 2.0)
        table_top_pb_z = float(table_top_sdk_z - t_sdk_minus_pb[2])
        table_center_xy = (bottle_pick_pb[:2] + box_center_pb[:2]) / 2.0
        half_xy = np.abs(bottle_pick_pb[:2] - box_center_pb[:2]) / 2.0 + TABLE_XY_PADDING_M
        table = {
            "source_obstacle_id": "table_from_assumed_bottle_bottom",
            "center": [float(table_center_xy[0]), float(table_center_xy[1]),
                       table_top_pb_z - TABLE_THICKNESS_M / 2.0],
            "half": [float(half_xy[0]), float(half_xy[1]), TABLE_THICKNESS_M / 2.0],
            "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
            "support_contact_tolerance_m": 0.0001,
            "minimum_robot_clearance_m": 0.0,
            "minimum_gripper_clearance_m": 0.0,
        }
        obstacles = [table] + _wall_obstacles(box_center_pb, box_dimensions, box_rotation)
        disabled = sorted([list(pair) for pair in gen.parent_planner.compute_disabled_pairs(robot)])

        assumptions = [
            "A1/B1 processed visual feature is temporarily treated as the bottle grasp center",
            "current bottle uses the prior 70x70x200mm conservative envelope",
            "A3/B3 processed material-box pose is temporarily treated as outer geometry center",
            "box dimensions 310x390x130mm are treated as local X/Y/Z outer dimensions",
            "the supplied box quaternion defines the local X/Y/Z axes in this demo",
            "placed bottle center uses a proxy-aware derived inset and box bottom plus half bottle height",
            "one offline-searched task orientation is held from pick descent through place ascent",
            "safe-to-task posture change is smooth at high clearance and reversed exactly on return",
            "safe Worlds/Joints establish a legacy Model-A EE-link-origin display anchor only; it conflicts with the snapshot TCP label and is not calibration",
            "transfer bottle bottom clears the highest assumed box rim by 20mm",
            "box wall thickness is a 10mm simulation assumption and floor is a thin support proxy",
        ]
        anchor_orientation_delta = cc.rotation_angle_deg(
            cc.uvw_to_matrix(cc.CONFIRMED_UVW_CANDIDATE,
                             snapshot["safe_candidate"]["pose"]["sdk_world_uvw_deg"])
            @ (safe_R_link @ gen._rotz(90.0)).T)
        meta = {
            "arm": ARM,
            "arm_id": profile["arm_id"],
            "gripper_id": profile["gripper_id"],
            "j6_flipped": True,
            "task": "tabletop_vision_ab_20260902_assumption_replay",
            **CANDIDATE_POLICY,
            "source_snapshot": str(snapshot_path.resolve().relative_to(WORK)),
            "source_snapshot_sha256": _sha256(snapshot_path),
            "generator": str(Path(__file__).resolve().relative_to(WORK)),
            "generator_sha256": _sha256(Path(__file__)),
            "assumptions": assumptions,
            "single_pose_anchor": {
                "status": ANCHOR_CONTRACT_STATUS,
                "snapshot_endpoint_reference": snapshot["contract"]["endpoint_reference"],
                "safe_sdk_world_m": safe_sdk_m.tolist(),
                "safe_ee_link_origin_pb_m": safe_link_pb.tolist(),
                "safe_tcp_pb_m": safe_tcp_pb.tolist(),
                "safe_grip_pb_m": safe_grip_pb.tolist(),
                "t_sdk_minus_pb_m": t_sdk_minus_pb.tolist(),
                "safe_worlds_vs_fk_orientation_delta_deg": anchor_orientation_delta,
                "axis_mapping": "IDENTITY_ASSUMPTION_MODEL_A_EE_LINK_ORIGIN",
            },
            "derived_geometry_pb_m": {
                "bottle_pick_center": bottle_pick_pb.tolist(),
                "bottle_place_center": bottle_place_pb.tolist(),
                "pick_high_grip": pick_high_pb.tolist(),
                "place_high_grip": place_high_pb.tolist(),
                "safe_high_grip": safe_high_pb.tolist(),
                "box_center": box_center_pb.tolist(),
                "table_top_z": table_top_pb_z,
            },
            "transfer_clearance": {
                "assumed_box_rim_sdk_z_mm": box_rim_sdk_z * 1000.0,
                "carried_bottle_bottom_clearance_mm": TRANSFER_CLEARANCE_M * 1000.0,
                "carried_only_transfer_center_sdk_z_mm": (
                    carried_transfer_center_sdk_z * 1000.0),
                "empty_gripper_proxy_bottom_from_grip_mm": (
                    proxy_bottom_from_grip_m * 1000.0),
                "empty_approach_center_sdk_z_mm": (
                    empty_approach_center_sdk_z * 1000.0),
                "transfer_bottle_center_sdk_z_mm": transfer_center_sdk_z * 1000.0,
            },
            "placement_inset_policy": {
                "selection": "LOCAL_X_MINUS_LOCAL_Y_PLUS_NEAR_ROBOT_REACHABLE_CORNER",
                "box_wall_assumption_mm": BOX_WALL_M * 1000.0,
                "bottle_half_xy_mm": (bottle_half_xy_m * 1000.0).tolist(),
                "gripper_proxy_half_extent_in_box_axes_mm": (
                    proxy_half_extent_box_m * 1000.0).tolist(),
                "gripper_proxy_center_from_grip_box_axes_mm": (
                    proxy_center_from_grip_box_m * 1000.0).tolist(),
                "combined_occupied_low_xy_from_grip_mm": (
                    occupied_low_xy_m * 1000.0).tolist(),
                "combined_occupied_high_xy_from_grip_mm": (
                    occupied_high_xy_m * 1000.0).tolist(),
                "minimum_bottle_inner_wall_clearance_mm": (
                    BOTTLE_INTERIOR_CLEARANCE_M * 1000.0),
                "minimum_gripper_wall_clearance_mm": (
                    GRIPPER_WALL_CLEARANCE_M * 1000.0),
                "derived_local_offset_mm": (place_local_m * 1000.0).tolist(),
                "derived_sdk_world_center_mm": (place_center_sdk * 1000.0).tolist(),
            },
            "stable_task_orientation": {
                "status": "SIMULATION_ANCHOR_UNCONFIRMED_FOR_REAL_ROBOT",
                "source": "bounded offline pose search seed 42 candidate 10",
                "R_link_pb": TASK_R_LINK_ANCHOR.tolist(),
                "tool_axis_pb": gen._tool_axis_from_link_rotation(
                    TASK_R_LINK_ANCHOR).tolist(),
                "safe_to_task_transition": "SLERP_AT_HIGH_CLEARANCE",
            },
            "tcp_grip_compensation": {
                "tcp_link11_mm": [float(v) for v in cc.CONFIRMED_R_TCP_LINK11_MM],
                "grip_center_link11_mm": profile["grip_center_link_mm"],
                "grip_minus_tcp_link11_mm": (
                    np.asarray(profile["grip_center_link_mm"], float)
                    - np.asarray(cc.CONFIRMED_R_TCP_LINK11_MM, float)).tolist(),
            },
            "controller_limit_evidence": limit_evidence,
            "local_limit_report": limit_report,
            "pulse": {
                "step_deg": profile["shared"]["pulse_step_deg"],
                "period_ms": profile["shared"]["pulse_period_ms"],
            },
            "maximum_exported_adjacent_step_deg": maximum_exported_step,
            "cartesian_chain_reports": {
                "safe_raise": safe_raise_report,
                "safe_high_to_pick_high": entry_report,
                "pick_descend": descend_report,
                "pick_lift": lift_report,
                "transfer_above_bin": transfer_report,
                "place_descend": place_down_report,
                "place_ascend": place_up_report,
                "empty_return_transfer": "EXACT_REVERSE_OF_TRANSFER_ABOVE_BIN",
                "pick_high_to_safe_high": "EXACT_REVERSE_OF_SAFE_HIGH_TO_PICK_HIGH",
                "safe_lower": "EXACT_REVERSE_OF_SAFE_RAISE",
            },
            "gui_review": {
                "required": True,
                "status": "PENDING_HUMAN_REVIEW",
                "dense_step_deg": profile["shared"]["pulse_step_deg"],
                "demo_script": "pick_place_coord/demo_bottle_trajectory.py",
            },
            "unresolved_real_motion_gates": [
                "feature_to_grasp transform is only assumed",
                "current object dimensions are only assumed",
                "material-box datum and inner geometry are only assumed",
                "single-point display anchor is not PB-SDK calibration",
                "live armTryWorlds path is absent",
                "live controller state and limits were not read",
                "human GUI review has not yet passed",
            ],
        }
        payload = {"meta": meta, "waypoints": waypoints}
        trajectory_out.parent.mkdir(parents=True, exist_ok=True)
        trajectory_out.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

        qtask = {
            "schema_version": "tabletop_assumption_qualification_task.v1",
            "status": "SIMULATION_ONLY_ASSUMPTIONS_UNCONFIRMED",
            **CANDIDATE_POLICY,
            "coordinate_contract_status": ANCHOR_CONTRACT_STATUS,
            "source_snapshot_sha256": _sha256(snapshot_path),
            "assumptions": assumptions,
            "self_collision_disabled_link_pairs": disabled,
            "pairs": [{
                "pick": bottle_pick_pb.tolist(),
                "place": bottle_place_pb.tolist(),
                "actual_pick_grip": np.asarray(actual_pick_grip).tolist(),
                "actual_place_grip": np.asarray(actual_place_grip).tolist(),
                "object_pick_center": bottle_pick_pb.tolist(),
                "object_place_center": bottle_place_pb.tolist(),
                "dimensions_m": BOTTLE_DIMENSIONS_M.tolist(),
                "carried_center_offset_grip_m": (
                    bottle_pick_pb - np.asarray(actual_pick_grip)).tolist(),
                "grasp_capture": {
                    "axis_range_m": profile["grasp_capture_axis_range_m"],
                    "lateral_tolerance_m": profile["grasp_capture_lateral_tolerance_m"],
                    "reference": "SIMULATION_ASSUMPTION_FEATURE_EQUALS_GRASP_CENTER",
                    "object_grasp_point_world_m": bottle_pick_pb.tolist(),
                },
                "object_attachment_mode": "RIGID_FROM_CLOSE",
            }],
            "obstacles": obstacles,
        }
        qualification_out.parent.mkdir(parents=True, exist_ok=True)
        qualification_out.write_text(
            json.dumps(qtask, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return {
            "trajectory": str(trajectory_out),
            "trajectory_sha256": _sha256(trajectory_out),
            "qualification_task": str(qualification_out),
            "qualification_task_sha256": _sha256(qualification_out),
            "motion_points": len(motion_rows),
            "gripper_events": 2,
            "maximum_exported_adjacent_step_deg": maximum_exported_step,
            "limit_status": limit_report["status"],
            "real_motion_authorized": False,
        }
    finally:
        if p.isConnected():
            p.disconnect()


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, required=True)
    parser.add_argument("--trajectory-out", type=Path, required=True)
    parser.add_argument("--qualification-out", type=Path, required=True)
    parser.add_argument("--diagnostic-out", type=Path)
    parser.add_argument("--accept-simulation-assumptions", action="store_true")
    args = parser.parse_args(argv)
    if not args.accept_simulation_assumptions:
        parser.error("必须显式给 --accept-simulation-assumptions；输出仅供仿真")
    try:
        result = build(args.snapshot.resolve(), args.trajectory_out.resolve(),
                       args.qualification_out.resolve())
        if args.diagnostic_out:
            generation_report = {
                "schema_version": "tabletop_replay_generation_report.v1",
                "verdict": "GENERATED_SIMULATION_CANDIDATE",
                **CANDIDATE_POLICY,
                "coordinate_contract_status": ANCHOR_CONTRACT_STATUS,
                "outputs_written": True,
                **result,
            }
            args.diagnostic_out.parent.mkdir(parents=True, exist_ok=True)
            args.diagnostic_out.write_text(
                json.dumps(generation_report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    except ReplayCandidateBlocked as exc:
        report = {
            "schema_version": "tabletop_replay_generation_report.v1",
            "verdict": "BLOCKED",
            "reason": "IK_PATH_UNREACHABLE_OR_DISCONTINUOUS",
            "stage": exc.stage,
            "cause": exc.cause,
            "attempted_start_pb_m": exc.start_pb_m,
            "attempted_end_pb_m": exc.end_pb_m,
            "source_snapshot": str(args.snapshot.resolve()),
            "source_snapshot_sha256": _sha256(args.snapshot.resolve()),
            **CANDIDATE_POLICY,
            "coordinate_contract_status": ANCHOR_CONTRACT_STATUS,
            "outputs_written": False,
            "required_next_evidence": [
                "valid current PB-SDK frame calibration",
                "or live armTryWorlds q_sdk_deg for PICK_HIGH, PICK, PLACE_HIGH, PLACE",
            ],
            "message": str(exc),
        }
        if args.diagnostic_out:
            args.diagnostic_out.parent.mkdir(parents=True, exist_ok=True)
            args.diagnostic_out.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
