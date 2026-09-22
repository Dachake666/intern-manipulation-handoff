#!/usr/bin/env python3
"""当前 ``tabletop_plan.v1`` 的 PyBullet 动态诊断回放。

本脚本不把 MoveWorlds 点位冒充为已解关节轨迹：它用用户同时给出的
safe Worlds/Joints 做单点显示锚，按真机执行器相同的 2 mm 直线密化逐点求
6D IK。完整路径未通过时，默认只返回 BLOCKED；显式给出
``--show-blocked`` 时才在 GUI 里播放已通过的前缀，到首个失败点停住并标红。

这是 SIMULATION ASSUMPTION 诊断，不生成 GUI PASS，不授权 Debian 真机运动。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
ROBOT_SIDE = WORK / "frame_calibration" / "robot_side"
sys.path[:0] = [str(HERE), str(WORK), str(ROBOT_SIDE)]

import arm_profiles  # noqa: E402
import gen_bottle_servo_candidate as ik_solver  # noqa: E402
import xifeng_pb as xf  # noqa: E402
from execute_tabletop_pick_place_worlds import (  # noqa: E402
    MAX_JOINT_STEP_DEG,
    SAMPLE_MM,
    densify_line,
    load_plan,
)
from frame_calibration.analysis import calib_common as cc  # noqa: E402
from robot_mission.contracts import load_and_validate, sha256_file  # noqa: E402
from robot_mission.grasp_pose import (  # noqa: E402
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)


DEFAULT_PLAN = (
    HERE / "trajectories" / "candidates" / "tabletop_pick_place_worlds.json")
DEFAULT_SNAPSHOT = HERE / "tasks" / "task_tabletop_vision_ab_20260902.json"
LIMIT_MARGIN_DEG = 5.0
IK_POSITION_TOLERANCE_MM = 0.8
IK_ROTATION_TOLERANCE_DEG = 0.5
BOX_WALL_M = 0.010
TABLE_THICKNESS_M = 0.050


def _sha256_bytes(value: dict) -> str:
    raw = json.dumps(value, ensure_ascii=False, sort_keys=True,
                     separators=(",", ":"), allow_nan=False).encode("utf-8")
    return hashlib.sha256(raw).hexdigest()


def _set_arm(robot: int, profile: dict, q_sdk_deg) -> None:
    q_urdf_deg = arm_profiles.sdk_to_urdf_deg(profile, list(q_sdk_deg))
    for joint, value in zip(profile["joint_ids"], q_urdf_deg):
        p.resetJointState(robot, joint, math.radians(float(value)))


def _link_pose(robot: int, profile: dict) -> tuple[np.ndarray, np.ndarray]:
    state = p.getLinkState(robot, profile["ee_link_id"],
                           computeForwardKinematics=True)
    rotation = np.asarray(p.getMatrixFromQuaternion(state[5]), float).reshape(3, 3)
    return np.asarray(state[4], float), rotation


def _self_pairs(robot: int) -> set[tuple[int, int]]:
    return {
        tuple(sorted((int(row[3]), int(row[4]))))
        for row in p.getClosestPoints(robot, robot, 0.0)
        if int(row[3]) != int(row[4])
    }


def _pose6(stage: dict) -> list[float]:
    pose = stage["pose"]
    return [float(v) for v in pose["position_mm"] + pose["sdk_world_uvw_deg"]]


def _scene_geometry(snapshot: dict, shift_pb_minus_sdk: np.ndarray,
                    object_dimensions_mm) -> dict:
    bottle = snapshot["bottle"]["processed_pose"]
    box = snapshot["material_box"]["processed_pose"]
    box_dimensions = np.asarray(snapshot["material_box"]["dimensions_mm"], float) / 1000.0
    bottle_dimensions = np.asarray(object_dimensions_mm, float) / 1000.0
    if bottle_dimensions.shape != (3,) or np.any(bottle_dimensions <= 0):
        raise ValueError("plan input.dimensions_mm 必须是 3 个正数")
    bottle_center = np.asarray(bottle["position_mm"], float) / 1000.0 + shift_pb_minus_sdk
    box_center = np.asarray(box["position_mm"], float) / 1000.0 + shift_pb_minus_sdk
    box_rotation = quaternion_xyzw_to_matrix(box["quaternion_xyzw"])
    table_top = (float(snapshot["material_box"].get(
        "assumed_table_surface_z_mm", bottle["position_mm"][2] -
        float(object_dimensions_mm[2]) / 2.0)) / 1000.0
        + float(shift_pb_minus_sdk[2]))
    xy = np.vstack((bottle_center[:2], box_center[:2]))
    table_center_xy = np.mean(xy, axis=0)
    table_half_xy = np.ptp(xy, axis=0) / 2.0 + np.array([0.28, 0.28])
    return {
        "bottle_center": bottle_center,
        "bottle_dimensions": bottle_dimensions,
        "box_center": box_center,
        "box_rotation": box_rotation,
        "box_quaternion": [float(v) for v in box["quaternion_xyzw"]],
        "box_dimensions": box_dimensions,
        "table_top": table_top,
        "table_center": np.array([table_center_xy[0], table_center_xy[1],
                                  table_top - TABLE_THICKNESS_M / 2.0]),
        "table_half": np.array([table_half_xy[0], table_half_xy[1],
                                TABLE_THICKNESS_M / 2.0]),
    }


def _create_scene(scene: dict, profile: dict, *, collisions: bool) -> dict:
    bodies: list[int] = []

    def box_body(half, center, rgba, orientation=(0, 0, 0, 1)) -> int:
        collision = (p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
                     if collisions else -1)
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=rgba)
        body = p.createMultiBody(0, collision, visual, basePosition=center,
                                 baseOrientation=orientation)
        bodies.append(body)
        return body

    table = box_body(scene["table_half"].tolist(), scene["table_center"].tolist(),
                     [0.70, 0.57, 0.40, 0.72])
    dx, dy, dz = (float(v) for v in scene["box_dimensions"])
    wall_specs = (
        ([BOX_WALL_M / 2, dy / 2, dz / 2], [-dx / 2 + BOX_WALL_M / 2, 0, 0]),
        ([BOX_WALL_M / 2, dy / 2, dz / 2], [dx / 2 - BOX_WALL_M / 2, 0, 0]),
        ([dx / 2, BOX_WALL_M / 2, dz / 2], [0, -dy / 2 + BOX_WALL_M / 2, 0]),
        ([dx / 2, BOX_WALL_M / 2, dz / 2], [0, dy / 2 - BOX_WALL_M / 2, 0]),
        ([dx / 2, dy / 2, .004], [0, 0, -dz / 2 + .004]),
    )
    box_bodies = []
    for half, local in wall_specs:
        center = scene["box_center"] + scene["box_rotation"] @ np.asarray(local, float)
        box_bodies.append(box_body(half, center.tolist(), [0.95, 0.48, 0.06, 0.60],
                                    scene["box_quaternion"]))

    radius = max(float(scene["bottle_dimensions"][0]),
                 float(scene["bottle_dimensions"][1])) / 2.0
    height = float(scene["bottle_dimensions"][2])
    bottle_collision = (p.createCollisionShape(
        p.GEOM_CYLINDER, radius=radius, height=height) if collisions else -1)
    bottle_visual = p.createVisualShape(
        p.GEOM_CYLINDER, radius=radius, length=height,
        rgbaColor=[0.12, 0.68, 0.92, 0.88])
    bottle = p.createMultiBody(0, bottle_collision, bottle_visual,
                               basePosition=scene["bottle_center"].tolist())
    bodies.append(bottle)

    proxy_half = np.asarray(profile["gripper_collision_box_m"], float) / 2.0
    proxy_collision = (p.createCollisionShape(p.GEOM_BOX, halfExtents=proxy_half.tolist())
                       if collisions else -1)
    proxy_visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=proxy_half.tolist(),
        rgbaColor=[1.0, 0.10, 0.02, 0.25])
    proxy = p.createMultiBody(0, proxy_collision, proxy_visual)
    bodies.append(proxy)
    return {"all": bodies, "table": table, "box": box_bodies,
            "environment": [table] + box_bodies, "bottle": bottle,
            "proxy": proxy}


def _update_proxy(robot: int, profile: dict, proxy: int) -> tuple[np.ndarray, np.ndarray]:
    link, rotation = _link_pose(robot, profile)
    center = link + rotation @ (
        np.asarray(profile["gripper_collision_center_link_mm"], float) / 1000.0)
    quaternion = matrix_to_quaternion_xyzw(rotation)
    p.resetBasePositionAndOrientation(proxy, center.tolist(), quaternion.tolist())
    return center, rotation


def build_diagnostic(robot: int, plan: dict, snapshot: dict) -> tuple[dict, dict]:
    profile = arm_profiles.arm_profile("left")
    if (plan["contract"]["arm"], plan["contract"]["arm_id"],
            plan["contract"]["gripper_id"]) != ("left", 1, 2):
        raise ValueError("本诊断入口只接受左臂 arm_id=1 / gripper_id=2")
    safe_q = [float(v) for v in snapshot["safe_candidate"]["joints_sdk_deg"]]
    safe_sdk = np.asarray(plan["resolved"]["safe_endpoint"]["position_mm"], float) / 1000.0
    snapshot_safe = np.asarray(
        snapshot["safe_candidate"]["pose"]["position_mm"], float) / 1000.0
    if np.linalg.norm(safe_sdk - snapshot_safe) > 0.001:
        raise ValueError("plan safe endpoint 与 snapshot safe joints 不是同一个锚点")

    _set_arm(robot, profile, safe_q)
    safe_link, safe_rotation = _link_pose(robot, profile)
    tcp_link = np.asarray(cc.CONFIRMED_R_TCP_LINK11_MM, float) / 1000.0
    grip_link = np.asarray(profile["grip_center_link_mm"], float) / 1000.0
    safe_tcp = safe_link + safe_rotation @ tcp_link
    shift = safe_tcp - safe_sdk
    scene = _scene_geometry(snapshot, shift, plan["input"]["dimensions_mm"])
    scene_bodies = _create_scene(scene, profile, collisions=True)
    _update_proxy(robot, profile, scene_bodies["proxy"])
    p.performCollisionDetection()
    baseline_self = _self_pairs(robot)

    limits = profile["controller_limits_deg"]
    bounds = ik_solver._solver_bounds_from_limits(limits, LIMIT_MARGIN_DEG)
    previous_pose = [float(v) for v in (
        plan["resolved"]["safe_endpoint"]["position_mm"] +
        plan["resolved"]["safe_endpoint"]["sdk_world_uvw_deg"])]
    previous_q = list(safe_q)
    timeline: list[dict] = [{"kind": "pose", "q_sdk_deg": safe_q,
                            "stage": "START_ASSERT_SAFE"}]
    accepted_frames: list[dict] = []
    segment_reports = []
    first_failure = None
    min_limit_margin = math.inf
    new_self: set[tuple[int, int]] = set()
    environment_collision = None

    for stage in plan["stages"]:
        kind = stage["kind"]
        if kind == "GRIPPER":
            timeline.append({"kind": "gripper", "action": stage["action"],
                             "stage": stage["name"]})
            continue
        if kind != "MOVE_WORLDS":
            continue
        target_pose = _pose6(stage)
        dense = densify_line(previous_pose, target_pose)
        report = {"stage": stage["name"], "dense_points": len(dense),
                  "accepted_points": 0, "status": "PENDING"}
        for sample_index, pose in enumerate(dense, 1):
            target_tcp = safe_tcp + (np.asarray(pose[:3], float) / 1000.0 - safe_sdk)
            target_grip = target_tcp + safe_rotation @ (grip_link - tcp_link)
            q, residual = ik_solver.solve_full_pose(
                robot, target_grip, safe_rotation, previous_q, bounds,
                iterations=500,
                position_tolerance_mm=IK_POSITION_TOLERANCE_MM,
                rotation_tolerance_deg=IK_ROTATION_TOLERANCE_DEG)
            if q is None:
                first_failure = {
                    "reason": "IK_UNREACHABLE_OR_SINGULAR", "stage": stage["name"],
                    "sample_index": sample_index, "segment_samples": len(dense),
                    "sdk_world_pose": pose, "target_grip_pb_m": target_grip.tolist(),
                    "ik_residual": residual,
                }
                break
            step = max(abs(float(a) - float(b)) for a, b in zip(q, previous_q))
            if step > MAX_JOINT_STEP_DEG:
                first_failure = {
                    "reason": "IK_BRANCH_JUMP", "stage": stage["name"],
                    "sample_index": sample_index, "segment_samples": len(dense),
                    "sdk_world_pose": pose, "target_grip_pb_m": target_grip.tolist(),
                    "max_adjacent_joint_step_deg": step,
                    "allowed_max_joint_step_deg": MAX_JOINT_STEP_DEG,
                    "ik_residual": residual,
                }
                break
            margins = [min(float(value) - (float(pair[0]) + LIMIT_MARGIN_DEG),
                           (float(pair[1]) - LIMIT_MARGIN_DEG) - float(value))
                       for value, pair in zip(q, limits)]
            if min(margins) < 0:
                first_failure = {
                    "reason": "CONTROLLER_LIMIT_MARGIN", "stage": stage["name"],
                    "sample_index": sample_index, "segment_samples": len(dense),
                    "sdk_world_pose": pose, "minimum_margin_deg": min(margins),
                }
                break
            min_limit_margin = min(min_limit_margin, min(margins))
            _set_arm(robot, profile, q)
            _update_proxy(robot, profile, scene_bodies["proxy"])
            p.performCollisionDetection()
            unexpected_self = _self_pairs(robot) - baseline_self
            if unexpected_self:
                new_self |= unexpected_self
                first_failure = {
                    "reason": "NEW_SELF_COLLISION", "stage": stage["name"],
                    "sample_index": sample_index, "segment_samples": len(dense),
                    "pairs": sorted([list(pair) for pair in unexpected_self]),
                }
                break
            hit = next((body for body in scene_bodies["environment"]
                        if p.getClosestPoints(robot, body, 0.0) or
                        p.getClosestPoints(scene_bodies["proxy"], body, 0.0)), None)
            if hit is not None:
                environment_collision = hit
                first_failure = {
                    "reason": "ROBOT_OR_GRIPPER_ENVIRONMENT_COLLISION",
                    "stage": stage["name"], "sample_index": sample_index,
                    "segment_samples": len(dense), "body_id": int(hit),
                }
                break
            frame = {"kind": "pose", "q_sdk_deg": [float(v) for v in q],
                     "stage": stage["name"], "sample_index": sample_index,
                     "sdk_world_pose": pose, "target_grip_pb_m": target_grip.tolist()}
            accepted_frames.append(frame)
            timeline.append(frame)
            previous_q = [float(v) for v in q]
            report["accepted_points"] += 1
        if first_failure:
            report["status"] = "BLOCKED"
            segment_reports.append(report)
            break
        report["status"] = "PASS"
        segment_reports.append(report)
        previous_pose = target_pose

    moves = [stage for stage in plan["stages"]
             if stage["kind"] == "MOVE_WORLDS"]
    dense_start = [float(v) for v in (
        plan["resolved"]["safe_endpoint"]["position_mm"] +
        plan["resolved"]["safe_endpoint"]["sdk_world_uvw_deg"])]
    expected_dense = 0
    for stage in moves:
        dense_target = _pose6(stage)
        expected_dense += len(densify_line(dense_start, dense_target))
        dense_start = dense_target
    status = "OFFLINE_DIAGNOSTIC_PASS" if first_failure is None else "BLOCKED"
    report = {
        "schema_version": "tabletop_move_worlds_pb_diagnostic.v1",
        "status": status,
        "qualification": "SIMULATION_ASSUMPTION_ONLY",
        "real_motion_authorized": False,
        "debian_execution_allowed": False,
        "plan_sha256": _sha256_bytes(plan),
        "snapshot_sha256": _sha256_bytes(snapshot),
        "executor_dense_sample_mm": SAMPLE_MM,
        "expected_dense_points": expected_dense,
        "accepted_dense_points": len(accepted_frames),
        "segment_reports": segment_reports,
        "first_failure": first_failure,
        "minimum_controller_margin_deg_in_accepted_prefix": (
            min_limit_margin if math.isfinite(min_limit_margin) else None),
        "new_self_collision_pairs": sorted([list(pair) for pair in new_self]),
        "frame_mapping": {
            "mode": "SINGLE_SAFE_WORLDS_JOINTS_TCP_DISPLAY_ANCHOR_ONLY",
            "shift_pb_minus_sdk_m": shift.tolist(),
            "safe_tcp_pb_m": safe_tcp.tolist(),
            "safe_link_pb_m": safe_link.tolist(),
            "not_a_calibration": True,
        },
        "notes": [
            "Mac PyBullet diagnostic only; Debian armTryWorlds is authoritative",
            "a blocked prefix may be shown only with --show-blocked",
            "no GUI PASS record is produced by this script",
        ],
    }
    return report, {"profile": profile, "safe_q": safe_q, "safe_tcp": safe_tcp,
                    "safe_rotation": safe_rotation, "scene": scene,
                    "scene_bodies": scene_bodies, "timeline": timeline,
                    "accepted_frames": accepted_frames}


def _draw_path(plan: dict, context: dict, failure: dict | None) -> None:
    safe_sdk = np.asarray(plan["resolved"]["safe_endpoint"]["position_mm"], float) / 1000.0
    safe_tcp = context["safe_tcp"]
    rotation = context["safe_rotation"]
    profile = context["profile"]
    tcp = np.asarray(cc.CONFIRMED_R_TCP_LINK11_MM, float) / 1000.0
    grip = np.asarray(profile["grip_center_link_mm"], float) / 1000.0

    def grip_point(pose: list[float]) -> np.ndarray:
        endpoint = safe_tcp + (np.asarray(pose[:3], float) / 1000.0 - safe_sdk)
        return endpoint + rotation @ (grip - tcp)

    previous = [float(v) for v in (
        plan["resolved"]["safe_endpoint"]["position_mm"] +
        plan["resolved"]["safe_endpoint"]["sdk_world_uvw_deg"])]
    for stage in plan["stages"]:
        if stage["kind"] != "MOVE_WORLDS":
            continue
        target = _pose6(stage)
        p.addUserDebugLine(grip_point(previous).tolist(), grip_point(target).tolist(),
                           [0.88, 0.08, 0.04], 2)
        previous = target
    frames = context["accepted_frames"]
    safe_pose = [float(v) for v in (
        plan["resolved"]["safe_endpoint"]["position_mm"] +
        plan["resolved"]["safe_endpoint"]["sdk_world_uvw_deg"])]
    points = [grip_point(safe_pose)]
    points.extend(np.asarray(frame["target_grip_pb_m"], float) for frame in frames)
    for left, right in zip(points, points[1:]):
        p.addUserDebugLine(left.tolist(), right.tolist(), [0.08, 0.48, 1.0], 5)
    if failure:
        marker = p.createVisualShape(p.GEOM_SPHERE, radius=.018,
                                     rgbaColor=[1.0, 0.0, 0.0, 1.0])
        p.createMultiBody(0, -1, marker, basePosition=failure["target_grip_pb_m"])


def play_gui(robot: int, plan: dict, report: dict, context: dict,
             speed_factor: float, stay_seconds: float) -> None:
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
    target = ((context["scene"]["bottle_center"] +
               context["scene"]["box_center"]) / 2.0)
    p.resetDebugVisualizerCamera(1.15, 150, -24, target.tolist())
    _draw_path(plan, context, report["first_failure"])
    message_position = target + np.array([0.0, 0.0, 0.38])
    p.addUserDebugText(
        "BLOCKED - SIMULATION ONLY\nred=planned  blue=verified prefix",
        message_position.tolist(), [0.92, 0.02, 0.02], 1.35)
    _set_arm(robot, context["profile"], context["safe_q"])
    previous_q = context["safe_q"]
    last_stage = None
    status_id = -1
    for index, frame in enumerate(context["accepted_frames"], 1):
        if not p.isConnected():
            return
        q = frame["q_sdk_deg"]
        for blend in range(1, 4):
            shown = [a + (b - a) * blend / 3.0 for a, b in zip(previous_q, q)]
            _set_arm(robot, context["profile"], shown)
            _update_proxy(robot, context["profile"], context["scene_bodies"]["proxy"])
            time.sleep(0.018 / speed_factor)
        previous_q = q
        if frame["stage"] != last_stage:
            status_id = p.addUserDebugText(
                f"{frame['stage']}  {index}/{report['expected_dense_points']}",
                (target + np.array([0, 0, .31])).tolist(), [0.02, 0.08, 0.15], 1.1,
                replaceItemUniqueId=status_id)
            last_stage = frame["stage"]
    failure = report["first_failure"]
    if failure and p.isConnected():
        p.addUserDebugText(
            f"STOP: {failure['reason']}  {failure['stage']} "
            f"{failure['sample_index']}/{failure['segment_samples']}",
            (target + np.array([0, 0, .25])).tolist(), [1.0, 0.0, 0.0], 1.15)
    deadline = None if stay_seconds == 0 else time.monotonic() + stay_seconds
    while p.isConnected() and (deadline is None or time.monotonic() < deadline):
        time.sleep(.05)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", default=str(DEFAULT_PLAN))
    parser.add_argument("--snapshot", default=str(DEFAULT_SNAPSHOT))
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--show-blocked", action="store_true")
    parser.add_argument("--speed-factor", type=float, default=1.0)
    parser.add_argument("--stay-seconds", type=float, default=15.0)
    parser.add_argument("--report-out")
    args = parser.parse_args(argv)
    if args.speed_factor <= 0 or args.stay_seconds < 0:
        parser.error("--speed-factor 必须 >0，--stay-seconds 必须 >=0")
    plan, raw_plan_sha = load_plan(args.plan)
    snapshot = load_and_validate(args.snapshot, "tabletop_scene_snapshot.v1")
    robot = xf.load_xifeng(gui=args.gui)
    try:
        report, context = build_diagnostic(robot, plan, snapshot)
        report["plan_raw_file_sha256"] = raw_plan_sha
        report["snapshot_raw_file_sha256"] = sha256_file(args.snapshot)
        report["demo_script_sha256"] = sha256_file(__file__)
        if args.report_out:
            path = Path(args.report_out)
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        blocked = report["status"] == "BLOCKED"
        if args.gui:
            if blocked and not args.show_blocked:
                print("GUI 未播放：路径 BLOCKED；如需查看已验证前缀，显式给 --show-blocked。")
            else:
                play_gui(robot, plan, report, context, args.speed_factor,
                         args.stay_seconds)
        return 2 if blocked else 0
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
