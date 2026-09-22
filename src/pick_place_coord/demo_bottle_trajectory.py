#!/usr/bin/env python3
"""在 PyBullet 中按 Servo 等效密集帧回放瓶子抓放候选，并生成 GUI 人工审查记录。"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WORK))

import arm_profiles
import xifeng_pb as xf
from robot_mission.contracts import sha256_file
from robot_mission.grasp_pose import normalize_quaternion_xyzw
from robot_mission.preflight import densify, load_trajectory, grasp_capture_geometry


def _set_robot(robot: int, active: dict, q_sdk, inactive: dict, inactive_q):
    for profile, values in ((active, q_sdk), (inactive, inactive_q)):
        q_urdf = arm_profiles.sdk_to_urdf_deg(profile, values)
        for joint, value in zip(profile["joint_ids"], q_urdf):
            p.resetJointState(robot, joint, math.radians(float(value)))


def _grip_pose(robot: int, profile: dict) -> tuple[np.ndarray, np.ndarray, tuple]:
    state = p.getLinkState(robot, profile["ee_link_id"], computeForwardKinematics=True)
    rotation = np.asarray(p.getMatrixFromQuaternion(state[5]), float).reshape(3, 3)
    grip = np.asarray(state[4], float) + rotation @ (
        np.asarray(profile["grip_center_link_mm"], float) / 1000.0)
    proxy = np.asarray(state[4], float) + rotation @ (
        np.asarray(profile["gripper_collision_center_link_mm"], float) / 1000.0)
    return grip, rotation, (proxy, state[5])


def _in_grasp_capture(profile: dict, grip, R_link, bottle_center, pair=None) -> dict:
    """close 时瓶心必须已在两指捕获区；Demo 不得无接触绑瓶。"""
    if pair is not None:
        return grasp_capture_geometry(profile, grip, R_link, pair)
    a = math.radians(90.0)
    rz = np.array([[math.cos(a), -math.sin(a), 0.0],
                   [math.sin(a), math.cos(a), 0.0],
                   [0.0, 0.0, 1.0]])
    axis = (np.asarray(R_link, float) @ rz)[:, 0]
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    delta = np.asarray(bottle_center, float) - np.asarray(grip, float)
    axial = float(np.dot(delta, axis))
    lateral = float(np.linalg.norm(delta - axial * axis))
    axial_range = [float(v) for v in profile["grasp_capture_axis_range_m"]]
    lateral_limit = float(profile["grasp_capture_lateral_tolerance_m"])
    passed = axial_range[0] <= axial <= axial_range[1] and lateral <= lateral_limit
    return {"status": "PASS" if passed else "BLOCKED",
            "axis_offset_m": axial, "lateral_error_m": lateral}


def _timeline(waypoints, step_deg: float):
    timeline, previous, carrying, carried_index = [], None, False, -1
    for waypoint in waypoints:
        if "gripper" in waypoint:
            action = waypoint["gripper"]
            if action == "close":
                carrying = True
                carried_index += 1
            elif action == "open":
                carrying = False
            timeline.append({"kind": "gripper", "action": action,
                             "seg": waypoint.get("seg", ""),
                             "carrying": carrying, "carried_index": carried_index})
            continue
        if "q_sdk_deg" not in waypoint:
            continue
        q = [float(v) for v in waypoint["q_sdk_deg"]]
        if previous is None:
            timeline.append({"kind": "move", "q_sdk_deg": q,
                             "seg": waypoint.get("seg", ""),
                             "carrying": carrying, "carried_index": None})
        else:
            for dense_q in densify(previous, q, step_deg):
                timeline.append({"kind": "move", "q_sdk_deg": dense_q,
                                 "seg": waypoint.get("seg", ""),
                                 "carrying": carrying,
                                 "carried_index": carried_index if carrying else None})
        previous = q
    return timeline


def _create_scene(task: dict, profile: dict):
    for obstacle in task.get("obstacles", []):
        orientation = normalize_quaternion_xyzw(
            obstacle.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]))
        collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=obstacle["half"])
        visual = p.createVisualShape(p.GEOM_BOX, halfExtents=obstacle["half"],
                                     rgbaColor=[0.48, 0.34, 0.20, 0.75])
        p.createMultiBody(0, collision, visual, basePosition=obstacle["center"],
                          baseOrientation=orientation.tolist())

    pair = task["pairs"][0]
    dimensions = np.asarray(pair["dimensions_m"], float)
    offset = np.asarray(pair.get("carried_center_offset_grip_m", [0, 0, 0]), float)
    pick_grip = np.asarray(pair.get("actual_pick_grip", pair["pick"]), float)
    place_grip = np.asarray(pair.get("actual_place_grip", pair["place"]), float)
    pick_center = np.asarray(pair.get("object_pick_center", pick_grip + offset), float)
    place_center = np.asarray(pair.get("object_place_center", place_grip + offset), float)
    radius, height = max(dimensions[0], dimensions[1]) / 2.0, dimensions[2]
    bottle_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=radius, height=height)
    bottle_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=radius, length=height,
                                        rgbaColor=[0.18, 0.72, 0.92, 0.92])
    bottle = p.createMultiBody(0, bottle_collision, bottle_visual,
                               basePosition=pick_center.tolist())
    proxy_half = (np.asarray(profile["gripper_collision_box_m"], float) / 2.0).tolist()
    proxy_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=proxy_half,
                                       rgbaColor=[1.0, 0.25, 0.1, 0.25])
    proxy = p.createMultiBody(0, -1, proxy_visual)
    object_pick = np.asarray(pair.get("object_pick_center", pick_center), float)
    for point, color in ((pick_grip, [0.1, 1.0, 0.1, 0.9]),
                         (place_grip, [1.0, 0.75, 0.05, 0.9]),
                         (object_pick, [0.7, 0.2, 1.0, 0.9])):
        visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.018, rgbaColor=color)
        p.createMultiBody(0, -1, visual, basePosition=point.tolist())
    p.addUserDebugLine(pick_grip.tolist(), place_grip.tolist(), [1.0, 0.75, 0.05], 4)
    return {"body": bottle, "pick_center": pick_center, "place_center": place_center,
            "object_pick_center": object_pick, "offset": offset, "proxy": proxy,
            "pair": pair, "rigid": pair.get("object_attachment_mode") == "RIGID_FROM_CLOSE"}


def _draw_planned_grip_path(robot: int, profile: dict, inactive: dict,
                            inactive_q, waypoints):
    points = []
    for waypoint in waypoints:
        if "q_sdk_deg" not in waypoint:
            continue
        _set_robot(robot, profile, waypoint["q_sdk_deg"], inactive, inactive_q)
        grip, _R, _ = _grip_pose(robot, profile)
        points.append(grip.tolist())
    for a, b in zip(points, points[1:]):
        p.addUserDebugLine(a, b, [0.15, 0.55, 1.0], 2)


def _write_review(path: Path, args, result: str, timeline, meta):
    record = {
        "schema_version": "pybullet_gui_review.v1",
        "created": dt.datetime.now(dt.timezone.utc).isoformat(),
        "result": result,
        "reviewer": args.reviewer,
        "trajectory": str(Path(args.trajectory).resolve()),
        "trajectory_sha256": sha256_file(args.trajectory),
        "qualification_task": str(Path(args.task).resolve()),
        "qualification_task_sha256": sha256_file(args.task),
        "demo_script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "dense_step_deg": args.step_deg,
        "period_ms": args.period_ms,
        "speed_factor": args.speed_factor,
        "dense_move_frames": sum(item["kind"] == "move" for item in timeline),
        "gripper_events": [item["action"] for item in timeline
                           if item["kind"] == "gripper"],
        "arm_id": meta["arm_id"],
        "note": "人工观察 PyBullet GUI 的整条密集回放；不代表真机授权。"
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")
    print("GUI 审查记录:", path.resolve())


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--trajectory", required=True)
    ap.add_argument("--task", required=True,
                    help="生成器输出的 pairs+obstacles 资格任务 JSON")
    ap.add_argument("--headless", action="store_true", help="仅自动回归，不打开 GUI/不写人工记录")
    ap.add_argument("--step-deg", type=float, default=0.4)
    ap.add_argument("--period-ms", type=float, default=20.0)
    ap.add_argument("--speed-factor", type=float, default=1.0)
    ap.add_argument("--event-pause-s", type=float, default=1.0)
    ap.add_argument("--review-out")
    ap.add_argument("--reviewer", default="operator")
    args = ap.parse_args(argv)
    if args.step_deg <= 0 or args.period_ms <= 0 or args.speed_factor <= 0:
        ap.error("step/period/speed-factor 必须为正数")
    meta, waypoints, _schema = load_trajectory(args.trajectory)
    task = json.loads(Path(args.task).read_text(encoding="utf-8"))
    profile = arm_profiles.arm_profile(meta["arm"])
    inactive = arm_profiles.arm_profile("right" if meta["arm"] == "left" else "left")
    inactive_q = inactive["home_deg"]
    timeline = _timeline(waypoints, args.step_deg)
    robot = xf.load_xifeng(gui=not args.headless)
    scene = _create_scene(task, profile)
    try:
        _draw_planned_grip_path(robot, profile, inactive, inactive_q, waypoints)
        first = next(item for item in timeline if item["kind"] == "move")
        _set_robot(robot, profile, first["q_sdk_deg"], inactive, inactive_q)
        if not args.headless:
            pair = task["pairs"][0]
            target_array = ((np.asarray(pair["pick"]) +
                             np.asarray(pair["place"])) / 2.0)
            target = target_array.tolist()
            p.resetDebugVisualizerCamera(1.25, 155, -22, target)
            if (meta.get("debian_execution_allowed") is False or
                    "SIMULATION" in str(meta.get("qualification", ""))):
                warning_at = target_array.copy()
                warning_at[2] += 0.32
                p.addUserDebugText(
                    "SIMULATION ASSUMPTION ONLY - NOT A DEBIAN / REAL MOTION TRAJECTORY",
                    warning_at.tolist(), [0.9, 0.05, 0.05], 1.25)
        released_center = scene["pick_center"].copy()
        released_orientation = [0, 0, 0, 1]
        local_offset = inverse_close = None
        bound = False
        capture_at_close = None
        status_id = -1
        last_segment = None
        for index, item in enumerate(timeline, 1):
            if not p.isConnected():
                raise RuntimeError("PyBullet GUI 已关闭")
            if item["kind"] == "gripper":
                if item["action"] == "close":
                    grip, R_link, (_proxy, close_orientation) = _grip_pose(robot, profile)
                    capture_at_close = _in_grasp_capture(
                        profile, grip, R_link, scene["object_pick_center"], scene["pair"])
                    bound = capture_at_close["status"] == "PASS"
                    if bound and scene["rigid"]:
                        local_offset = R_link.T @ (scene["pick_center"] - grip)
                        _zero, inverse_close = p.invertTransform([0, 0, 0], close_orientation)
                    if not bound:
                        print("grasp capture BLOCKED; demo will not bind the bottle:",
                              json.dumps(capture_at_close))
                elif item["action"] == "open":
                    bound = False
                    released_center = np.asarray(p.getBasePositionAndOrientation(
                        scene["body"])[0], float)
                if not args.headless:
                    time.sleep(args.event_pause_s / args.speed_factor)
                continue
            _set_robot(robot, profile, item["q_sdk_deg"], inactive, inactive_q)
            grip, R, (proxy_center, proxy_orn) = _grip_pose(robot, profile)
            p.resetBasePositionAndOrientation(scene["proxy"], proxy_center.tolist(), proxy_orn)
            if bound:
                if scene["rigid"]:
                    released_center = grip + R @ local_offset
                    _zero, released_orientation = p.multiplyTransforms(
                        [0, 0, 0], proxy_orn, [0, 0, 0], inverse_close)
                else:
                    released_center = grip + scene["offset"]
            p.resetBasePositionAndOrientation(scene["body"], released_center.tolist(),
                                              released_orientation)
            p.performCollisionDetection()
            if not args.headless and item["seg"] != last_segment:
                text = f"{index}/{len(timeline)}  {item['seg']}"
                status_id = p.addUserDebugText(text, [0.12, 0.0, 1.38],
                                               [0.05, 0.05, 0.05], 1.4,
                                               replaceItemUniqueId=status_id)
                last_segment = item["seg"]
            # 这是关节轨迹的运动学回放，不推进动力学。推进 stepSimulation 会让未被
            # 轨迹控制的腰部等关节受默认马达/重力影响，反而把末端路径画错。
            if not args.headless:
                time.sleep((args.period_ms / 1000.0) / args.speed_factor)
        print(json.dumps({"trajectory_sha256": sha256_file(args.trajectory),
                          "dense_move_frames": sum(x["kind"] == "move" for x in timeline),
                          "gripper_events": [x["action"] for x in timeline
                                             if x["kind"] == "gripper"],
                          "grasp_capture_at_close": capture_at_close,
                          "final_bottle_center_m": released_center.tolist(),
                          "final_bottle_orientation_xyzw": list(released_orientation),
                          "object_attachment_mode": scene["pair"].get("object_attachment_mode", "LEGACY_WORLD_OFFSET")},
                         ensure_ascii=False, indent=2))
        if not args.headless:
            input("\n回放完成。请在 GUI 中检查桌面、接近、抓取、携带物和放置；回车后判定: ")
            result = input("输入 PASS 或 FAIL: ").strip().upper()
            if result not in ("PASS", "FAIL"):
                raise SystemExit("未输入 PASS/FAIL，不生成审查记录")
            if result == "PASS" and (not capture_at_close or capture_at_close["status"] != "PASS"):
                raise SystemExit("捕获区未通过，不能记录 GUI PASS")
            if args.review_out:
                _write_review(Path(args.review_out), args, result, timeline, meta)
    finally:
        if p.isConnected():
            p.disconnect()
    return 0 if capture_at_close and capture_at_close["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
