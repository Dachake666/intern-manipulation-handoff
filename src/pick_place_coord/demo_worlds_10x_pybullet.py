#!/usr/bin/env python3
"""XF0112048 左臂世界坐标十步控制的 PyBullet 真模型演示。

这不是手画示意图：程序会加载工作区 XF0112048/robot.urdf，从一条
REALVERIFIED 左臂轨迹中取安全悬停姿态，按 execute_worlds_10x.py 的同一十步
相对世界坐标配方逐 2mm 做数值 IK，并逐帧检查关节限位、新增自碰撞和桌面碰撞。
任何硬门失败都不会进入 GUI 播放。

用法（当前 Mac 已有 PyBullet 的 Anaconda Python）：
  /opt/anaconda3/bin/python3 demo_worlds_10x_pybullet.py          # DIRECT 验证
  /opt/anaconda3/bin/python3 demo_worlds_10x_pybullet.py --gui    # 窗口循环播放

GUI 中橙色小球是十步目标位置，蓝线是笛卡尔路径；每到一站会显示序号及 FK
反馈误差。关闭 PyBullet 窗口即可结束。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
ROBOT_SIDE = WORK / "frame_calibration" / "robot_side"
ANALYSIS = WORK / "frame_calibration" / "analysis"
sys.path[:0] = [str(HERE), str(ROBOT_SIDE), str(ANALYSIS)]

import calib_common as cc  # noqa: E402
import execute_worlds_10x as core  # noqa: E402
import left_arm_ik as ik  # noqa: E402
import xifeng_pb as xf  # noqa: E402


SOURCE_TRAJECTORY = (
    WORK / "pick_place_coord" / "trajectories" / "verified" /
    "traj_multi_2grasp_20260804_REALVERIFIED.json"
)
# 取 REALVERIFIED 放置后上升段的中间姿态，而不是最高悬停点。
# 0/1 号更低，但当前 URDF+桌面模型会发生碰撞；2 号是复核通过的较低档。
SOURCE_SEGMENT = "PLACE1_ASCEND"
SOURCE_OCCURRENCE = 2
OBJECT_HALF_M = 0.025
SAMPLE_MM = 2.0


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def set_arm(robot: int, q_rad) -> None:
    for joint, value in zip(ik.ARM, q_rad):
        p.resetJointState(robot, joint, float(value))


def rotation_error_vector(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """返回从 current 转到 target 的世界系旋转向量(rad)。"""
    delta = target @ current.T
    angle = math.acos(float(np.clip((np.trace(delta) - 1.0) / 2.0, -1.0, 1.0)))
    if angle < 1e-10:
        return np.zeros(3)
    sine = math.sin(angle)
    if abs(sine) < 1e-10:
        raise RuntimeError("完整姿态 IK 遇到接近 180° 的旋转奇异")
    axis = np.array([
        delta[2, 1] - delta[1, 2],
        delta[0, 2] - delta[2, 0],
        delta[1, 0] - delta[0, 1],
    ]) / (2.0 * sine)
    return axis * angle


def solve_full_pose(robot: int, target_position, target_rotation: np.ndarray,
                    seed_rad, iterations: int = 200):
    """7关节 DLS 逆解：同时锁定抓取中心 XYZ 和完整末端旋转矩阵。"""
    lower, upper = ik._limits(robot)  # 与项目左臂 IK 共用真机 j7 覆盖和软余量
    q = np.asarray(seed_rad, float)

    def error(values):
        grip, rotation = ik._fk(robot, values)
        return np.concatenate([
            np.asarray(target_position, float) - grip,
            rotation_error_vector(rotation, target_rotation),
        ])

    for iteration in range(iterations):
        e = error(q)
        pos_error = float(np.linalg.norm(e[:3]))
        orientation_error = float(np.linalg.norm(e[3:]))
        if pos_error < 0.0005 and orientation_error < math.radians(0.1):
            return q.tolist(), {
                "pos_err_mm": pos_error * 1000.0,
                "orientation_err_deg": math.degrees(orientation_error),
                "iterations": iteration,
            }
        jacobian = np.zeros((6, 7))
        epsilon = 1e-5
        for axis in range(7):
            perturbed = q.copy()
            perturbed[axis] += epsilon
            # error=target-FK，因此取负号得到正向 FK 雅可比。
            jacobian[:, axis] = -(error(perturbed) - e) / epsilon
        damping = 0.03
        delta_q = jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping ** 2 * np.eye(6), e)
        delta_q = np.clip(delta_q, -math.radians(5.0), math.radians(5.0))
        q = np.clip(q + delta_q, lower, upper)
    final = error(q)
    return None, {
        "pos_err_mm": float(np.linalg.norm(final[:3]) * 1000.0),
        "orientation_err_deg": float(math.degrees(np.linalg.norm(final[3:]))),
        "iterations": iterations,
    }


def load_verified_anchor() -> tuple[list[float], dict]:
    with SOURCE_TRAJECTORY.open(encoding="utf-8") as handle:
        payload = json.load(handle)
    candidates = [
        wp for wp in payload["waypoints"]
        if wp.get("seg") == SOURCE_SEGMENT and "q_sdk_deg" in wp
    ]
    if not candidates:
        raise RuntimeError(f"源轨迹中找不到 {SOURCE_SEGMENT}")
    if not 0 <= SOURCE_OCCURRENCE < len(candidates):
        raise RuntimeError(
            f"{SOURCE_SEGMENT} 只有 {len(candidates)} 个点，"
            f"无法取第 {SOURCE_OCCURRENCE} 个")
    source = candidates[SOURCE_OCCURRENCE]
    q_urdf_deg = cc.sdk_q_to_urdf_q(source["q_sdk_deg"])
    return [math.radians(v) for v in q_urdf_deg], payload["meta"]


def create_table(anchor, source_meta: dict) -> tuple[int, float]:
    pick_z = float(source_meta["pairs_pb_m"][0]["pick"][2])
    table_top = pick_z - OBJECT_HALF_M
    # 与本候选局部路径对应的 0.5m 方桌；再扩大到躯干下方会把“桌穿身体”
    # 这种场景搭建错误误报成手臂轨迹碰撞。
    half = [0.25, 0.25, 0.02]
    collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
    visual = p.createVisualShape(
        p.GEOM_BOX, halfExtents=half, rgbaColor=[0.72, 0.60, 0.44, 0.65])
    body = p.createMultiBody(
        baseMass=0,
        baseCollisionShapeIndex=collision,
        baseVisualShapeIndex=visual,
        basePosition=[float(anchor[0]), float(anchor[1]), table_top - half[2]])
    return body, table_top


def self_collision_pairs(robot: int) -> set[tuple[int, int]]:
    return {
        tuple(sorted((int(contact[3]), int(contact[4]))))
        for contact in p.getClosestPoints(robot, robot, 0.0)
        if contact[3] != contact[4]
    }


def solve_candidate(robot: int, step_mm: float) -> dict:
    q_start, source_meta = load_verified_anchor()
    set_arm(robot, q_start)
    anchor, rotation_start = ik.grip_state(robot)
    table, table_top = create_table(anchor, source_meta)
    p.performCollisionDetection()
    baseline_self = self_collision_pairs(robot)
    if p.getClosestPoints(robot, table, 0.0):
        raise RuntimeError("REALVERIFIED 锚点姿态与模拟桌面碰撞，模型契约不一致")

    offsets = core.build_offsets(step_mm)
    targets = [(anchor + np.asarray(off[:3], float) / 1000.0).tolist()
               for off in offsets]
    previous_position = anchor.tolist()
    seed = list(q_start)
    frames = []
    move_ends = []
    max_pos_error_mm = 0.0
    max_ik_orientation_error_deg = 0.0

    for move_index, target in enumerate(targets, start=1):
        distance = float(np.linalg.norm(
            np.asarray(target) - np.asarray(previous_position)))
        count = max(1, math.ceil(distance / (SAMPLE_MM / 1000.0)))
        for sample_index in range(1, count + 1):
            position = (
                np.asarray(previous_position) +
                (np.asarray(target) - np.asarray(previous_position)) *
                sample_index / count)
            q, info = solve_full_pose(
                robot, position.tolist(), rotation_start, seed_rad=seed)
            if q is None:
                raise RuntimeError(
                    f"move {move_index} sample {sample_index}/{count} IK 失败: {info}")
            q_sdk_deg = cc.urdf_q_to_sdk_q([math.degrees(v) for v in q])
            bad = core.limit_violations(
                q_sdk_deg, core.LEFT_OPERATIONAL_LIMITS_DEG,
                core.DEFAULT_LIMIT_MARGIN_DEG, source="本地实测")
            if bad:
                raise RuntimeError(
                    f"move {move_index} sample {sample_index}/{count} 限位:\n  " +
                    "\n  ".join(bad))
            seed = list(q)
            max_pos_error_mm = max(max_pos_error_mm, float(info["pos_err_mm"]))
            max_ik_orientation_error_deg = max(
                max_ik_orientation_error_deg,
                float(info.get("orientation_err_deg", 0.0)))
            frames.append({
                "move": move_index,
                "q_rad": list(q),
                "target_pb_m": position.tolist(),
                "ik": info,
            })
        move_ends.append(len(frames) - 1)
        previous_position = target

    previous_q = q_start
    max_joint_step_deg = 0.0
    joint_ranges_sdk = [[math.inf, -math.inf] for _ in range(7)]
    new_self = set()
    table_collision_frames = []
    minimum_table_clearance_m = math.inf
    feedback = []
    end_set = set(move_ends)

    for frame_index, frame in enumerate(frames):
        q = frame["q_rad"]
        jump = max(abs(math.degrees(a - b)) for a, b in zip(q, previous_q))
        max_joint_step_deg = max(max_joint_step_deg, jump)
        previous_q = q
        q_sdk = cc.urdf_q_to_sdk_q([math.degrees(v) for v in q])
        for axis, value in enumerate(q_sdk):
            joint_ranges_sdk[axis][0] = min(joint_ranges_sdk[axis][0], value)
            joint_ranges_sdk[axis][1] = max(joint_ranges_sdk[axis][1], value)
        set_arm(robot, q)
        p.performCollisionDetection()
        new_self |= self_collision_pairs(robot) - baseline_self
        if p.getClosestPoints(robot, table, 0.0):
            table_collision_frames.append(frame_index)
        nearby = p.getClosestPoints(robot, table, 0.20)
        if nearby:
            minimum_table_clearance_m = min(
                minimum_table_clearance_m, min(float(c[8]) for c in nearby))
        if frame_index in end_set:
            actual, rotation = ik.grip_state(robot)
            target = np.asarray(frame["target_pb_m"])
            move = int(frame["move"])
            feedback.append({
                "move": move,
                "target_relative_mm": ((target - anchor) * 1000.0).tolist(),
                "actual_relative_mm": ((actual - anchor) * 1000.0).tolist(),
                "position_error_mm": float(np.linalg.norm(actual - target) * 1000.0),
                "orientation_drift_deg": float(
                    cc.rotation_angle_deg(rotation_start.T @ rotation)),
            })

    if new_self:
        raise RuntimeError(f"出现锚点基线之外的新自碰撞连杆对: {sorted(new_self)}")
    if table_collision_frames:
        raise RuntimeError(f"桌面碰撞帧: {table_collision_frames[:10]}")
    if len(feedback) != core.MOVE_COUNT:
        raise RuntimeError(f"反馈数量不是10: {len(feedback)}")

    margins = []
    for (used_lo, used_hi), (hard_lo, hard_hi) in zip(
            joint_ranges_sdk, core.LEFT_OPERATIONAL_LIMITS_DEG):
        margins.append(min(used_lo - hard_lo, hard_hi - used_hi))
    return {
        "q_start": q_start,
        "anchor_pb_m": anchor.tolist(),
        "targets_pb_m": targets,
        "frames": frames,
        "move_ends": move_ends,
        "feedback": feedback,
        "table": table,
        "table_top_m": table_top,
        "source_sha256": sha256(SOURCE_TRAJECTORY),
        "frames_count": len(frames),
        "max_joint_step_deg": max_joint_step_deg,
        "joint_ranges_sdk_deg": joint_ranges_sdk,
        "minimum_hard_limit_margin_deg": margins,
        "max_ik_position_error_mm": max_pos_error_mm,
        "max_ik_orientation_error_deg": max_ik_orientation_error_deg,
        "max_feedback_position_error_mm": max(
            item["position_error_mm"] for item in feedback),
        "max_orientation_drift_deg": max(
            item["orientation_drift_deg"] for item in feedback),
        "minimum_table_clearance_mm": (
            minimum_table_clearance_m * 1000.0
            if math.isfinite(minimum_table_clearance_m) else None),
        "new_self_collisions": sorted(new_self),
        "table_collision_frames": table_collision_frames,
    }


def print_report(result: dict) -> None:
    print("=" * 70)
    print("PyBullet · XF0112048 左臂世界坐标十步候选")
    print("=" * 70)
    print(f"源轨迹: {SOURCE_TRAJECTORY.name}")
    print(f"锚点来源: {SOURCE_SEGMENT}[{SOURCE_OCCURRENCE}]（较低安全姿态）")
    print(f"源 SHA-256: {result['source_sha256']}")
    print(f"锚点 PB XYZ: {[round(v, 5) for v in result['anchor_pb_m']]}")
    print(f"运动: 10 次 / 密集 IK: {result['frames_count']} 帧 / 2mm")
    print(f"最大相邻关节变化: {result['max_joint_step_deg']:.3f}°")
    print(f"最大 IK 位置误差: {result['max_ik_position_error_mm']:.3f}mm")
    print(f"最大 IK 完整姿态误差: {result['max_ik_orientation_error_deg']:.3f}°")
    print(f"最大完整姿态漂移: {result['max_orientation_drift_deg']:.3f}°")
    print(f"最大站点位置误差: {result['max_feedback_position_error_mm']:.3f}mm")
    print(f"最小硬限位余量: {min(result['minimum_hard_limit_margin_deg']):.2f}°")
    print(f"最小桌面距离: {result['minimum_table_clearance_mm']:.2f}mm")
    print("新增自碰撞: 0；桌面碰撞帧: 0")
    print("结论: OFFLINE_PASS（PyBullet 候选，不等于真机验证）")


def add_scene_guides(result: dict) -> None:
    points = [result["anchor_pb_m"]] + result["targets_pb_m"]
    for a, b in zip(points, points[1:]):
        p.addUserDebugLine(a, b, [0.15, 0.55, 1.0], 4, 0)
    seen = set()
    for index, target in enumerate(result["targets_pb_m"], start=1):
        key = tuple(round(v, 6) for v in target)
        if key in seen:
            continue
        seen.add(key)
        visual = p.createVisualShape(
            p.GEOM_SPHERE, radius=0.0045, rgbaColor=[1.0, 0.45, 0.05, 0.95])
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual,
                          basePosition=target)


def play_gui(robot: int, result: dict, speed: float) -> None:
    p.resetDebugVisualizerCamera(
        cameraDistance=0.75, cameraYaw=150, cameraPitch=-22,
        cameraTargetPosition=result["anchor_pb_m"])
    add_scene_guides(result)
    frames = result["frames"]
    ends = set(result["move_ends"])
    status_id = -1
    cycle = 0
    while p.isConnected():
        cycle += 1
        set_arm(robot, result["q_start"])
        time.sleep(0.5 / speed)
        previous_q = result["q_start"]
        for frame_index, frame in enumerate(frames):
            if not p.isConnected():
                return
            q = frame["q_rad"]
            # 每个 2mm IK 点再插 5 帧，只改变显示帧率，不改变已验证路径。
            for sub in range(1, 6):
                if not p.isConnected():
                    return
                blended = [a + (b - a) * sub / 5 for a, b in zip(previous_q, q)]
                try:
                    set_arm(robot, blended)
                except p.error:
                    # 用户关闭 GUI 时 Physics Server 会先断开；这是正常退出。
                    return
                time.sleep(1.0 / (60.0 * speed))
            previous_q = q
            if frame_index in ends:
                move = frame["move"]
                feedback = result["feedback"][move - 1]
                text = (
                    f"cycle {cycle}  move {move}/10\n"
                    f"FK error {feedback['position_error_mm']:.3f} mm")
                status_id = p.addUserDebugText(
                    text,
                    [result["anchor_pb_m"][0], result["anchor_pb_m"][1],
                     result["anchor_pb_m"][2] + 0.09],
                    textColorRGB=[0.05, 0.15, 0.35], textSize=1.5,
                    replaceItemUniqueId=status_id)
                print(
                    f"cycle {cycle} move {move:02d}: actual relative XYZ="
                    f"{[round(v, 3) for v in feedback['actual_relative_mm']]} mm")
                time.sleep(0.35 / speed)


def main() -> int:
    parser = argparse.ArgumentParser(description="XF0112048 世界坐标十步 PyBullet demo")
    parser.add_argument("--gui", action="store_true", help="打开 PyBullet 窗口循环播放")
    parser.add_argument("--step-mm", type=float, default=20.0,
                        help="十步图案幅值；默认与真机候选脚本一致")
    parser.add_argument("--speed", type=float, default=1.0, help="GUI 播放倍速")
    args = parser.parse_args()
    if args.speed <= 0:
        parser.error("--speed 必须大于0")

    robot = xf.load_xifeng(gui=args.gui)
    try:
        result = solve_candidate(robot, args.step_mm)
        print_report(result)
        if args.gui:
            print("PyBullet 窗口将循环播放；关闭窗口结束。")
            play_gui(robot, result, args.speed)
        return 0
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    sys.exit(main())
