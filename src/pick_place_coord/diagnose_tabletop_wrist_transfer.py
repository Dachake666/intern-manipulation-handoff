#!/usr/bin/env python3
"""只读 J6/J7 相对运动诊断；不生成 SDK 轨迹，不签发真机授权。

SAFE 来自已存实测关节；PICK_ASCEND 仅为单点锚定的 URDF 相对 IK 假设，
不是 Debian 实际冗余构型。v2 仅解释回读变化，不用于物理碰撞判断。
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
sys.path[:0] = [str(HERE), str(WORK), str(WORK / "frame_calibration/robot_side")]
import arm_profiles
import gen_bottle_servo_candidate as ik
import sdk_session as ss
import xifeng_pb as xf
from frame_calibration.analysis import calib_common as cc
from robot_mission.contracts import sha256_file

DIAGNOSTIC_JOINT_STEP_DEG = 0.25  # 仅离线诊断密化，不是控制器/Servo 步长


def finite(values, size):
    out = np.asarray(values, float)
    if out.shape != (size,) or not np.all(np.isfinite(out)):
        raise ValueError(f"需要 {size} 个有限数值")
    return out


def probe(robot, q):
    q = finite(q, 7)
    for jid, angle in zip(cc.ARM_JOINT_IDS["left"], cc.sdk_q_to_urdf_q(q)):
        p.resetJointState(robot, jid, math.radians(angle))
    st = p.getLinkState(robot, cc.EE_LINK_ID["left"], computeForwardKinematics=True)
    link = np.asarray(st[4]) * 1000
    rotation = np.asarray(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
    forearm_link = cc.ARM_JOINT_IDS["left"][4]
    forearm = p.getLinkState(robot, forearm_link, computeForwardKinematics=True)
    r9 = np.asarray(p.getMatrixFromQuaternion(forearm[5])).reshape(3, 3)
    tcp = link + rotation @ cc.CONFIRMED_R_TCP_LINK11_MM
    return {
        "link_origin": link,
        "physical_grip": link + rotation @ cc.GRIP_R_LINK_MM["left"],
        "physical_tcp": tcp,
        "sdk_v2_without_session_t": tcp + r9 @ cc.CONFIRMED_D_FOREARM_LINK9_MM,
        "rotation": rotation,
    }


def grid(lo, hi, step):
    if lo == hi:
        return np.asarray([lo])
    return np.linspace(lo, hi, max(1, int(math.ceil((hi - lo) / step))) + 1)


def segment_diagnostic(robot, start, end, limits):
    """只评估假定同步关节插值，不能代替控制器轨迹或环境扫掠验证。"""
    start, end = finite(start, 7), finite(end, 7)
    baseline = probe(robot, start)
    step = DIAGNOSTIC_JOINT_STEP_DEG
    count = max(1, int(math.ceil(np.max(np.abs(end - start)) / step)))
    frames = [probe(robot, start + t * (end - start)) for t in np.linspace(0, 1, count + 1)]
    delta = np.asarray([frame["physical_grip"] - baseline["physical_grip"] for frame in frames])
    assumed_bottle_tilt = []
    for frame in frames:
        up = frame["rotation"] @ baseline["rotation"].T @ np.array([0, 0, 1])
        assumed_bottle_tilt.append(math.degrees(math.acos(float(np.clip(up[2], -1, 1)))))
    endpoint_margins = [
        min(start[i] - lo, hi - start[i], end[i] - lo, hi - end[i])
        for i, (lo, hi) in enumerate(limits)]
    return {"interpolation": "JOINT_LINEAR_ASSUMPTION_NOT_CONTROLLER_TRACE",
            "dense_frame_count": count + 1,
            "max_adjacent_joint_step_deg": float(np.max(np.abs(end - start)) / count),
            "minimum_margin_per_joint_deg": endpoint_margins,
            "grip_z_delta_range_mm": [float(delta[:, 2].min()), float(delta[:, 2].max())],
            "grip_xyz_delta_range_mm": [delta.min(axis=0).tolist(), delta.max(axis=0).tolist()],
            "bottle_orientation_assumption": "UPRIGHT_AT_START_RIGIDLY_ATTACHED_NO_SLIP",
            "max_assumed_bottle_tilt_deg": max(assumed_bottle_tilt),
            "wrist_gripper_bottle_collision_status": "NOT_CHECKED_UNREGISTERED_SCENE"}


def sweep(robot, start, limits, j7_window, target_delta, step):
    start = finite(start, 7)
    baseline = probe(robot, start)
    margin = ss.LIMIT_MARGIN_DEG
    lo6, hi6 = limits[5][0] + margin, limits[5][1] - margin
    lo7, hi7 = limits[6][0] + margin, limits[6][1] - margin
    if j7_window is not None:
        lo7, hi7 = max(lo7, start[6] - j7_window), min(hi7, start[6] + j7_window)
    if lo7 > hi7:
        raise ValueError("J7 扫描范围为空")
    keys = ("link_origin", "physical_grip", "physical_tcp", "sdk_v2_without_session_t")
    samples, qs = [], []
    for j6 in grid(lo6, hi6, step):
        for j7 in grid(lo7, hi7, step):
            q = start.copy()
            q[5:] = [j6, j7]
            item = probe(robot, q)
            samples.append([item[k] - baseline[k] for k in keys])
            qs.append(q.tolist())
    data = np.asarray(samples)
    displacement = data[:, keys.index("physical_grip"), :]
    error = np.linalg.norm(displacement - target_delta, axis=1)
    best = int(np.argmin(error))
    height_band = np.flatnonzero(np.abs(displacement[:, 2]) <= 5.0)
    height_best = (int(height_band[np.argmin(error[height_band])])
                   if height_band.size else None)

    def solution(index):
        if index is None:
            return None
        return {"q_sdk_deg": qs[index],
                "grip_delta_pb_mm": displacement[index].tolist(),
                "distance_to_reference_delta_mm": float(error[index]),
                "sdk_v2_delta_mm": data[index, 3].tolist()}

    report = {
        "j1_to_j5_fixed_deg": start[:5].tolist(),
        "start_q_sdk_deg": start.tolist(),
        "j6_range_deg": [float(lo6), float(hi6)],
        "j7_range_deg": [float(lo7), float(hi7)],
        "grid_step_deg_at_most": step,
        "sample_count": len(samples),
        "required_limit_margin_deg": margin,
        "reference_delta_mm": target_delta.tolist(),
        "reference_delta_role": "OLD_HOVER_TRANSLATION_COMPARISON_NOT_CURRENT_BIN_BOUNDS",
        "ranges_relative_to_start_mm": {
            key: {"minimum_xyz": data[:, i].min(axis=0).tolist(),
                  "maximum_xyz": data[:, i].max(axis=0).tolist(),
                  "max_displacement_norm": float(np.linalg.norm(data[:, i], axis=1).max())}
            for i, key in enumerate(keys)},
        "closest_reference_sample": solution(best),
        "height_filter_scope": "ENDPOINT_ONLY_SEE_SELECTED_SEGMENT_FOR_DENSE_PATH_HEIGHT",
        "height_within_5mm_sample_count": int(height_band.size),
        "closest_reference_sample_within_5mm_height": solution(height_best),
        "not_a_collision_check": True,
    }
    selected = height_best if height_best is not None else best
    report["selected_segment_diagnostic"] = segment_diagnostic(robot, start, qs[selected], limits)
    return report, qs[selected]


def build(robot, snapshot, ascent, target_delta, start_override=None, step=0.5):
    profile = arm_profiles.arm_profile("left")
    if not np.array_equal(profile["controller_limits_deg"], ss.LIMITS_DEG[1]):
        raise ValueError("共享 arm_profile 与 sdk_session 左臂限位漂移")
    if profile["joint_ids"] != cc.ARM_JOINT_IDS["left"]:
        raise ValueError("共享关节映射漂移")
    if not np.allclose(profile["grip_center_link_mm"], cc.GRIP_R_LINK_MM["left"]):
        raise ValueError("共享夹爪偏移漂移")
    safe = finite(snapshot["safe_candidate"]["joints_sdk_deg"], 7)
    safe_xyz = finite(snapshot["safe_candidate"]["pose"]["position_mm"], 3)
    safe_fk = probe(robot, safe)
    cases = [("MEASURED_SAFE_REFERENCE_NOT_PICK_ASCEND", safe)]
    assumed = None
    if start_override is not None:
        cases.append(("USER_SUPPLIED_START_JOINTS_SCENE_UNQUALIFIED", finite(start_override, 7)))
    else:
        target = (safe_fk["physical_grip"] + ascent - safe_xyz) / 1000
        bounds = ik._solver_bounds_from_limits(ss.LIMITS_DEG[1], ss.LIMIT_MARGIN_DEG)
        q, residual = ik.solve_full_pose(robot, target, safe_fk["rotation"], safe,
                                        bounds, iterations=500)
        assumed = {"mode": "SINGLE_SAFE_RELATIVE_URDF_IK_ASSUMPTION_ONLY",
                   "requested_sdk_xyz_mm": ascent.tolist(),
                   "not_actual_debian_joints": True, "residual": residual,
                   "solved": q is not None}
        if q is not None:
            cases.append(("ASSUMED_PICK_ASCEND_NOT_REAL_CONTROLLER_BRANCH", q))
    reports, playback = [], None
    for name, start in cases:
        bad = ss.check_limits(1, start, margin=ss.LIMIT_MARGIN_DEG)
        if bad:
            reports.append({"case": name, "status": "START_LIMIT_REJECT", "details": bad})
            continue
        scans = []
        for label, j7_window in (("J6_ONLY", 0.0), ("J6_WITH_J7_PLUS_MINUS_5", 5.0),
                                 ("J6_J7_FULL_ALLOWED_ENVELOPE_DIAGNOSTIC", None)):
            result, best = sweep(robot, start, ss.LIMITS_DEG[1], j7_window, target_delta, step)
            result["mode"] = label
            scans.append(result)
            if label == "J6_WITH_J7_PLUS_MINUS_5":
                playback = (list(start), best)
        reports.append({"case": name, "scans": scans})
    return {
        "schema_version": "tabletop_wrist_transfer_diagnostic.v1",
        "status": "BLOCKED_FOR_TRAJECTORY_QUALIFICATION",
        "real_motion_authorized": False, "debian_execution_allowed": False,
        "physical_frame": "URDF_RELATIVE_ONLY_NO_ABSOLUTE_SDK_TO_PB_MAPPING",
        "v2_scope": "RELATIVE_SDK_READOUT_EXPLANATION_ONLY_NOT_PHYSICAL_CALIBRATION",
        "ascent_ik_assumption": assumed, "cases": reports,
        "blockers": [
            "Latest successful Debian JSON and executor originals not synchronized",
            "Actual successful PICK_ASCEND redundant joint branch not established",
            "Model-A PB-SDK gate blocked; no current physical bin/table registration",
            "Wrist/gripper/held-object swept collisions not qualified",
            "No full mixed-path GUI review or controller precheck"],
    }, playback


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path, default=HERE / "tasks/task_tabletop_vision_ab_20260902.json")
    parser.add_argument("--ascent-sdk-mm", nargs=3, type=float, default=[700, 250, 550])
    parser.add_argument("--reference-delta-mm", nargs=3, type=float, default=[20, -150, 0])
    parser.add_argument("--start-joints", nargs=7, type=float)
    parser.add_argument("--grid-step-deg", type=float, default=0.5)
    parser.add_argument("--report-out", type=Path)
    parser.add_argument("--gui", action="store_true")
    args = parser.parse_args(argv)
    if not math.isfinite(args.grid_step_deg) or not 0.1 <= args.grid_step_deg <= 2:
        parser.error("grid step 必须在 0.1~2 度之间")
    snapshot = json.loads(args.snapshot.read_text())
    robot = xf.load_xifeng(gui=args.gui)
    try:
        report, playback = build(robot, snapshot, finite(args.ascent_sdk_mm, 3),
                                 finite(args.reference_delta_mm, 3), args.start_joints,
                                 args.grid_step_deg)
        report["source_hashes"] = {
            str(path.resolve().relative_to(WORK)): sha256_file(path)
            for path in (Path(__file__), args.snapshot, Path(cc.DEFAULT_URDF),
                         WORK / "frame_calibration/analysis/calib_common.py",
                         WORK / "frame_calibration/robot_side/sdk_session.py",
                         WORK / "arm_profiles.v1.json", Path(ik.__file__), Path(xf.__file__))}
        if args.report_out:
            output = args.report_out.resolve()
            output.relative_to(WORK)
            output.parent.mkdir(parents=True, exist_ok=True)
            output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if args.gui and playback:
            start, target = map(np.asarray, playback)
            position = probe(robot, start)["physical_grip"] / 1000
            p.resetDebugVisualizerCamera(1.0, 150, -24, position.tolist())
            p.addUserDebugText("WRIST DIAGNOSTIC ONLY - NO BIN/COLLISION PASS",
                              (position + [0, 0, .3]).tolist(), [1, 0, 0], 1.1)
            count = max(1, int(math.ceil(np.max(np.abs(target - start)) / DIAGNOSTIC_JOINT_STEP_DEG)))
            for begin, end in ((start, target), (target, start)):
                for t in np.linspace(0, 1, count + 1):
                    if not p.isConnected():
                        return 2
                    probe(robot, begin + (end - begin) * t)
                    time.sleep(0.02)
        return 2  # 诊断有数值结果仍不构成整条轨迹 PASS
    finally:
        if p.isConnected():
            p.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
