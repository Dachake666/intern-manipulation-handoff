#!/usr/bin/env python3
"""[待核 20260907] Worlds 抓放 + 两段 MoveJ 转运的完整离线候选。

不导入 pypilot，不连接机器人，不支持 --run。原 SDK EE_POSE 不再补偿 TCP。
v2 只解释报告中的 SDK 回读和新关节的预期回读，绝不作为场景碰撞标定。
回放中的 Worlds 是预测 IK，MoveJ 是同步关节线性假设，均不冒充实际控制器轨迹。
"""
from __future__ import annotations

import argparse
import ast
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
sys.path[:0] = [str(HERE), str(WORK), str(WORK / "frame_calibration/robot_side")]
import diagnose_tabletop_wrist_transfer as d
from execute_tabletop_pick_place_worlds import densify_line, MAX_JOINT_STEP_DEG, SAMPLE_MM, SAMPLE_DEG
from robot_mission.contracts import sha256_file
from robot_mission import grasp_pose
import tabletop_servo_contract as servo_contract

SCHEMA = "tabletop_hybrid_candidate.v1"
OPTIMIZED_SCHEMA = "tabletop_hybrid_candidate.v2"
DEFAULT_OUTPUT = HERE / "trajectories/candidates/tabletop_pick_place_hybrid_CANDIDATE.json"
OPTIMIZED_OUTPUT = HERE / "trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED.json"
SERVO_OUTPUT = HERE / "trajectories/candidates/tabletop_pick_place_SERVO_CANDIDATE.json"
# 仅显示/数值诊断密度，沿用既有腕部诊断的密化，不是 Servo 控制周期。
REPLAY_STEP_DEG = d.DIAGNOSTIC_JOINT_STEP_DEG


def finite(values, size):
    if not isinstance(values, (list, tuple, np.ndarray)) or any(isinstance(v, bool) for v in values):
        raise ValueError("需要数值数组，不接受 bool")
    return d.finite(values, size)


def unpack_report(report):
    if (report.get("schema_version") != "tabletop_readonly_ik_capture.v1"
            or report.get("status") != "WORLDS_IK_PRECHECK_PASS_NOT_MOTION_AUTHORIZED"
            or report.get("real_motion_authorized") is not False
            or report.get("session_stopped") is not True):
        raise ValueError("需要完整结束的只读 IK PASS 报告，不是运动授权")
    stages = report["plan_content"]["stages"]
    expected = [s for s in stages if s["kind"] == "MOVE_WORLDS"]
    segments = report["precheck_summary"]["segments"]
    if [s["name"] for s in expected] != [s["stage"] for s in segments]:
        raise ValueError("报告段序与原计划不一致")
    rows, groups, offset = report["dense_ik_samples"], {}, 0
    for stage, segment in zip(expected, segments):
        count = segment["dense_points"]
        if isinstance(count, bool) or not isinstance(count, int) or count <= 0:
            raise ValueError("段采样数无效")
        group = rows[offset:offset + count]
        if len(group) != count:
            raise ValueError("密集 IK 记录缺失")
        for index, row in enumerate(group, offset + 1):
            if row["sample"] != index:
                raise ValueError("密集 IK 编号不连续")
            finite(row["q_sdk_deg"], 7)
            finite(row["worlds_xyzuvw"], 6)
        if not np.allclose(group[-1]["q_sdk_deg"], report["endpoint_ik"][stage["name"]], atol=.002, rtol=0):
            raise ValueError("端点关节和密集记录不一致")
        if not np.allclose(group[-1]["worlds_xyzuvw"], pose6(stage["pose"]), atol=1e-6, rtol=0):
            raise ValueError("端点 Worlds 和原计划不一致")
        groups[stage["name"]] = group
        offset += count
    if offset != len(rows) or offset != report["precheck_summary"]["dense_points"]:
        raise ValueError("报告密集点数不一致")
    return groups


def pose6(pose):
    return finite(pose["position_mm"], 3).tolist() + finite(pose["sdk_world_uvw_deg"], 3).tolist()


def pose_dict(xyzuvw):
    v = finite(xyzuvw, 6)
    return {"position_mm": v[:3].tolist(), "sdk_world_uvw_deg": v[3:].tolist(),
            "quaternion_xyzw": Rotation.from_euler("xyz", v[3:], degrees=True).as_quat().tolist(),
            "reference_point": "EE_POSE", "source": "V2_RELATIVE_READOUT_PREDICTION_NOT_SCENE_CALIBRATION"}


def predicted_worlds(robot, q, explanation_offset):
    frame = d.probe(robot, q)
    return np.r_[frame["sdk_v2_without_session_t"] + explanation_offset,
                 d.ik._matrix_to_uvw(frame["rotation"] @ d.ik._rotz(90))]


def solve_grip(robot, seed, target, target_rotation, bounds, *, joint_reference=None,
               joint_weights=None, prefer_j6=None):
    seed = finite(seed, 7)
    reference = seed if joint_reference is None else finite(joint_reference, 7)
    weights = np.full(7, .06) if joint_weights is None else finite(joint_weights, 7)

    def error(q):
        frame = d.probe(robot, q)
        rerr = Rotation.from_matrix(target_rotation @ frame["rotation"].T).as_rotvec() * 180 / math.pi
        parts = [4 * (frame["physical_grip"] - target), .04 * rerr, weights * (q - reference)]
        if prefer_j6 is not None:
            parts.append(np.array([.5 * (q[5] - prefer_j6)]))
        return np.concatenate(parts)

    result = least_squares(error, np.clip(seed, *bounds), bounds=bounds, diff_step=1e-3, max_nfev=400)
    q = result.x
    residual = float(np.linalg.norm(d.probe(robot, q)["physical_grip"] - target))
    if residual > .2:
        raise ValueError(f"夹持点 IK 位置误差过大: {residual:.3f} mm")
    return q


def solve_worlds(robot, seed, target, explanation_offset, bounds, joint_reference=None):
    target = finite(target, 6)
    target_r = Rotation.from_euler("xyz", target[3:], degrees=True).as_matrix() @ d.ik._rotz(-90)

    def error(q):
        frame = d.probe(robot, q)
        pos = frame["sdk_v2_without_session_t"] + explanation_offset - target[:3]
        rot = Rotation.from_matrix(target_r @ frame["rotation"].T).as_rotvec() * 180 / math.pi
        residual = np.r_[pos, 3 * rot]
        # 仅离线冗余分支软引导；armTryWorlds 没有这个 seed 接口，不能继承为 live PASS。
        if joint_reference is not None:
            residual = np.r_[residual, .01 * (q - joint_reference)]
        return residual

    def jacobian(q):
        # PB link state 为浮点回读；相对差分在接近 0° 时会退化，使用绝对角差分。
        step = .01
        return np.column_stack([(error(q + np.eye(7)[j] * step) -
                                 error(q - np.eye(7)[j] * step)) / (2 * step) for j in range(7)])

    result = least_squares(error, np.clip(seed, *bounds), bounds=bounds, jac=jacobian, max_nfev=200)
    residual = error(result.x)
    if np.linalg.norm(residual[:3]) > .2 or np.linalg.norm(residual[3:6]) / 3 > .1:
        raise ValueError(f"后段 Worlds 模型 IK 不通过: {residual.tolist()}")
    return result.x, float(np.linalg.norm(residual[:3])), float(np.linalg.norm(residual[3:6]) / 3)


def interpolate(a, b):
    a, b = finite(a, 7), finite(b, 7)
    count = max(1, math.ceil(float(np.max(np.abs(b - a))) / REPLAY_STEP_DEG))
    return [a + (b - a) * t for t in np.linspace(0, 1, count + 1)[1:]]


def qualify_frames(robot, frames, limits):
    motions = [f for f in frames if "q_sdk_deg" in f]
    qs = np.asarray([f["q_sdk_deg"] for f in motions])
    if qs.ndim != 2 or qs.shape[1] != 7 or not np.isfinite(qs).all():
        raise ValueError("回放关节数组无效")
    margins = np.minimum(qs.min(0) - np.asarray(limits)[:, 0], np.asarray(limits)[:, 1] - qs.max(0))
    if np.min(margins) < d.ss.LIMIT_MARGIN_DEG - 1e-6:
        raise ValueError(f"全程关节限位余量不足: {margins.tolist()}")
    steps = np.max(np.abs(np.diff(qs, axis=0)), axis=1)
    if len(steps) and max(steps) > REPLAY_STEP_DEG + 1e-6:
        raise ValueError("回放密化不足")
    transfer = [f for f in motions if f["stage"] in ("PICK_ASCEND", "TRANSFER_MID", "PLACE_HOVER")]
    start = next(f for f in reversed(transfer) if f["stage"] == "PICK_ASCEND")
    relevant = [start] + [f for f in transfer if f["stage"] != "PICK_ASCEND"]
    probes = [d.probe(robot, f["q_sdk_deg"]) for f in relevant]
    grip = np.array([p["physical_grip"] for p in probes])
    rot0 = probes[0]["rotation"]
    tilts = [math.degrees(math.acos(float(np.clip((p["rotation"] @ rot0.T)[2, 2], -1, 1)))) for p in probes]
    return {"joint_limits_status": "PASS", "joint_used_ranges_deg": np.stack((qs.min(0), qs.max(0)), axis=1).tolist(),
            "minimum_margin_per_joint_deg": margins.tolist(), "motion_frame_count": len(motions),
            "max_dense_joint_step_deg": float(max(steps, default=0)),
            "transfer_grip_delta_xyz_min_mm": (grip - grip[0]).min(0).tolist(),
            "transfer_grip_delta_xyz_max_mm": (grip - grip[0]).max(0).tolist(),
            "transfer_y_monotone_negative": bool(np.all(np.diff(grip[:, 1]) <= .01)),
            "transfer_max_assumed_upright_bottle_tilt_deg": max(tilts),
            "bottle_assumption": "UPRIGHT_AT_ASCENT_RIGID_NO_SLIP_NOT_MEASURED"}


def self_mesh_diagnostic(robot, frames):
    """只排除 URDF 真正父子相邻，不用初始重叠建立白名单。"""
    adjacent = {tuple(sorted((j, d.p.getJointInfo(robot, j)[16]))) for j in range(d.p.getNumJoints(robot))}
    active = set(d.cc.ARM_JOINT_IDS["left"])
    pairs, total = {}, 0
    for frame in frames:
        if "q_sdk_deg" not in frame:
            continue
        total += 1
        d.probe(robot, frame["q_sdk_deg"])
        for row in d.p.getClosestPoints(robot, robot, 0):
            pair = tuple(sorted((int(row[3]), int(row[4]))))
            if pair[0] == pair[1] or pair in adjacent or not active.intersection(pair) or row[8] >= 0:
                continue
            entry = pairs.setdefault(pair, {"links": list(pair), "deepest_penetration_mm": 0., "stage": None})
            depth = -float(row[8]) * 1000
            if depth > entry["deepest_penetration_mm"]:
                entry.update(deepest_penetration_mm=depth, stage=frame["stage"])
    return {"status": "URDF_SELF_INTERSECTION_DETECTED" if pairs else "NO_INTERSECTION_IN_THIS_MODEL_ONLY",
            "frames_checked": total, "pairs": list(pairs.values()),
            "excluded_pairs": sorted([list(p) for p in adjacent]),
            "inactive_joints": "DEFAULT_ZERO_NOT_MEASURED", "gripper_bottle_scene_covered": False}


def load_success_reference(plan_path, run_path, executor_path):
    """只读取/解析证据；不 import 外来执行器，不把现场 PASS 升级为碰撞证明。"""
    plan = json.loads(Path(plan_path).read_text())
    run = json.loads(Path(run_path).read_text())
    if (plan.get("schema_version") != SCHEMA
            or run.get("schema_version") != "tabletop_hybrid_trial_run.v1"
            or run.get("status") != "PASS_RETURNED_SAFE"):
        raise ValueError("优化输入必须是 v1 混合计划及配套完整现场 PASS 记录")
    if plan.get("contract", {}).get("world_frame") != "sdk_world":
        raise ValueError("现场父计划没有明确 SDK world 坐标契约")
    if run.get("plan_sha256") != sha256_file(plan_path):
        raise ValueError("现场运行记录与原计划 SHA 不匹配")
    if run.get("executor_sha256") != sha256_file(executor_path):
        raise ValueError("现场运行记录与原执行器 SHA 不匹配")
    expected_names = [s["name"] for s in plan["stages"]]
    if [e["stage"] for e in run["events"]] != expected_names:
        raise ValueError("现场运行事件不完整或与计划段序不同")
    # 固定 Home 来自匹配的现场执行器，不另抄一组共享安全常量或旧 arm_profile home。
    values = []
    for node in ast.parse(Path(executor_path).read_text()).body:
        if isinstance(node, ast.Assign) and any(
                isinstance(t, ast.Name) and t.id == "SAFE_HOME_JOINTS_DEG" for t in node.targets):
            values.append(finite(ast.literal_eval(node.value), 7))
    if len(values) != 1:
        raise ValueError("现场执行器没有唯一可审计的 SAFE_HOME_JOINTS_DEG")
    home = values[0]
    final = finite(run["final_snapshot"]["joints"], 7)
    if np.max(np.abs(final - home)) > 1.0:
        raise ValueError("现场末次关节回读没有到达固定 Home")
    return plan, run, home


def retarget_endpoint_orientation(pose, uvw_deg):
    """保持已有 SDK 夹持点不变，姿态改变后重新反算 EE；不再施加 Model-A T。"""
    if pose.get("reference_point") != "EE_POSE":
        raise ValueError("本接口需要明确的 EE_POSE；物体坐标应先在上游解析")
    uvw = finite(uvw_deg, 3)
    old = finite(pose6(pose), 6)
    if np.array_equal(old[3:], uvw):
        return copy.deepcopy(pose)  # 原成功 EE 位姿零改动、零二次补偿。
    T_e_g, provenance = grasp_pose.sdk_endpoint_to_grasp_transform("left")
    R_old = grasp_pose.sdk_uvw_deg_to_matrix(old[3:])
    R_new = grasp_pose.sdk_uvw_deg_to_matrix(uvw)
    grip_sdk = old[:3] + R_old @ T_e_g[:3, 3]
    converted = pose_dict(np.r_[grip_sdk - R_new @ T_e_g[:3, 3], uvw])
    converted["source"] = "SDK_GRASP_CENTER_PRESERVED_ORIENTATION_RETARGET"
    converted["grasp_transform_provenance"] = provenance
    return converted


def settled_snapshot(event):
    snapshot = event["after"]
    finite(snapshot["joints"], 7)
    finite(snapshot["worlds"], 6)
    return snapshot


def endpoint_long_axis():
    """从已测夹持中心方向推导工具长轴，不能把 Euler U/V 都置零当作物品直立。"""
    T_e_g, provenance = grasp_pose.sdk_endpoint_to_grasp_transform("left")
    profile = d.arm_profiles.arm_profile("left")
    _, _, link_tool = grasp_pose.local_grasp_axes(profile)
    axis = T_e_g[:3, :3] @ link_tool
    return axis / np.linalg.norm(axis), provenance


def long_axis_tilt_deg(uvw):
    axis, _ = endpoint_long_axis()
    world_axis = grasp_pose.sdk_uvw_deg_to_matrix(uvw) @ axis
    return math.degrees(math.asin(float(np.clip(world_axis[2], -1, 1))))


def select_grasp_pose(robot, stage_map, event_map, offset, bounds, templates=None, horizontal_long_axis=False):
    """仅在配置模板内筛选姿态；没有物品约束时只复用实跑姿态，不猜水平/瓶轴。"""
    original = finite(stage_map["PICK_DESCEND"]["pose"]["sdk_world_uvw_deg"], 3)
    alternatives = [original.tolist()] if not templates else [finite(t, 3).tolist() for t in templates]
    if horizontal_long_axis:
        if templates:
            raise ValueError("长轴水平自动模板与手动姿态模板不能同时指定")
        axis, _ = endpoint_long_axis()
        if not np.allclose(axis, [1., 0., 0.], atol=1e-8, rtol=0):
            raise ValueError("工具长轴与 SDK endpoint +X 的关系已改变，需要重建水平模板")
        alternatives = [np.r_[original[0], v, original[2]].tolist() for v in (0., 5., 10., original[1])]
    if len(alternatives) > 25:
        raise ValueError("第一版姿态模板最多 25 个；不执行无界全姿态搜索")
    records, selected = [], None
    for index, uvw in enumerate(alternatives):
        record = {"template": index, "endpoint_uvw_deg": uvw,
                  "status": "REJECT", "scope": "ENDPOINT_IK_SCREEN_NOT_SWEPT_PATH_PASS"}
        poses, qs = {}, {}
        try:
            for name in ("PICK_HOVER", "PICK_DESCEND", "PICK_ASCEND"):
                poses[name] = retarget_endpoint_orientation(stage_map[name]["pose"], uvw)
                seed = finite(settled_snapshot(event_map[name])["joints"], 7)
                qs[name], pe, re = solve_worlds(robot, seed, pose6(poses[name]), offset, bounds, seed)
            margin = min(float(np.min(np.minimum(q - bounds[0], bounds[1] - q))) for q in qs.values())
            rotation_delta = np.linalg.norm(Rotation.from_matrix(
                grasp_pose.sdk_uvw_deg_to_matrix(uvw) @ grasp_pose.sdk_uvw_deg_to_matrix(original).T).as_rotvec())
            joint_change = sum(float(np.linalg.norm(qs[name] - settled_snapshot(event_map[name])["joints"])) for name in qs)
            # 此处只比较可达性/改动量，不把较小 U/V 误当夹爪水平。
            score = joint_change + math.degrees(rotation_delta) - .1 * margin
            if horizontal_long_axis:
                score += 100. * abs(long_axis_tilt_deg(uvw))
            record.update(status="ENDPOINT_IK_PASS_PENDING_DENSE_CHECK", score=score,
                          residual_margin_beyond_required_deg=margin,
                          long_axis_tilt_from_horizontal_deg=long_axis_tilt_deg(uvw))
            if selected is None or score < selected[0]:
                selected = (score, poses, qs, index)
        except ValueError as exc:
            record["reason"] = str(exc)
        records.append(record)
    if selected is None:
        raise ValueError(f"全部抓姿态模板的端点 IK 不可行: {records}")
    _, poses, qs, index = selected
    T_e_g, provenance = grasp_pose.sdk_endpoint_to_grasp_transform("left")
    return poses, qs, {
        "policy": "HORIZONTAL_LONG_AXIS_TEMPLATE_IK_SELECTION" if horizontal_long_axis else "CONFIGURED_TEMPLATE_IK_SELECTION" if templates else "FIELD_TESTED_POSE_PRESERVED",
        "independent_of_home": True, "templates": records, "selected_template": index,
        "selected_endpoint_uvw_deg": poses["PICK_DESCEND"]["sdk_world_uvw_deg"],
        "T_endpoint_grasp_mm": T_e_g.tolist(), "grasp_transform_provenance": provenance,
        "grip_center_uncertainty_mm": 10.0 if provenance["grip_center_status"] == "MEASURED_20260707_PLUS_MINUS_10MM" else None,
        "orientation_changed": not np.array_equal(finite(poses["PICK_DESCEND"]["sdk_world_uvw_deg"], 3), original),
        "level_gripper_definition": "TOOL_LONG_AXIS_LINK11_PLUS_Y_EQUALS_SDK_ENDPOINT_PLUS_X" if horizontal_long_axis else "NO_LEVEL_TARGET_REQUESTED",
        "selected_tool_long_axis_tilt_deg": long_axis_tilt_deg(poses["PICK_DESCEND"]["sdk_world_uvw_deg"]),
        "object_upright_axis": "NOT_SUPPLIED_NO_ACTUAL_BOTTLE_TILT_GUARANTEE",
        "note": "只有确认物品/夹爪允许姿态后才传模板；新姿态仍需密集扫掠、GUI 和实时 SDK 预检",
    }


def optimized_replay(robot, stages, run, offset, bounds, references):
    events = {e["stage"]: e for e in run["events"]}
    previous = finite(run["final_snapshot"]["joints"], 7)
    current_pose = finite(run["final_snapshot"]["worlds"], 6)
    frames = [{"stage": "RECORDED_FINAL_HOME_NOT_CURRENT_LIVE_START", "q_sdk_deg": previous.tolist(),
               "basis": "MEASURED_REFERENCE_NOT_CURRENT_ROBOT", "carrying": False}]
    metrics, max_pos, max_rot = [], 0., 0.
    for stage in stages:
        name, kind = stage["name"], stage["kind"]
        if kind == "GRIPPER":
            frames.append({"stage": name, "action": stage["action"], "basis": "VISUAL_EVENT_ONLY"})
            continue
        if kind == "ASSERT_ENDPOINT":
            continue
        count0 = len(frames)
        if kind == "MOVE_JOINTS":
            targets = [finite(stage["q_sdk_deg"], 7)]
            basis = "JOINT_LINEAR_ASSUMPTION_NOT_CONTROLLER_TRACE"
        else:
            targets, seed, start_q = [], previous.copy(), previous.copy()
            end_q = finite(references.get(name, settled_snapshot(events[name])["joints"]), 7)
            samples = densify_line(current_pose.tolist(), pose6(stage["pose"]))
            for i, pose in enumerate(samples, 1):
                reference = start_q + (end_q - start_q) * i / len(samples)
                q, pe, re = solve_worlds(robot, seed, pose, offset, bounds, reference)
                if np.max(np.abs(q - seed)) > MAX_JOINT_STEP_DEG:
                    raise ValueError(f"{name} 密集模型 IK 分支跳变")
                max_pos, max_rot = max(max_pos, pe), max(max_rot, re)
                targets.append(q)
                seed = q
            basis = "V2_READOUT_IK_RECONSTRUCTION_NOT_CONTROLLER_TRACE"
        for q in targets:
            for point in interpolate(previous, q):
                frames.append({"stage": name, "q_sdk_deg": point.tolist(), "basis": basis,
                               "carrying": stage["carrying"]})
            previous = q
        current_pose = predicted_worlds(robot, previous, offset) if kind == "MOVE_JOINTS" else np.asarray(pose6(stage["pose"]))
        metrics.append({"stage": name, "kind": kind, "dense_frame_count": len(frames) - count0})
    return frames, {"segments": metrics, "max_new_worlds_model_position_residual_mm": max_pos,
                    "max_new_worlds_model_orientation_residual_deg": max_rot}


def build_optimized(plan_path, run_path, executor_path, robot, templates=None, horizontal_long_axis=False,
                    placement_follow_grasp_tilt=False):
    if placement_follow_grasp_tilt and (not templates or horizontal_long_axis):
        raise ValueError("放置继承抓取U/V需要显式姿态模板，不能与长轴水平搜索混用")
    parent, run, home = load_success_reference(plan_path, run_path, executor_path)
    event_map = {e["stage"]: e for e in run["events"]}
    stage_map = {s["name"]: copy.deepcopy(s) for s in parent["stages"]}
    for stage in stage_map.values():
        for field in ("pose", "expected_endpoint"):
            if field in stage:
                # v1 父计划 Worlds API 的实跑输入已经是 SDK endpoint，而非相机物体坐标。
                stage[field].setdefault("reference_point", "EE_POSE")
    limits = d.ss.LIMITS_DEG[1]
    bounds = (np.asarray(limits)[:, 0] + d.ss.LIMIT_MARGIN_DEG,
              np.asarray(limits)[:, 1] - d.ss.LIMIT_MARGIN_DEG)
    offsets = np.asarray([finite(settled_snapshot(e)["worlds"], 6)[:3]
                          - d.probe(robot, settled_snapshot(e)["joints"])["sdk_v2_without_session_t"]
                          for e in run["events"] if "arrival_worlds" in e])
    offset = np.median(offsets, axis=0)
    spread = float(np.max(np.linalg.norm(offsets - offset, axis=1)))
    if spread > .5:
        raise ValueError(f"当前现场回读与 v2 解释模型不一致: {spread:.3f} mm")
    poses, references, selection = select_grasp_pose(robot, stage_map, event_map, offset, bounds, templates, horizontal_long_axis)
    field_hover_q = finite(stage_map["PLACE_HOVER"]["q_sdk_deg"], 7)
    new_hover_q = field_hover_q.copy()
    retarget_placement = horizontal_long_axis or placement_follow_grasp_tilt
    if retarget_placement:
        placement_templates = []
        for name in ("PLACE_HOVER", "PLACE_DESCEND", "PLACE_ASCEND"):
            old_pose = stage_map[name]["expected_endpoint" if name == "PLACE_HOVER" else "pose"]
            uvw = finite(old_pose["sdk_world_uvw_deg"], 3)
            if placement_follow_grasp_tilt:
                # [20260908] 现场认可的U/V延续到放置；保留原朝框W，不回退旧抓取倾角。
                uvw[:2] = selection["selected_endpoint_uvw_deg"][:2]
            else:
                uvw[1] = selection["selected_endpoint_uvw_deg"][1]
            poses[name] = retarget_endpoint_orientation(old_pose, uvw)
            seed = finite(settled_snapshot(event_map[name])["joints"], 7)
            references[name], pe, re = solve_worlds(robot, seed, pose6(poses[name]), offset, bounds, seed)
            placement_templates.append({"stage": name, "selected_endpoint_uvw_deg": uvw.tolist(),
                                         "tool_long_axis_tilt_deg": long_axis_tilt_deg(uvw),
                                         "grasp_center_policy": "PRESERVE_FIELD_SDK_GRASP_CENTER"})
        new_hover_q = references["PLACE_HOVER"]
        selection["placement_orientation_targets"] = placement_templates
    selection["placement_orientation_policy"] = (
        "FOLLOW_SELECTED_GRASP_UV_PRESERVE_FIELD_PLACEMENT_W" if placement_follow_grasp_tilt
        else "FOLLOW_SELECTED_LONG_AXIS_PITCH" if horizontal_long_axis
        else "PRESERVE_FIELD_PLACEMENT_ORIENTATION")
    selection["target_height_policy"] = "PRESERVE_FIELD_SDK_GRASP_TARGETS_NO_OBJECT_HEIGHT_INFERENCE"
    stages = []
    for original in parent["stages"]:
        stage = copy.deepcopy(original)
        if stage["name"] == "TRANSFER_MID":
            continue
        if stage["name"] in poses and stage["kind"] == "MOVE_WORLDS":
            stage["pose"] = poses[stage["name"]]
        if stage["name"] == "PLACE_HOVER" and retarget_placement:
            stage["q_sdk_deg"] = new_hover_q.tolist()
            stage["expected_endpoint"] = poses["PLACE_HOVER"]
        if stage["name"] == "RETURN_SAFE":
            stage = {"name": "RETURN_SAFE", "kind": "MOVE_JOINTS", "q_sdk_deg": home.tolist(),
                     "expected_endpoint": copy.deepcopy(stage["pose"]), "carrying": False,
                     "wait_until_joints": True, "handoff_check": "FIXED_SUCCESSFUL_HOME_JOINTS"}
        stages.append(stage)
    frames, replay_metrics = optimized_replay(robot, stages, run, offset, bounds, references)
    metrics = qualify_frames(robot, frames, limits)
    metrics.update(replay_metrics)
    qa = finite(next(f["q_sdk_deg"] for f in reversed(frames) if f["stage"] == "PICK_ASCEND"), 7)
    qb = new_hover_q
    metrics["direct_transfer"] = d.segment_diagnostic(robot, qa, qb, limits)
    metrics["transfer_joints_delta_deg"] = (qb - qa).tolist()
    if not metrics["transfer_y_monotone_negative"]:
        raise ValueError("单段 MoveJ 模型路径出现 Y 正向回摆，拒绝自动采用")
    # 诊断包络来自现场原两段模型路径并留出报告位置误差；不是桌面净空阈值。
    metrics["direct_transfer_clearance_status"] = "NO_SCENE_CLEARANCE_CLAIM"
    qdesc = next(f["q_sdk_deg"] for f in reversed(frames) if f["stage"] == "PLACE_DESCEND")
    metrics["replayed_place_grip_delta_from_recorded_arrival_mm"] = (
        d.probe(robot, qdesc)["physical_grip"]
        - d.probe(robot, settled_snapshot(event_map["PLACE_DESCEND"])["joints"])["physical_grip"]).tolist()
    metrics["placement_endpoint_policy"] = ("SDK_GRASP_CENTER_PRESERVED_CONFIGURED_UV_FIELD_W" if placement_follow_grasp_tilt
                                             else "SDK_GRASP_CENTER_PRESERVED_NEW_HORIZONTAL_ORIENTATION" if horizontal_long_axis
                                             else "ORIGINAL_FIELD_TESTED_EE_TARGET_UNCHANGED")
    transfer_qs = [qa] + [finite(f["q_sdk_deg"], 7) for f in frames if f["stage"] == "PLACE_HOVER"]
    tool_tilts = [long_axis_tilt_deg(predicted_worlds(robot, q, offset)[3:]) for q in transfer_qs]
    metrics["transfer_tool_long_axis_tilt_range_deg"] = [min(tool_tilts), max(tool_tilts)]
    metrics["sdk_grasp_center_preservation_error_mm"] = {}
    T_e_g, _ = grasp_pose.sdk_endpoint_to_grasp_transform("left")
    for name, new_pose in poses.items():
        source = stage_map[name]["expected_endpoint" if name == "PLACE_HOVER" else "pose"]
        old_grip = np.asarray(source["position_mm"]) + grasp_pose.sdk_uvw_deg_to_matrix(source["sdk_world_uvw_deg"]) @ T_e_g[:3, 3]
        new_grip = np.asarray(new_pose["position_mm"]) + grasp_pose.sdk_uvw_deg_to_matrix(new_pose["sdk_world_uvw_deg"]) @ T_e_g[:3, 3]
        metrics["sdk_grasp_center_preservation_error_mm"][name] = float(np.linalg.norm(new_grip - old_grip))
    metrics["sdk_vs_pb_physical_grasp_warning"] = (
        "SDK transform preserves intended grasp center, but v2 includes pose-dependent forearm correction; "
        "relative PB grip drift is reported separately and is NOT a physical-scene calibration PASS")
    regressions = []
    for path in sorted(Path(run_path).parent.glob("hybrid_trial_run_*.json")):
        _, reference_run, _ = load_success_reference(plan_path, path, executor_path)
        reference_event = next(e for e in reference_run["events"] if e["stage"] == "PICK_ASCEND")
        start = finite(settled_snapshot(reference_event)["joints"], 7)
        diagnostic = d.segment_diagnostic(robot, start, field_hover_q, limits)
        sampled = [start] + interpolate(start, field_hover_q)
        xyz = np.asarray([d.probe(robot, q)["physical_grip"] for q in sampled])
        diagnostic["transfer_y_monotone_negative"] = bool(np.all(np.diff(xyz[:, 1]) <= .01))
        regressions.append({"pass_log_sha256": sha256_file(path), "filename": path.name,
                            "start_source": "PICK_ASCEND.after.joints_SETTLED",
                            "scope": "BASELINE_SINGLE_MOVEJ_WITH_ORIGINAL_GRASP_AND_PLACEMENT_NOT_NEW_HORIZONTAL_POSE",
                            "diagnostic": diagnostic})
    metrics["field_start_direct_transfer_regression"] = regressions
    metrics["prior_two_movej_model_max_drop_mm"] = abs(parent["qualification"]["kinematics"]["transfer_grip_delta_xyz_min_mm"][2])
    metrics["single_movej_height_tradeoff"] = "FEWER_STOPS_NOT_STRICTLY_HORIZONTAL_SEE_MEASURED_MODEL_Z_RANGE"
    calibration_proc = subprocess.run([sys.executable, "-B", str(WORK / "frame_calibration/analysis/verify_worlds_record.py")],
                                     cwd=WORK, text=True, capture_output=True)
    calibration = json.loads(calibration_proc.stdout)
    plan = copy.deepcopy(parent)
    plan.update(schema_version=OPTIMIZED_SCHEMA, plan_id="tabletop_hybrid_single_movej_20260908",
                home_joints_sdk_deg=home.tolist(), stages=stages, pose_selection=selection,
                reviewed_start=copy.deepcopy(run["final_snapshot"]),
                expected_pick_ascent_joints_sdk_deg=qa.tolist(),
                offline_replay={"joint_step_deg": REPLAY_STEP_DEG, "worlds_sample_mm": SAMPLE_MM,
                                "worlds_sample_deg": SAMPLE_DEG, "frames": frames,
                                "display_period_s": .02, "display_duration_not_robot_timing": True})
    if not np.array_equal(finite(plan["reviewed_start"]["joints"], 7), finite(frames[0]["q_sdk_deg"], 7)):
        raise ValueError("GUI 审阅起始关节必须与绑定现场快照完全一致")
    finite(plan["reviewed_start"]["worlds"], 6)
    plan["contract"].update(required_executor="execute_tabletop_hybrid_trial.py_WITH_V2_SUPPORT",
                            runtime_authorization="EXPLICIT_STAGED_TRIAL_WITH_GUI_AND_LIVE_PRECHECK",
                            return_home_policy="EXPLICIT_FIXED_JOINTS_NO_EXECUTOR_HIDDEN_OVERRIDE")
    plan["policy"].update(startup="LIVE_START_RECHECK_AND_OPERATOR_APPROVAL_NOT_SAFE_RADIUS",
                          worlds_arrival_at_ascent="LIVE_JOINTS_TO_NEXT_MOVEJ_RECHECK_BRANCH_DIFFERENCE_DIAGNOSTIC",
                          transfer_policy="ONE_MOVEJ_NO_INTERMEDIATE_STOP_NO_QUEUE_BLEND_ASSUMPTION")
    plan["gripper_policy"]["executor_binding"] = "execute_tabletop_hybrid_trial.py.v2"
    plan["provenance"].update(parent_plan_sha256=sha256_file(plan_path), field_pass_log_sha256=sha256_file(run_path),
                              field_executor_sha256=sha256_file(executor_path), generator_sha256=sha256_file(__file__),
                              field_home_source="MATCHED_EXECUTOR_AST_SAFE_HOME_JOINTS_DEG",
                              field_pass_filename=Path(run_path).name,
                              grasp_pose_sha256=sha256_file(Path(grasp_pose.__file__)),
                              arm_profiles_sha256=sha256_file(Path(d.arm_profiles.__file__)))
    plan["readout_model_diagnostic"] = {"scope": "V2_EXPLANATION_ONLY_NOT_PHYSICAL_SCENE_TRANSFORM",
                                         "report_local_offset_mm": offset.tolist(),
                                         "max_report_readout_residual_mm": spread}
    plan["qualification"].update(kinematics=metrics, calibration_gate=calibration,
                                 self_collision=self_mesh_diagnostic(robot, frames), gui_review="PENDING",
                                 controller_mixed_path_precheck="NOT_RUN_FOR_THIS_V2_PLAN")
    plan["motion_qualification"] = {
        "status": "BLOCKED", "geometry_status": "UNREGISTERED_SCENE",
        "calibration_status": "BLOCKED_MODEL_A_NOT_QUALIFIED",
        "dense_collision_status": "NOT_QUALIFIED_FULL_SCENE",
        "controller_interpolation_status": "JOINT_LINEAR_ASSUMPTION_NOT_TRACE",
        "evidence": {"field_parent_plan_sha256": sha256_file(plan_path),
                     "field_parent_pass_log_sha256": sha256_file(run_path),
                     "inheritance": "PARENT_PASS_DOES_NOT_QUALIFY_CHANGED_SINGLE_MOVEJ"}}
    plan["blockers"] = [
        "New single-MoveJ interpolation is a model assumption, not a controller trace or prior run PASS",
        "Full table/bin/wrist/gripper/held-object swept clearance is not qualified; Model-A gate pending",
        "Exact-candidate dense GUI human review and current actual-start controller precheck required",
        "Grasp geometry has measured +/-10mm uncertainty; bottle attachment/upright axis and jaw-plane roll not measured",
        "New startup region needs operator scene approval; an EE point above the table is not whole-arm clearance",
    ]
    return plan


def build(report_path, robot):
    report = json.loads(Path(report_path).read_text())
    groups = unpack_report(report)
    parent = report["plan_content"]
    limits = d.ss.intersect_axis_limits(report["precheck_summary"]["controller_limits_deg"], d.ss.LIMITS_DEG[1])
    bounds = (np.asarray(limits)[:, 0] + d.ss.LIMIT_MARGIN_DEG, np.asarray(limits)[:, 1] - d.ss.LIMIT_MARGIN_DEG)
    rows = report["dense_ik_samples"]
    offsets = np.array([np.asarray(r["worlds_xyzuvw"][:3]) - d.probe(robot, r["q_sdk_deg"])["sdk_v2_without_session_t"] for r in rows])
    # 不写 calib_common，不使用/替换 Model-A T；只对当前报告做回读一致性说明。
    offset = np.median(offsets, axis=0)
    spread = float(np.max(np.linalg.norm(offsets - offset, axis=1)))
    if spread > .5:
        raise ValueError(f"当前报告与v2解释模型不符: {spread:.3f} mm")
    a = finite(report["endpoint_ik"]["PICK_ASCEND"], 7)
    old_end = finite(report["endpoint_ik"]["PLACE_HOVER"], 7)
    pa, old = d.probe(robot, a), d.probe(robot, old_end)
    end = solve_grip(robot, old_end, old["physical_grip"] + [0, -.1, 0], pa["rotation"], bounds,
                     joint_reference=a, joint_weights=[.08, .08, .08, .08, .08, 0, .08], prefer_j6=-25)
    pe = d.probe(robot, end)
    half_rot = Rotation.from_rotvec(.5 * Rotation.from_matrix(pe["rotation"] @ pa["rotation"].T).as_rotvec()).as_matrix() @ pa["rotation"]
    mid = solve_grip(robot, (a + end) / 2, (pa["physical_grip"] + pe["physical_grip"]) / 2, half_rot, bounds)
    # 将新的朝向延续到下降；目标是真实夹持点的旧落点，不是把旧 EE 再补偿一次。
    old_desc = d.probe(robot, report["endpoint_ik"]["PLACE_DESCEND"])
    qdesc, desc_ik = d.ik.solve_full_pose(robot, (old_desc["physical_grip"] + [0, -.1, 0]) / 1000,
                         pe["rotation"], end, d.ik._solver_bounds_from_limits(limits, d.ss.LIMIT_MARGIN_DEG),
                         iterations=500, position_tolerance_mm=.1, rotation_tolerance_deg=.1)
    if qdesc is None:
        raise ValueError(f"新朝向下放置 IK 不通过: {desc_ik}")
    hover_pose, descend_pose = predicted_worlds(robot, end, offset), predicted_worlds(robot, qdesc, offset)
    stages = copy.deepcopy(parent["stages"])
    new = []
    for stage in stages:
        if stage["name"] == "PLACE_HOVER":
            for name, q in (("TRANSFER_MID", mid), ("PLACE_HOVER", end)):
                new.append({"name": name, "kind": "MOVE_JOINTS", "q_sdk_deg": q.tolist(),
                            "expected_endpoint": pose_dict(predicted_worlds(robot, q, offset)),
                            "carrying": True, "wait_until_joints": True,
                            "handoff_check": "ACTUAL_JOINTS_AND_WORLDS_REQUIRED_BEFORE_DESCENT"})
        else:
            if stage["name"] in ("PLACE_DESCEND", "PLACE_ASCEND"):
                stage["pose"] = pose_dict(descend_pose if stage["name"] == "PLACE_DESCEND" else hover_pose)
            new.append(stage)
    frames = [{"stage": "CAPTURED_LIVE_START_NOT_ASSERTED_SAFE", "q_sdk_deg": report["live_start"]["joints"],
               "basis": "LIVE_CAPTURE_NOT_CURRENT_ROBOT"}]
    previous_q = finite(report["live_start"]["joints"], 7)
    current_pose = finite(report["live_start"]["worlds"], 6)
    max_pos, max_rot, segment_metrics = 0., 0., []
    for stage in new:
        name, kind = stage["name"], stage["kind"]
        if kind == "GRIPPER":
            frames.append({"stage": name, "action": stage["action"], "basis": "VISUAL_EVENT_ONLY"})
            continue
        if kind == "ASSERT_ENDPOINT":
            continue
        count0 = len(frames)
        if kind == "MOVE_JOINTS":
            targets = [finite(stage["q_sdk_deg"], 7)]
            basis = "JOINT_LINEAR_ASSUMPTION_NOT_CONTROLLER_TRACE"
        elif name in ("PICK_HOVER", "PICK_DESCEND", "PICK_ASCEND"):
            targets = [finite(row["q_sdk_deg"], 7) for row in groups[name]]
            basis = "SDK_IK_PREDICTED_NOT_MEASURED_ARRIVAL"
        else:
            targets, seed = [], previous_q.copy()
            reference_end = np.asarray(qdesc if name == "PLACE_DESCEND" else end if name == "PLACE_ASCEND"
                                       else report["endpoint_ik"]["RETURN_SAFE"])
            reference_start = seed.copy()
            poses = densify_line(current_pose.tolist(), pose6(stage["pose"]))
            for sample, pose in enumerate(poses, 1):
                try:
                    reference = reference_start + (reference_end - reference_start) * sample / len(poses)
                    q, poserr, roterr = solve_worlds(robot, seed, pose, offset, bounds, reference)
                except ValueError as exc:
                    raise ValueError(f"{name} sample {sample}: {exc}") from exc
                max_pos, max_rot = max(max_pos, poserr), max(max_rot, roterr)
                if np.max(np.abs(q - seed)) > MAX_JOINT_STEP_DEG:
                    raise ValueError(f"{name} 模型 IK 分支跳变")
                targets.append(q)
                seed = q
            basis = "V2_MODEL_IK_SOFT_BRANCH_GUIDANCE_NOT_CONTROLLER_IK"
        for q in targets:
            if kind == "MOVE_WORLDS" and np.max(np.abs(q - previous_q)) > MAX_JOINT_STEP_DEG:
                raise ValueError(f"{name} 原 IK 分支跳变")
            for point in interpolate(previous_q, q):
                frames.append({"stage": name, "q_sdk_deg": point.tolist(), "basis": basis, "carrying": stage["carrying"]})
            previous_q = q
        current_pose = (predicted_worlds(robot, previous_q, offset) if kind == "MOVE_JOINTS"
                        else np.asarray(pose6(stage["pose"])))
        segment_metrics.append({"stage": name, "kind": kind, "dense_frame_count": len(frames) - count0})
    metrics = qualify_frames(robot, frames, limits)
    reached = {name: d.probe(robot, next(f["q_sdk_deg"] for f in reversed(frames) if f["stage"] == name))
               for name in ("PLACE_DESCEND", "PLACE_ASCEND")}
    place_delta = reached["PLACE_DESCEND"]["physical_grip"] - old_desc["physical_grip"]
    if place_delta[1] > 0:
        raise ValueError(f"实际模型回放落点偏到旧放置点 Y 正侧: {place_delta.tolist()}")
    metrics.update({"segments": segment_metrics, "max_new_worlds_model_position_residual_mm": max_pos,
                    "max_new_worlds_model_orientation_residual_deg": max_rot,
                    "transfer_joints_delta_deg": (end - a).tolist(),
                    "new_hover_grip_delta_from_original_mm": (pe["physical_grip"] - old["physical_grip"]).tolist(),
                    "new_place_grip_delta_from_original_mm": place_delta.tolist(),
                    "place_ascend_grip_return_error_mm": (reached["PLACE_ASCEND"]["physical_grip"] - pe["physical_grip"]).tolist()})
    self_diagnostic = self_mesh_diagnostic(robot, frames)
    gate = subprocess.run([sys.executable, "-B", str(WORK / "frame_calibration/analysis/verify_worlds_record.py")],
                          cwd=WORK, text=True, capture_output=True)
    calibration = json.loads(gate.stdout)
    gripper_policy = copy.deepcopy(parent["gripper_policy"])
    gripper_policy["parent_source_sha256"] = gripper_policy.pop("source_sha256", None)
    gripper_policy["executor_binding"] = "MIXED_CONSUMER_BINDING_PENDING_NOT_WORLDS_V1"
    return {"schema_version": SCHEMA, "status": "OFFLINE_CANDIDATE_BLOCKED", "real_motion_authorized": False,
            "debian_execution_allowed": False, "plan_id": "tabletop_hybrid_j6_preferred_20260907",
            "meta": {"arm": "left", "arm_id": 1, "gripper_id": 2},
            "contract": {**parent["contract"], "dynamic_transport": "MOVE_WORLDS_AND_MOVE_JOINTS",
                         "move_j_policy": "ALL_SEVEN_ABSOLUTE_JOINTS_WRIST_SOFT_PREFERENCE",
                         "runtime_authorization": "NONE_OFFLINE_CANDIDATE_ONLY",
                         "required_executor": "NEW_MIXED_CONSUMER_REQUIRED_NOT_EXISTING_WORLDS_V1"},
            "provenance": {"precheck_report_sha256": sha256_file(report_path),
                           "parent_plan_sha256": report["plan_sha256"],
                           "parent_sources": report["source_hashes"],
                           "generator_sha256": sha256_file(__file__),
                           "model_sources": {str(path.relative_to(WORK)): sha256_file(path) for path in
                            (Path(d.cc.__file__), Path(d.ik.__file__), Path(d.__file__), Path(d.ss.__file__), Path(d.cc.DEFAULT_URDF))}},
            "policy": {"fixed_joints": [], "j6_preferred_deg": -25, "preference_is_not_hard_constraint": True,
                       "placement_y_reference": "ORIGINAL_PHYSICAL_PLACE_HOVER", "placement_y_positive_allowed": False,
                       "joint_margin_deg": d.ss.LIMIT_MARGIN_DEG,
                       "startup": "RECHECK_CURRENT_POSE_AND_BRANCH_NO_BLIND_RECOVERY",
                       "worlds_arrival_at_ascent": "MUST_MATCH_EXPECTED_JOINT_BRANCH_BEFORE_MOVE_J"},
            "expected_pick_ascent_joints_sdk_deg": a.tolist(),
            "input_reference_only": copy.deepcopy(parent["input"]),
            "gripper_policy": gripper_policy,
            "stages": new,
            "offline_replay": {"joint_step_deg": REPLAY_STEP_DEG, "worlds_sample_mm": SAMPLE_MM,
                               "worlds_sample_deg": SAMPLE_DEG, "frames": frames,
                               "display_period_s": .02, "display_duration_not_robot_timing": True},
            "qualification": {"verdict": "REJECT_FOR_HARDWARE_PENDING_QUALIFICATION", "kinematics": metrics,
                              "calibration_gate": calibration, "gui_review": "PENDING",
                              "self_collision": self_diagnostic, "robot_environment_collision": "NOT_QUALIFIED",
                              "wrist_gripper_held_object_collision": "NOT_QUALIFIED",
                              "controller_mixed_path_precheck": "NOT_RUN", "robot_duration_seconds": None},
            "readout_model_diagnostic": {"scope": "V2_EXPLANATION_ONLY_NOT_PHYSICAL_SCENE_TRANSFORM",
                                         "report_local_offset_mm": offset.tolist(),
                                         "max_report_readout_residual_mm": spread},
            "blockers": ["Physical table/bin scene not registered; Model-A gate blocked",
                         "Full wrist/gripper/held-object swept collisions not qualified",
                         "GUI completion is not human review or exact controller interpolation proof",
                         "Existing Worlds-only executor rejects mixed schema; new tested consumer required",
                         "New post-transfer Worlds IK uses offline soft branch guidance unavailable in armTryWorlds",
                         "Actual start and PICK_ASCEND redundant branch must be rechecked before motion"]}


def replay_frames(plan):
    return plan["waypoints"] if plan.get("schema_version") == servo_contract.SCHEMA else plan["offline_replay"]["frames"]


def build_servo(parent_path, robot):
    plan = servo_contract.build(parent_path)
    frames = plan["waypoints"]
    plan["offline_replay"] = {"joint_step_deg": plan["stream_policy"]["max_step_deg"],
                              "display_period_s": plan["stream_policy"]["period_s"],
                              "scope": "EXACT_EXPORTED_SERVO_SAMPLES_NO_EXTRA_INTERPOLATION"}
    plan["qualification"]["kinematics"] = qualify_frames(robot, frames, servo_contract.limits())
    plan["qualification"]["self_collision"] = self_mesh_diagnostic(robot, frames)
    # 缺实际桌框配准/持物扫掠时如实阻断，不继承父 MoveWorlds/MoveJ 的运动验收。
    plan["qualification"]["robot_environment_collision"] = "UNREGISTERED_CURRENT_SCENE"
    plan["qualification"]["wrist_gripper_held_object_collision"] = "NOT_QUALIFIED"
    return plan


def replay(robot, plan):
    """显示整条候选，不伪造桌子/框绝对位置，不写 GUI PASS。"""
    frames = replay_frames(plan)
    pick_q = next(f["q_sdk_deg"] for f in reversed(frames) if f["stage"] == "PICK_DESCEND")
    pick_frame = d.probe(robot, pick_q)
    grip_positions = [d.probe(robot, f["q_sdk_deg"])["physical_grip"] / 1000 for f in frames if "q_sdk_deg" in f]
    for a, b in zip(grip_positions[::8], grip_positions[8::8]):
        d.p.addUserDebugLine(a.tolist(), b.tolist(), [.1, .65, .25], 2)
    d.p.resetDebugVisualizerCamera(1.5, 115, -22, [.35, .1, 1.0])
    d.p.configureDebugVisualizer(d.p.COV_ENABLE_GUI, 0)
    d.p.addUserDebugText("KINEMATIC REPLAY - NOT A COLLISION / HARDWARE PASS", [.05, .1, 1.65], [1, .2, .05], 1.1)
    d.p.addUserDebugText("Object marker only; table/bin registration pending", [.05, .1, 1.58], [.15, .15, .15], 1)
    marker = d.p.createMultiBody(0, -1, d.p.createVisualShape(d.p.GEOM_SPHERE, radius=.025, rgbaColor=[.1, .65, 1, 1]),
                                basePosition=(pick_frame["physical_grip"] / 1000).tolist())
    held, stage_text, previous_stage = False, -1, None
    played = 0
    for f in frames:
        if not d.p.isConnected():
            return {"status": "INTERRUPTED", "played_motion_frames": played}
        if f["stage"] != previous_stage:
            print("GUI", f["stage"], flush=True)
            stage_text = d.p.addUserDebugText(f["stage"], [.05, .1, 1.48], [.1, .15, .7], 1.3, replaceItemUniqueId=stage_text)
            previous_stage = f["stage"]
        if "action" in f:
            held = f["action"] == "close"
            time.sleep(f.get("settle_s", .5))
            continue
        state = d.probe(robot, f["q_sdk_deg"])
        if held:
            d.p.resetBasePositionAndOrientation(marker, (state["physical_grip"] / 1000).tolist(), [0, 0, 0, 1])
        played += 1
        time.sleep(plan["offline_replay"]["display_period_s"])
    return {"status": "PLAYBACK_COMPLETED_NOT_HUMAN_REVIEW", "played_motion_frames": played,
            "human_gui_review": "PENDING", "real_motion_authorized": False}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--servo-parent", type=Path, help="按已收到的最终 A 关节折线生成 Servo 候选，不覆盖 A")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--optimize", action="store_true", help="生成 v2 单段转运；不覆盖 v1 成功候选")
    parser.add_argument("--parent-plan", type=Path, help="现场原成功 v1 JSON")
    parser.add_argument("--success-log", type=Path, help="与原 JSON/执行器 SHA 匹配的完整 PASS 日志")
    parser.add_argument("--field-executor", type=Path, help="只解析现场执行器 Home 字面量，绝不 import")
    parser.add_argument("--pose-template", nargs=3, type=float, action="append", metavar=("U", "V", "W"),
                        help="现场确认可用的抓姿态模板，可重复；不填则保持实跑抓姿态")
    parser.add_argument("--horizontal-long-axis", action="store_true",
                        help="搜索工具长轴近水平抓放姿态；不是瓶子直立或碰撞PASS")
    parser.add_argument("--placement-follow-grasp-tilt", action="store_true",
                        help="放置继承显式姿态模板的U/V，保留原朝框W；保持SDK夹持目标而非旧EE坐标")
    parser.add_argument("--replay", type=Path)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--gui-report", type=Path, help="可选：保存绑定候选SHA的播放完成记录，不是人工GUI PASS")
    parser.add_argument("--gui-review", type=Path, help="完整播放后终端人工确认，生成绑定轨迹SHA的GUI审阅记录")
    parser.add_argument("--reviewer", help="人工GUI审阅人；需配合--gui-review")
    args = parser.parse_args(argv)
    if args.replay is None and args.report is None and not args.optimize and args.servo_parent is None:
        parser.error("生成需 --report；回放需 --replay")
    if args.replay is not None and not args.gui:
        parser.error("--replay 需 --gui")
    if args.servo_parent and (args.replay or args.report or args.optimize or args.pose_template or args.horizontal_long_axis):
        parser.error("--servo-parent 独立使用，不与重新求解/改姿态选项混用")
    if args.optimize and (args.replay or not all((args.parent_plan, args.success_log, args.field_executor))):
        parser.error("--optimize 需要 --parent-plan --success-log --field-executor，不能与 --replay 混用")
    if args.pose_template and not args.optimize:
        parser.error("--pose-template 只用于 --optimize")
    if args.horizontal_long_axis and (not args.optimize or args.pose_template):
        parser.error("--horizontal-long-axis 只用于 --optimize，不能同时提供 --pose-template")
    if args.placement_follow_grasp_tilt and (not args.optimize or not args.pose_template or args.horizontal_long_axis):
        parser.error("--placement-follow-grasp-tilt 需要 --optimize --pose-template，不能与长轴水平搜索混用")
    if args.gui_review and (not args.gui or not args.reviewer or not args.reviewer.strip()):
        parser.error("--gui-review 需要 --gui 及非空 --reviewer")
    robot = d.xf.load_xifeng(gui=args.gui)
    try:
        if args.replay:
            plan = json.loads(args.replay.read_text())
            if plan.get("schema_version") not in (SCHEMA, OPTIMIZED_SCHEMA, servo_contract.SCHEMA) or plan.get("real_motion_authorized") is not False:
                raise ValueError("不是本脚本的离线候选")
            if plan.get("schema_version") == servo_contract.SCHEMA:
                servo_contract.validate(plan)
            output = args.replay
        else:
            plan = (build_servo(args.servo_parent.resolve(), robot) if args.servo_parent else
                    build_optimized(args.parent_plan.resolve(), args.success_log.resolve(),
                                    args.field_executor.resolve(), robot, args.pose_template, args.horizontal_long_axis,
                                    args.placement_follow_grasp_tilt)
                    if args.optimize else build(args.report.resolve(), robot))
            output = (args.out or (SERVO_OUTPUT if args.servo_parent else OPTIMIZED_OUTPUT if args.optimize else DEFAULT_OUTPUT)).resolve()
            output.relative_to(WORK)
            if args.optimize and output == args.parent_plan.resolve():
                raise ValueError("优化输出不能覆盖原成功 JSON")
            if args.servo_parent and (output == args.servo_parent.resolve() or output.name in (DEFAULT_OUTPUT.name, OPTIMIZED_OUTPUT.name)):
                raise ValueError("Servo 候选不能覆盖 A 的原件/现有候选")
            output.write_text(json.dumps(plan, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
        print(json.dumps({"file": str(output.resolve()), "sha256": sha256_file(output),
                          "status": plan["status"], "kinematics": plan["qualification"]["kinematics"]}, ensure_ascii=False, indent=2), flush=True)
        if args.gui:
            gui_result = {"schema_version": "pybullet_gui_playback.v1", "candidate_sha256": sha256_file(output),
                          "player_sha256": sha256_file(__file__), "gui": replay(robot, plan)}
            if args.gui_report:
                gui_output = args.gui_report.resolve()
                gui_output.relative_to(WORK)
                if gui_output == output.resolve():
                    raise ValueError("GUI记录不能覆盖候选轨迹")
                gui_output.write_text(json.dumps(gui_result, ensure_ascii=False, indent=2) + "\n")
            print(json.dumps(gui_result, ensure_ascii=False), flush=True)
            if args.gui_review:
                review_path = args.gui_review.resolve()
                review_path.relative_to(WORK)
                if review_path == output.resolve() or (args.gui_report and review_path == args.gui_report.resolve()):
                    raise ValueError("GUI审阅记录不能覆盖轨迹/播放记录")
                expected_frames = sum("q_sdk_deg" in f for f in replay_frames(plan))
                complete = (gui_result["gui"]["status"] == "PLAYBACK_COMPLETED_NOT_HUMAN_REVIEW"
                            and gui_result["gui"]["played_motion_frames"] == expected_frames)
                if not complete or not sys.stdin.isatty():
                    raise ValueError("未完整播放或非交互终端，拒绝生成 GUI PASS")
                answer = input("已完整观察上述轨迹？仅确认画面/运动符合预期请输入 PASS，否则输入 FAIL：").strip()
                if answer not in ("PASS", "FAIL"):
                    raise ValueError("没有明确人工 PASS/FAIL 答复，不写 GUI 审阅记录")
                review = {"schema_version": "pybullet_gui_review.v1", "result": answer,
                          "trajectory_sha256": sha256_file(output), "reviewer": args.reviewer.strip(),
                          "arm_id": plan["meta"]["arm_id"], "dense_step_deg": plan["offline_replay"]["joint_step_deg"],
                          "frame_count": expected_frames,
                          "completed_stages": list(dict.fromkeys(f["stage"] for f in replay_frames(plan))),
                          "reviewed_motion_stages": [s["name"] for s in plan.get("stages", plan.get("source_stages", [])) if s["kind"].startswith("MOVE_")],
                          "full_replay_completed": True,
                          "player_sha256": sha256_file(__file__), "reviewed_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                          "model_limitation": "GUI checks reconstructed kinematics only; unregistered table/bin/gripper/held-object and controller interpolation remain unqualified",
                          "real_motion_authorized": False}
                review_path.write_text(json.dumps(review, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
                print(json.dumps({"gui_review_file": str(review_path), "result": answer}, ensure_ascii=False), flush=True)
        return 0  # 文件生成成功，不代表硬件资格通过。
    finally:
        if d.p.isConnected():
            d.p.disconnect()


if __name__ == "__main__":
    raise SystemExit(main())
