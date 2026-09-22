#!/usr/bin/env python3
"""在 SDK world 中核对桌上视觉快照（场景预览，不是机器人轨迹回放）。

默认只画相机/用户已经给出的 SDK-world 数据、当前 endpoint 姿态下的夹爪
保守包络，以及在“料框合并点是外形中心”的预览假设下计算出的框口高度。
``--robot-overlay`` 可加载机器人 URDF，并用同一组 safe Worlds/Joints 做单姿态
显示锚定；它不使用旧 PB<->SDK 会话 T，也不把单点锚定冒充标定或路径证明。

只有关节轨迹、完整碰撞预检和哈希绑定的 GUI 人工记录全部通过后，才能使用
``demo_bottle_trajectory.py`` 做正式轨迹审查；本脚本永远不会生成 PASS 记录。
"""
from __future__ import annotations

import argparse
import datetime as dt
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
from frame_calibration.analysis import calib_common
import xifeng_pb as xf
from robot_mission.contracts import load_and_validate, sha256_file
from robot_mission.grasp_pose import (
    matrix_to_quaternion_xyzw,
    quaternion_xyzw_to_matrix,
)


PREVIEW_ASSUMPTIONS = (
    "material_box.processed_pose is treated as outer geometry center for display only",
    "material_box.dimensions_mm order is treated as local X/Y/Z for display only",
    "prior 70x70x200mm bottle envelope is centered on the unresolved visual feature for display only",
    "no SDK-world to PyBullet robot-frame transform is applied",
)

PRIOR_BOTTLE_TASK = HERE / "tasks" / "task_bottle_taught_tcp_20260831.json"


def _pose(item: dict) -> tuple[np.ndarray, np.ndarray, list[float]]:
    position = np.asarray(item["position_mm"], dtype=float) / 1000.0
    quaternion = [float(v) for v in item["quaternion_xyzw"]]
    return position, quaternion_xyzw_to_matrix(quaternion), quaternion


def _z_extents(center: np.ndarray, rotation: np.ndarray,
               dimensions_m: np.ndarray) -> tuple[float, float]:
    half_z_projection = float(np.abs(rotation[2]) @ (dimensions_m / 2.0))
    return float(center[2] - half_z_projection), float(center[2] + half_z_projection)


def _rotation_delta_deg(left: np.ndarray, right: np.ndarray) -> float:
    cosine = float(np.clip((np.trace(left.T @ right) - 1.0) / 2.0, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def _prior_bottle_envelope_m() -> tuple[np.ndarray, dict]:
    task = json.loads(PRIOR_BOTTLE_TASK.read_text(encoding="utf-8"))
    scene = task["scene"]
    radius = float(scene["bottle_radius_m"])
    dimensions = np.array([2.0 * radius, 2.0 * radius,
                           float(scene["bottle_height_m"])])
    return dimensions, {
        "path": str(PRIOR_BOTTLE_TASK.resolve()),
        "sha256": sha256_file(PRIOR_BOTTLE_TASK),
        "status": "PRIOR_WORKSPACE_ENVELOPE_FOR_DISPLAY_ONLY_NOT_CURRENT_OBJECT_EVIDENCE",
    }


def analyze_preview(snapshot: dict) -> dict:
    bottle_position, bottle_rotation, _ = _pose(snapshot["bottle"]["processed_pose"])
    box_position, box_rotation, _ = _pose(snapshot["material_box"]["processed_pose"])
    safe_position, safe_rotation, _ = _pose(snapshot["safe_candidate"]["pose"])
    box_dimensions = np.asarray(snapshot["material_box"]["dimensions_mm"], float) / 1000.0
    raw_bottle = snapshot["bottle"]["raw_observations"]
    raw_a_position, raw_a_rotation, _ = _pose(raw_bottle[0]["pose"])
    raw_b_position, raw_b_rotation, _ = _pose(raw_bottle[1]["pose"])

    # 已确认关系：R_sdk_endpoint = R_link11 * Rz(+90deg)。以下只计算夹爪
    # proxy 相对 SDK endpoint 的几何，不涉及 PB world 会话平移。
    a = math.radians(-90.0)
    rz_minus_90 = np.array([
        [math.cos(a), -math.sin(a), 0.0],
        [math.sin(a), math.cos(a), 0.0],
        [0.0, 0.0, 1.0],
    ])
    link_rotation = safe_rotation @ rz_minus_90
    tcp_link_m = np.asarray(calib_common.CONFIRMED_R_TCP_LINK11_MM, float) / 1000.0
    profile = arm_profiles.arm_profile("left")
    proxy_local_m = np.asarray(profile["gripper_collision_center_link_mm"], float) / 1000.0
    proxy_dimensions_m = np.asarray(profile["gripper_collision_box_m"], float)
    link_position = safe_position - link_rotation @ tcp_link_m
    proxy_center = link_position + link_rotation @ proxy_local_m

    box_bottom_z, box_top_z = _z_extents(box_position, box_rotation, box_dimensions)
    proxy_bottom_z, proxy_top_z = _z_extents(
        proxy_center, link_rotation, proxy_dimensions_m)
    proxy_bottom_offset_from_endpoint = proxy_bottom_z - safe_position[2]
    endpoint_z_clear_box = box_top_z - proxy_bottom_offset_from_endpoint
    clearance_m = float(profile["minimum_table_clearance_m"])
    endpoint_z_with_clearance = endpoint_z_clear_box + clearance_m
    safe_joints = np.asarray(snapshot["safe_candidate"]["joints_sdk_deg"], float)
    limits = np.asarray(profile["controller_limits_deg"], float)
    raw_joint_margins = np.minimum(safe_joints - limits[:, 0],
                                   limits[:, 1] - safe_joints)
    configured_margin = float(profile["shared"]["minimum_joint_margin_deg"])

    return {
        "schema_version": "tabletop_scene_preview_report.v1",
        "status": "SCENE_PREVIEW_ONLY_NOT_A_TRAJECTORY",
        "created": dt.datetime.now(dt.timezone.utc).isoformat(),
        "real_motion_authorized": False,
        "snapshot_id": snapshot["snapshot_id"],
        "contract": snapshot["contract"],
        "assumptions": list(PREVIEW_ASSUMPTIONS),
        "metrics": {
            "safe_to_bottle_distance_mm": float(np.linalg.norm(
                safe_position - bottle_position) * 1000.0),
            "bottle_to_box_distance_mm": float(np.linalg.norm(
                bottle_position - box_position) * 1000.0),
            "bottle_raw_ab_position_delta_mm": (
                (raw_b_position - raw_a_position) * 1000.0).tolist(),
            "bottle_raw_ab_distance_mm": float(np.linalg.norm(
                raw_b_position - raw_a_position) * 1000.0),
            "bottle_raw_ab_orientation_delta_deg": _rotation_delta_deg(
                raw_a_rotation, raw_b_rotation),
            "bottle_to_safe_endpoint_orientation_delta_deg": _rotation_delta_deg(
                bottle_rotation, safe_rotation),
            "box_assumed_bottom_z_mm": box_bottom_z * 1000.0,
            "box_assumed_top_z_mm": box_top_z * 1000.0,
            "safe_endpoint_z_mm": float(safe_position[2] * 1000.0),
            "safe_gripper_proxy_bottom_z_mm": proxy_bottom_z * 1000.0,
            "safe_gripper_proxy_top_z_mm": proxy_top_z * 1000.0,
            "safe_proxy_clearance_over_box_top_mm": (
                proxy_bottom_z - box_top_z) * 1000.0,
            "minimum_endpoint_z_to_clear_box_proxy_mm": (
                endpoint_z_clear_box * 1000.0),
            "minimum_endpoint_z_with_profile_clearance_mm": (
                endpoint_z_with_clearance * 1000.0),
            "safe_joint_min_raw_limit_margin_deg": float(np.min(raw_joint_margins)),
            "safe_joint_min_margin_after_configured_buffer_deg": float(
                np.min(raw_joint_margins) - configured_margin),
        },
        "gates": {
            "source_observation_preserved": "PASS",
            "bottle_ab_repeatability": "REVIEW_REQUIRED_NO_ACCEPTANCE_THRESHOLD_DEFINED",
            "bottle_feature_to_object_transform": "BLOCKED_PENDING_T_OBSERVATION_OBJECT",
            "object_to_grasp_transform": "BLOCKED_PENDING_OBJECT_TEMPLATE",
            "bin_datum_to_place_transform": "BLOCKED_PENDING_T_BIN_OBJECT_PLACE",
            "open_bin_collision_geometry": "BLOCKED_PENDING_INNER_GEOMETRY_AND_DATUM_ROLE",
            "safe_candidate_as_transfer_corridor": (
                "PASS" if proxy_bottom_z >= box_top_z + clearance_m
                else "BLOCKED_PROXY_BELOW_ASSUMED_RIM_CLEARANCE"),
            "safe_worlds_joints_same_sample_and_tcp": "BLOCKED_PENDING_LIVE_COHERENCE_CHECK",
            "safe_joint_values_against_stored_limits": (
                "PRELIMINARY_PASS_NOT_LIVE_READBACK" if np.all(raw_joint_margins >= configured_margin)
                else "BLOCKED_STORED_LIMIT_MARGIN"),
            "pb_sdk_frame_gate": "NOT_USED_FOR_SCENE_ONLY_PREVIEW",
            "joint_trajectory_and_dense_collision_preflight": "PENDING",
            "formal_gui_trajectory_review": "PENDING",
        },
        "provenance": {
            "bottle_preview_envelope": _prior_bottle_envelope_m()[1],
        },
    }


def _visual_box(half, rgba, position, orientation=(0, 0, 0, 1)) -> int:
    shape = p.createVisualShape(p.GEOM_BOX, halfExtents=list(map(float, half)),
                                rgbaColor=rgba)
    return p.createMultiBody(0, -1, shape, basePosition=list(map(float, position)),
                             baseOrientation=list(map(float, orientation)))


def _world_from_local(center: np.ndarray, rotation: np.ndarray,
                      local: list[float]) -> np.ndarray:
    return center + rotation @ np.asarray(local, dtype=float)


def _draw_axes(center: np.ndarray, rotation: np.ndarray, length: float = .09):
    for axis, color in zip(rotation.T, ([1, 0, 0], [0, .8, 0], [0, .35, 1])):
        p.addUserDebugLine(center.tolist(), (center + axis * length).tolist(), color, 3)


def _draw_label(text: str, position, color=(.08, .08, .08), size=.65):
    p.addUserDebugText(text, list(map(float, position)), list(color),
                       textSize=float(size), lifeTime=0)


def _draw_tag(text: str, source: np.ndarray, label_position,
              color, size=.65):
    label = np.asarray(label_position, dtype=float)
    p.addUserDebugLine(source.tolist(), label.tolist(), list(color), 1)
    _draw_label(text, label, color, size)


def _draw_scene(snapshot: dict, report: dict,
                display_shift_m: np.ndarray | None = None):
    shift = (np.zeros(3) if display_shift_m is None
             else np.asarray(display_shift_m, dtype=float))
    if shift.shape != (3,) or not np.all(np.isfinite(shift)):
        raise ValueError("display_shift_m 必须是 3 个有限数")
    bottle_position, bottle_rotation, bottle_quaternion = _pose(
        snapshot["bottle"]["processed_pose"])
    box_position, box_rotation, box_quaternion = _pose(
        snapshot["material_box"]["processed_pose"])
    safe_position, safe_rotation, safe_quaternion = _pose(
        snapshot["safe_candidate"]["pose"])
    bottle_position += shift
    box_position += shift
    safe_position += shift
    box_dimensions = np.asarray(snapshot["material_box"]["dimensions_mm"], float) / 1000.0
    bottle_dimensions, _bottle_source = _prior_bottle_envelope_m()

    # 用合并框位姿的最低角点推断一张预览桌面；这不是已确认 table_surface_z。
    box_bottom_z = (report["metrics"]["box_assumed_bottom_z_mm"] / 1000.0
                    + shift[2])
    floor_center = [float((bottle_position[0] + box_position[0]) / 2.0),
                    float((bottle_position[1] + box_position[1]) / 2.0),
                    box_bottom_z - .006]
    _visual_box([.45, .48, .006], [.72, .62, .47, .30], floor_center)

    # 料框以局部 X/Y/Z = 310/390/130mm 画成开口框；壁厚仅为视觉效果。
    dx, dy, dz = (float(v) for v in box_dimensions)
    wall = .010
    wall_specs = (
        ([wall / 2, dy / 2, dz / 2], [-dx / 2 + wall / 2, 0, 0]),
        ([wall / 2, dy / 2, dz / 2], [ dx / 2 - wall / 2, 0, 0]),
        ([dx / 2, wall / 2, dz / 2], [0, -dy / 2 + wall / 2, 0]),
        ([dx / 2, wall / 2, dz / 2], [0,  dy / 2 - wall / 2, 0]),
        ([dx / 2, dy / 2, .004], [0, 0, -dz / 2 + .004]),
    )
    for half, local_center in wall_specs:
        _visual_box(half, [.95, .55, .08, .48],
                    _world_from_local(box_position, box_rotation, local_center),
                    box_quaternion)
    _draw_axes(box_position, box_rotation)
    box_marker = p.createVisualShape(p.GEOM_SPHERE, radius=.013,
                                     rgbaColor=[1, .10, .05, 1])
    p.createMultiBody(0, -1, box_marker, basePosition=box_position.tolist())
    _draw_tag("B  BIN DATUM", box_position,
              box_position + np.array([0, 0, .163]), (.65, .20, .02))

    # 瓶子只作为半透明的旧包络先验；紫色球才是实际合并观测点。
    radius = max(float(bottle_dimensions[0]), float(bottle_dimensions[1])) / 2.0
    height = float(bottle_dimensions[2])
    bottle_shape = p.createVisualShape(
        p.GEOM_CYLINDER, radius=radius, length=height,
        rgbaColor=[.12, .62, .92, .24])
    p.createMultiBody(0, -1, bottle_shape, basePosition=bottle_position.tolist())
    feature_shape = p.createVisualShape(p.GEOM_SPHERE, radius=.016,
                                        rgbaColor=[.70, .05, .92, 1])
    p.createMultiBody(0, -1, feature_shape, basePosition=bottle_position.tolist())
    _draw_axes(bottle_position, bottle_rotation)
    _draw_tag("V  VISION FEATURE", bottle_position,
              bottle_position + np.array([0, 0, .17]), (.35, .04, .55))

    # endpoint 与夹爪 proxy 只使用已确认的 endpoint/link 相对几何。
    a = math.radians(-90.0)
    rz_minus_90 = np.array([[math.cos(a), -math.sin(a), 0],
                            [math.sin(a), math.cos(a), 0], [0, 0, 1]], float)
    link_rotation = safe_rotation @ rz_minus_90
    tcp_link = np.asarray(calib_common.CONFIRMED_R_TCP_LINK11_MM, float) / 1000.0
    profile = arm_profiles.arm_profile("left")
    link_position = safe_position - link_rotation @ tcp_link
    proxy_center = link_position + link_rotation @ (
        np.asarray(profile["gripper_collision_center_link_mm"], float) / 1000.0)
    proxy_half = np.asarray(profile["gripper_collision_box_m"], float) / 2.0
    # Matrix->quaternion implementation already lives in the shared transform module.
    proxy_q = matrix_to_quaternion_xyzw(link_rotation).tolist()
    _visual_box(proxy_half, [.95, .08, .05, .30], proxy_center, proxy_q)
    safe_shape = p.createVisualShape(p.GEOM_SPHERE, radius=.014,
                                     rgbaColor=[.05, .82, .25, 1])
    p.createMultiBody(0, -1, safe_shape, basePosition=safe_position.tolist())
    _draw_axes(safe_position, safe_rotation)
    _draw_tag("S  SAFE EE", safe_position,
              safe_position + np.array([0, 0, .09]), (.02, .38, .10))

    # 灰线只表达点之间的数值关系，不表达可执行路径。
    p.addUserDebugLine(safe_position.tolist(), bottle_position.tolist(),
                       [.45, .45, .45], 1)
    p.addUserDebugLine(bottle_position.tolist(), box_position.tolist(),
                       [.45, .45, .45], 1)
    required_z = (report["metrics"]["minimum_endpoint_z_with_profile_clearance_mm"]
                  / 1000.0 + shift[2])
    # TinyRenderer 不显示 debug line/text；薄红条让 headless 截图也能看见最低空夹爪
    # endpoint 高度。它不是规划路点。
    _visual_box([.26, .004, .004], [1, .03, .01, .95],
                [box_position[0], box_position[1], required_z])
    p.addUserDebugLine([box_position[0] - .26, box_position[1], required_z],
                       [box_position[0] + .26, box_position[1], required_z],
                       [1, .05, .02], 5)
    z_label = [box_position[0] + .245, box_position[1], required_z + .025]
    _draw_tag(f"Zmin EE = {required_z * 1000:.1f} mm",
              np.array([box_position[0] + .26, box_position[1], required_z]),
              z_label, (.75, .03, .02))
    _draw_label("PREVIEW ONLY / NO MOTION",
                [float((safe_position[0] + box_position[0]) / 2.0),
                 float((safe_position[1] + bottle_position[1]) / 2.0),
                 .72 + float(shift[2])], (.80, .02, .02), .75)

    return {
        "target": ((safe_position + bottle_position + box_position) / 3.0).tolist(),
        "safe_quaternion": safe_quaternion,
    }


def _save_screenshot(path: Path, target: list[float], distance: float,
                     yaw: float, pitch: float, width=1280, height=900):
    view = p.computeViewMatrixFromYawPitchRoll(
        cameraTargetPosition=target, distance=float(distance), yaw=float(yaw),
        pitch=float(pitch),
        roll=0, upAxisIndex=2)
    projection = p.computeProjectionMatrixFOV(
        fov=52, aspect=float(width) / height, nearVal=.02, farVal=4.0)
    _, _, rgba, _, _ = p.getCameraImage(
        width, height, viewMatrix=view, projectionMatrix=projection,
        renderer=p.ER_TINY_RENDERER)
    from PIL import Image
    path.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(np.asarray(rgba, dtype=np.uint8), mode="RGBA").save(path)


def _prime_gui_camera(target: list[float], distance: float,
                      yaw: float, pitch: float) -> dict:
    """macOS ExampleBrowser 会在 GUI 线程首帧覆盖一次相机，连续数帧设定后交还鼠标。"""
    camera = None
    matched = False
    for index in range(60):
        p.resetDebugVisualizerCamera(
            cameraDistance=float(distance), cameraYaw=float(yaw),
            cameraPitch=float(pitch), cameraTargetPosition=target)
        p.stepSimulation()
        time.sleep(.025)
        camera = p.getDebugVisualizerCamera()
        target_error = float(np.linalg.norm(
            np.asarray(camera[11], float) - np.asarray(target, float)))
        matched = (index >= 10 and camera[0] > 0 and camera[1] > 0 and
                   abs(float(camera[10]) - float(distance)) < .02 and
                   target_error < .01)
        if matched:
            # 再给 macOS GUI 线程几帧；若首帧稍晚覆盖，下一轮会重新设置。
            for _ in range(5):
                p.resetDebugVisualizerCamera(
                    cameraDistance=float(distance), cameraYaw=float(yaw),
                    cameraPitch=float(pitch), cameraTargetPosition=target)
                p.stepSimulation()
                time.sleep(.025)
            camera = p.getDebugVisualizerCamera()
            break
    if camera is None:
        raise RuntimeError("PyBullet GUI 相机状态不可读")
    target_error = float(np.linalg.norm(
        np.asarray(camera[11], float) - np.asarray(target, float)))
    if (not matched or abs(float(camera[10]) - float(distance)) >= .02 or
            target_error >= .01):
        raise RuntimeError(
            "PyBullet GUI 相机初始化未生效；拒绝显示默认远景 "
            f"(distance={camera[10]}, target_error={target_error:.4f}m)")
    return {
        "viewport_px": [int(camera[0]), int(camera[1])],
        "requested_distance": float(distance),
        "actual_distance": float(camera[10]),
        "requested_target": list(map(float, target)),
        "actual_target": [float(v) for v in camera[11]],
    }


def _set_robot_pose(robot: int, snapshot: dict) -> None:
    left = arm_profiles.arm_profile("left")
    right = arm_profiles.arm_profile("right")
    for profile, values in (
            (left, snapshot["safe_candidate"]["joints_sdk_deg"]),
            (right, right["home_deg"])):
        urdf_deg = arm_profiles.sdk_to_urdf_deg(profile, values)
        for joint, value in zip(profile["joint_ids"], urdf_deg):
            p.resetJointState(robot, joint, math.radians(float(value)))


def _single_safe_pose_robot_anchor(robot: int, snapshot: dict) -> tuple[np.ndarray, dict]:
    """用同一 safe Worlds/Joints 只做显示平移；绝不提升为 frame gate PASS。"""
    _set_robot_pose(robot, snapshot)
    profile = arm_profiles.arm_profile("left")
    state = p.getLinkState(robot, profile["ee_link_id"],
                           computeForwardKinematics=True)
    pb_link_position = np.asarray(state[4], dtype=float)
    pb_link_rotation = np.asarray(
        p.getMatrixFromQuaternion(state[5]), dtype=float).reshape(3, 3)
    tcp_link_m = np.asarray(
        calib_common.CONFIRMED_R_TCP_LINK11_MM, dtype=float) / 1000.0
    pb_tcp_position = pb_link_position + pb_link_rotation @ tcp_link_m
    sdk_safe_position, sdk_safe_rotation, _ = _pose(
        snapshot["safe_candidate"]["pose"])
    display_shift = pb_tcp_position - sdk_safe_position

    a = math.radians(90.0)
    rz_plus_90 = np.array([[math.cos(a), -math.sin(a), 0.0],
                           [math.sin(a), math.cos(a), 0.0],
                           [0.0, 0.0, 1.0]])
    predicted_sdk_rotation = pb_link_rotation @ rz_plus_90
    orientation_delta = _rotation_delta_deg(
        predicted_sdk_rotation, sdk_safe_rotation)
    t_sdk_minus_pb_m = -display_shift
    reference_t_m = np.asarray(
        calib_common.CONFIRMED_T_SESSIONS_MM["20260707"], dtype=float) / 1000.0
    return display_shift, {
        "mode": "VISUAL_ONLY_SINGLE_SAFE_POSE_ANCHOR",
        "qualification_status": "NOT_A_FRAME_GATE_NOT_VALID_FOR_TRAJECTORY_OR_COLLISION_PASS",
        "sdk_safe_endpoint_position_m": sdk_safe_position.tolist(),
        "pb_safe_tcp_position_m": pb_tcp_position.tolist(),
        "display_shift_pb_minus_sdk_m": display_shift.tolist(),
        "single_anchor_t_sdk_minus_pb_m": t_sdk_minus_pb_m.tolist(),
        "difference_from_in_use_model_a_t_mm": (
            (t_sdk_minus_pb_m - reference_t_m) * 1000.0).tolist(),
        "difference_from_in_use_model_a_t_norm_mm": float(
            np.linalg.norm(t_sdk_minus_pb_m - reference_t_m) * 1000.0),
        "safe_orientation_residual_deg": orientation_delta,
        "note": "safe joints/worlds are rounded user input; overlay is a visual sanity check only",
    }


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--snapshot", required=True)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--screenshot")
    ap.add_argument("--report-out")
    ap.add_argument("--robot-overlay", action="store_true",
                    help="加载机器人并用 safe Worlds/Joints 做单姿态显示锚定；仍非轨迹资格通过")
    ap.add_argument("--stay-seconds", type=float, default=0.0,
                    help="GUI 保持秒数；0 表示直到手动关闭")
    ap.add_argument("--camera-distance", type=float, default=.78)
    ap.add_argument("--camera-yaw", type=float, default=142.0)
    ap.add_argument("--camera-pitch", type=float, default=-30.0)
    args = ap.parse_args(argv)
    if args.stay_seconds < 0 or args.camera_distance <= 0:
        ap.error("--stay-seconds 必须 >= 0 且 --camera-distance 必须 > 0")

    snapshot = load_and_validate(args.snapshot, "tabletop_scene_snapshot.v1")
    report = analyze_preview(snapshot)
    report["snapshot_path"] = str(Path(args.snapshot).resolve())
    report["snapshot_sha256"] = sha256_file(args.snapshot)
    report["preview_script_sha256"] = sha256_file(__file__)

    display_shift = np.zeros(3)
    if args.robot_overlay:
        robot = xf.load_xifeng(gui=not args.headless)
        display_shift, overlay = _single_safe_pose_robot_anchor(robot, snapshot)
        report["assumptions"][-1] = (
            "a visual-only translation from one safe Worlds/Joints pair is applied; "
            "no global SDK-world to PyBullet frame transform is qualified")
        report["gates"]["pb_sdk_frame_gate"] = (
            "BLOCKED_VISUAL_SINGLE_POSE_ANCHOR_ONLY")
        report["visualization"] = {
            "robot_overlay": overlay,
            "collision_evaluation": (
                "NOT_PERFORMED_PREVIEW_OBJECTS_USE_VISUAL_SHAPES_ONLY"),
        }
    else:
        client = p.connect(p.DIRECT if args.headless else p.GUI)
        if client < 0:
            raise RuntimeError("无法连接 PyBullet")
        report["visualization"] = {
            "robot_overlay": {
                "mode": "DISABLED_SDK_WORLD_SCENE_ONLY",
                "qualification_status": "NOT_A_TRAJECTORY_OR_COLLISION_PASS",
            }}
    try:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        if not args.headless:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
        p.setGravity(0, 0, 0)
        scene = _draw_scene(snapshot, report, display_shift)
        target = scene["target"]
        if args.screenshot:
            _save_screenshot(Path(args.screenshot), target,
                             args.camera_distance, args.camera_yaw,
                             args.camera_pitch)
        if args.report_out:
            out = Path(args.report_out)
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if not args.headless:
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
            camera_check = _prime_gui_camera(
                target, args.camera_distance, args.camera_yaw, args.camera_pitch)
            print("GUI camera:", json.dumps(camera_check, ensure_ascii=False))
            print("颜色: 蓝=瓶子显示包络, 橙=料框, 红体=safe夹爪包络, 红线=最低空夹爪端点Z")
            print("说明: 这是场景数据预览，不是机械臂轨迹回放。")
            started = time.monotonic()
            while p.isConnected() and (
                    args.stay_seconds == 0 or
                    time.monotonic() - started < args.stay_seconds):
                time.sleep(.05)
    finally:
        if p.isConnected():
            p.disconnect()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
