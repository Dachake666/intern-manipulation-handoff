#!/usr/bin/env python3
"""桌边/桌下障碍场景的后适配几何与无运动补充门。

本模块故意不定义新的视觉或任务 JSON 契约。它只接收统一输入层已经
转换到 PB world/米的数据，并把几何编译成现有 ``preflight`` 的
``pairs + obstacles`` 边界。这样桌上 MVP 与障碍版可以共用同一个上游契约。

安全常量只从 ``arm_profiles`` 传入；此处不复制 HOME、J7、0.4°/20ms 或
夹爪包络字面量。
"""
from __future__ import annotations

import copy
import math
from typing import Iterable, Sequence

import numpy as np

from robot_mission.contracts import (canonical_bytes, sha256_bytes,
                                     validate_document)
from robot_mission.grasp_pose import (
    matrix_to_quaternion_xyzw,
    normalize_quaternion_xyzw,
    pose_matrix,
    quaternion_xyzw_to_matrix,
)
from robot_mission.preflight import _set_robot, densify


def _finite_vector(name: str, value: Sequence[float], size: int,
                   *, positive: bool = False,
                   nonnegative: bool = False) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape != (size,) or not np.isfinite(array).all():
        raise ValueError(f"{name} 必须是 {size} 个有限数")
    if positive and np.any(array <= 0.0):
        raise ValueError(f"{name} 必须全部大于 0")
    if nonnegative and np.any(array < 0.0):
        raise ValueError(f"{name} 必须全部大于等于 0")
    return array


def _inflation_xyz(value: float | Sequence[float]) -> np.ndarray:
    if np.isscalar(value):
        value = [float(value)] * 3
    return _finite_vector("inflation_m", value, 3, nonnegative=True)


def quaternion_matrix_xyzw(quaternion_xyzw: Sequence[float]) -> np.ndarray:
    """复用桌上 MVP 的 xyzw 规一化与旋转实现。"""
    return quaternion_xyzw_to_matrix(quaternion_xyzw)


def upright_axis_aligned_orientation_gate(
        quaternion_xyzw: Sequence[float], tolerance: float = 1e-6) -> dict:
    """当前 AABB/盒体预检只允许直立且与 PB XY 轴对齐的开口几何。

    实心障碍使用保守 AABB 可以接受假阳性；开口容器不行，因为旋转后四壁
    分别转 AABB 可能把开口虚假封死。因此非 90° 轴对齐时明确 BLOCKED。
    """
    limit = float(tolerance)
    if not math.isfinite(limit) or limit < 0.0:
        raise ValueError("tolerance 必须为非负有限数")
    rotation = quaternion_matrix_xyzw(quaternion_xyzw)
    absolute = np.abs(rotation)
    permutation = np.zeros((3, 3))
    columns = np.argmax(absolute, axis=1)
    if len(set(int(v) for v in columns)) == 3:
        permutation[np.arange(3), columns] = 1.0
        axis_error = float(np.max(np.abs(absolute - permutation)))
    else:
        axis_error = 1.0
    # 必须是容器本地 +Z 指向世界 +Z；不能取绝对值，否则
    # roll=180deg 的翻倒容器也会被当成“直立”。
    upright_error = float(np.max(np.abs(rotation[:, 2] - [0.0, 0.0, 1.0])))
    passed = axis_error <= limit and upright_error <= limit
    return {
        "status": "PASS" if passed else "BLOCKED",
        "axis_alignment_error": axis_error,
        "upright_z_error": upright_error,
        "tolerance": limit,
        "supported_orientation": "upright PB-axis-aligned, including yaw multiples of 90deg",
    }


def placed_object_from_tabletop_plan(plan: dict) -> dict:
    """从共享 ``tabletop_plan.v1`` 恢复放置后物体位姿。

    障碍版不再解释一次原始相机 XYZ/四元数，只消费桌上 MVP 已经
    解算好的 endpoint 与同一组 ``T_endpoint_grasp``/
    ``T_object_grasp``。
    """
    validate_document(plan, "tabletop_plan.v1")
    if (plan["contract"]["arm"], plan["contract"]["arm_id"],
            plan["contract"]["gripper_id"]) != ("left", 1, 2):
        raise ValueError("首版障碍场景只允许左臂 arm_id=1/夹爪2")
    closure = tabletop_plan_closure_gate(plan)
    if closure["status"] != "PASS":
        raise ValueError(f"tabletop_plan 语义闭环失败: {closure['checks']}")
    placed = plan["resolved"]["place_object_after_release"]
    # 固定工具姿态时，必须消费共享 plan 在闭爪瞬间冻结的
    # T_endpoint_object_at_pick。resolved 已是该刚性附件关系的解算结果，
    # 不得再用理想 T_object_grasp 重算。
    T_world_object = pose_matrix(
        placed["position_mm"], placed["quaternion_xyzw"])
    quaternion = matrix_to_quaternion_xyzw(T_world_object[:3, :3])
    return {
        "frame": "pb_world_pending_sdk_to_pb_gate",
        "source_frame": "sdk_world",
        "position_sdk_world_mm": [float(v) for v in T_world_object[:3, 3]],
        "quaternion_xyzw": [float(v) for v in quaternion],
        "dimensions_mm": [float(v) for v in plan["input"]["dimensions_mm"]],
        "T_world_object_mm": [[float(v) for v in row]
                              for row in T_world_object],
        "plan_id": plan["plan_id"],
        "tabletop_plan_sha256": sha256_bytes(canonical_bytes(plan)),
        "T_endpoint_object_at_pick_sha256": sha256_bytes(canonical_bytes(
            plan["transforms"]["T_endpoint_object_at_pick"])),
        "note": "转 PB world 前仍须通过现行 PB<->SDK frame gate；本函数不猜旧 T。",
    }


def _matrix_close(left: np.ndarray, right: np.ndarray) -> bool:
    """仅用于 JSON 序列化/四元数往返的数值闭环，不是物理安全余量。"""
    return bool(np.allclose(left, right, atol=1e-8, rtol=0.0))


def tabletop_plan_closure_gate(plan: dict) -> dict:
    """补充 schema 形状校验之上的统一 plan 语义闭环。"""
    try:
        validate_document(plan, "tabletop_plan.v1")
    except ValueError as exc:
        return {
            "status": "BLOCKED",
            "checks": {"tabletop_plan_contract_valid": False},
            "contract_error": str(exc),
            "tabletop_plan_sha256": sha256_bytes(canonical_bytes(plan)),
        }
    T_pick = pose_matrix(**{
        "position_mm": plan["resolved"]["pick_endpoint"]["position_mm"],
        "quaternion_xyzw": plan["resolved"]["pick_endpoint"]["quaternion_xyzw"],
    })
    T_place = pose_matrix(**{
        "position_mm": plan["resolved"]["place_endpoint"]["position_mm"],
        "quaternion_xyzw": plan["resolved"]["place_endpoint"]["quaternion_xyzw"],
    })
    T_safe = pose_matrix(**{
        "position_mm": plan["resolved"]["safe_endpoint"]["position_mm"],
        "quaternion_xyzw": plan["resolved"]["safe_endpoint"]["quaternion_xyzw"],
    })
    T_endpoint_object = np.asarray(
        plan["transforms"]["T_endpoint_object_at_pick"], dtype=float)
    placed = plan["resolved"]["place_object_after_release"]
    T_placed = pose_matrix(placed["position_mm"], placed["quaternion_xyzw"])
    stages = {stage["name"]: stage for stage in plan["stages"]}
    expected_stage_names = [
        "START_ASSERT_SAFE", "OPEN_BEFORE_PICK", "PICK_HOVER",
        "PICK_DESCEND", "CLOSE_AT_PICK", "PICK_ASCEND",
        "TRANSFER_HIGH", "PLACE_DESCEND", "OPEN_AT_PLACE",
        "PLACE_ASCEND", "RETURN_HIGH", "END_ASSERT_SAFE",
    ]
    expected_stage_state = [
        ("ASSERT_ENDPOINT", None, False),
        ("GRIPPER", "open", False),
        ("MOVE_WORLDS", None, False),
        ("MOVE_WORLDS", None, False),
        ("GRIPPER", "close", True),
        ("MOVE_WORLDS", None, True),
        ("MOVE_WORLDS", None, True),
        ("MOVE_WORLDS", None, True),
        ("GRIPPER", "open", False),
        ("MOVE_WORLDS", None, False),
        ("MOVE_WORLDS", None, False),
        ("ASSERT_ENDPOINT", None, False),
    ]
    actual_stage_state = [
        (stage.get("kind"), stage.get("action"),
         stage.get("carrying_after") if stage.get("kind") == "GRIPPER"
         else stage.get("carrying"))
        for stage in plan["stages"]]
    T_pick_hover = T_pick.copy(); T_pick_hover[2, 3] = T_safe[2, 3]
    T_place_hover = T_place.copy(); T_place_hover[2, 3] = T_safe[2, 3]

    def stage_matches(name: str, expected: np.ndarray) -> bool:
        stage = stages.get(name, {})
        pose = stage.get("pose")
        return bool(pose) and _matrix_close(
            pose_matrix(pose["position_mm"], pose["quaternion_xyzw"]),
            expected)

    checks = {
        "left_arm_only": (
            plan["contract"]["arm"], plan["contract"]["arm_id"],
            plan["contract"]["gripper_id"]) == ("left", 1, 2),
        "shared_sdk_world_mm_xyzw_contract": (
            plan["contract"]["world_frame"],
            plan["contract"]["length_unit"],
            plan["contract"]["quaternion_order"]) == (
                "sdk_world", "millimeter", "xyzw"),
        "upstream_plan_is_pose_only_not_joint_execution":
            plan["contract"]["dynamic_transport"] == "MOVE_WORLDS_ONLY",
        "real_motion_remains_disabled": plan.get("real_motion_authorized") is False,
        "safe_corridor_scalar_matches_safe_endpoint":
            float(plan["resolved"]["safe_corridor_endpoint_z_mm"]) ==
            float(T_safe[2, 3]),
        "stage_sequence_is_exact_and_unique":
            [stage["name"] for stage in plan["stages"]] ==
            expected_stage_names and len(stages) == len(expected_stage_names),
        "stage_kind_action_and_carrying_state_are_exact":
            actual_stage_state == expected_stage_state,
        "place_object_uses_frozen_attachment": _matrix_close(
            T_placed, T_place @ T_endpoint_object),
        "pick_descend_matches_resolved": stage_matches("PICK_DESCEND", T_pick),
        "pick_hover_matches_safe_corridor":
            stage_matches("PICK_HOVER", T_pick_hover),
        "pick_ascend_matches_safe_corridor":
            stage_matches("PICK_ASCEND", T_pick_hover),
        "place_descend_matches_resolved": stage_matches("PLACE_DESCEND", T_place),
        "transfer_high_matches_safe_corridor":
            stage_matches("TRANSFER_HIGH", T_place_hover),
        "place_ascend_matches_safe_corridor":
            stage_matches("PLACE_ASCEND", T_place_hover),
        "start_safe_matches_resolved": stage_matches("START_ASSERT_SAFE", T_safe),
        "return_high_matches_resolved": stage_matches("RETURN_HIGH", T_safe),
        "end_safe_matches_resolved": stage_matches("END_ASSERT_SAFE", T_safe),
    }
    return {"status": "PASS" if all(checks.values()) else "BLOCKED",
            "checks": checks,
            "tabletop_plan_sha256": sha256_bytes(canonical_bytes(plan))}


def _validated_frame_translation_mm(frame_gate_report: dict) -> np.ndarray:
    """只接受现行已通过的左臂平移模型，不猜测旧会话 T。"""
    required = {
        "schema_version": "frame_gate_report.v1",
        "model": "A_TRANSLATION_ONLY",
        "transform": "p_sdk_mm = p_pb_ee_link_origin_mm + t_session_mm",
        "arm": "left",
        "verdict": "PASS",
    }
    mismatches = {key: [frame_gate_report.get(key), expected]
                  for key, expected in required.items()
                  if frame_gate_report.get(key) != expected}
    if mismatches:
        raise ValueError(f"PB<->SDK frame gate 不可用: {mismatches}")
    reference = frame_gate_report.get("reference")
    candidate = frame_gate_report.get("candidate")
    thresholds = frame_gate_report.get("thresholds")
    checks = frame_gate_report.get("checks")
    if not all(isinstance(value, dict) for value in
               (reference, candidate, thresholds, checks)):
        raise ValueError("frame gate 缺 reference/candidate/thresholds/checks")

    def valid_sha256(value) -> bool:
        if not isinstance(value, str) or len(value) != 64:
            return False
        try:
            int(value, 16)
        except ValueError:
            return False
        return value == value.lower()

    for label, record in (("reference", reference),
                          ("candidate", candidate)):
        if (not valid_sha256(record.get("sha256")) or
                not isinstance(record.get("path"), str) or
                not record["path"] or
                not isinstance(record.get("sample_count"), int) or
                record["sample_count"] < 1):
            raise ValueError(f"frame gate {label} 记录身份不完整")
        _finite_vector(f"frame_gate.{label}.t_session_mm",
                       record.get("t_session_mm"), 3)
        residual = [record.get("residual_rms_mm"),
                    record.get("residual_max_mm")]
        residual.extend(record.get("residual_mean_abs_axis_mm", []))
        if (len(residual) != 5 or
                not all(isinstance(value, (int, float)) and
                        math.isfinite(float(value)) and value >= 0.0
                        for value in residual)):
            raise ValueError(f"frame gate {label} 残差证据不完整")
    if not isinstance(candidate.get("captured_at"), str) or not candidate["captured_at"]:
        raise ValueError("frame gate candidate 缺 captured_at")

    reference_t = _finite_vector(
        "frame_gate.reference.t_session_mm", reference["t_session_mm"], 3)
    candidate_t = _finite_vector(
        "frame_gate.candidate.t_session_mm", candidate["t_session_mm"], 3)
    drift = _finite_vector("frame_gate.drift_xyz_mm",
                           frame_gate_report.get("drift_xyz_mm"), 3)
    drift_norm = float(frame_gate_report.get("drift_euclidean_mm", float("nan")))
    age = float(frame_gate_report.get("age_days", float("nan")))
    axis_limit = float(thresholds.get("axis_abs_lt_mm", float("nan")))
    norm_limit = float(thresholds.get("euclidean_lt_mm", float("nan")))
    age_limit = float(thresholds.get("age_lte_days", float("nan")))
    numeric_ok = all(math.isfinite(value) and value >= 0.0 for value in
                     (drift_norm, age, axis_limit, norm_limit, age_limit))
    semantic_checks = {
        "reported_checks_all_true":
            all(checks.get(key) is True for key in
                ("axis_drift", "euclidean_drift", "freshness")),
        "candidate_minus_reference_matches_report":
            _matrix_close(candidate_t - reference_t, drift),
        "drift_norm_matches_report":
            numeric_ok and math.isclose(
                float(np.linalg.norm(drift)), drift_norm,
                rel_tol=0.0, abs_tol=1e-8),
        "axis_threshold_recomputed":
            numeric_ok and bool(np.all(np.abs(drift) < axis_limit)),
        "euclidean_threshold_recomputed":
            numeric_ok and drift_norm < norm_limit,
        "freshness_threshold_recomputed":
            numeric_ok and age <= age_limit,
    }
    if not all(semantic_checks.values()):
        raise ValueError(f"frame gate 内部语义不自洽: {semantic_checks}")
    return candidate_t


def _sdk_mm_to_pb_m(position_sdk_mm: Sequence[float],
                    translation_mm: np.ndarray) -> list[float]:
    position = _finite_vector("position_sdk_mm", position_sdk_mm, 3)
    return ((position - translation_mm) / 1000.0).tolist()


def _plan_object_matrices(plan: dict) -> tuple[np.ndarray, np.ndarray,
                                                  np.ndarray, np.ndarray]:
    resolved = plan["resolved"]
    transforms = plan["transforms"]
    T_pick_endpoint = pose_matrix(
        resolved["pick_endpoint"]["position_mm"],
        resolved["pick_endpoint"]["quaternion_xyzw"])
    T_place_endpoint = pose_matrix(
        resolved["place_endpoint"]["position_mm"],
        resolved["place_endpoint"]["quaternion_xyzw"])
    T_endpoint_object = np.asarray(
        transforms["T_endpoint_object_at_pick"], dtype=float)
    T_endpoint_grasp = np.asarray(transforms["T_endpoint_grasp"], dtype=float)
    return (T_pick_endpoint @ T_endpoint_object,
            T_place_endpoint @ T_endpoint_object,
            T_pick_endpoint @ T_endpoint_grasp,
            T_place_endpoint @ T_endpoint_grasp)


def _identity_box_orientation_equivalence_gate(plan: dict) -> dict:
    """现有 preflight 静止物体使用 identity 姿态时的保守限制。

    首版只允许 XY 正方形包络且物体 +Z 始终朝世界 +Z。这样
    对于盒体碰撞近似，yaw 不会改变占用体积；任意倾斜或 XY 非对称
    都阻断，直到主 preflight 真正消费 T_endpoint_object_at_pick。
    """
    dimensions = _finite_vector(
        "plan.input.dimensions_mm", plan["input"]["dimensions_mm"], 3,
        positive=True)
    pick_object, place_object, _pick_grasp, _place_grasp = (
        _plan_object_matrices(plan))

    def upright(T: np.ndarray) -> bool:
        return bool(np.allclose(T[:3, 2], [0.0, 0.0, 1.0],
                                atol=1e-8, rtol=0.0))

    checks = {
        "xy_collision_box_is_rotationally_symmetric":
            bool(dimensions[0] == dimensions[1]),
        "pick_object_is_upright": upright(pick_object),
        "place_object_is_upright": upright(place_object),
    }
    return {"status": "PASS" if all(checks.values()) else "BLOCKED",
            "checks": checks,
            "policy": "IDENTITY_PROXY_EQUIVALENT_ONLY; no tilted/asymmetric object"}


def _derive_obstacle_pair(plan: dict, frame_gate_report: dict,
                          grasp_capture_template: dict) -> dict:
    translation = _validated_frame_translation_mm(frame_gate_report)
    pick_object, place_object, pick_grasp, place_grasp = (
        _plan_object_matrices(plan))
    pick_center = _sdk_mm_to_pb_m(pick_object[:3, 3], translation)
    place_center = _sdk_mm_to_pb_m(place_object[:3, 3], translation)
    pick_grip = _sdk_mm_to_pb_m(pick_grasp[:3, 3], translation)
    place_grip = _sdk_mm_to_pb_m(place_grasp[:3, 3], translation)
    capture = copy.deepcopy(grasp_capture_template)
    if not isinstance(capture, dict):
        raise ValueError("必须提供已测量的 grasp_capture 证据模板")
    axis_range = _finite_vector(
        "grasp_capture.axis_range_m", capture.get("axis_range_m"), 2)
    lateral_tolerance = float(capture.get("lateral_tolerance_m", float("nan")))
    if (axis_range[0] > axis_range[1] or
            not math.isfinite(lateral_tolerance) or lateral_tolerance <= 0.0 or
            not isinstance(capture.get("reference"), str) or
            not capture["reference"]):
        raise ValueError("grasp_capture 缺已测 axis/lateral/reference 证据")
    capture["object_grasp_point_world_m"] = pick_grip
    capture["object_grasp_point_source"] = (
        "tabletop_plan.transforms.T_endpoint_grasp + passed frame gate")
    return {
        "pick": pick_grip,
        "place": place_grip,
        "actual_pick_grip": pick_grip,
        "actual_place_grip": place_grip,
        "object_pick_center": pick_center,
        "object_place_center": place_center,
        "dimensions_m": (np.asarray(plan["input"]["dimensions_mm"], float) /
                         1000.0).tolist(),
        "carried_center_offset_grip_m": (
            np.asarray(pick_center) - np.asarray(pick_grip)).tolist(),
        "grasp_capture": capture,
        "object_attachment_mode": "RIGID_FROM_CLOSE",
    }


def adapt_tabletop_plan_to_pb_task(
        plan: dict, frame_gate_report: dict, safety_template: dict, *,
        static_obstacles: Sequence[dict], container: dict) -> dict:
    """统一 plan -> 已绑定 PB 任务的唯一障碍版后适配入口。

    该函数不是新的视觉输入契约；它只消费已验证的
    ``tabletop_plan.v1``、帧门报告和现有 preflight 的几何边界。
    坐标字段全部重新派生，不保留 safety_template 中的旧坐标。
    """
    closure = tabletop_plan_closure_gate(plan)
    if closure["status"] != "PASS":
        raise ValueError(f"tabletop_plan 语义闭环失败: {closure['checks']}")
    _validated_frame_translation_mm(frame_gate_report)
    orientation = _identity_box_orientation_equivalence_gate(plan)
    if orientation["status"] != "PASS":
        raise ValueError(f"现有 preflight 不能安全等价该物体姿态: {orientation['checks']}")
    pairs = safety_template.get("pairs")
    disabled = safety_template.get("self_collision_disabled_link_pairs")
    if not isinstance(pairs, list) or len(pairs) != 1:
        raise ValueError("首版只允许一个已测 grasp_capture 模板")
    if not isinstance(disabled, list) or not disabled:
        raise ValueError("缺显式 self_collision_disabled_link_pairs")
    pair = _derive_obstacle_pair(
        plan, frame_gate_report, pairs[0].get("grasp_capture"))
    obstacles = [copy.deepcopy(item) for item in static_obstacles]
    obstacles.extend(copy.deepcopy(container["obstacles"]))
    plan_sha = sha256_bytes(canonical_bytes(plan))
    frame_sha = sha256_bytes(canonical_bytes(frame_gate_report))
    template_sha = sha256_bytes(canonical_bytes(safety_template))
    obstacle_sha = sha256_bytes(canonical_bytes(obstacles))
    container_sha = sha256_bytes(canonical_bytes(container))
    binding = {
        "adapter": "tabletop_plan_to_obstacle_task.TEST_SCAFFOLD_ONLY",
        "tabletop_plan_sha256": plan_sha,
        "frame_gate_report_canonical_sha256": frame_sha,
        "safety_template_sha256": template_sha,
        "disabled_collision_pairs_sha256": sha256_bytes(canonical_bytes(
            disabled)),
        "grasp_capture_template_sha256": sha256_bytes(canonical_bytes(
            pairs[0]["grasp_capture"])),
        "frame_candidate_record_sha256":
            frame_gate_report["candidate"].get("sha256"),
        "t_session_mm": [float(v) for v in
                         frame_gate_report["candidate"]["t_session_mm"]],
        "derived_pair_sha256": sha256_bytes(canonical_bytes(pair)),
        "obstacle_geometry_sha256": obstacle_sha,
        "target_container_sha256": container_sha,
        "T_endpoint_object_at_pick": copy.deepcopy(
            plan["transforms"]["T_endpoint_object_at_pick"]),
        "object_orientation_approximation": orientation,
        "joint_trajectory_plan_binding": "MISSING",
        "formal_preflight_integration": "NOT_INSTALLED",
    }
    adapted_task = {
        "pairs": [pair], "obstacles": obstacles,
        "self_collision_disabled_link_pairs": copy.deepcopy(disabled),
    }
    # 顶层故意不放 pairs/obstacles：现有 preflight 只看这两个字段且
    # 尚未认识补充门。用非运行包装层可防止脚手架产物被直接送入
    # 旧 preflight 而绕过本文件的恒 BLOCKED 结论。
    return {
        "artifact_type": "obstacle_task.TEST_SCAFFOLD_ONLY",
        "real_motion_authorized": False,
        "scene_geometry_status": "TEST_SCAFFOLD_ONLY_NOT_AUTHORIZED",
        "adapted_task": adapted_task,
        "binding": binding,
        "comment": (
            "只用于离线脚手架；未绑定由 tabletop plan 生成的 Track C "
            "关节轨迹，且补充门尚未接入正式 preflight。"
        ),
    }


def conservative_aabb_from_local_box(
        origin_world_m: Sequence[float], quaternion_xyzw: Sequence[float],
        local_center_m: Sequence[float], dimensions_m: Sequence[float],
        inflation_m: float | Sequence[float] = 0.0) -> dict:
    """将带旋转的局部盒保守编译为当前 preflight 可消费的世界 AABB。"""
    origin = _finite_vector("origin_world_m", origin_world_m, 3)
    local_center = _finite_vector("local_center_m", local_center_m, 3)
    dimensions = _finite_vector("dimensions_m", dimensions_m, 3, positive=True)
    inflation = _inflation_xyz(inflation_m)
    rotation = quaternion_matrix_xyzw(quaternion_xyzw)
    center = origin + rotation @ local_center
    obb_half = dimensions / 2.0
    aabb_half = np.abs(rotation) @ obb_half + inflation
    obb_volume = float(np.prod(dimensions))
    aabb_volume = float(np.prod(2.0 * aabb_half))
    return {
        "center": center.tolist(),
        "half": aabb_half.tolist(),
        "geometry_approximation": "CONSERVATIVE_AABB_FROM_OBB",
        "aabb_overapprox_volume_ratio": aabb_volume / obb_volume,
        "inflation_m": inflation.tolist(),
        "source_obb": {
            "origin_world_m": origin.tolist(),
            "local_center_m": local_center.tolist(),
            "dimensions_m": dimensions.tolist(),
            "quaternion_xyzw": [float(v) for v in
                                normalize_quaternion_xyzw(quaternion_xyzw)],
        },
    }


def build_static_obstacle(
        profile: dict, *, obstacle_id: str, scene_role: str,
        origin_world_m: Sequence[float], quaternion_xyzw: Sequence[float],
        dimensions_m: Sequence[float],
        inflation_m: float | Sequence[float]) -> dict:
    """生成桌板、桌腿、裙板、桌下柜体或桌边障碍的保守 AABB。"""
    if profile.get("arm") != "left" or profile.get("arm_id") != 1:
        raise ValueError("首版障碍场景只允许左臂 arm_id=1")
    allowed_roles = {
        "table_slab", "table_edge", "table_leg", "table_apron",
        "under_table_obstacle", "beside_table_obstacle",
    }
    if scene_role not in allowed_roles:
        raise ValueError(f"scene_role 必须是 {sorted(allowed_roles)}")
    if not obstacle_id:
        raise ValueError("obstacle_id 不能为空")
    result = conservative_aabb_from_local_box(
        origin_world_m, quaternion_xyzw, [0, 0, 0], dimensions_m,
        inflation_m)
    minimum_clearance = float(profile["minimum_table_clearance_m"])
    result.update({
        "source_obstacle_id": obstacle_id,
        "scene_role": scene_role,
        "minimum_robot_clearance_m": minimum_clearance,
        "minimum_gripper_clearance_m": minimum_clearance,
        "solid_obstacle_orientation_policy":
            "CONSERVATIVE_AABB_ACCEPTS_FALSE_POSITIVE_NOT_FALSE_NEGATIVE",
    })
    return result


def build_open_container_obstacles(
        profile: dict, *, container_id: str,
        floor_center_world_m: Sequence[float],
        quaternion_xyzw: Sequence[float],
        inner_size_xy_m: Sequence[float],
        wall_thickness_xy_m: Sequence[float],
        wall_height_m: float, floor_thickness_m: float,
        inflation_m: float | Sequence[float] = 0.0,
        support_contact_tolerance_m: float = 0.0) -> dict:
    """把目标框编译为底板+四条独立框壁，绝不使用一个实心盒。

    ``floor_center_world_m`` 是框内支撑底面中心，不是底板实体中心。
    输入必须已在 PB world/米；坐标转换由上游统一契约完成。
    """
    if profile.get("arm") != "left" or profile.get("arm_id") != 1:
        raise ValueError("首版障碍场景只允许左臂 arm_id=1")
    if not container_id:
        raise ValueError("container_id 不能为空")
    inner = _finite_vector("inner_size_xy_m", inner_size_xy_m, 2,
                           positive=True)
    thickness = _finite_vector("wall_thickness_xy_m",
                               wall_thickness_xy_m, 2, positive=True)
    wall_height = float(wall_height_m)
    floor_thickness = float(floor_thickness_m)
    support_tolerance = float(support_contact_tolerance_m)
    if (not math.isfinite(wall_height) or wall_height <= 0.0 or
            not math.isfinite(floor_thickness) or floor_thickness <= 0.0):
        raise ValueError("框壁高度与底板厚度必须是正有限数")
    if not math.isfinite(support_tolerance) or support_tolerance < 0.0:
        raise ValueError("support_contact_tolerance_m 必须为非负有限数")
    if not np.isscalar(inflation_m):
        requested = _finite_vector("inflation_m", inflation_m, 3,
                                   nonnegative=True)
        if not bool(np.all(requested == requested[0])):
            raise ValueError(
                "开口容器首版只允许标量/各向同性 inflation；"
                "各向异性余量在 yaw 后的轴系尚未实现")
    minimum_clearance = float(profile["minimum_table_clearance_m"])
    outer = inner + 2.0 * thickness
    requested_inflation = _inflation_xyz(inflation_m)
    orientation_qualification = upright_axis_aligned_orientation_gate(
        quaternion_xyzw)

    # local z=0 是框内支撑面；底板向下，四壁向上。
    pieces = [
        ("floor", [0.0, 0.0, -floor_thickness / 2.0],
         [outer[0], outer[1], floor_thickness]),
        ("rim_x_negative", [-(inner[0] + thickness[0]) / 2.0, 0.0,
                            wall_height / 2.0],
         [thickness[0], outer[1], wall_height]),
        ("rim_x_positive", [(inner[0] + thickness[0]) / 2.0, 0.0,
                            wall_height / 2.0],
         [thickness[0], outer[1], wall_height]),
        ("rim_y_negative", [0.0, -(inner[1] + thickness[1]) / 2.0,
                            wall_height / 2.0],
         [inner[0], thickness[1], wall_height]),
        ("rim_y_positive", [0.0, (inner[1] + thickness[1]) / 2.0,
                            wall_height / 2.0],
         [inner[0], thickness[1], wall_height]),
    ]
    obstacles = []
    for role, center, dimensions in pieces:
        # 支撑底面的 Z 是明确放置基准，不能对称膨胀后虚假抬高。
        # Z 测量误差必须单独进入 support tolerance/场景证据门。
        piece_inflation = (requested_inflation if role != "floor" else
                           np.array([requested_inflation[0],
                                     requested_inflation[1], 0.0]))
        obstacle = conservative_aabb_from_local_box(
            floor_center_world_m, quaternion_xyzw, center, dimensions,
            piece_inflation)
        obstacle.update({
            "source_obstacle_id": f"{container_id}:{role}",
            "scene_role": "target_container_floor" if role == "floor"
                          else "target_container_rim",
            "minimum_robot_clearance_m": minimum_clearance,
            "minimum_gripper_clearance_m": minimum_clearance,
            "open_geometry_orientation_status":
                orientation_qualification["status"],
        })
        if role == "floor":
            obstacle["support_contact_tolerance_m"] = support_tolerance
        obstacles.append(obstacle)
    return {
        "container_id": container_id,
        "frame": "pb_world",
        "length_unit": "meter",
        "floor_center_world_m": list(map(float, floor_center_world_m)),
        "quaternion_xyzw": [float(v) for v in
                            normalize_quaternion_xyzw(quaternion_xyzw)],
        "inner_size_xy_m": inner.tolist(),
        "outer_size_xy_m": outer.tolist(),
        "wall_height_m": wall_height,
        "floor_thickness_m": floor_thickness,
        "inflation_m": requested_inflation.tolist(),
        "support_contact_tolerance_m": support_tolerance,
        "floor_inflation_policy": "XY_ONLY; support Z uncertainty is a separate gate",
        "orientation_qualification": orientation_qualification,
        "obstacles": obstacles,
        "note": "输入契约由上游统一；此处仅是保守几何编译结果。",
    }


def placement_fit_gate(container: dict, *,
                       object_center_world_m: Sequence[float],
                       object_dimensions_m: Sequence[float],
                       object_quaternion_xyzw: Sequence[float] = (0, 0, 0, 1),
                       minimum_wall_clearance_m: float = 0.0,
                       support_tolerance_m: float | None = None) -> dict:
    """检查物体包络能否通过框口，且瓶底是否落在支撑面上。"""
    center = _finite_vector("object_center_world_m", object_center_world_m, 3)
    dimensions = _finite_vector("object_dimensions_m", object_dimensions_m, 3,
                                positive=True)
    clearance = float(minimum_wall_clearance_m)
    if not math.isfinite(clearance) or clearance < 0.0:
        raise ValueError("minimum_wall_clearance_m 必须为非负有限数")
    tolerance = (container["support_contact_tolerance_m"]
                 if support_tolerance_m is None else float(support_tolerance_m))
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("support_tolerance_m 必须为非负有限数")
    rotation_container = quaternion_matrix_xyzw(container["quaternion_xyzw"])
    rotation_object = quaternion_matrix_xyzw(object_quaternion_xyzw)
    floor = _finite_vector("floor_center_world_m",
                           container["floor_center_world_m"], 3)
    local_center = rotation_container.T @ (center - floor)
    projected_half = (np.abs(rotation_container.T @ rotation_object) @
                      (dimensions / 2.0))
    aperture_half = np.asarray(container["inner_size_xy_m"], dtype=float) / 2.0
    margins_xy = aperture_half - np.abs(local_center[:2]) - projected_half[:2]
    inflation_xy = np.asarray(container.get("inflation_m", [0, 0, 0]),
                              dtype=float)[:2]
    effective_required_xy = clearance + inflation_xy
    bottom_from_floor = float(local_center[2] - projected_half[2])
    checks = {
        "aperture_x": bool(margins_xy[0] >= effective_required_xy[0]),
        "aperture_y": bool(margins_xy[1] >= effective_required_xy[1]),
        "support_height": bool(abs(bottom_from_floor) <= tolerance),
        "container_orientation_supported":
            container.get("orientation_qualification", {}).get("status") == "PASS",
    }
    return {
        "status": "PASS" if all(checks.values()) else "BLOCKED",
        "checks": checks,
        "object_center_container_m": local_center.tolist(),
        "projected_object_half_extents_container_m": projected_half.tolist(),
        "aperture_margin_xy_m": margins_xy.tolist(),
        "required_wall_clearance_m": clearance,
        "effective_required_clearance_xy_m": effective_required_xy.tolist(),
        "object_bottom_from_floor_m": bottom_from_floor,
        "support_tolerance_m": tolerance,
    }


def _motion_rows(waypoints: Iterable[dict]) -> list[dict]:
    rows = [row for row in waypoints if "q_sdk_deg" in row]
    if not rows:
        raise ValueError("轨迹没有运动路点")
    for index, row in enumerate(rows):
        _finite_vector(f"waypoints[{index}].q_sdk_deg", row["q_sdk_deg"], 7)
    return rows


def _post_release_dense_frames(waypoints: Iterable[dict], step_deg: float) -> list[dict]:
    previous = None
    holding = False
    released = False
    frames = []
    for waypoint in waypoints:
        if "gripper" in waypoint:
            action = waypoint["gripper"]
            if action == "close":
                holding = True
            elif action == "open" and holding:
                holding = False
                released = True
                if previous is not None:
                    # 执行器在上一个关节目标停稳后原地开爪。该姿态
                    # 是放置物体的初始基线，不能从下一段 k=1 才开始检查。
                    frames.append({
                        "q_sdk_deg": list(previous),
                        "seg": "RELEASE_EVENT",
                        "carrying": False,
                        "carried_index": None,
                        "release_event": True,
                    })
            continue
        if "q_sdk_deg" not in waypoint:
            continue
        current = waypoint["q_sdk_deg"]
        if previous is not None and released:
            frames.extend({"q_sdk_deg": q,
                           "seg": waypoint.get("seg", ""),
                           "carrying": False, "carried_index": None,
                           "release_event": False}
                          for q in densify(previous, current, step_deg))
        previous = current
    return frames


def executor_sequence_gate(
        waypoints: list[dict], bound_current_q_sdk_deg: Sequence[float],
        profile: dict, *, live_current_q_sdk_deg: Sequence[float],
        bound_inactive_q_sdk_deg: Sequence[float],
        live_inactive_q_sdk_deg: Sequence[float], trajectory_document: dict,
        runtime_step_deg: float, runtime_period_ms: float,
        exact_start_tolerance_deg: float) -> dict:
    """以绑定快照+现场真值复核 Track C 首点与插值语义。

    当前正式执行器还没有调用本门，因此本函数只返回脚手架检查
    结果，``status`` 保持 BLOCKED。特别地，它不把执行器允许的任何
    非零漂移伪装成已扫掠的 ``armMoveJoints`` 路径。
    """
    if profile.get("arm") != "left" or profile.get("arm_id") != 1:
        raise ValueError("首版障碍场景只允许左臂 arm_id=1")
    bound = _finite_vector("bound_current_q_sdk_deg",
                           bound_current_q_sdk_deg, 7)
    live = _finite_vector("live_current_q_sdk_deg", live_current_q_sdk_deg, 7)
    bound_inactive = _finite_vector("bound_inactive_q_sdk_deg",
                                    bound_inactive_q_sdk_deg, 7)
    live_inactive = _finite_vector("live_inactive_q_sdk_deg",
                                   live_inactive_q_sdk_deg, 7)
    tolerance = float(exact_start_tolerance_deg)
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("exact_start_tolerance_deg 必须为非负有限数")
    moves = _motion_rows(waypoints)
    first = np.asarray(moves[0]["q_sdk_deg"], float)
    bound_first_gap = float(np.max(np.abs(bound - first)))
    live_first_gap = float(np.max(np.abs(live - first)))
    active_snapshot_drift = float(np.max(np.abs(live - bound)))
    inactive_snapshot_drift = float(np.max(np.abs(
        live_inactive - bound_inactive)))
    home = _finite_vector("profile.home_deg", profile["home_deg"], 7)
    final_home_error = float(np.max(np.abs(home - np.asarray(moves[-1]["q_sdk_deg"], float))))
    events = [row.get("gripper") for row in waypoints if "gripper" in row]
    valid_cycle = events == ["close", "open"]
    validate_document(trajectory_document, "trajectory.v2")
    trajectory_pulse = trajectory_document["pulse"]
    step = float(trajectory_pulse.get("step_deg", float("nan")))
    period = float(trajectory_pulse.get("period_ms", float("nan")))
    runtime_step = float(runtime_step_deg)
    runtime_period = float(runtime_period_ms)
    if not all(math.isfinite(value) and value > 0.0 for value in
               (step, period, runtime_step, runtime_period)):
        raise ValueError("轨迹与运行时 pulse 必须是正有限数")
    profile_step = float(profile["shared"]["pulse_step_deg"])
    profile_period = float(profile["shared"]["pulse_period_ms"])
    trajectory_waypoints_match = (
        canonical_bytes(trajectory_document["waypoints"]) ==
        canonical_bytes(waypoints))
    trajectory_contract_match = (
        trajectory_document["contract"]["arm"],
        trajectory_document["contract"]["arm_id"],
        trajectory_document["contract"]["gripper_id"],
        trajectory_document["contract"]["executor"]) == (
            "left", 1, 2, "track_c_armPluseToServo")
    post_release = _post_release_dense_frames(waypoints, step)
    post_segments = list(dict.fromkeys(row["seg"] for row in post_release))
    checks = {
        "preflight_snapshot_equals_first_point": bound_first_gap <= tolerance,
        "live_active_equals_bound_snapshot": active_snapshot_drift <= tolerance,
        "live_inactive_equals_bound_snapshot": inactive_snapshot_drift <= tolerance,
        # 非零差值即意味 armMoveJoints 有未知扫掠路径；首版不放宽。
        "live_current_is_exact_first_point_no_movej_sweep": live_first_gap == 0.0,
        "trajectory_pulse_matches_shared_track_c":
            step == profile_step and period == profile_period,
        "validated_trajectory_v2_waypoints_match_input":
            trajectory_waypoints_match,
        "trajectory_contract_is_left_track_c": trajectory_contract_match,
        "runtime_pulse_matches_bound_trajectory":
            runtime_step == step and runtime_period == period,
        "single_serial_gripper_cycle": valid_cycle,
        "post_release_motion_present": bool(post_release),
        "return_ends_at_shared_home": final_home_error <= tolerance,
    }
    scaffold_status = "PASS" if all(checks.values()) else "BLOCKED"
    return {
        "status": "BLOCKED",
        "scaffold_status": scaffold_status,
        "authorization_blockers": [
            "executor does not call this live-current/pulse gate",
            "nonzero current-to-first armMoveJoints sweep has no qualified envelope",
        ],
        "checks": checks,
        "bound_first_point_gap_deg": bound_first_gap,
        "live_first_point_gap_deg": live_first_gap,
        "active_snapshot_drift_deg": active_snapshot_drift,
        "inactive_snapshot_drift_deg": inactive_snapshot_drift,
        "exact_start_tolerance_deg": tolerance,
        "gripper_events": events,
        "post_release_dense_frames": len(post_release),
        "post_release_segments": post_segments,
        "final_home_error_deg": final_home_error,
        "interpolation_contract": {
            "between_exported_waypoints": "linear_joint_densify",
            "step_deg": step,
            "period_ms": period,
            "current_to_first": (
                "armMoveJoints; only a bit-exact live first point makes it a no-op; "
                "otherwise BLOCKED until a sweep envelope is qualified"),
        },
        "trajectory_sha256": sha256_bytes(canonical_bytes(
            trajectory_document)),
    }


def adapted_obstacle_task_gate(
        artifact: dict, *, container_id: str, tabletop_plan: dict,
        frame_gate_report: dict, container: dict, profile: dict,
        inactive_profile: dict, safety_template: dict) -> dict:
    """重算并逐字段复核 plan -> PB 后适配数据。

    ``scaffold_status`` 只表示这一离线边界内部自洽。因正式
    ``preflight.build_report`` 尚未调用本门，且尚无从该 plan 生成并绑定
    Track C 关节轨迹的证据，总 ``status`` 故意恒为 BLOCKED。
    """
    wrapper_ok = (
        artifact.get("artifact_type") == "obstacle_task.TEST_SCAFFOLD_ONLY" and
        artifact.get("real_motion_authorized") is False and
        artifact.get("scene_geometry_status") ==
        "TEST_SCAFFOLD_ONLY_NOT_AUTHORIZED" and
        "pairs" not in artifact and "obstacles" not in artifact and
        isinstance(artifact.get("adapted_task"), dict))
    task_data = artifact.get("adapted_task", {})
    pairs = task_data.get("pairs")
    disabled = task_data.get("self_collision_disabled_link_pairs")
    obstacles = task_data.get("obstacles", [])
    binding = artifact.get("binding", {})
    template_pairs = safety_template.get("pairs")
    template_capture = (
        template_pairs[0].get("grasp_capture")
        if isinstance(template_pairs, list) and len(template_pairs) == 1 and
        isinstance(template_pairs[0], dict) else None)
    roles = [item.get("scene_role") for item in obstacles
             if str(item.get("source_obstacle_id", "")).startswith(f"{container_id}:")]
    container_obstacles = [item for item in obstacles
                           if str(item.get("source_obstacle_id", "")).startswith(
                               f"{container_id}:")]
    pair_ok = (isinstance(pairs, list) and len(pairs) == 1 and
               pairs[0].get("object_attachment_mode") == "RIGID_FROM_CLOSE" and
               isinstance(pairs[0].get("grasp_capture"), dict))
    plan_closure = {"status": "BLOCKED", "checks": {}}
    orientation = {"status": "BLOCKED", "checks": {}}
    expected_pair = None
    derivation_error = None
    try:
        plan_closure = tabletop_plan_closure_gate(tabletop_plan)
        orientation = _identity_box_orientation_equivalence_gate(
            tabletop_plan)
        if pair_ok:
            expected_pair = _derive_obstacle_pair(
                tabletop_plan, frame_gate_report,
                template_capture)
    except (KeyError, TypeError, ValueError) as exc:
        derivation_error = f"{type(exc).__name__}: {exc}"
    pair_matches_plan = bool(expected_pair is not None and
                             canonical_bytes(pairs[0]) ==
                             canonical_bytes(expected_pair))

    all_joint_ids = list(profile.get("joint_ids", [])) + list(
        inactive_profile.get("joint_ids", []))
    max_link_id = max(all_joint_ids) if all_joint_ids else -2
    template_disabled = safety_template.get(
        "self_collision_disabled_link_pairs")
    disabled_ok = (
        isinstance(disabled, list) and bool(disabled) and
        disabled == template_disabled and
        all(isinstance(item, list) and len(item) == 2 and
            all(isinstance(value, int) and -1 <= value <= max_link_id
                for value in item) and item[0] != item[1]
            for item in disabled))
    rim_ok = roles.count("target_container_rim") == 4
    floor_ok = roles.count("target_container_floor") == 1
    container_orientation_ok = bool(container_obstacles) and all(
        item.get("open_geometry_orientation_status") == "PASS"
        for item in container_obstacles)
    scene_roles = {item.get("scene_role") for item in obstacles}
    scene_obstacle_ok = bool(scene_roles & {
        "under_table_obstacle", "beside_table_obstacle"})
    table_geometry_ok = (
        "table_slab" in scene_roles and "table_leg" in scene_roles)
    obstacle_ids = [item.get("source_obstacle_id") for item in obstacles]
    stable_ids_ok = (all(isinstance(value, str) and value for value in obstacle_ids)
                     and len(set(obstacle_ids)) == len(obstacle_ids))
    obstacle_hash = sha256_bytes(canonical_bytes(obstacles))
    expected_plan_hash = sha256_bytes(canonical_bytes(tabletop_plan))
    expected_frame_hash = sha256_bytes(canonical_bytes(frame_gate_report))
    expected_container_hash = sha256_bytes(canonical_bytes(container))
    expected_template_hash = sha256_bytes(canonical_bytes(safety_template))
    container_matches_task = (
        canonical_bytes(container_obstacles) ==
        canonical_bytes(container.get("obstacles", [])))
    binding_ok = bool(
        isinstance(binding, dict) and expected_pair is not None and
        binding.get("tabletop_plan_sha256") == expected_plan_hash and
        binding.get("frame_gate_report_canonical_sha256") ==
        expected_frame_hash and
        binding.get("safety_template_sha256") == expected_template_hash and
        binding.get("disabled_collision_pairs_sha256") ==
        sha256_bytes(canonical_bytes(template_disabled)) and
        binding.get("grasp_capture_template_sha256") ==
        sha256_bytes(canonical_bytes(template_capture)) and
        binding.get("derived_pair_sha256") ==
        sha256_bytes(canonical_bytes(expected_pair)) and
        binding.get("obstacle_geometry_sha256") == obstacle_hash and
        binding.get("target_container_sha256") == expected_container_hash and
        binding.get("t_session_mm") ==
        frame_gate_report.get("candidate", {}).get("t_session_mm") and
        binding.get("T_endpoint_object_at_pick") ==
        tabletop_plan.get("transforms", {}).get(
            "T_endpoint_object_at_pick"))

    def positive_inflation(item: dict) -> bool:
        try:
            values = _finite_vector("obstacle.inflation_m",
                                    item.get("inflation_m"), 3,
                                    nonnegative=True)
        except (TypeError, ValueError):
            return False
        if item.get("scene_role") == "target_container_floor":
            return bool(np.all(values[:2] > 0.0) and values[2] == 0.0)
        return bool(np.all(values > 0.0))

    uncertainty_inflation_ok = bool(obstacles) and all(
        positive_inflation(item) for item in obstacles)
    profile_clearance = float(profile["minimum_table_clearance_m"])
    obstacle_clearances_match_profile = bool(obstacles) and all(
        float(item.get("minimum_robot_clearance_m", float("nan"))) ==
        profile_clearance and
        float(item.get("minimum_gripper_clearance_m", float("nan"))) ==
        profile_clearance for item in obstacles)
    placement = {"status": "BLOCKED", "checks": {}}
    if expected_pair is not None:
        place_object = tabletop_plan["resolved"]["place_object_after_release"]
        placement = placement_fit_gate(
            container,
            object_center_world_m=expected_pair["object_place_center"],
            object_dimensions_m=expected_pair["dimensions_m"],
            object_quaternion_xyzw=place_object["quaternion_xyzw"],
            minimum_wall_clearance_m=0.0)

    scaffold_checks = {
        "non_runtime_wrapper_prevents_legacy_preflight_bypass": wrapper_ok,
        "left_arm_serial_single_pair": pair_ok,
        "tabletop_plan_semantic_closure": plan_closure["status"] == "PASS",
        "plan_object_orientation_is_safe_for_identity_box_proxy":
            orientation["status"] == "PASS",
        "adapted_pair_rederived_exactly_from_plan_and_frame_gate":
            pair_matches_plan,
        "plan_frame_pair_obstacle_binding_hashes_match": binding_ok,
        "explicit_self_collision_disabled_pairs": disabled_ok,
        "table_slab_and_support_geometry_present": table_geometry_ok,
        "under_or_beside_scope_obstacle_present": scene_obstacle_ok,
        "stable_unique_obstacle_ids": stable_ids_ok,
        "positive_measurement_inflation_on_non_support_axes":
            uncertainty_inflation_ok,
        "robot_and_gripper_clearances_match_shared_profile":
            obstacle_clearances_match_profile,
        "target_container_has_four_rims": rim_ok,
        "target_container_has_support_floor": floor_ok,
        "target_container_obstacles_match_bound_model_exactly":
            container_matches_task,
        "target_container_orientation_supported": container_orientation_ok,
        "placed_object_fits_bound_target_container":
            placement["status"] == "PASS",
    }
    return {"status": "BLOCKED",
            "scaffold_status": ("PASS" if all(scaffold_checks.values())
                                else "BLOCKED"),
            "authorization_blockers": [
                "joint trajectory is not provenance-bound to this tabletop plan",
                "supplemental obstacle gates are not called by formal preflight",
                "formal preflight does not consume T_endpoint_object_at_pick",
                "frame report source files/hashes and CONFIRMED_T session are not reopened here",
                "measured scene inventory completeness is not independently attested",
                "safety template provenance is not a REAL_VERIFIED qualification artifact",
            ],
            "checks": scaffold_checks, "container_roles": roles,
            "scene_roles": sorted(str(value) for value in scene_roles),
            "obstacle_geometry_sha256": obstacle_hash,
            "tabletop_plan_sha256": expected_plan_hash,
            "frame_gate_report_canonical_sha256": expected_frame_hash,
            "plan_closure": plan_closure,
            "object_orientation_approximation": orientation,
            "placement_fit": placement,
            "derivation_error": derivation_error,
            "pair_count": len(pairs) if isinstance(pairs, list) else None,
            "disabled_pair_count": len(disabled) if isinstance(disabled, list) else None}


def clearance_series_gate(
        active_distances_m: Sequence[float],
        inactive_distances_m: Sequence[float], *,
        minimum_clearance_m: float, exit_progress_tolerance_m: float,
        maximum_exit_frames: int) -> dict:
    """纯函数状态机：开爪基线 -> 单调退出 -> 不得重入。"""
    active = _finite_vector("active_distances_m", active_distances_m,
                            len(active_distances_m))
    inactive = _finite_vector("inactive_distances_m", inactive_distances_m,
                              len(inactive_distances_m))
    if not len(active) or len(active) != len(inactive):
        raise ValueError("两夹爪距离序列必须等长且非空")
    clearance = float(minimum_clearance_m)
    tolerance = float(exit_progress_tolerance_m)
    if not math.isfinite(clearance) or clearance <= 0.0:
        raise ValueError("minimum_clearance_m 必须是正有限数")
    if not math.isfinite(tolerance) or tolerance < 0.0:
        raise ValueError("exit_progress_tolerance_m 必须为非负有限数")
    if not isinstance(maximum_exit_frames, int) or maximum_exit_frames < 1:
        raise ValueError("maximum_exit_frames 必须是正整数")
    cleared = False
    first_clear = None
    regressions = []
    reentries = []
    inactive_violations = []
    best_exit_distance = float(active[0])
    for index, (active_distance, inactive_distance) in enumerate(
            zip(active.tolist(), inactive.tolist())):
        if inactive_distance < clearance:
            inactive_violations.append({"frame": index,
                                        "distance_m": inactive_distance})
        # 与历史最佳退出距离比，不与上一帧比。否则每帧小于容差的
        # 恶化可以在多帧中累积成毫米级更深穿透。
        if (not cleared and index and
                active_distance < best_exit_distance - tolerance):
            regressions.append({
                "frame": index,
                "best_prior_distance_m": best_exit_distance,
                "distance_m": active_distance})
        if not cleared:
            best_exit_distance = max(best_exit_distance, active_distance)
        if not cleared and active_distance >= clearance:
            cleared = True
            first_clear = index
        elif cleared and active_distance < clearance:
            reentries.append({"frame": index, "distance_m": active_distance})
    checks = {
        "active_proxy_never_deepens_before_exit": not regressions,
        "active_proxy_reaches_positive_clearance_in_time":
            first_clear is not None and first_clear <= maximum_exit_frames,
        "active_proxy_never_reenters": not reentries,
        "inactive_proxy_clear_for_all_frames": not inactive_violations,
    }
    return {"status": "PASS" if all(checks.values()) else "BLOCKED",
            "checks": checks, "first_clear_frame": first_clear,
            "exit_regressions": regressions,
            "reentries": reentries,
            "inactive_violations": inactive_violations}


def released_object_clearance_gate(
        profile: dict, inactive_profile: dict,
        inactive_q_sdk_deg: Sequence[float], waypoints: list[dict],
        collision_result: dict, object_index: int, *,
        trajectory_document: dict,
        minimum_clearance_m: float, exit_progress_tolerance_m: float,
        maximum_exit_frames: int) -> dict:
    """用主预检实际释放位姿检查两只夹爪与放置物的撤退/返程。

    开爪瞬间显式纳入帧列。实心整夹爪代理若与物体初始重叠，
    清空前的距离必须单调不恶化，在限定帧数内达到经资格验证的
    正间隙；之后活动夹爪不得重入，非活动夹爪也必须全程保持该间隙。

    此处仍是静态运动学代理，无手指分体几何、开爪扫掠与瓶体动力学。
    因此即使 ``kinematic_scaffold_status`` 通过，总 ``status`` 仍为 BLOCKED。
    """
    try:
        import pybullet as p
    except ImportError:
        return {"status": "UNRESOLVED", "reason": "pybullet_missing"}
    if collision_result.get("status") != "PASS":
        return {"status": "BLOCKED",
                "kinematic_scaffold_status": "BLOCKED",
                "reason": "source_collision_check_not_pass"}
    details = collision_result.get("details", {})
    releases = details.get("actual_release_poses")
    dimensions_list = details.get("carried_object_dimensions_m")
    if (not isinstance(object_index, int) or object_index < 0 or
            not isinstance(releases, list) or object_index >= len(releases) or
            releases[object_index] is None or
            not isinstance(dimensions_list, list) or
            object_index >= len(dimensions_list)):
        return {"status": "BLOCKED",
                "kinematic_scaffold_status": "BLOCKED",
                "reason": "actual_release_pose_or_dimensions_missing"}
    release_pose = releases[object_index]
    dimensions = _finite_vector(
        "collision_result.carried_object_dimensions_m",
        dimensions_list[object_index], 3, positive=True)
    center = _finite_vector("actual_release_pose.position",
                            release_pose[0], 3)
    quaternion_matrix_xyzw(release_pose[1])
    clearance = float(minimum_clearance_m)
    progress_tolerance = float(exit_progress_tolerance_m)
    if not math.isfinite(clearance) or clearance <= 0.0:
        raise ValueError("minimum_clearance_m 必须是经资格验证的正有限数")
    if not math.isfinite(progress_tolerance) or progress_tolerance < 0.0:
        raise ValueError("exit_progress_tolerance_m 必须为非负有限数")
    if not isinstance(maximum_exit_frames, int) or maximum_exit_frames < 1:
        raise ValueError("maximum_exit_frames 必须是正整数")
    validate_document(trajectory_document, "trajectory.v2")
    trajectory_pulse = trajectory_document["pulse"]
    step = float(trajectory_pulse.get("step_deg", float("nan")))
    period = float(trajectory_pulse.get("period_ms", float("nan")))
    if (not math.isfinite(step) or step <= 0.0 or
            not math.isfinite(period) or period <= 0.0):
        raise ValueError("trajectory pulse 必须是正有限数")
    pulse_matches_profile = (
        step == float(profile["shared"]["pulse_step_deg"]) and
        period == float(profile["shared"]["pulse_period_ms"]))
    waypoints_match_trajectory = (
        canonical_bytes(trajectory_document["waypoints"]) ==
        canonical_bytes(waypoints))
    frames = _post_release_dense_frames(waypoints, step)
    if not frames:
        return {"status": "BLOCKED", "reason": "no_post_release_frames",
                "checked_frames": 0}
    client = p.connect(p.DIRECT)
    try:
        # 与主 preflight 相同：供应商末端幽灵网格由 arm_profiles 中的实测代理替代。
        from frame_calibration.analysis import calib_common as cc

        robot = p.loadURDF(cc.DEFAULT_URDF, useFixedBase=True)
        active_proxy = p.createMultiBody(0, p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(np.asarray(profile["gripper_collision_box_m"], float) / 2.0).tolist()))
        inactive_proxy = p.createMultiBody(0, p.createCollisionShape(
            p.GEOM_BOX,
            halfExtents=(np.asarray(
                inactive_profile["gripper_collision_box_m"], float) /
                2.0).tolist()))
        placed = p.createMultiBody(0, p.createCollisionShape(
            p.GEOM_BOX, halfExtents=(dimensions / 2.0).tolist()),
            basePosition=center.tolist(),
            baseOrientation=list(map(float, release_pose[1])))
        active_distances = []
        inactive_distances = []

        def reset_proxy(body: int, arm_profile: dict) -> None:
            state = p.getLinkState(robot, arm_profile["ee_link_id"],
                                   computeForwardKinematics=True)
            rotation = np.asarray(
                p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
            proxy_center = np.asarray(state[4]) + rotation @ (
                np.asarray(
                    arm_profile["gripper_collision_center_link_mm"],
                    float) / 1000.0)
            p.resetBasePositionAndOrientation(body, proxy_center, state[5])

        for index, frame in enumerate(frames):
            _set_robot(p, robot, profile, frame["q_sdk_deg"],
                       inactive_profile, inactive_q_sdk_deg)
            reset_proxy(active_proxy, profile)
            reset_proxy(inactive_proxy, inactive_profile)
            p.performCollisionDetection()
            active_near = p.getClosestPoints(
                active_proxy, placed, clearance)
            inactive_near = p.getClosestPoints(
                inactive_proxy, placed, clearance)
            active_distance = (min(float(row[8]) for row in active_near)
                               if active_near else clearance)
            inactive_distance = (min(float(row[8]) for row in inactive_near)
                                 if inactive_near else clearance)
            active_distances.append(active_distance)
            inactive_distances.append(inactive_distance)
        series = clearance_series_gate(
            active_distances, inactive_distances,
            minimum_clearance_m=clearance,
            exit_progress_tolerance_m=progress_tolerance,
            maximum_exit_frames=maximum_exit_frames)
        first_clear_frame = series["first_clear_frame"]
        kinematic_checks = {
            "source_collision_check_passed": True,
            "actual_release_pose_present": True,
            "release_event_frame_included": bool(
                frames and frames[0].get("release_event")),
            "trajectory_pulse_matches_shared_track_c": pulse_matches_profile,
            "validated_trajectory_v2_waypoints_match_input":
                waypoints_match_trajectory,
            "active_gripper_proxy_has_measured_verified_evidence":
                "VERIFIED" in str(profile.get(
                    "gripper_collision_status", "")),
            "inactive_gripper_proxy_has_measured_verified_evidence":
                "VERIFIED" in str(inactive_profile.get(
                    "gripper_collision_status", "")),
            **series["checks"],
            "all_post_release_frames_checked":
                len(active_distances) == len(frames),
        }
        kinematic_status = (
            "PASS" if all(kinematic_checks.values()) else "BLOCKED")
        return {
            "status": "BLOCKED",
            "kinematic_scaffold_status": kinematic_status,
            "authorization_blockers": [
                "no measured palm/two-finger opening-sweep geometry",
                "static massless object cannot prove release dynamics or stability",
                "gripper event actual-q uncertainty envelope is not qualified",
                "supplemental gate is not integrated into formal preflight",
            ],
            "checks": kinematic_checks,
            "source_collision_result_sha256": sha256_bytes(
                canonical_bytes(collision_result)),
            "trajectory_sha256": sha256_bytes(canonical_bytes(
                trajectory_document)),
            "actual_release_pose": copy.deepcopy(release_pose),
            "checked_frames": len(active_distances),
            "planned_post_release_frames": len(frames),
            "first_clear_frame": first_clear_frame,
            "frames_before_clear": first_clear_frame,
            "maximum_exit_frames": maximum_exit_frames,
            "time_to_clear_ms": (None if first_clear_frame is None else
                                 first_clear_frame * period),
            "minimum_clearance_after_exit_m": (
                None if first_clear_frame is None else
                min(active_distances[first_clear_frame:])),
            "final_active_clearance_m": (
                active_distances[-1] if active_distances else None),
            "minimum_inactive_clearance_m": (
                min(inactive_distances) if inactive_distances else None),
            "required_clearance_m": clearance,
            "exit_progress_tolerance_m": progress_tolerance,
            "exit_regressions": series["exit_regressions"][:20],
            "reentries": series["reentries"][:20],
            "inactive_violations": series["inactive_violations"][:20],
            "post_release_segments": list(dict.fromkeys(row["seg"] for row in frames)),
        }
    finally:
        p.disconnect(client)
