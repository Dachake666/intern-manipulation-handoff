#!/usr/bin/env python3
"""SDK-world 桌上抓放候选计划生成器（不连接、不使能、不运动机器人）。

相机侧每次只需更新 ``observation.pose``；固定料框和安全起点放在同一请求的
``station`` 中，后续可由上层长期保存。物体类别变化时，更新一次物体尺寸和
``T_object_grasp``，不需要人工重填本次物体坐标。

本模块只生成 ``tabletop_plan.v1`` 离线候选。真机前仍必须经过 live
``armTryWorlds``、控制器限位和碰撞完整预检；这里绝不把候选标成可执行。
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

import arm_profiles
from .contracts import (
    TABLETOP_STAGE_CONTRACT,
    canonical_bytes,
    sha256_file,
    validate_document,
)
from .grasp_pose import (
    POSE_ENDPOINT,
    POSE_GRASP,
    POSE_OBJECT,
    POSE_ROLES,
    matrix_to_sdk_uvw_deg,
    pose_from_matrix,
    pose_matrix,
    resolve_sdk_endpoint_pose,
    rigid_inverse,
    sdk_endpoint_to_grasp_transform,
    validate_rigid_transform,
)
from .placement_clearance import compute_placement_clearance


REQUEST_SCHEMA = "tabletop_request.v1"
PLAN_SCHEMA = "tabletop_plan.v1"
WORLD_FRAME = "sdk_world"


def _number(value, label):
    out = float(value)
    if not math.isfinite(out):
        raise ValueError(f"{label} 必须是有限数值")
    return out


def _positive_vector(values, label):
    out = np.asarray(values, dtype=float)
    if out.shape != (3,) or not np.all(np.isfinite(out)) or np.any(out <= 0):
        raise ValueError(f"{label} 必须是 3 个正有限数值")
    return out


def _matrix_list(value):
    return [[float(v) for v in row] for row in np.asarray(value, dtype=float)]


def _pose_with_uvw(T):
    pose = pose_from_matrix(T)
    pose["sdk_world_uvw_deg"] = [
        float(v) for v in matrix_to_sdk_uvw_deg(np.asarray(T)[:3, :3])]
    return pose


def _world_stage(name, T, *, carrying, critical=True):
    return {
        "name": name,
        "kind": "MOVE_WORLDS",
        "pose": _pose_with_uvw(T),
        "carrying": bool(carrying),
        "interpolation_en": False,
        "wait_until_worlds": True,
        "critical": bool(critical),
    }


def _gripper_stage(name, action, carrying_after, policy_id):
    return {"name": name, "kind": "GRIPPER", "action": action,
            "policy_id": policy_id, "carrying_after": bool(carrying_after),
            "wait_until_done": True}


def _translate_world_z(T, target_z_mm):
    out = np.array(validate_rigid_transform(T), copy=True)
    out[2, 3] = float(target_z_mm)
    return out


def _box_min_world_z(T_world_object, dimensions_mm):
    T = validate_rigid_transform(T_world_object, "T_world_object")
    half = _positive_vector(dimensions_mm, "object.dimensions_mm") / 2.0
    corners = np.array([[x, y, z, 1.0]
                        for x in (-half[0], half[0])
                        for y in (-half[1], half[1])
                        for z in (-half[2], half[2])])
    return float(np.min((T @ corners.T).T[:, 2]))


def _require_pose_source(item, label):
    if not isinstance(item, dict):
        raise ValueError(f"{label} 必须是对象")
    if item.get("frame_id") != WORLD_FRAME:
        raise ValueError(f"{label}.frame_id 必须显式为 sdk_world")
    if item.get("length_unit") != "millimeter":
        raise ValueError(f"{label}.length_unit 必须为 millimeter")
    if item.get("quaternion_order") != "xyzw":
        raise ValueError(f"{label}.quaternion_order 必须为 xyzw")
    role = item.get("pose_role")
    if role not in POSE_ROLES:
        raise ValueError(f"{label}.pose_role 必须是 {sorted(POSE_ROLES)}")
    pose = item.get("pose")
    if not isinstance(pose, dict):
        raise ValueError(f"{label}.pose 缺失")
    return role, pose


def _resolved_endpoint(item, label, T_endpoint_grasp, T_object_grasp,
                       fixed_endpoint_quaternion):
    role, pose = _require_pose_source(item, label)
    kwargs = {"T_endpoint_grasp": T_endpoint_grasp}
    if role == POSE_OBJECT:
        kwargs["T_object_grasp"] = T_object_grasp
    if role != POSE_ENDPOINT and fixed_endpoint_quaternion is not None:
        kwargs["fixed_endpoint_quaternion_xyzw"] = fixed_endpoint_quaternion
    return resolve_sdk_endpoint_pose(pose, role, **kwargs)


def _source_object_pose(item, resolved, T_endpoint_grasp, T_object_grasp):
    """恢复抓取瞬间的实际物体姿态，用于冻结刚性附件关系。"""
    role, source_pose = _require_pose_source(item, "pose source")
    T_source = pose_matrix(source_pose["position_mm"],
                           source_pose["quaternion_xyzw"])
    if role == POSE_OBJECT:
        return T_source
    if role == POSE_GRASP:
        return T_source @ rigid_inverse(T_object_grasp)
    return (resolved["T_world_endpoint"] @
            validate_rigid_transform(T_endpoint_grasp) @
            rigid_inverse(T_object_grasp))


def _place_source_from_station(station):
    """返回最终放置抓取点；料框输入用 T_W_G=T_W_B*T_B_G 自动解析。"""
    if "place" in station:
        return station["place"], {
            "kind": "EXPLICIT_PLACE",
            "description": "调用方直接给出 sdk_world 放置点",
        }
    box = station.get("material_box")
    if not isinstance(box, dict):
        raise ValueError("station 必须且只能提供 place 或 material_box")
    outer = _positive_vector(box.get("dimensions_mm"),
                             "station.material_box.dimensions_mm")
    inner = _positive_vector(box.get("inner_dimensions_mm"),
                             "station.material_box.inner_dimensions_mm")
    if np.any(inner > outer + 1e-9):
        raise ValueError("料框 inner_dimensions_mm 不能大于外形 dimensions_mm")
    margin = _number(box.get("placement_wall_margin_mm"),
                     "station.material_box.placement_wall_margin_mm")
    if margin < 0:
        raise ValueError("placement_wall_margin_mm 不能为负")
    T_box_place = validate_rigid_transform(
        box.get("T_box_place_grasp"), "station.material_box.T_box_place_grasp")
    local = T_box_place[:3, 3]
    usable_xy = inner[:2] / 2.0 - margin
    if np.any(usable_xy <= 0) or np.any(np.abs(local[:2]) > usable_xy + 1e-9):
        raise ValueError(
            "料框局部放置点超出扣除 placement_wall_margin_mm 后的框内 XY 区域")
    pose = box.get("pose", {})
    T_world_box = pose_matrix(pose.get("position_mm"),
                              pose.get("quaternion_xyzw"))
    T_world_place = T_world_box @ T_box_place
    source = {
        "frame_id": WORLD_FRAME,
        "length_unit": "millimeter",
        "quaternion_order": "xyzw",
        "pose_role": POSE_GRASP,
        "pose": pose_from_matrix(T_world_place),
    }
    return source, {
        "kind": "MATERIAL_BOX_LOCAL_OFFSET",
        "datum_role": box["datum_role"],
        "geometry_status": box["geometry_status"],
        "dimensions_mm": [float(v) for v in outer],
        "inner_dimensions_mm": [float(v) for v in inner],
        "placement_wall_margin_mm": margin,
        "T_box_place_grasp": _matrix_list(T_box_place),
    }


def _same_pose(a, b, *, position=True):
    if position and not np.allclose(a["position_mm"], b["position_mm"], atol=1e-9):
        return False
    return np.allclose(
        pose_matrix([0, 0, 0], a["quaternion_xyzw"])[:3, :3],
        pose_matrix([0, 0, 0], b["quaternion_xyzw"])[:3, :3],
        atol=1e-9)


def _validate_stage_invariants(stages, pick_mode="VERTICAL"):
    by_name = {x["name"]: x for x in stages}
    for stage, expected in zip(stages, TABLETOP_STAGE_CONTRACT):
        name, kind, action, state_key, state_value = expected
        if (stage["name"] != name or stage["kind"] != kind or
                (action is not None and stage.get("action") != action) or
                stage.get(state_key) is not state_value):
            raise AssertionError("桌上抓放状态机动作或夹持状态漂移")
    if any(x["kind"] == "MOVE_J" and x.get("carrying") for x in stages):
        raise AssertionError("首版禁止持物 MoveJ")
    approach = by_name["PICK_HOVER"]["pose"]["position_mm"]
    pick = by_name["PICK_DESCEND"]["pose"]["position_mm"]
    ascend = by_name["PICK_ASCEND"]["pose"]["position_mm"]
    if not np.allclose(pick[:2], ascend[:2], atol=1e-9, rtol=0) or ascend[2] <= pick[2]:
        raise AssertionError("抓取后必须同 XY 沿 +Z 抬升")
    if pick_mode == "HORIZONTAL_X_POSITIVE":
        if approach[0] >= pick[0] or not np.allclose(approach[1:], pick[1:], atol=1e-9, rtol=0):
            raise AssertionError("水平接近必须纯 +X，锁定 Y/Z")
    for name in ("PLACE_ASCEND",):
        if not _same_pose(by_name["PLACE_HOVER"]["pose"],
                          by_name[name]["pose"]):
            raise AssertionError("放置上方只能有一个 hover 点，转运/抬升必须复用该点")
    if not _same_pose(by_name["RETURN_SAFE"]["pose"],
                      by_name["END_ASSERT_SAFE"]["pose"]):
        raise AssertionError("最终 MoveWorlds 必须回到用户给出的 safe endpoint")
    vertical_pairs = [("PLACE_DESCEND", "PLACE_HOVER")]
    if pick_mode == "VERTICAL":
        vertical_pairs.append(("PICK_DESCEND", "PICK_HOVER"))
    for low, high in vertical_pairs:
        a, b = by_name[low]["pose"], by_name[high]["pose"]
        if not np.allclose(a["position_mm"][:2], b["position_mm"][:2], atol=1e-9):
            raise AssertionError(f"{low} 必须是同 XY 竖直段")
        if not np.allclose(
                pose_matrix([0, 0, 0], a["quaternion_xyzw"])[:3, :3],
                pose_matrix([0, 0, 0], b["quaternion_xyzw"])[:3, :3],
                atol=1e-9):
            raise AssertionError(f"{low} 必须锁定姿态")
    for stage in stages:
        if stage["kind"] == "MOVE_WORLDS" and (
                stage["interpolation_en"] or not stage["wait_until_worlds"]):
            raise AssertionError("关键 MoveWorlds 必须非混合且逐段等到位")
    move_poses = [stage["pose"] for stage in stages
                  if stage["kind"] in ("MOVE_WORLDS", "ASSERT_ENDPOINT")]
    if any(not _same_pose(move_poses[0], pose, position=False)
           for pose in move_poses[1:]):
        raise AssertionError("MoveWorlds 首版要求全程固定 endpoint 姿态")


def build_tabletop_plan(request):
    """校验请求并生成不可直接授权真机的 ``tabletop_plan.v1``。"""
    validate_document(request, REQUEST_SCHEMA)
    if request.get("schema_version") != REQUEST_SCHEMA:
        raise ValueError(f"schema_version 必须为 {REQUEST_SCHEMA}")
    if request.get("arm") != "left":
        raise ValueError("当前 endpoint/TCP 证据只允许 left")
    request_id = str(request.get("request_id", "")).strip()
    if not request_id:
        raise ValueError("request_id 不能为空")
    observation = request.get("observation", {})
    if not str(observation.get("observation_id", "")).strip():
        raise ValueError("observation.observation_id 不能为空")
    gripper_policy = request["gripper_policy"]
    gripper_policy_id = gripper_policy["policy_id"]

    obj = request.get("object", {})
    dimensions_mm = _positive_vector(obj.get("dimensions_mm"),
                                     "object.dimensions_mm")
    T_object_grasp = validate_rigid_transform(
        obj.get("T_object_grasp"), "object.T_object_grasp")
    fixed_q = obj.get("fixed_endpoint_quaternion_xyzw")
    orientation_policy = obj.get("orientation_policy")
    if orientation_policy != "FIXED_ENDPOINT_ORIENTATION" or fixed_q is None:
        raise ValueError(
            "MoveWorlds 首版为保证姿态稳定，只接受 FIXED_ENDPOINT_ORIENTATION，"
            "且必须给 fixed_endpoint_quaternion_xyzw")

    transforms = request.get("transforms", {})
    if transforms.get("T_endpoint_grasp") is None:
        T_endpoint_grasp, endpoint_source = sdk_endpoint_to_grasp_transform("left")
    else:
        T_endpoint_grasp = validate_rigid_transform(
            transforms["T_endpoint_grasp"], "transforms.T_endpoint_grasp")
        endpoint_source = transforms.get("endpoint_transform_source")
        if not isinstance(endpoint_source, dict) or not endpoint_source.get("status"):
            raise ValueError("自定义 T_endpoint_grasp 必须附 endpoint_transform_source.status")

    pick = _resolved_endpoint(observation, "observation", T_endpoint_grasp,
                              T_object_grasp, fixed_q)
    station = request.get("station", {})
    place_source, place_provenance = _place_source_from_station(station)
    place = _resolved_endpoint(place_source, "station.place",
                               T_endpoint_grasp, T_object_grasp, fixed_q)
    safe_role, safe_pose = _require_pose_source(
        station.get("safe_endpoint", {}), "station.safe_endpoint")
    if safe_role != POSE_ENDPOINT:
        raise ValueError("station.safe_endpoint 必须是 EE_POSE")
    safe = resolve_sdk_endpoint_pose(safe_pose, POSE_ENDPOINT)

    T_pick = pick["T_world_endpoint"]
    T_place = place["T_world_endpoint"]
    T_safe = safe["T_world_endpoint"]
    fixed_rotation = pose_matrix([0, 0, 0], fixed_q)[:3, :3]
    if not np.allclose(T_safe[:3, :3], fixed_rotation, atol=1e-8, rtol=0):
        raise ValueError(
            "safe_endpoint 姿态必须等于 fixed_endpoint_quaternion_xyzw；"
            "首版全程不改变 endpoint 姿态")
    minimum_lift = _number(request.get("motion_policy", {}).get("minimum_lift_mm"),
                           "motion_policy.minimum_lift_mm")
    if minimum_lift <= 0:
        raise ValueError("minimum_lift_mm 必须 > 0")
    pick_hover_offset = _number(
        request.get("motion_policy", {}).get("pick_hover_offset_mm"),
        "motion_policy.pick_hover_offset_mm")
    place_hover_offset = _number(
        request.get("motion_policy", {}).get("place_hover_offset_mm"),
        "motion_policy.place_hover_offset_mm")
    placement_policy = request["motion_policy"].get("placement_clearance")
    if pick_hover_offset < minimum_lift or (
            placement_policy is None and place_hover_offset < minimum_lift):
        raise ValueError("两个 hover 抬高量都必须 >= minimum_lift_mm")

    T_pick_hover = _translate_world_z(
        T_pick, float(T_pick[2, 3]) + pick_hover_offset)
    pick_mode = request["motion_policy"].get("pick_mode", "VERTICAL")
    T_pick_ascend = T_pick_hover.copy()
    if pick_mode == "HORIZONTAL_X_POSITIVE":
        distance = _number(request["motion_policy"].get("approach_distance_mm"), "approach_distance_mm")
        lift = _number(request["motion_policy"].get("pick_ascend_offset_mm"), "pick_ascend_offset_mm")
        if distance <= 0 or lift <= 0:
            raise ValueError("水平接近距离、抬升高度必须为正数")
        T_pick_hover = T_pick.copy()
        T_pick_hover[0, 3] -= distance
        T_pick_ascend = _translate_world_z(T_pick, float(T_pick[2, 3]) + lift)
    T_place_hover = _translate_world_z(
        T_place, float(T_place[2, 3]) + place_hover_offset)
    # 物体闭爪后是 endpoint 的刚性附件。固定工具姿态时，不能再用理想
    # T_object_grasp 反推每个路径点，否则会把物体旋转成一个并未真实发生的姿态。
    T_pick_object = _source_object_pose(
        observation, pick, T_endpoint_grasp, T_object_grasp)
    T_endpoint_object_at_pick = rigid_inverse(T_pick) @ T_pick_object
    # 相机 feature 原点未必是瓶子几何中心。尺寸包络必须使用独立的几何变换。
    T_object_geometry = validate_rigid_transform(
        obj.get("T_object_geometry", np.eye(4)), "object.T_object_geometry")
    T_endpoint_geometry = T_endpoint_object_at_pick @ T_object_geometry
    top = _number(station.get("table_surface_z_mm"),
                  "station.table_surface_z_mm")
    rim = _number(station.get("bin_rim_z_mm"), "station.bin_rim_z_mm")
    clearance = _number(request["motion_policy"].get("held_clearance_mm"),
                        "motion_policy.held_clearance_mm")
    placement_blockers = []
    placement_report = {"status": "MISSING", "real_motion_authorized": False}
    if placement_policy is None:
        placement_blockers.append("缺少放置段手腕/开爪包络及支持面净空校验")
    else:
        result = compute_placement_clearance(
            T_world_endpoint=T_place,
            envelope_points_endpoint_mm=placement_policy["envelope_points_endpoint_mm"],
            T_endpoint_object=T_endpoint_geometry,
            object_dimensions_mm=dimensions_mm,
            bin_rim_z_mm=rim, table_surface_z_mm=top,
            rim_clearance_mm=placement_policy["rim_clearance_mm"],
            held_clearance_mm=clearance,
            hover_z_mm=placement_policy["hover_z_mm"],
            minimum_hover_gap_mm=placement_policy["minimum_hover_gap_mm"],
            support_z_mm=placement_policy["support_z_mm"],
            release_mode=placement_policy["release_mode"],
            max_drop_mm=placement_policy["max_drop_mm"],
            support_tolerance_mm=placement_policy["support_tolerance_mm"])
        T_place = result.pop("T_world_release_endpoint")
        T_place_hover = result.pop("T_world_hover_endpoint")
        placement_report = result
        placement_report["policy"] = copy.deepcopy(placement_policy)
        placement_report["T_object_geometry"] = _matrix_list(T_object_geometry)
        placement_report["scope"] = (
            "CONSERVATIVE_PLACE_VERTICAL_ENVELOPE_ONLY_NOT_FULL_ROBOT_COLLISION")
        placement_report["released_object_pose_semantics"] = (
            "AT_OPEN_COMMAND_NOT_SETTLED_POSE")
        placement_report["wrist_envelope_path_binding"] = "PENDING_DENSE_IK_REVALIDATION"
        placement_blockers.extend(result["blockers"])
        if (placement_policy["envelope_status"] != "CONFIRMED" or
                not placement_policy["covers_wrist_and_open_gripper"]):
            placement_blockers.append("放置上下行的完整手腕及开爪包络尚未确认")
        if ("T_object_geometry" not in obj or
                obj.get("object_geometry_transform_status") != "CONFIRMED"):
            placement_blockers.append("相机 feature 到物体几何中心的变换尚未确认")
        if placement_policy["support_status"] != "CONFIRMED":
            placement_blockers.append("料框内底支持面的世界高度尚未确认")
        # 不要求精确框中心：也支持一次确认并保存的 SDK-world 内缩可放区域。
        # 区域必须已经扣除框壁/定位误差；检查整个物体投影，而非仅检查 TCP 点。
        half = dimensions_mm / 2.0
        corners = np.array([[x, y, z, 1.] for x in (-half[0], half[0])
                            for y in (-half[1], half[1])
                            for z in (-half[2], half[2])])
        box = station.get("material_box")
        region = station.get("place_region")
        if region is not None:
            bounds = np.asarray([region["x_bounds_mm"], region["y_bounds_mm"]], float)
            if (bounds.shape != (2, 2) or not np.all(np.isfinite(bounds)) or
                    np.any(bounds[:, 1] <= bounds[:, 0])):
                raise ValueError("station.place_region 必须是有限且有正面积的框内 XY 区域")
            world = (T_place @ T_endpoint_geometry @ corners.T).T
            inside = bool(np.all(world[:, :2] >= bounds[:, 0] - 1e-9) and
                          np.all(world[:, :2] <= bounds[:, 1] + 1e-9))
            placement_report["object_xy_inside_bin"] = inside
            placement_report["place_region"] = copy.deepcopy(region)
            if region["geometry_status"] != "CONFIRMED":
                placement_blockers.append("保存的框内可放区域尚未现场确认")
            if not inside:
                placement_blockers.append("物体完整投影超出保存的框内可放区域")
        elif not box or box["datum_role"] != "BOX_GEOMETRY_CENTER":
            placement_blockers.append("缺少当前框内可放区域或料框几何位姿，未证明物体完整落在框内")
        else:
            T_world_box = pose_matrix(box["pose"]["position_mm"],
                                      box["pose"]["quaternion_xyzw"])
            if not np.allclose(T_world_box[:3, 2], [0, 0, 1], atol=1e-6):
                placement_blockers.append("倾斜料框需三维支持面/重力落点校验，不能仅用水平内底高度")
            else:
                # inner_dimensions 的 z 是从内底到口沿的深度，不是外形中心两侧对称。
                model_rim = float(T_world_box[2, 3]) + float(box["dimensions_mm"][2]) / 2.0
                model_floor = model_rim - float(box["inner_dimensions_mm"][2])
                if (abs(rim - model_rim) > 1e-6 or abs(
                        float(placement_policy["support_z_mm"]) - model_floor) > 1e-6):
                    placement_blockers.append("料框位姿/尺寸推导的口沿或内底与提供的世界高度不一致")
            local = (rigid_inverse(T_world_box) @ T_place @
                     T_endpoint_geometry @ corners.T).T
            usable = np.asarray(box["inner_dimensions_mm"][:2]) / 2.0 - float(
                box["placement_wall_margin_mm"])
            inside = bool(np.all(np.abs(local[:, :2]) <= usable + 1e-9))
            placement_report["object_xy_inside_bin"] = inside
            if not inside:
                placement_blockers.append("物体完整几何包络超出料框内壁余量，不能只检查抓取点在框内")
        placement_report["blockers"] = list(placement_blockers)
        placement_report["status"] = "BLOCKED" if placement_blockers else "CANDIDATE"
    T_pick_object_hover = T_pick_ascend @ T_endpoint_object_at_pick
    T_place_object_hover = T_place_hover @ T_endpoint_object_at_pick
    T_place_object_actual = T_place @ T_endpoint_object_at_pick
    if place_source["pose_role"] == POSE_OBJECT:
        requested_pose = place_source["pose"]
        T_place_object_requested = pose_matrix(
            requested_pose["position_mm"], requested_pose["quaternion_xyzw"])
        if not np.allclose(T_place_object_actual, T_place_object_requested,
                           atol=1e-7, rtol=0):
            raise ValueError(
                "固定附件关系或净空抬高后无法实现 station.place 的严格完整 OBJECT_POSE；"
                "不会静默改变物体目标位姿，请改用兼容姿态或允许调整高度的 GRASP_POSE/EE_POSE")
    if clearance <= 0:
        raise ValueError("held_clearance_mm 必须 > 0")
    required_bottom_z = max(top, rim) + clearance
    pick_bottom_z = _box_min_world_z(T_pick_object_hover @ T_object_geometry, dimensions_mm)
    place_bottom_z = _box_min_world_z(T_place_object_hover @ T_object_geometry, dimensions_mm)
    if min(pick_bottom_z, place_bottom_z) + 1e-9 < required_bottom_z:
        raise ValueError(
            "安全起点/走廊过低：持物最低点低于桌面或料框口沿加余量 "
            f"({min(pick_bottom_z, place_bottom_z):.3f} < {required_bottom_z:.3f}mm)")

    stages = [
        {"name": "START_ASSERT_SAFE", "kind": "ASSERT_ENDPOINT",
         "pose": _pose_with_uvw(T_safe), "carrying": False},
        _gripper_stage("OPEN_BEFORE_PICK", "open", False, gripper_policy_id),
        _world_stage("PICK_HOVER", T_pick_hover, carrying=False),
        _world_stage("PICK_DESCEND", T_pick, carrying=False),
        _gripper_stage("CLOSE_AT_PICK", "close", True, gripper_policy_id),
        _world_stage("PICK_ASCEND", T_pick_ascend, carrying=True),
        _world_stage("PLACE_HOVER", T_place_hover, carrying=True),
        _world_stage("PLACE_DESCEND", T_place, carrying=True),
        _gripper_stage("OPEN_AT_PLACE", "open", False, gripper_policy_id),
        _world_stage("PLACE_ASCEND", T_place_hover, carrying=False),
        _world_stage("RETURN_SAFE", T_safe, carrying=False),
        {"name": "END_ASSERT_SAFE", "kind": "ASSERT_ENDPOINT",
         "pose": _pose_with_uvw(T_safe), "carrying": False},
    ]
    _validate_stage_invariants(stages, pick_mode)

    profile_path = Path(arm_profiles.__file__).with_name("arm_profiles.v1.json")
    endpoint_matrix = _matrix_list(T_endpoint_grasp)
    object_matrix = _matrix_list(T_object_grasp)
    request_sha = hashlib.sha256(canonical_bytes(request)).hexdigest()
    # 计划必须冻结本次现场参数，避免调用方随后修改 request 时让已生成计划静默漂移。
    plan_gripper_policy = copy.deepcopy(gripper_policy)
    plan_gripper_policy["source_sha256"] = hashlib.sha256(
        canonical_bytes(gripper_policy)).hexdigest()
    blockers = list(request["input_status"]["blocking_assumptions"])
    assumed_components = [
        label for label, status in (
            ("object.dimensions_mm", obj["dimensions_status"]),
            ("object.T_object_grasp", obj["object_grasp_transform_status"]),
            ("station.table_surface_z_mm", station["table_surface_status"]),
            ("station.bin_rim_z_mm", station["bin_rim_status"]),
            ("station.material_box", place_provenance.get("geometry_status", "CONFIRMED")),
        ) if status != "CONFIRMED"
    ]
    if request["input_status"]["geometry_status"] != "CONFIRMED" and not blockers:
        raise ValueError("SIMULATION_ASSUMPTION 必须列出 blocking_assumptions")
    if request["input_status"]["geometry_status"] == "CONFIRMED" and blockers:
        raise ValueError("geometry_status=CONFIRMED 时 blocking_assumptions 必须为空")
    if request["input_status"]["geometry_status"] == "CONFIRMED" and assumed_components:
        raise ValueError(
            "仍有 SIMULATION_ASSUMPTION 组件，input_status.geometry_status 不能标 CONFIRMED: "
            + ", ".join(assumed_components))
    blockers = list(dict.fromkeys(blockers + placement_blockers))
    if (transforms.get("T_endpoint_grasp") is not None and
            endpoint_source.get("status") != "CONFIRMED"):
        blockers.append("自定义 endpoint/TCP 到抓点的变换尚未确认")
    plan = {
        "schema_version": PLAN_SCHEMA,
        "pick_mode": pick_mode,
        "plan_id": f"{request_id}-{request_sha[:12]}",
        "status": ("OFFLINE_CANDIDATE_BLOCKED" if blockers else
                   "OFFLINE_CANDIDATE_PRECHECK_REQUIRED"),
        "real_motion_authorized": False,
        "contract": {
            "arm": "left", "arm_id": 1, "gripper_id": 2,
            "world_frame": WORLD_FRAME, "length_unit": "millimeter",
            "quaternion_order": "xyzw", "sdk_uvw_convention": "Rz(W)*Ry(V)*Rx(U)",
            "dynamic_transport": "MOVE_WORLDS_ONLY",
            "move_j_policy": "VERIFIED_FIXED_SEGMENTS_ONLY_NOT_USED_IN_THIS_PLAN",
            "runtime_authorization": "RUN_FLAG_PLUS_ENV_GATE_PLUS_OPERATOR_CONFIRMATION",
        },
        "input": {
            "request_id": request_id,
            "request_sha256": request_sha,
            "observation_id": observation["observation_id"],
            "object_id": obj.get("object_id"), "class_id": obj.get("class_id"),
            "dimensions_mm": [float(v) for v in dimensions_mm],
            "pick_pose_role": observation["pose_role"],
            "place_pose_role": place_source["pose_role"],
            "place_source_kind": place_provenance["kind"],
            "station_id": station.get("station_id"),
        },
        "gripper_policy": plan_gripper_policy,
        "transforms": {
            "notation": "T_A_B maps coordinates in B into A",
            "T_endpoint_grasp": endpoint_matrix,
            "T_endpoint_grasp_sha256": hashlib.sha256(
                canonical_bytes(endpoint_matrix)).hexdigest(),
            "endpoint_transform_source": endpoint_source,
            "T_object_grasp": object_matrix,
            "T_object_grasp_sha256": hashlib.sha256(
                canonical_bytes(object_matrix)).hexdigest(),
            "T_endpoint_object_at_pick": _matrix_list(T_endpoint_object_at_pick),
            "orientation_policy": orientation_policy,
            "place_source": place_provenance,
        },
        "resolved": {
            "pick_endpoint": _pose_with_uvw(T_pick),
            "pick_hover_endpoint": _pose_with_uvw(T_pick_hover),
            "pick_ascend_endpoint": _pose_with_uvw(T_pick_ascend),
            "place_endpoint": _pose_with_uvw(T_place),
            "place_hover_endpoint": _pose_with_uvw(T_place_hover),
            "place_object_after_release": _pose_with_uvw(T_place_object_actual),
            "safe_endpoint": _pose_with_uvw(T_safe),
        },
        "stages": stages,
        "safety": {
            "input_geometry_gate": "BLOCKED" if blockers else "PASS",
            "input_blockers": blockers,
            "structural_sequence_check": "PASS",
            "held_object_hover_endpoint_check": "PASS",
            "held_object_hover_endpoint_bottom_min_mm": min(pick_bottom_z, place_bottom_z),
            "held_object_hover_endpoint_bottom_required_mm": required_bottom_z,
            "placement_clearance": placement_report,
            "dense_swept_held_object_clearance": "PENDING",
            "table_bin_volume_geometry": "PENDING",
            "live_armTryWorlds_dense_precheck": "PENDING",
            "controller_limit_intersection": "PENDING",
            "robot_gripper_held_object_collision_preflight": "PENDING",
            "gui_review": "PENDING",
            "real_robot_evidence": "PENDING",
        },
        "provenance": {
            "arm_profiles_path": str(profile_path),
            "arm_profiles_sha256": sha256_file(profile_path),
            "generator_path": str(Path(__file__).resolve()),
            "generator_sha256": sha256_file(Path(__file__).resolve()),
        },
    }
    if "startup_policy" in request:
        plan["startup_policy"] = copy.deepcopy(request["startup_policy"])
    validate_document(plan, PLAN_SCHEMA)
    return plan


def build_working_reference_variant(reference, recipe, *, reference_sha256):
    """从有原件哈希的 EE 工作参考构建新候选；不把现场成功外推成新路径净空 PASS。"""
    from frame_calibration.robot_side.execute_tabletop_pick_place_worlds import validate_plan
    validate_plan(reference)
    expected = {"schema_version", "source_sha256", "summary_source", "reported_latest",
                "pick_endpoint_mm", "approach_distance_mm", "pick_lift_mm",
                "place_endpoint_mm", "place_hover_endpoint_mm", "startup_policy"}
    if set(recipe) != expected or recipe["schema_version"] != "tabletop_working_reference_update.v1":
        raise ValueError("working-reference recipe 字段缺失或含未知字段")
    if reference_sha256 != recipe["source_sha256"]:
        raise ValueError("工作参考 SHA-256 不匹配，拒绝在未知文件上改点")
    from frame_calibration.robot_side.execute_tabletop_pick_place_worlds import _finite_vector
    pick = _finite_vector(recipe["pick_endpoint_mm"], 3, "pick_endpoint_mm")
    place = _finite_vector(recipe["place_endpoint_mm"], 3, "place_endpoint_mm")
    place_hover = _finite_vector(recipe["place_hover_endpoint_mm"], 3, "place_hover_endpoint_mm")
    distance, lift = _finite_vector([recipe["approach_distance_mm"], recipe["pick_lift_mm"]], 2, "距离/抬升")
    if distance <= 0 or lift <= 0:
        raise ValueError("水平距离与抬升必须大于 0")
    if place_hover[:2] != place[:2] or place_hover[2] <= place[2]:
        raise ValueError("放置 hover 必须在放置点正上方")
    plan = copy.deepcopy(reference)
    plan["pick_mode"] = "HORIZONTAL_X_POSITIVE"
    recipe_sha = hashlib.sha256(canonical_bytes(recipe)).hexdigest()
    plan["plan_id"] = f"horizontal-recovery-{recipe_sha[:12]}"
    plan["startup_policy"] = copy.deepcopy(recipe["startup_policy"])
    targets = {
        "pick_endpoint": pick,
        "pick_hover_endpoint": [pick[0] - distance, pick[1], pick[2]],
        "pick_ascend_endpoint": [pick[0], pick[1], pick[2] + lift],
        "place_endpoint": place,
        "place_hover_endpoint": place_hover,
    }
    for key, xyz in targets.items():
        pose = copy.deepcopy(plan["resolved"]["safe_endpoint"])
        pose["position_mm"] = xyz
        plan["resolved"][key] = pose
    from frame_calibration.robot_side.execute_tabletop_pick_place_worlds import RESOLVED_STAGE_KEYS
    for stage in plan["stages"]:
        if stage["kind"] != "GRIPPER":
            stage["pose"] = copy.deepcopy(plan["resolved"][RESOLVED_STAGE_KEYS[stage["name"]]])
    # 手改现场 EE 点后，旧相机 feature 及派生落点不能继续冒充新场景计算结果。
    plan["resolved"]["place_object_after_release"] = None
    plan["input"].update({"request_id": plan["plan_id"], "request_sha256": recipe_sha,
                          "observation_id": f"field-ee-{recipe_sha[:12]}",
                          "pick_pose_role": "EE_POSE", "place_pose_role": "EE_POSE",
                          "place_source_kind": "EXPLICIT_PLACE"})
    plan["transforms"]["place_source"] = {
        "kind": "EXPLICIT_PLACE", "description": "现场 EE 放置参考，不重新补偿 TCP"}
    plan["derivation"] = {
        "parent_plan_sha256": reference_sha256,
        "recipe_sha256": recipe_sha,
        "summary_source": recipe["summary_source"],
        "reported_latest": copy.deepcopy(recipe["reported_latest"]),
        "latest_artifacts_received": False,
        "inherited_object_geometry_usable": False,
        "reference_semantics": "EE_POSE_NO_SECOND_TCP_COMPENSATION",
        "note": "只重建用户明确提供的点位；新路径净空及现场新增参数尚待验证。",
    }
    plan["status"] = "OFFLINE_CANDIDATE_BLOCKED"
    plan["safety"].update({
        "input_geometry_gate": "BLOCKED",
        "input_blockers": ["新水平接近、持物抬升及斜线转运的整臂/夹爪/持瓶净空尚未确认"],
        "held_object_hover_endpoint_check": "PENDING",
        "held_object_hover_endpoint_bottom_min_mm": None,
        "held_object_hover_endpoint_bottom_required_mm": None,
        "dense_swept_held_object_clearance": "PENDING",
        "table_bin_volume_geometry": "PENDING",
        "live_armTryWorlds_dense_precheck": "PENDING",
        "controller_limit_intersection": "PENDING",
        "robot_gripper_held_object_collision_preflight": "PENDING",
        "gui_review": "PENDING", "real_robot_evidence": "PENDING",
    })
    plan["safety"].pop("placement_clearance", None)
    plan["provenance"]["generator_path"] = str(Path(__file__).resolve())
    plan["provenance"]["generator_sha256"] = sha256_file(Path(__file__).resolve())
    validate_document(plan, PLAN_SCHEMA)
    return plan


def verify_reported_baseline(reference_path, recipe):
    """对收到的 9/3 原件只在内存重建 9/5；只有精确文件哈希一致才记为重建成功。"""
    plan_text = reference_path.read_text(encoding="utf-8").replace("495.0", "445.0")
    executor_path = reference_path.with_name("execute_tabletop_pick_place_worlds.py")
    executor_text = executor_path.read_text(encoding="utf-8")
    for old, new in (
            ("SAFE_POSITION_TOL_MM = 20.0", "SAFE_POSITION_TOL_MM = 50.0"),
            ("SAFE_ORIENTATION_TOL_DEG = 8.0", "SAFE_ORIENTATION_TOL_DEG = 15.0"),
            ("0.1 <= speed <= 5.0", "0.1 <= speed <= 20.0"),
            ("--speed 必须在 [0.1,5.0]%", "--speed 必须在 [0.1,20.0]%")):
        executor_text = executor_text.replace(old, new)
    hashes = {"plan_sha256": hashlib.sha256(plan_text.encode()).hexdigest(),
              "executor_sha256": hashlib.sha256(executor_text.encode()).hexdigest()}
    if any(hashes[key] != recipe["reported_latest"][key] for key in hashes):
        raise ValueError("无法精确重建摘要中的最新基线哈希，停止发布")
    return hashes


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="把 sdk_world XYZ+xyzw 解析成桌上抓放 endpoint 候选计划；不连接机器人")
    parser.add_argument("request", help="tabletop_request.v1 或 working-reference recipe JSON")
    parser.add_argument("--working-reference", help="从已归档 EE 工作参考应用 recipe；保留其原件不动")
    parser.add_argument("--out", required=True, help="输出 tabletop_plan.v1 JSON")
    args = parser.parse_args(argv)
    request_path = Path(args.request)
    request = json.loads(request_path.read_text(encoding="utf-8"))
    if args.working_reference:
        reference_path = Path(args.working_reference)
        reference = json.loads(reference_path.read_text(encoding="utf-8"))
        plan = build_working_reference_variant(reference, request,
                                               reference_sha256=sha256_file(reference_path))
        plan["derivation"]["reconstructed_latest_file_hashes"] = verify_reported_baseline(reference_path, request)
        print("9/5 executor 与 Z445 JSON 均已从原件精确重建并匹配所报 SHA-256。")
    else:
        plan = build_tabletop_plan(request)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
                   encoding="utf-8")
    print(f"{plan['status']}: {plan['plan_id']} -> {out}")
    print("real_motion_authorized=false；仍需 live SDK + 碰撞完整预检")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
