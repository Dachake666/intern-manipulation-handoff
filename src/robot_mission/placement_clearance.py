#!/usr/bin/env python3
"""保守的放置/hover 高度计算，不构成轨迹或真机安全授权。

输入已经是 SDK endpoint 位姿；本模块不再执行 endpoint/TCP 补偿。所有
长度单位为 mm，矩阵记号 ``T_A_B`` 将 B 系坐标变换到 A 系。物体坐标原点
必须是实际包围盒的几何中心，不能直接把未知偏移的相机特征点当作中心。

调用方必须提供覆盖腕部、工具及张开夹爪的保守包络顶点，并记录其来源与
适用关节姿态。腕部多关节（如 link9）不永远刚性附着于 endpoint；应对
本次 IK/完整关节路径重新计算包络或提供覆盖这些姿态的联合包络。本模块
只按给定包络计算，不能证明输入包络真的覆盖了腕部或证明路径无碰撞。

策略为整个工具/腕部包络都保持在水平框沿最高处之上，因而较保守；框内
XY、整臂、携物横移、释放后退离、可达性和关节限位仍需主链路验证。
"""
from __future__ import annotations

import itertools
import math

import numpy as np

from robot_mission.grasp_pose import validate_rigid_transform


def _scalar(value, name, *, nonnegative=False, positive=False):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} 必须是有限数值")
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{name} 必须是有限数值") from exc
    if not math.isfinite(result):
        raise ValueError(f"{name} 必须是有限数值")
    if positive and result <= 0:
        raise ValueError(f"{name} 必须大于 0")
    if nonnegative and result < 0:
        raise ValueError(f"{name} 不得为负数")
    return result


def _points(value, name):
    try:
        result = np.asarray(value, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} 必须是非空有限 N x 3 点云") from exc
    if (result.ndim != 2 or result.shape[1] != 3 or len(result) == 0
            or not np.all(np.isfinite(result))):
        raise ValueError(f"{name} 必须是非空有限 N x 3 点云")
    return result


def compute_placement_clearance(
        *, T_world_endpoint, envelope_points_endpoint_mm, T_endpoint_object,
        object_dimensions_mm, bin_rim_z_mm, table_surface_z_mm,
        rim_clearance_mm, held_clearance_mm, hover_z_mm,
        minimum_hover_gap_mm, support_z_mm, release_mode,
        max_drop_mm=None, support_tolerance_mm):
    """仅向上调整 release，并独立计算 hover 的最低高度。

    ``T_endpoint_object`` 描述抓取后实际物体几何中心相对 endpoint 的刚性
    附着关系；物体姿态随 ``T_world_endpoint`` 一起旋转。``hover_z_mm``
    是独立世界高度下限，不是 release 的固定增量。hover 还必须高于
    release 至少 ``minimum_hover_gap_mm``，且所持物的最低角点高于框沿
    /桌面的较高者加 ``held_clearance_mm``。

    ``SUPPORTED_ONLY`` 仅接受最低角点位于水平支持面容差范围内，不证明
    物体稳定落座。``ALLOW_BOUNDED_DROP`` 需显式非负 ``max_drop_mm``；
    抬高导致的下落间隙超过上限时返回 BLOCKED，不悄悄改成高空释放。

    返回两个 ndarray 位姿和可 JSON 化的标量/列表报告。无几何冲突时
    也只返回 CANDIDATE，永不授予真实运动权限。无效输入抛出 ValueError。
    不在这里猜测腕部包络、内底高度、尺寸、框沿或允许的下落距离。
    """
    T_release = validate_rigid_transform(
        T_world_endpoint, "T_world_endpoint").copy()
    T_attached = validate_rigid_transform(
        T_endpoint_object, "T_endpoint_object")
    envelope = _points(envelope_points_endpoint_mm,
                       "envelope_points_endpoint_mm")
    try:
        dimensions = np.asarray(object_dimensions_mm, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("object_dimensions_mm 必须是 3 个正的有限数值") from exc
    if (dimensions.shape != (3,) or not np.all(np.isfinite(dimensions))
            or np.any(dimensions <= 0)):
        raise ValueError("object_dimensions_mm 必须是 3 个正的有限数值")

    rim = _scalar(bin_rim_z_mm, "bin_rim_z_mm")
    table = _scalar(table_surface_z_mm, "table_surface_z_mm")
    rim_clearance = _scalar(rim_clearance_mm, "rim_clearance_mm", nonnegative=True)
    held_clearance = _scalar(held_clearance_mm, "held_clearance_mm", nonnegative=True)
    requested_hover = _scalar(hover_z_mm, "hover_z_mm")
    hover_gap = _scalar(minimum_hover_gap_mm, "minimum_hover_gap_mm", positive=True)
    support = _scalar(support_z_mm, "support_z_mm")
    support_tolerance = _scalar(support_tolerance_mm, "support_tolerance_mm",
                                nonnegative=True)
    if support > rim:
        raise ValueError("support_z_mm 不得高于 bin_rim_z_mm")
    if release_mode not in ("SUPPORTED_ONLY", "ALLOW_BOUNDED_DROP"):
        raise ValueError("release_mode 必须是 SUPPORTED_ONLY 或 ALLOW_BOUNDED_DROP")
    if release_mode == "ALLOW_BOUNDED_DROP":
        if max_drop_mm is None:
            raise ValueError("ALLOW_BOUNDED_DROP 必须显式提供 max_drop_mm")
        maximum_drop = _scalar(max_drop_mm, "max_drop_mm", nonnegative=True)
    elif max_drop_mm is not None:
        maximum_drop = _scalar(max_drop_mm, "max_drop_mm", nonnegative=True)
        if maximum_drop != 0:
            raise ValueError("SUPPORTED_ONLY 不得设置非零 max_drop_mm")
    else:
        maximum_drop = 0.0

    rotation = T_release[:3, :3]
    # 局部点云先随 endpoint 姿态旋转；不使用固定世界 Z 的 TCP 偏置。
    envelope_offset_z = float(np.min((envelope @ rotation.T)[:, 2]))
    object_corners = np.asarray(list(itertools.product((-0.5, 0.5), repeat=3)))
    object_corners *= dimensions
    object_points_endpoint = (
        object_corners @ T_attached[:3, :3].T + T_attached[:3, 3])
    object_offset_z = float(np.min((object_points_endpoint @ rotation.T)[:, 2]))
    if not all(math.isfinite(v) for v in (envelope_offset_z, object_offset_z)):
        raise ValueError("包络或物体变换后产生非有限数值")

    requested_release = float(T_release[2, 3])
    minimum_release = rim + rim_clearance - envelope_offset_z
    release = max(requested_release, minimum_release)
    minimum_hover = max(release + hover_gap,
                        max(rim, table) + held_clearance - object_offset_z)
    hover = max(requested_hover, minimum_hover)
    object_bottom = release + object_offset_z
    release_gap = object_bottom - support
    if not all(math.isfinite(v) for v in (
            minimum_release, release, minimum_hover, hover,
            object_bottom, release_gap)):
        raise ValueError("放置高度计算产生非有限数值")

    blockers = []
    if release_gap < -support_tolerance:
        blockers.append(
            f"release_object_below_support: 物体最低点穿过支持面 {-release_gap:.3f} mm")
    if release_mode == "SUPPORTED_ONLY" and release_gap > support_tolerance:
        blockers.append(
            f"release_not_supported: 物体距支持面 {release_gap:.3f} mm，禁止高空松爪")
    elif release_mode == "ALLOW_BOUNDED_DROP" and release_gap > maximum_drop:
        blockers.append(
            f"release_drop_exceeds_max: 下落间隙 {release_gap:.3f} mm "
            f"> 显式上限 {maximum_drop:.3f} mm")

    T_release[2, 3] = release
    T_hover = T_release.copy()
    T_hover[2, 3] = hover
    return {
        "T_world_release_endpoint": T_release,
        "T_world_hover_endpoint": T_hover,
        "strategy": "KEEP_ENVELOPE_ABOVE_RIM",
        "status": "BLOCKED" if blockers else "CANDIDATE",
        "real_motion_authorized": False,
        "requested_release_z_mm": requested_release,
        "minimum_release_z_mm": float(minimum_release),
        "release_z_mm": float(release),
        "release_raise_mm": float(release - requested_release),
        "requested_hover_z_mm": requested_hover,
        "minimum_hover_z_mm": float(minimum_hover),
        "hover_z_mm": float(hover),
        "hover_raise_mm": float(hover - requested_hover),
        "release_to_hover_mm": float(hover - release),
        "envelope_lowest_offset_world_z_mm": envelope_offset_z,
        "envelope_min_z_mm": float(release + envelope_offset_z),
        "held_object_lowest_offset_world_z_mm": object_offset_z,
        "held_object_bottom_z_mm": float(object_bottom),
        "hover_held_object_bottom_z_mm": float(hover + object_offset_z),
        "release_gap_mm": float(release_gap),
        "release_mode": release_mode,
        "max_drop_mm": float(maximum_drop),
        "blockers": blockers,
    }
