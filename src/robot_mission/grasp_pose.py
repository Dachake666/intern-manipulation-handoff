#!/usr/bin/env python3
"""抓取姿态、SDK endpoint 补偿与 6D DLS 逆解。

矩阵记号统一为 ``T_A_B``：把 B 系坐标变换到 A 系。相机已经给出
``sdk_world`` 位姿时，本模块不会再应用手眼外参或 PB↔SDK 会话平移。
"""
from __future__ import annotations

import math

import numpy as np


POSE_OBJECT = "OBJECT_POSE"
POSE_GRASP = "GRASP_POSE"
POSE_ENDPOINT = "EE_POSE"
POSE_ROLES = frozenset((POSE_OBJECT, POSE_GRASP, POSE_ENDPOINT))


def _finite_vector(values, size, label):
    out = np.asarray(values, dtype=float)
    if out.shape != (size,) or not np.all(np.isfinite(out)):
        raise ValueError(f"{label} 必须是 {size} 个有限数值")
    return out


def normalize_quaternion_xyzw(values, max_norm_error=1e-3):
    """校验并轻微归一化 xyzw 四元数；固定符号以便 JSON/hash 稳定。"""
    q = _finite_vector(values, 4, "quaternion_xyzw")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12 or abs(norm - 1.0) > float(max_norm_error):
        raise ValueError(
            f"quaternion_xyzw 模长={norm:.9f}，与 1 的偏差超过 {max_norm_error}")
    q /= norm
    # q 与 -q 表示同一旋转；统一到 w>0（w=0 时首个非零分量为正）。
    if q[3] < 0 or (abs(q[3]) < 1e-15 and q[np.argmax(np.abs(q))] < 0):
        q = -q
    return q


def quaternion_xyzw_to_matrix(values):
    x, y, z, w = normalize_quaternion_xyzw(values)
    return np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ], dtype=float)


def validate_rigid_transform(value, label="transform", atol=1e-8):
    T = np.asarray(value, dtype=float)
    if T.shape != (4, 4) or not np.all(np.isfinite(T)):
        raise ValueError(f"{label} 必须是有限 4x4 矩阵")
    if not np.allclose(T[3], [0, 0, 0, 1], atol=atol, rtol=0):
        raise ValueError(f"{label} 末行必须是 [0,0,0,1]")
    R = T[:3, :3]
    if not np.allclose(R.T @ R, np.eye(3), atol=atol, rtol=0):
        raise ValueError(f"{label} 旋转块不是正交矩阵")
    if abs(float(np.linalg.det(R)) - 1.0) > atol:
        raise ValueError(f"{label} 旋转块 det 必须为 +1")
    return T


def matrix_to_quaternion_xyzw(rotation):
    R = np.asarray(rotation, dtype=float)
    T = np.eye(4); T[:3, :3] = R
    validate_rigid_transform(T, "rotation")
    trace = float(np.trace(R))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array([(R[2, 1] - R[1, 2]) / s,
                      (R[0, 2] - R[2, 0]) / s,
                      (R[1, 0] - R[0, 1]) / s, s / 4])
    else:
        i = int(np.argmax(np.diag(R)))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            q = np.array([s / 4, (R[0, 1] + R[1, 0]) / s,
                          (R[0, 2] + R[2, 0]) / s,
                          (R[2, 1] - R[1, 2]) / s])
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            q = np.array([(R[0, 1] + R[1, 0]) / s, s / 4,
                          (R[1, 2] + R[2, 1]) / s,
                          (R[0, 2] - R[2, 0]) / s])
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            q = np.array([(R[0, 2] + R[2, 0]) / s,
                          (R[1, 2] + R[2, 1]) / s, s / 4,
                          (R[1, 0] - R[0, 1]) / s])
    return normalize_quaternion_xyzw(q)


def pose_matrix(position_mm, quaternion_xyzw):
    T = np.eye(4)
    T[:3, :3] = quaternion_xyzw_to_matrix(quaternion_xyzw)
    T[:3, 3] = _finite_vector(position_mm, 3, "position_mm")
    return T


def pose_from_matrix(value):
    T = validate_rigid_transform(value)
    return {
        "position_mm": [float(v) for v in T[:3, 3]],
        "quaternion_xyzw": [float(v) for v in matrix_to_quaternion_xyzw(T[:3, :3])],
    }


def rigid_inverse(value):
    T = validate_rigid_transform(value)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def _rotz(degrees):
    a = math.radians(float(degrees))
    return np.array([[math.cos(a), -math.sin(a), 0.0],
                     [math.sin(a), math.cos(a), 0.0],
                     [0.0, 0.0, 1.0]])


def sdk_endpoint_to_grasp_transform(arm="left"):
    """返回 ``T_endpoint_grasp`` 以及来源。

    endpoint 是 armMoveWorlds/armGetWorlds 的 SDK TCP。当前只有左臂有足够证据：
    ``T_link_endpoint=[Rz(+90deg), (0,168,-39)mm]``，抓取中心在 link 系
    ``(0,150,0)mm``。因此平移不是直接照抄 (0,150,0)，而是 endpoint 系的
    约 ``(-18,0,39)mm``。
    """
    if arm != "left":
        raise RuntimeError("右臂 endpoint/TCP 尚未测量确认，拒绝自动生成 T_endpoint_grasp")
    import arm_profiles
    from frame_calibration.analysis import calib_common

    profile = arm_profiles.arm_profile(arm)
    T_link_endpoint = np.eye(4)
    T_link_endpoint[:3, :3] = _rotz(90.0)
    T_link_endpoint[:3, 3] = np.asarray(
        calib_common.CONFIRMED_R_TCP_LINK11_MM, dtype=float)
    T_link_grasp = np.eye(4)
    T_link_grasp[:3, 3] = np.asarray(profile["grip_center_link_mm"], dtype=float)
    T_endpoint_grasp = rigid_inverse(T_link_endpoint) @ T_link_grasp
    return T_endpoint_grasp, {
        "arm": arm,
        "endpoint_frame": "sdk_arm_worlds_tcp",
        "grasp_frame": "grasp_center_axes_parallel_to_link11",
        "grip_center_status": profile["grip_center_status"],
        "tcp_source": "calib_common.CONFIRMED_R_TCP_LINK11_MM",
        "sdk_orientation_relation": "R_sdk=R_link11*Rz(+90deg)",
    }


def resolve_sdk_endpoint_pose(source_pose, pose_role, *,
                              T_object_grasp=None,
                              T_endpoint_grasp=None,
                              fixed_endpoint_quaternion_xyzw=None):
    """把 sdk_world 中的物体/抓取/endpoint 位姿解析成 SDK endpoint 位姿。

    ``OBJECT_POSE``: ``T_W_E=T_W_O*T_O_G*inv(T_E_G)``
    ``GRASP_POSE``:  ``T_W_E=T_W_G*inv(T_E_G)``
    ``EE_POSE``:     直接透传，不做任何补偿

    ``fixed_endpoint_quaternion_xyzw`` 用于圆瓶等轴对称物体：物体四元数仍作为
    输入证据并用于旋转局部抓点偏移，但 endpoint 姿态锁定到已示教姿态。
    """
    if pose_role not in POSE_ROLES:
        raise ValueError(f"pose_role 必须是 {sorted(POSE_ROLES)}")
    if set(source_pose) != {"position_mm", "quaternion_xyzw"}:
        raise ValueError("source_pose 必须且只能包含 position_mm/quaternion_xyzw")
    T_source = pose_matrix(source_pose["position_mm"],
                           source_pose["quaternion_xyzw"])
    if pose_role == POSE_ENDPOINT:
        if T_object_grasp is not None or fixed_endpoint_quaternion_xyzw is not None:
            raise ValueError("EE_POSE 是直通语义，禁止再给 object offset 或固定姿态补偿")
        return {"T_world_endpoint": T_source,
                "T_world_grasp": None,
                "pose": pose_from_matrix(T_source),
                "pose_role": pose_role,
                "orientation_policy": "DIRECT_ENDPOINT"}

    if T_endpoint_grasp is None:
        T_endpoint_grasp, _ = sdk_endpoint_to_grasp_transform("left")
    T_endpoint_grasp = validate_rigid_transform(
        T_endpoint_grasp, "T_endpoint_grasp")
    if pose_role == POSE_OBJECT:
        if T_object_grasp is None:
            raise ValueError("OBJECT_POSE 必须提供 T_object_grasp")
        T_world_grasp = T_source @ validate_rigid_transform(
            T_object_grasp, "T_object_grasp")
    else:
        if T_object_grasp is not None:
            raise ValueError("GRASP_POSE 已是抓取中心，禁止再次应用 T_object_grasp")
        T_world_grasp = T_source

    if fixed_endpoint_quaternion_xyzw is None:
        T_world_endpoint = T_world_grasp @ rigid_inverse(T_endpoint_grasp)
        policy = "FOLLOW_INPUT_FULL_POSE"
    else:
        R_world_endpoint = quaternion_xyzw_to_matrix(
            fixed_endpoint_quaternion_xyzw)
        T_world_endpoint = np.eye(4)
        T_world_endpoint[:3, :3] = R_world_endpoint
        T_world_endpoint[:3, 3] = (
            T_world_grasp[:3, 3] -
            R_world_endpoint @ T_endpoint_grasp[:3, 3])
        policy = "FIXED_ENDPOINT_ORIENTATION"
    requested_world_grasp = T_world_grasp
    actual_world_grasp = T_world_endpoint @ T_endpoint_grasp
    if not np.allclose(actual_world_grasp[:3, 3], requested_world_grasp[:3, 3],
                       atol=1e-8, rtol=0):
        raise AssertionError("endpoint/grasp 平移闭环失败")
    if fixed_endpoint_quaternion_xyzw is None and not np.allclose(
            actual_world_grasp, requested_world_grasp, atol=1e-8, rtol=0):
        raise AssertionError("endpoint/grasp 4x4 闭环失败")
    return {"T_world_endpoint": T_world_endpoint,
            "T_world_grasp": actual_world_grasp,
            "T_world_grasp_requested": requested_world_grasp,
            "pose": pose_from_matrix(T_world_endpoint),
            "pose_role": pose_role,
            "orientation_policy": policy}


def sdk_uvw_deg_to_matrix(uvw_deg):
    u, v, w = (math.radians(x) for x in _finite_vector(uvw_deg, 3, "sdk_uvw_deg"))
    cu, su = math.cos(u), math.sin(u)
    cv, sv = math.cos(v), math.sin(v)
    cw, sw = math.cos(w), math.sin(w)
    return np.array([
        [cw*cv, cw*sv*su-sw*cu, cw*sv*cu+sw*su],
        [sw*cv, sw*sv*su+cw*cu, sw*sv*cu-cw*su],
        [-sv, cv*su, cv*cu],
    ])


def matrix_to_sdk_uvw_deg(rotation):
    """分解 ``R=Rz(W)*Ry(V)*Rx(U)``，并以矩阵回环判定结果。"""
    R = np.asarray(rotation, dtype=float)
    T = np.eye(4); T[:3, :3] = R
    validate_rigid_transform(T, "SDK rotation")
    cv = math.hypot(float(R[0, 0]), float(R[1, 0]))
    v = math.atan2(float(-R[2, 0]), cv)
    if cv > 1e-10:
        u = math.atan2(float(R[2, 1]), float(R[2, 2]))
        w = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        u = 0.0
        w = math.atan2(float(-R[0, 1]), float(R[1, 1]))
    values = [math.degrees(x) for x in (u, v, w)]
    values = [((x + 180.0) % 360.0) - 180.0 for x in values]
    if not np.allclose(sdk_uvw_deg_to_matrix(values), R, atol=1e-8, rtol=0):
        raise ValueError("SDK UVW 位于不可稳定分解的奇异姿态")
    return values


def local_grasp_axes(profile):
    tool = np.asarray(profile["grip_center_link_mm"], float)
    tool /= np.linalg.norm(tool)
    # 左腕工具轴 +Y 时夹持轴取 +X；右腕工具轴 +X 时取 +Y。
    jaw = np.array([1., 0., 0.]) if abs(tool[1]) > .5 else np.array([0., 1., 0.])
    jaw -= tool * float(jaw @ tool); jaw /= np.linalg.norm(jaw)
    side = np.cross(tool, jaw)
    return jaw, side, tool


def top_grasp_rotations(profile, object_yaw_rad):
    """返回绕工具轴相差 180° 的两个等价完整姿态。"""
    local_jaw, local_side, local_tool = local_grasp_axes(profile)
    L = np.column_stack((local_jaw, local_side, local_tool))
    rotations = []
    for yaw in (object_yaw_rad, object_yaw_rad + math.pi):
        world_jaw = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        world_tool = np.array([0.0, 0.0, -1.0])
        world_side = np.cross(world_tool, world_jaw)
        W = np.column_stack((world_jaw, world_side, world_tool))
        R = W @ L.T
        if np.linalg.det(R) < 0.999:
            raise ValueError("顶抓姿态不是右手旋转矩阵")
        rotations.append(R)
    return rotations


def _rotation_error(R_current, R_target):
    E = R_target @ R_current.T
    angle = math.acos(float(np.clip((np.trace(E) - 1) / 2, -1, 1)))
    if angle < 1e-9:
        return np.zeros(3), 0.0
    axis = np.array([E[2,1]-E[1,2], E[0,2]-E[2,0], E[1,0]-E[0,1]])
    axis /= 2 * math.sin(angle)
    return axis * angle, math.degrees(angle)


def solve_full_pose(robot, profile, target_position_m, target_rotation,
                    seed_rad, iters=300, position_tolerance_m=.001,
                    orientation_tolerance_deg=1.5):
    import pybullet as p
    joints, ee = profile["joint_ids"], profile["ee_link_id"]
    grip = np.asarray(profile["grip_center_link_mm"], float) / 1000.0
    margins = profile["shared"]["ik_limit_margin_deg"]
    lo, hi = [], []
    for i, jid in enumerate(joints):
        info = p.getJointInfo(robot, jid)
        a, b = math.degrees(info[8]) + margins[i], math.degrees(info[9]) - margins[i]
        if profile.get("controller_limits_deg"):
            ca, cb = profile["controller_limits_deg"][i]
            a, b = max(a, ca + 5), min(b, cb - 5)
        lo.append(math.radians(a)); hi.append(math.radians(b))
    lo, hi = np.asarray(lo), np.asarray(hi)

    def state(q):
        for jid, value in zip(joints, q):
            p.resetJointState(robot, jid, float(value))
        st = p.getLinkState(robot, ee, computeForwardKinematics=True)
        R = np.asarray(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
        return np.asarray(st[4]) + R @ grip, R

    def error(q):
        pos, R = state(q)
        rot, angle = _rotation_error(R, target_rotation)
        return np.r_[np.asarray(target_position_m) - pos, rot], \
            float(np.linalg.norm(np.asarray(target_position_m) - pos)), angle

    q = np.clip(np.asarray(seed_rad, float), lo, hi)
    damping, clamp = .05, math.radians(10)
    for _ in range(iters):
        e, pe, ae = error(q)
        if pe < position_tolerance_m and ae < orientation_tolerance_deg:
            return q.tolist(), {"pos_err_mm": pe*1000, "ori_err_deg": ae}
        J = np.zeros((6, 7))
        for k in range(7):
            qk = q.copy(); qk[k] += 1e-5
            J[:, k] = -(error(qk)[0] - e) / 1e-5
        dq = J.T @ np.linalg.solve(J @ J.T + damping*damping*np.eye(6), e)
        q = np.clip(q + np.clip(dq, -clamp, clamp), lo, hi)
    _, pe, ae = error(q)
    return None, {"pos_err_mm": pe*1000, "ori_err_deg": ae}


def qualify_top_grasps(robot, profile, position_m, object_yaw_rad):
    seed = np.radians(profile["ik_seed_deg"])
    out = []
    for index, R in enumerate(top_grasp_rotations(profile, object_yaw_rad)):
        q, info = solve_full_pose(robot, profile, position_m, R, seed)
        out.append({"variant": index, "q_urdf_rad": q, "ik": info,
                    "reachable": q is not None})
    return out
