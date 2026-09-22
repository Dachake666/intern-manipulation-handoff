#!/usr/bin/env python3
"""标定常量与 SDK<->URDF 关节换算的共享权威。

============================ 恢复重建说明(务必读) ============================
2026-07-22 从聊天记录重建(原文件随工程被误删)。分两类:

【A. 高置信 —— 标定活动的实测结论, 数值逐字来自对话/总结, 可信】
    CONFIRMED_R_TCP_LINK11_MM   TCP 在 link11 系的偏移(0.003mm 拟合)
    CONFIRMED_D_FOREARM_LINK9_MM SDK 内部模型前臂比 URDF 长 85mm 的修正
    CONFIRMED_T_SESSIONS_V2_MM  v2 位置模型的会话级平移 t
    CONFIRMED_UVW_CANDIDATE     UVW=标准 RPY, R_sdk=R_link11·Rz(90°)
    j6(腕滚)符号相对 URDF 取反  —— urdf_q_to_sdk_q / sdk_q_to_urdf_q

【B. 待核 —— 结构/数值我只有片段, 首次真机使用前必须逐项复核】
    CONFIRMED_T_SESSIONS_MM     坐标换算用的轴对齐平移(可视化脚本曾显示
                                ≈[275,-20,-727]mm, 与 v2 模型的 t 不同,
                                二者服务不同模型, 不要混用)
    J6_FLIP_INDEX               取反的关节在 7 元组里的下标(见下方告警)
    FK 类、load_samples、uvw_to_matrix、rotation_angle_deg 的函数体为骨架,
    逻辑注释保留, 但未逐字复原。
详见工作区根目录 RECOVERY_STATUS.md。
=============================================================================
"""
from __future__ import annotations

import json
import math
import os

import numpy as np

# ---------------------------------------------------------------- 路径
_ANALYSIS_DIR = os.path.dirname(os.path.abspath(__file__))
FRAME_CALIB_DIR = os.path.dirname(_ANALYSIS_DIR)
WORK_DIR = os.path.dirname(FRAME_CALIB_DIR)
DATA_DIR = os.path.join(FRAME_CALIB_DIR, "data")
# URDF 路径 —— 幸存的 XF0112048/robot.urdf(2026-07-22 确认真实文件名)。
DEFAULT_URDF = os.path.join(WORK_DIR, "XF0112048", "robot.urdf")
EXISTING_SAMPLES = os.path.join(DATA_DIR, "wrist_orientation_samples_latest.json")

# ---------------------------------------------------------------- 关节映射
# [已核实] 关节 id 与末端 link id —— 取自幸存的原始脚本
# workspace/pybullet_tests/urdf/XF0112048/(xf0112048_left_arm_sdk_fk_compare.py
# 与 ..._right_arm_numeric_jacobian_ik_demo.py)。左右臂对称。
#   左臂 [5..11]: 肩pitch/肩roll/肩yaw/肘pitch/腕yaw/腕roll/腕pitch, 末端 link=11
#   右臂 [12..18], 末端 link=18
ARM_JOINT_IDS = {"left": [5, 6, 7, 8, 9, 10, 11],
                 "right": [12, 13, 14, 15, 16, 17, 18]}
EE_LINK_ID = {"left": 11, "right": 18}

# 规划层抓取中心在末端连杆局部系中的偏移(mm)。左臂为尺量值；右臂是 URDF
# 几何候选，现场测量前不得作为 REAL_VERIFIED。权威状态见 arm_profiles.v1.json。
GRIP_R_LINK_MM = {"left": np.array([0.0, 150.0, 0.0]),
                  "right": np.array([150.0, 0.0, 0.0])}

# ---------------------------------------------------------------- 标定常量(A 类, 高置信)
# TCP 在 link11(末端)局部系中的位置(mm)。r=[0,168,-39], 0.003mm 拟合残差。
CONFIRMED_R_TCP_LINK11_MM = np.array([0.0, 168.0, -39.0])
# SDK 内部运动学模型的前臂比 URDF 长 85mm; 仅用于"解释 armGetWorlds / 生成
# armMoveWorlds", 物理真值是 URDF(卷尺裁决: 肘->腕≈300mm 与 URDF 311mm 相符),
# 故仿真抓取点无需此修正。(link9 系, mm)
CONFIRMED_D_FOREARM_LINK9_MM = np.array([-0.08, 0.24, -85.06])
# v2 位置模型的会话级平移 t(mm), 腰部上下移动会使其漂移(严格矢状面特征)。
CONFIRMED_T_SESSIONS_V2_MM = {
    "20260710": [223.2998, -0.1639, -669.1295],
    "20260707": [207.6747, -0.1510, -634.1624],
}
# UVW = 标准 RPY: R = Rz(W)·Ry(V)·Rx(U); R_sdk = R_link11·Rz(90°) 精确成立。
CONFIRMED_UVW_CANDIDATE = {"id": "rpy_zyx", "seq": "zyx",
                           "perm": (2, 1, 0), "signs": (1, 1, 1)}

# ---------------------------------------------------------------- 坐标换算常量(B 类, 待核)
# [待核] 轴对齐(MODEL A)下, SDK 世界原点在 PB 世界中的平移(mm)。
# 可视化脚本 visualize_calibration.py 曾把它画作 ≈[275,-20,-727]mm。
# 注意: 这与 CONFIRMED_T_SESSIONS_V2_MM 是两套模型, 不可互换。
# pick_place_coord.py 用 CONFIRMED_T_SESSIONS_MM 做 PB<->SDK 坐标换算。
CONFIRMED_T_SESSIONS_MM = {
    "20260707": [275.0, -20.0, -727.0],   # <-- 首次真机前务必用一次逐点轨迹重标
    "20260710": [275.0, -20.0, -727.0],
}
T_PB_TO_SDK_MM = np.array(CONFIRMED_T_SESSIONS_MM["20260707"])


# ---------------------------------------------------------------- SDK<->URDF 关节换算
# [已核实] 取反关节 = 腕滚(left_wrist_roll = jid10 = 7 元组里的第 6 个 => 下标 5)。
#   来自幸存脚本的关节映射: [5,6,7,8,9,10,11] 依次为 肩pitch/肩roll/肩yaw/
#   肘pitch/腕yaw/腕roll(下标5)/腕pitch(下标6=j7, 实限 76°)。
J6_FLIP_INDEX = 5


def sdk_q_to_urdf_q(q_sdk_deg):
    """SDK 7 关节角(度) -> URDF 7 关节角(度): 仅腕滚取反。"""
    q = list(q_sdk_deg)
    q[J6_FLIP_INDEX] = -q[J6_FLIP_INDEX]
    return q


def urdf_q_to_sdk_q(q_urdf_deg):
    """URDF 7 关节角(度) -> SDK 7 关节角(度): 仅腕滚取反(对合运算)。"""
    q = list(q_urdf_deg)
    q[J6_FLIP_INDEX] = -q[J6_FLIP_INDEX]
    return q


# ---------------------------------------------------------------- 姿态工具(B 类骨架)
def uvw_to_matrix(candidate, uvw_deg):
    """按 UVW 约定把 (U,V,W) 度解码为旋转矩阵。
    [重建] R = Rz(W)·Ry(V)·Rx(U) 标准 RPY(seq=zyx)。"""
    u, v, w = (math.radians(a) for a in uvw_deg)

    def rx(a): return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)],
                                [0, math.sin(a), math.cos(a)]])

    def ry(a): return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0],
                                [-math.sin(a), 0, math.cos(a)]])

    def rz(a): return np.array([[math.cos(a), -math.sin(a), 0],
                                [math.sin(a), math.cos(a), 0], [0, 0, 1]])
    return rz(w) @ ry(v) @ rx(u)


def rotation_angle_deg(R):
    """旋转矩阵对应的转角(度): acos((tr-1)/2)。"""
    tr = np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0)
    return math.degrees(math.acos(tr))


def load_samples(paths):
    """读取一个或多个样本 JSON, 合并为列表。
    [重建] 每条样本含 name / q_deg / xyz_mm / uvw_deg 字段。"""
    out = []
    for path in paths:
        if not os.path.exists(path):
            continue
        with open(path) as f:
            d = json.load(f)
        out.extend(d if isinstance(d, list) else d.get("samples", []))
    return out


class FK:
    """URDF 正运动学封装(基于 PyBullet DIRECT)。
    [重建骨架] 原类提供 poses_of(q) -> 各 link 的 (pos, R)。需 PyBullet + URDF。
    首次使用前请对照 XF0112048 URDF 复核连杆号与关节顺序。"""

    def __init__(self, urdf=DEFAULT_URDF, arm="left"):
        self.urdf = urdf
        self.arm = arm
        self.joints = ARM_JOINT_IDS[arm]
        self.ee = EE_LINK_ID[arm]
        self._rid = None

    def _ensure(self):
        import pybullet as p
        import pybullet_data
        if self._rid is None:
            if p.getConnectionInfo().get("isConnected", 0) == 0:
                p.connect(p.DIRECT)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setAdditionalSearchPath(WORK_DIR)
            self._rid = p.loadURDF(self.urdf, [0, 0, 0], [0, 0, 0, 1],
                                   useFixedBase=True)

    def poses_of(self, q_urdf_deg):
        """返回 {link_id: (pos(np3), R(np3x3))}。"""
        import pybullet as p
        self._ensure()
        for j in range(p.getNumJoints(self._rid)):
            p.resetJointState(self._rid, j, 0.0)
        for j, q in zip(self.joints, q_urdf_deg):
            p.resetJointState(self._rid, j, math.radians(q))
        poses = {}
        for link in range(p.getNumJoints(self._rid)):
            st = p.getLinkState(self._rid, link, computeForwardKinematics=True)
            R = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
            poses[link] = (np.array(st[4]), R)
        return poses
