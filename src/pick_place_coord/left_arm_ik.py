#!/usr/bin/env python3
"""左臂 6D 数值逆解(DLS + 数值雅可比): 位置 + 工具轴朝向。

============================ 恢复重建说明 ============================
2026-07-22 从聊天记录重建(原文件随工程被误删)。
【高置信-常量】GRIP_R_M、LIMIT_MARGIN_DEG、REAL_LIMITS_DEG、REAL_MARGIN_DEG
  的数值逐字来自对话总结, 可信。
【重建-求解体】DLS 迭代、数值雅可比、收敛判据按标准 6D 数值 IK 写成, 逻辑
  等价但阻尼系数/步长等超参可能与原版有出入。真机使用前请用 --scan 与已归档
  的成功轨迹对比验证(同坐标应解出与历史一致的关节角)。
详见工作区根目录 RECOVERY_STATUS.md。
=====================================================================
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np
import pybullet as p

_WORK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _WORK)
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "analysis"))
import calib_common as cc
import arm_profiles as profiles

ARM_NAME = "left"
ARM = cc.ARM_JOINT_IDS[ARM_NAME]
EE = cc.EE_LINK_ID[ARM_NAME]

# [高置信] 抓取中心相对末端(link11)的偏移(m)。目测+测量确定到 [0, 0.150, 0]。
GRIP_R_M = cc.GRIP_R_LINK_MM[ARM_NAME] / 1000.0
# [高置信] 各关节软限位余量(度): 腕三关节留 12°, 其余 5°。
LIMIT_MARGIN_DEG = [5, 5, 5, 5, 12, 12, 12]
# [高置信] 真机实测限位覆盖(1-indexed 关节 -> (min,max) 度)。
#   左 j7 实际最大约 +76°(URDF 标称 90°); 轨迹在 1.05° 误差处卡死即因此。
REAL_LIMITS_BY_ARM = {"left": {7: (None, 76.0)}, "right": {}}
REAL_LIMITS_DEG = REAL_LIMITS_BY_ARM[ARM_NAME]
REAL_MARGIN_DEG = 5.0

# 夹爪伸出方向(link11 局部系) = GRIP_R_M 的方向 = +Y。抓取要让【这根轴】朝下,
# 而不是 Z 轴 —— 之前对齐 Z 导致夹爪水平、抓不到(20260723 修正)。
TOOL_DIR_LINK = GRIP_R_M / (np.linalg.norm(GRIP_R_M) + 1e-12)


def set_arm(arm):
    """切换 IK 绑定；右臂仅可生成/检查 CANDIDATE，不能证明真机就绪。"""
    global ARM_NAME, ARM, EE, GRIP_R_M, TOOL_DIR_LINK, REAL_LIMITS_DEG
    profile = profiles.arm_profile(arm)
    ARM_NAME = arm
    ARM = list(profile["joint_ids"])
    EE = int(profile["ee_link_id"])
    GRIP_R_M = np.asarray(profile["grip_center_link_mm"], float) / 1000.0
    TOOL_DIR_LINK = GRIP_R_M / (np.linalg.norm(GRIP_R_M) + 1e-12)
    REAL_LIMITS_DEG = REAL_LIMITS_BY_ARM[arm]
    return ARM_NAME

# [重建] DLS 超参
_DLS_LAMBDA = 0.05
_POS_TOL_M = 1e-3
_STEP_CLAMP = math.radians(10.0)


def _limits(robot):
    lo, hi = [], []
    for k, j in enumerate(ARM):
        info = p.getJointInfo(robot, j)
        a = math.degrees(info[8]) + LIMIT_MARGIN_DEG[k]
        b = math.degrees(info[9]) - LIMIT_MARGIN_DEG[k]
        rl = REAL_LIMITS_DEG.get(k + 1)
        if rl:
            if rl[0] is not None:
                a = max(a, rl[0] + REAL_MARGIN_DEG)
            if rl[1] is not None:
                b = min(b, rl[1] - REAL_MARGIN_DEG)
        lo.append(math.radians(a))
        hi.append(math.radians(b))
    return np.array(lo), np.array(hi)


def _fk(robot, q):
    for j, v in zip(ARM, q):
        p.resetJointState(robot, j, float(v))
    st = p.getLinkState(robot, EE, computeForwardKinematics=True)
    pos = np.array(st[4])
    R = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
    grip = pos + R @ GRIP_R_M
    return grip, R


def grip_state(robot):
    """返回当前姿态下 (抓取中心世界坐标, 末端旋转矩阵)。"""
    q = [p.getJointState(robot, j)[0] for j in ARM]
    return _fk(robot, q)


def _error(robot, q, target, tool_axis):
    """6D 误差: 位置(3) + 工具 z 轴与期望轴的叉积(3)。"""
    grip, R = _fk(robot, q)
    e_pos = np.asarray(target) - grip
    tool_world = R @ TOOL_DIR_LINK          # 夹爪伸出方向(世界系)
    axis = np.asarray(tool_axis, float)
    axis = axis / (np.linalg.norm(axis) + 1e-12)
    e_rot = np.cross(tool_world, axis)      # 使夹爪伸出方向对齐期望轴(朝下)
    return np.concatenate([e_pos, e_rot]), grip, R


def _jacobian(robot, q, target, tool_axis, e0):
    J = np.zeros((6, len(q)))
    dq = 1e-5
    for k in range(len(q)):
        qk = list(q)
        qk[k] += dq
        ek, _, _ = _error(robot, qk, target, tool_axis)
        J[:, k] = (ek - e0) / dq
    return -J        # e = target - fk(q), de/dq = -dfk/dq


def solve(robot, target, tool_axis=(0, 0, -1.0), seed_rad=None,
          axis_tol_deg=6.0, iters=150):
    """DLS 数值逆解。返回 (q_rad(list) 或 None, info)。
    收敛判据: 位置误差 < 1mm 且工具轴偏差 < axis_tol_deg。"""
    lo, hi = _limits(robot)
    if seed_rad is None:
        q = np.array([(a + b) / 2 for a, b in zip(lo, hi)])
    else:
        q = np.array(seed_rad, float)
    tool_axis = np.asarray(tool_axis, float)
    for _ in range(iters):
        e, grip, R = _error(robot, q, target, tool_axis)
        pos_err = np.linalg.norm(e[:3])
        tool_world = R @ TOOL_DIR_LINK
        axis_err = math.degrees(math.asin(
            min(1.0, np.linalg.norm(np.cross(tool_world,
                tool_axis / np.linalg.norm(tool_axis))))))
        if pos_err < _POS_TOL_M and axis_err < axis_tol_deg:
            return q.tolist(), {"pos_err_mm": pos_err * 1000,
                                "axis_err_deg": axis_err}
        J = _jacobian(robot, q, target, tool_axis, e)
        JT = J.T
        dq = JT @ np.linalg.solve(
            J @ JT + (_DLS_LAMBDA ** 2) * np.eye(6), e)
        dq = np.clip(dq, -_STEP_CLAMP, _STEP_CLAMP)
        q = np.clip(q + dq, lo, hi)
    e, grip, R = _error(robot, q, target, tool_axis)
    if np.linalg.norm(e[:3]) < _POS_TOL_M:
        return q.tolist(), {"pos_err_mm": np.linalg.norm(e[:3]) * 1000,
                            "axis_err_deg": axis_err}
    return None, {"pos_err_mm": np.linalg.norm(e[:3]) * 1000}
