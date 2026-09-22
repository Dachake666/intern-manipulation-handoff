#!/usr/bin/env python3
"""隙锋机器人 PyBullet 加载 / 可视化 / 抓取约束封装。

============================ 恢复重建说明(置信度最低) ============================
2026-07-22 重建(原文件随工程被误删)。原文件我几乎只掌握函数名与调用方式,
函数体基本靠标准 PyBullet 用法补齐, URDF 路径/连杆号/基座位姿等关键项均为
猜测, 未经运行验证。这是恢复件里最需要人工核对的文件:
  * DEFAULT_URDF 路径与真实 XF0112048 文件名
  * 基座位姿(原工程可能对机器人有平移/旋转)
  * EE_LINK_ID / ARM_JOINT_IDS 是否与 URDF 一致(与 calib_common 保持同一份)
  * attach_grasp 的父连杆与约束姿态
真实祖先脚本(逐字节幸存, 可直接参考/复制其加载逻辑):
  workspace/pybullet_tests/urdf/XF0112048/gui_test_xf0112048.py        (URDF 加载)
  workspace/pybullet_tests/urdf/XF0112048/xf0112048_*_numeric_jacobian_ik_demo.py (IK)
  workspace/pybullet_tests/urdf/XF0112048/xf0112048_left_arm_fit_sdk_world_from_samples.py (T 拟合)
已据此核实: 左臂关节 [5,6,7,8,9,10,11]、末端 link=11(见 calib_common)。
[仍待核] basePosition: 祖先脚本里 [0,0,0](最常见)/[0,0,0.5]/[0,0,1.0] 都出现过,
  原规划器用哪个未定 —— 影响 RANGE_BOX 可达性。本文件暂用 [0,0,0], 载入后按
  已归档 133 点轨迹的可达坐标反推确认。详见工作区根目录 RECOVERY_STATUS.md。
================================================================================
"""
from __future__ import annotations

import os
import sys

import numpy as np
import pybullet as p
import pybullet_data

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), "frame_calibration", "analysis"))
import calib_common as cc

# 与 calib_common 共用同一份连杆/关节定义, 避免两处漂移。
ARM_NAME = "left"
ARM_JOINT_IDS = cc.ARM_JOINT_IDS[ARM_NAME]
EE_LINK_ID = cc.EE_LINK_ID[ARM_NAME]
DEFAULT_URDF = cc.DEFAULT_URDF


def set_arm(arm):
    """切换 PyBullet 关节/末端映射；默认保持左臂。"""
    global ARM_NAME, ARM_JOINT_IDS, EE_LINK_ID
    if arm not in cc.ARM_JOINT_IDS:
        raise ValueError(f"arm 只能是 left/right, 收到 {arm!r}")
    ARM_NAME = arm
    ARM_JOINT_IDS = cc.ARM_JOINT_IDS[arm]
    EE_LINK_ID = cc.EE_LINK_ID[arm]
    return ARM_NAME


def load_xifeng(gui=True):
    """连接 PyBullet 并加载隙锋 URDF, 返回 robot body id。"""
    p.connect(p.GUI if gui else p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.setAdditionalSearchPath(cc.WORK_DIR)   # 解析 URDF 里 package:// 的 mesh
    robot = p.loadURDF(DEFAULT_URDF, [0, 0, 0], [0, 0, 0, 1],
                       useFixedBase=True)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=150,
                                     cameraPitch=-30, cameraTargetPosition=[0.3, 0.2, 1.0])
    return robot


def draw_frame(robot, link, length=0.1, width=2):
    """在指定连杆原点画 RGB 三轴(调试用)。"""
    rgb = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]
    for k in range(3):
        axis = [0, 0, 0]
        axis[k] = length
        p.addUserDebugLine([0, 0, 0], axis, rgb[k], width,
                           parentObjectUniqueId=robot, parentLinkIndex=link)


def set_robot_object_collision(robot, obj, enable):
    """开/关 机器人所有连杆 <-> 指定物体 的碰撞。"""
    n = p.getNumJoints(robot)
    for link in range(-1, n):
        p.setCollisionFilterPair(robot, obj, link, -1, 1 if enable else 0)


def attach_grasp(robot, obj, arm_joints=None, ee_link=None):
    """在末端与物体之间建立固定约束(模拟夹爪夹住), 返回约束 id。
    [重建] 约束姿态取物体相对末端的当前相对位姿。"""
    arm_joints = ARM_JOINT_IDS if arm_joints is None else arm_joints
    ee_link = EE_LINK_ID if ee_link is None else ee_link
    ee_state = p.getLinkState(robot, ee_link, computeForwardKinematics=True)
    # createConstraint 的 frame 相对 link 的惯性/质心系，而非 worldLinkFrame。
    # 用 [4]/[5] 会在第一帧把物体拉偏数厘米。
    ee_pos, ee_orn = np.array(ee_state[0]), ee_state[1]
    obj_pos, obj_orn = p.getBasePositionAndOrientation(obj)
    inv_pos, inv_orn = p.invertTransform(ee_pos, ee_orn)
    rel_pos, rel_orn = p.multiplyTransforms(inv_pos, inv_orn, obj_pos, obj_orn)
    cid = p.createConstraint(
        robot, ee_link, obj, -1, p.JOINT_FIXED,
        [0, 0, 0], rel_pos, [0, 0, 0], rel_orn)
    return cid
