#!/usr/bin/env python3
"""三点连续交接能力测试轨迹生成器 —— 解锁 --continuous 的前置实验。

目的: 实测 SDK 黑盒对"上一目标未完全停稳就收到下一条 armMoveJoints"的真实反应
(平滑续走 / 减速重规划 / 覆盖 / 拒绝)。官方 armMoveJoints 无 blend 参数, 此行为
无文档, 必须用最小实验确认后才允许在完整抓取任务上启用连续模式。

轨迹设计: HOME -> A -> B -> C -> HOME, 三个开阔区位姿(z>=1.12, 远离桌面高度),
无夹爪事件, 无障碍物; 段名全部取自执行器连续白名单, meta 声明
continuous_candidate=True —— 满足 --continuous 的全部准入检查。

生成(Mac):
    /opt/anaconda3/bin/python3 pick_place_coord/gen_three_point_test.py
    /opt/anaconda3/bin/python3 pick_place_coord/validate_trajectory.py \
        pick_place_coord/trajectories/traj_three_point_test.json

真机实验流程(容器内, 人守急停):
    1. python3 execute_trajectory.py traj_three_point_test.json            # 干跑
    2. 逐点停稳模式空跑一遍(ENABLE_REAL_MOTION=True), 确认轨迹本身无问题;
    3. 改 ENABLE_EXPERIMENTAL_CONTINUOUS=True, 加 --continuous 重跑;
    4. 观察平滑度与到位误差, 通过后才给完整抓取任务开连续。

============================ 恢复重建说明 ============================
2026-07-22 重建: 上方 docstring 与设计意图逐字来自原文件; 生成主体(位姿选择、
段名、meta)按已知行为重建。产出的 traj_three_point_test.json(47 路点)已作为
幸存件恢复在 trajectories/ 下, 本脚本仅用于将来重新生成。依赖 xf/ik 链(重建件),
运行前请先确认那两个模块已按 RECOVERY_STATUS.md 复核。
=====================================================================
"""
from __future__ import annotations

import datetime
import math
import os
import sys

import numpy as np
import pybullet as p

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

import xifeng_pb as xf
import left_arm_ik as ik
import pick_place_coord as ppc

# 三个开阔区位姿目标(PB 世界系, m), z>=1.12 远离桌面, 彼此拉开做转运幅度。
POINTS = [
    [0.34, 0.30, 1.14],
    [0.44, 0.20, 1.16],
    [0.30, 0.16, 1.12],
]
# 段名全部取自执行器连续白名单(含 "->"), 满足 --continuous 准入。
SEGMENTS = ["HOME->READY", "READY->PICK_HOVER", "PICK_HOVER->READY",
            "READY->PLACE_HOVER", "PLACE_HOVER->READY", "READY->HOME"]


def main():
    robot = xf.load_xifeng(gui=False)
    ppc.DISABLED = ppc.compute_disabled_pairs(robot)
    down = (0, 0, -1.0)

    targets = [ppc.HOME]
    seed = ppc.HOME
    for pt in POINTS:
        q, _ = ik.solve(robot, pt, down, seed_rad=seed, axis_tol_deg=30.0)
        if q is None:
            print(f"IK 失败: {pt}")
            return 1
        targets.append(q)
        seed = q
    targets.append(ppc.HOME)

    traj = ppc.Traj()
    obstacles = []
    mon = ppc.Monitor(robot, obstacles)
    ppc._kin_set_rad(robot, ppc.HOME)
    traj.record("HOME", ppc.HOME)
    ppc.PLAN_METRICS.update(legs=0, time_s=0.0, len_rad=0.0)

    for k in range(1, len(targets)):
        name = SEGMENTS[min(k - 1, len(SEGMENTS) - 1)]
        path = ppc.plan_verified(robot, targets[k], obstacles)
        if path is None:
            print(f"规划失败: {name}")
            return 1
        ppc.play(robot, name, path, traj, mon, gui=False)

    meta = {
        "arm_id": 1, "arm": "left", "j6_flipped": True,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "task": "three_point_capability",
        "continuous_candidate": True,
        "continuous_candidate_segments": sorted(
            {it["seg"] for it in traj.items if "->" in it["seg"]}),
        "export_step_deg": ppc.EXPORT_STEP_DEG,
        "note": "三点连续能力测试: 开阔区, 无夹爪无障碍; 用于真机验证 --continuous",
    }
    out = os.path.join(_HERE, "trajectories", "traj_three_point_test.json")
    traj.export(out, meta)
    p.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
