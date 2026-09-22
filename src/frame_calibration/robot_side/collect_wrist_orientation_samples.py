#!/usr/bin/env python3
"""Session 1: 采集腕部大幅姿态样本 —— 一份数据同时服务两个分析:
    * 问题1  UVW 约定判别   (analysis/identify_uvw_convention.py)
    * 问题2a TCP 偏移拟合   (analysis/fit_tcp_wrist_samples.py)

流程:
  1. 读取当前姿态作为 base(先用你们已有脚本把左臂摆到之前采样用的安全姿态, 如
     [22, 17, 0, -78, 3, 2, 3] 附近);
  2. 依 SAMPLE_PLAN 相对 base 摆腕部(j5/j6/j7)各姿态, 每个姿态到位+稳定后
     记录 关节角 + XYZ + UVW;
  3. 全部目标先做限位校验, DRY RUN 只打印计划不动机器人。

运行(容器内):
    python3 collect_wrist_orientation_samples.py
安全习惯: 跑完后确认 ENABLE_REAL_MOTION 恢复为 False。
"""
from __future__ import annotations

import sys
import time

import sdk_session as ss

# ---------------------------------------------------------------- 配置
ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080

ARM_ID = 1                     # 1=左臂
GLOBAL_SPEED = 5.0             # 全局速度百分比, 保持低速
ENABLE_REAL_MOTION = False     # False=只打印计划; True=真实运动
CONFIRM_EACH_STEP = True       # 每个姿态前等回车(q 中止并保存已采样本)
SETTLE_SECONDS = 2.0           # 到位后稳定时间再读数
OUT_DIR = "data"

# 采样计划: (名字, {SDK关节号: 相对 base 的角度增量 deg})
# j5=腕yaw(±170) j6=腕roll(±45) j7=腕pitch(±90); 增量非累积, 都相对 base。
SAMPLE_PLAN = [
    ("w01_base", {}),
    ("w02_j5_p20", {5: +20.0}),
    ("w03_j5_m20", {5: -20.0}),
    ("w04_j6_p25", {6: +25.0}),
    ("w05_j6_m25", {6: -25.0}),
    ("w06_j7_p20", {7: +20.0}),
    ("w07_j7_m20", {7: -20.0}),
    ("w08_j5p15_j6p15", {5: +15.0, 6: +15.0}),
    ("w09_j5m15_j7p15", {5: -15.0, 7: +15.0}),
    ("w10_j6p15_j7m15", {6: +15.0, 7: -15.0}),
    ("w11_j5p10_j6m15_j7m10", {5: +10.0, 6: -15.0, 7: -10.0}),
    ("w12_back_base", {}),
]


def build_targets(base):
    targets = []
    for name, deltas in SAMPLE_PLAN:
        q = base[:]
        for jn, d in deltas.items():
            q[jn - 1] = base[jn - 1] + d
        targets.append((name, q))
    return targets


def main():
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    samples = []
    try:
        base = ss.read_joints(sdk, ARM_ID)
        print(f"\nbase 关节角: {[round(v, 3) for v in base]}")

        targets = build_targets(base)
        print(f"\n采样计划({len(targets)} 个姿态, 全部相对 base):")
        bad_any = False
        for name, q in targets:
            bad = ss.check_limits(ARM_ID, q)
            flag = "  ✗ " + "; ".join(bad) if bad else ""
            print(f"  {name:24s} {[round(v, 2) for v in q]}{flag}")
            bad_any = bad_any or bool(bad)
        if bad_any:
            print("\n存在越限目标, 中止。请调整 base 姿态或减小增量。")
            return 1

        print(f"\nENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未发送任何运动。确认计划无误后改 ENABLE_REAL_MOTION=True 重跑。")
            return 0

        for i, (name, q) in enumerate(targets, 1):
            if CONFIRM_EACH_STEP:
                ans = input(f"\n[{i}/{len(targets)}] {name} -> 回车执行 / q 中止: ").strip().lower()
                if ans == "q":
                    print("用户中止, 保存已采样本。")
                    break
            ss.check_soft_stop(sdk, name)
            ss.move_joints_abs(sdk, ARM_ID, q)
            ss.wait_until_joints(sdk, ARM_ID, q)
            time.sleep(SETTLE_SECONDS)
            samples.append(ss.snapshot(sdk, ARM_ID, name, "wrist orientation sample"))

        if samples:
            ss.save_samples(OUT_DIR, "wrist_orientation_samples", {
                "arm_id": ARM_ID,
                "base_q_deg": base,
                "global_speed": GLOBAL_SPEED,
                "description": "腕部大幅姿态样本(UVW 约定判别 + TCP 拟合共用)",
            }, samples)
        return 0
    finally:
        ss.close_session(sdk)
        print("\n提醒: 请确认 ENABLE_REAL_MOTION 已恢复为 False。")


if __name__ == "__main__":
    sys.exit(main())
