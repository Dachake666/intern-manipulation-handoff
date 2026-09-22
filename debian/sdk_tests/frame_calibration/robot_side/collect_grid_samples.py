#!/usr/bin/env python3
"""Session 2: 采集大范围网格样本 —— 服务问题3(大工作空间坐标变换重标定)。

在 base 姿态基础上, 对肩/肘三个关节(j1/j2/j4)做网格偏移, 覆盖 pick-and-place
实际要用的工作空间; 腕部保持 base 值。每个网格点记录 关节角 + XYZ + UVW。
之后用 analysis/fit_frame_transform.py 拟合并对比 translation vs rigid 模型。

网格默认 3x3x3 = 27 点(蛇形排序, 相邻点只小幅移动), 5% 速度约 20~25 分钟。
运行(容器内):
    python3 collect_grid_samples.py
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

ARM_ID = 1
GLOBAL_SPEED = 5.0
ENABLE_REAL_MOTION = False
CONFIRM_EACH_STEP = True       # 熟练后可改 False 自动连续采集(人仍需盯守)
SETTLE_SECONDS = 2.0
OUT_DIR = "data"

# 网格: 相对 base 的角度增量(deg)。j1=肩pitch j2=肩roll j4=肘pitch。
# 以之前的 base≈[22,17,0,-78,...] 计, 目标角均在 URDF 限位收 5deg 内。
GRID_J1 = (-20.0, 0.0, +12.0)
GRID_J2 = (0.0, +12.0, +25.0)
GRID_J4 = (-15.0, 0.0, +12.0)


def build_grid_targets(base):
    """蛇形排序的网格目标: 相邻两点只有一个轴变化一档, 移动幅度小。"""
    targets = []
    for i1, d1 in enumerate(GRID_J1):
        j2_list = GRID_J2 if i1 % 2 == 0 else tuple(reversed(GRID_J2))
        for i2, d2 in enumerate(j2_list):
            j4_list = GRID_J4 if (i1 + i2) % 2 == 0 else tuple(reversed(GRID_J4))
            for d4 in j4_list:
                q = base[:]
                q[0] = base[0] + d1
                q[1] = base[1] + d2
                q[3] = base[3] + d4
                name = f"g_j1{d1:+05.1f}_j2{d2:+05.1f}_j4{d4:+05.1f}".replace(".0", "")
                targets.append((name, q))
    return targets


def main():
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    samples = []
    try:
        base = ss.read_joints(sdk, ARM_ID)
        print(f"\nbase 关节角: {[round(v, 3) for v in base]}")

        targets = build_grid_targets(base)
        targets.append(("g_back_base", base[:]))
        print(f"\n网格计划({len(targets)} 个点, 蛇形排序):")
        bad_any = False
        for name, q in targets:
            bad = ss.check_limits(ARM_ID, q)
            flag = "  ✗ " + "; ".join(bad) if bad else ""
            print(f"  {name:28s} {[round(v, 2) for v in q]}{flag}")
            bad_any = bad_any or bool(bad)
        if bad_any:
            print("\n存在越限目标, 中止。请调整 base 或网格增量。")
            return 1
        print(f"\n预计耗时约 {len(targets) * 0.8:.0f} 分钟(5% 速度, 含稳定与读数)")

        print(f"ENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未发送任何运动。确认计划无误后改 ENABLE_REAL_MOTION=True 重跑。")
            return 0

        for i, (name, q) in enumerate(targets, 1):
            if CONFIRM_EACH_STEP:
                ans = input(f"\n[{i}/{len(targets)}] {name} -> 回车执行 / q 中止: ").strip().lower()
                if ans == "q":
                    print("用户中止, 保存已采样本。")
                    break
            else:
                print(f"\n[{i}/{len(targets)}] {name}")
            ss.check_soft_stop(sdk, name)
            ss.move_joints_abs(sdk, ARM_ID, q)
            ss.wait_until_joints(sdk, ARM_ID, q)
            time.sleep(SETTLE_SECONDS)
            samples.append(ss.snapshot(sdk, ARM_ID, name, "workspace grid sample"))

        if samples:
            ss.save_samples(OUT_DIR, "grid_samples", {
                "arm_id": ARM_ID,
                "base_q_deg": base,
                "grid_j1": list(GRID_J1), "grid_j2": list(GRID_J2), "grid_j4": list(GRID_J4),
                "global_speed": GLOBAL_SPEED,
                "description": "大范围工作空间网格样本(坐标变换重标定)",
            }, samples)
        return 0
    finally:
        ss.close_session(sdk)
        print("\n提醒: 请确认 ENABLE_REAL_MOTION 已恢复为 False。")


if __name__ == "__main__":
    sys.exit(main())
