#!/usr/bin/env python3
"""问题1 收尾 + 问题2c: 命令侧 UVW 单轴验证(armMoveWorlds 首次真机试用)。

作用:
  1. 验证下发侧 UVW 与判别出的反馈侧约定一致(轴/方向/单位);
  2. 顺带做 TCP 行为判别: XYZ 不变只转姿态时, 观察哪个物理点保持不动 ——
     指尖不动/手腕绕转 -> 控制点是 TCP; 手腕不动/指尖扫弧 -> 控制点是腕。

流程: 读当前 [X,Y,Z,U,V,W] -> 只把 AXIS 加 DELTA_DEG 下发 -> 等稳定 ->
      读回并打印前后差(XYZ 变化、UVW 变化、关节变化) -> 可选转回原姿态。

每次只测一个轴!首测建议 DELTA_DEG=5。人守在急停旁。
运行(容器内):
    python3 verify_uvw_command_side.py
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
GLOBAL_SPEED = 3.0
ENABLE_REAL_MOTION = False
AXIS = "U"                    # 本次测试的轴: "U" / "V" / "W"
DELTA_DEG = 5.0               # 单轴增量(|值| <= 10)
RETURN_BACK = True            # 结束后转回原姿态
STABLE_EPS = 0.05             # 连续两次读数最大变化 < 此值视为稳定
TIMEOUT_S = 20.0

AXIS_INDEX = {"U": 3, "V": 4, "W": 5}


def wait_stable(sdk):
    prev = ss.read_worlds(sdk, ARM_ID)
    deadline = time.time() + TIMEOUT_S
    time.sleep(0.5)
    while time.time() < deadline:
        cur = ss.read_worlds(sdk, ARM_ID)
        change = max(abs(a - b) for a, b in zip(cur, prev))
        if change < STABLE_EPS:
            return cur
        prev = cur
        time.sleep(0.4)
    print("⚠ 超时仍未完全稳定, 用最后一次读数继续")
    return prev


def send_pose(sdk, pose6, tag):
    print(f"下发 armMoveWorlds({tag}): {[round(v, 3) for v in pose6]}")
    code = sdk.armMoveWorlds(ARM_ID, ss.make_float_vector(pose6), False, False, 0)
    print("armMoveWorlds 返回:", code)
    ss.require_code_zero(code, f"armMoveWorlds {tag}")
    return wait_stable(sdk)


def main():
    if AXIS not in AXIS_INDEX:
        raise SystemExit('AXIS 必须是 "U"/"V"/"W"')
    if abs(DELTA_DEG) > 10.0:
        raise SystemExit("安全限制: |DELTA_DEG| <= 10")

    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    try:
        before_w = ss.read_worlds(sdk, ARM_ID)
        before_j = ss.read_joints(sdk, ARM_ID)
        print(f"\n当前 world: xyz={[round(v, 3) for v in before_w[:3]]} "
              f"uvw={[round(v, 3) for v in before_w[3:]]}")
        print(f"当前 joints: {[round(v, 3) for v in before_j]}")

        target = before_w[:]
        target[AXIS_INDEX[AXIS]] += DELTA_DEG
        print(f"\n计划: 仅 {AXIS} {before_w[AXIS_INDEX[AXIS]]:+.3f} -> "
              f"{target[AXIS_INDEX[AXIS]]:+.3f} deg (XYZ 与另两轴不变)")
        print("观察要点: ① 腕部绕哪根物理轴转、方向; ② 指尖和手腕哪个点基本不动。")

        print(f"\nENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未发送。确认无误后改 ENABLE_REAL_MOTION=True 重跑。")
            return 0

        input("回车开始(人守急停): ")
        ss.check_soft_stop(sdk, "UVW 测试前")
        after_w = send_pose(sdk, target, f"{AXIS}{DELTA_DEG:+.1f}")
        after_j = ss.read_joints(sdk, ARM_ID)

        print("\n===== 前后对比 =====")
        print(f"XYZ 变化(mm):  {[round(a - b, 3) for a, b in zip(after_w[:3], before_w[:3])]}"
              "   <- 若明显非零, 说明控制点带工具偏移或标定有偏")
        print(f"UVW 变化(deg): {[round(a - b, 3) for a, b in zip(after_w[3:], before_w[3:])]}")
        print(f"关节变化(deg): {[round(a - b, 3) for a, b in zip(after_j, before_j)]}")
        print("\n请记录人工观察: 实际转轴方向 / 不动点是指尖还是手腕。")

        if RETURN_BACK:
            ans = input("\n回车让机械臂转回原姿态 / s 跳过: ").strip().lower()
            if ans != "s":
                send_pose(sdk, before_w, "return")
                print("已回到原姿态附近。")
        return 0
    finally:
        ss.close_session(sdk)
        print("\n提醒: 请确认 ENABLE_REAL_MOTION 已恢复为 False。")


if __name__ == "__main__":
    sys.exit(main())
