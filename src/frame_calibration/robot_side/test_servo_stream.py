#!/usr/bin/env python3
"""伺服透传平滑能力测试 —— 验证"连续不停"的正确做法。

背景(为什么建这个脚本):
  之前的"连续模式"用 armMoveJoints 逐点下发, 每点自带完整加减速并停稳,
  "提前发下一条"也救不了每点减速 —— 这是点到点 API 的架构上限, 不是参数问题。
  查 SDK 手册后确认: 真正的平滑连续要用【伺服透传】(手册 2.7.31 armPluseToServo,
  "关节摇操透传"), 关闭保护状态后高频把【密集插值】的路点直接流给伺服, 由伺服在
  点间插值, 走出一气呵成的连续运动。本脚本用一小段安全运动验证这条路是否成立、
  并帮你标定合适的 步长 / 周期。

范式(手册依据):
  set_protect_status(False)  →  循环 pulse_to_servo(密集点, 固定周期)  →  set_protect_status(True)

安全:
  * ENABLE_REAL_MOTION=False 时只打印计划(点数/最大步长/周期), 不下发;
  * 关闭保护期间失去"突变保护", 所以【步长必须很小】(默认 <=0.2 deg/点), 且
    过 check_limits; finally 里无论如何恢复保护;
  * 真跑请守急停, 从最小幅度 + 较大周期开始, 平滑后再缩短周期/加大幅度。

运行(Linux 容器内): python3 test_servo_stream.py
"""
import time

import sdk_session as ss

# ⚠️ 首次运行前核对 IP: execute_trajectory.py 用的是 192.168.8.148;
#    你今天的 test_move_*.py 写的是 .147。以真机当前实际为准, 两者要一致。
ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1                  # 1=左臂 2=右臂
GLOBAL_SPEED = 3.0            # % (透传下速度主要由 周期+步长 决定, 全局速度设低更稳)
ENABLE_REAL_MOTION = False

# 透传参数(本测试要标定的就是这两个)
STEP_DEG = 0.2               # 相邻透传点单关节最大步长(度); 越小越平滑越安全
PERIOD_S = 0.02             # 透传下发周期(秒); 与手册位置刷新周期(默认关节40ms)相称
                            # 20ms=50Hz 起步, 平滑后可试 10ms; 太快或步长太大都危险

# 相对起点的关节增量(度) —— 一小段来回轨迹的"关键点", 之间会被密集插值
KEY_DELTAS = [
    [0,  0, 0,  0, 0, 0, 0],   # 起点
    [8,  0, 0,  0, 0, 0, 0],
    [8,  0, 0, -8, 0, 0, 0],
    [0,  0, 0,  0, 0, 0, 0],   # 回到起点附近
]


def densify(waypoints, step_deg):
    """把关键点之间按 step_deg 线性插值成密集点流。"""
    dense = [waypoints[0]]
    for a, b in zip(waypoints, waypoints[1:]):
        span = max(abs(y - x) for x, y in zip(a, b))
        n = max(1, int(round(span / step_deg)))
        for k in range(1, n + 1):
            dense.append([x + (y - x) * k / n for x, y in zip(a, b)])
    return dense


def main():
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    protect_off = False
    try:
        home = ss.read_joints(sdk, ARM_ID)
        print("起点关节角:", [round(v, 2) for v in home])
        key = [[h + d for h, d in zip(home, delta)] for delta in KEY_DELTAS]
        for i, q in enumerate(key, 1):                 # 关键点先过限位
            bad = ss.check_limits(ARM_ID, q)
            if bad:
                raise SystemExit(f"关键点{i} 超限, 中止: {bad}")

        dense = densify(key, STEP_DEG)
        max_step = max(
            max(abs(y - x) for x, y in zip(a, b))
            for a, b in zip(dense, dense[1:]))
        dur = len(dense) * PERIOD_S
        print(f"密集透传点: {len(dense)} 个, 相邻最大步长 {max_step:.3f} deg, "
              f"周期 {PERIOD_S*1000:.0f}ms, 预计时长 {dur:.1f}s")

        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未下发。确认无误后改 ENABLE_REAL_MOTION=True 重跑。")
            return

        input("回车开始【伺服透传】流式运动(人守急停): ")
        print("关闭保护状态(透传前置条件)...")
        ss.set_protect_status(sdk, ARM_ID, False)
        protect_off = True

        t_next = time.time()
        for i, q in enumerate(dense):
            ss.check_soft_stop(sdk, f"透传点{i}")
            ss.pulse_to_servo(sdk, ARM_ID, q)
            t_next += PERIOD_S                          # 固定节拍, 补偿调用耗时
            sleep = t_next - time.time()
            if sleep > 0:
                time.sleep(sleep)
        time.sleep(0.3)
        actual = ss.read_joints(sdk, ARM_ID)
        err = max(abs(a - t) for a, t in zip(actual, key[-1]))
        print(f"透传完成: 终点最大误差 {err:.3f} deg")
        print("观察: 全程是否一气呵成不停顿? 有无抖动/异响? 据此调 STEP_DEG/PERIOD_S。")
    finally:
        if protect_off:
            print("恢复保护状态...")
            try:
                ss.set_protect_status(sdk, ARM_ID, True)
            except Exception as exc:                    # noqa: BLE001
                print("⚠️ 恢复保护失败, 请手动确认机械臂保护已开:", exc)
        ss.close_session(sdk)


if __name__ == "__main__":
    main()
