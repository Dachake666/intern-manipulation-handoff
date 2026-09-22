#!/usr/bin/env python3
"""move joints 轨迹测试: 用 armMoveJoints 逐点走一小段关节轨迹, 验证到位。

armMoveJoints 是点到点、每点完整加减速停稳(官方无平滑/blend 参数)。
本脚本从当前关节角出发, 按 DELTAS 生成几个路点, 逐点下发并轮询到位。

运行(Linux 操作电脑容器内): python3 test_move_joints.py
安全: ENABLE_REAL_MOTION=False 时只检查不下发; 真跑时人守急停。
"""
import sdk_session as ss

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.147"
ARM_PORT = 8080
ARM_ID   = 1              # 1=左臂 2=右臂
GLOBAL_SPEED = 3.0        # %
ENABLE_REAL_MOTION = False

# 相对起点的关节增量(度), 每行 7 个关节 —— 一小段来回轨迹
DELTAS = [
    [5, 0, 0,  0, 0, 0, 0],
    [5, 0, 0, -5, 0, 0, 0],
    [0, 0, 0,  0, 0, 0, 0],   # 回到起点附近
]


def main():
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    try:
        home = ss.read_joints(sdk, ARM_ID)
        print("起点关节角:", [round(v, 2) for v in home])
        waypoints = [[h + d for h, d in zip(home, delta)] for delta in DELTAS]

        for i, q in enumerate(waypoints, 1):        # 先全部过一遍限位
            bad = ss.check_limits(ARM_ID, q)
            if bad:
                raise SystemExit(f"路点{i} 超限, 中止: {bad}")

        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未下发。确认无误后改 ENABLE_REAL_MOTION=True 重跑。")
            return
        input("回车开始逐点走关节轨迹(人守急停): ")
        for i, q in enumerate(waypoints, 1):
            ss.check_soft_stop(sdk, f"路点{i} 前")
            print(f"→ 路点{i}: {[round(v, 2) for v in q]}")
            ss.move_joints_abs(sdk, ARM_ID, q)
            ss.wait_until_joints(sdk, ARM_ID, q)
        print("关节轨迹走完。")
    finally:
        ss.close_session(sdk)


if __name__ == "__main__":
    main()
