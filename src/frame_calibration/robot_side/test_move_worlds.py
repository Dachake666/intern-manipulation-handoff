#!/usr/bin/env python3
"""move worlds 轨迹测试: 用 armMoveWorlds 逐点走一小段世界坐标方框轨迹。

SMOOTH = armMoveWorlds 的 interpolation_en(手册 2.7.27): "是否启用平滑衔接"。
【关键理解】它平滑的是"当前这条移动"与"紧接的下一条移动"之间的【拐角】——
要衔接, 下一条必须在当前这条【还在运动时】就发出去, 控制器才能提前拐弯不停。

⚠️ 20260722 修正: 旧版每发一条就 wait_stable 等完全停稳再发下一条, 等于每条都
走到停、没有"相邻的下一条"可衔接, interpolation_en 形同虚设(所以开了 SMOOTH 也
不丝滑)。正确做法: 背靠背连续发出全部路点、中间不等停稳, 只在最后才等停。
翻 SMOOTH 对比 True/False 的拐角差别, 才是本脚本用途。

运行(Linux 容器内): python3 test_move_worlds.py  ENABLE_REAL_MOTION=False 只检查不下发, 真跑人守急停。
"""
import time

import sdk_session as ss

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.147"
ARM_PORT = 8080
ARM_ID   = 1              # 1=左臂 2=右臂
GLOBAL_SPEED = 3.0        # %
ENABLE_REAL_MOTION = False
SMOOTH = True             # ← 平滑轨迹开关(armMoveWorlds interpolation_en)

# 相对起点的位姿增量 [X, Y, Z(mm), U, V, W(deg)] —— 在 XY 平面走个小方框
DELTAS = [
    [30,  0, 0, 0, 0, 0],
    [30, 30, 0, 0, 0, 0],
    [ 0, 30, 0, 0, 0, 0],
    [ 0,  0, 0, 0, 0, 0],   # 回到起点附近
]


def wait_stable(sdk, eps=0.1, timeout=20.0):
    """轮询直到世界位姿基本不再变化(视为到位)。"""
    prev = ss.read_worlds(sdk, ARM_ID)
    deadline = time.time() + timeout
    time.sleep(0.5)
    while time.time() < deadline:
        cur = ss.read_worlds(sdk, ARM_ID)
        if max(abs(a - b) for a, b in zip(cur, prev)) < eps:
            return
        prev = cur
        time.sleep(0.4)


def main():
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    try:
        home = ss.read_worlds(sdk, ARM_ID)
        print("起点世界位姿:", [round(v, 2) for v in home])
        waypoints = [[h + d for h, d in zip(home, delta)] for delta in DELTAS]
        print(f"SMOOTH(平滑轨迹 interpolation_en) = {SMOOTH}")

        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未下发。确认无误后改 ENABLE_REAL_MOTION=True 重跑。")
            return
        input("回车开始走世界坐标轨迹(人守急停): ")
        # 背靠背连续下发, 中间【不】等停稳 —— 这样相邻两条才能被 interpolation_en
        # 在拐角处衔接。若 SDK 在运动中拒绝新命令(code!=0), require_code_zero 会报出来,
        # 那说明这条路要改用路由/队列方式(armClearRobotRoute 暗示存在路径队列)。
        for i, pose in enumerate(waypoints, 1):
            ss.check_soft_stop(sdk, f"路点{i} 前")
            print(f"→ 路点{i}: {[round(v, 2) for v in pose]}")
            code = sdk.armMoveWorlds(ARM_ID, ss.make_float_vector(pose), SMOOTH, False, 0)
            ss.require_code_zero(code, f"armMoveWorlds 路点{i}")
            time.sleep(0.05)          # 只给 SDK 一点登记时间, 不等运动完成
        wait_stable(sdk)              # 仅在全部发完后等最终停稳
        print(f"世界坐标轨迹走完(SMOOTH={SMOOTH})。观察拐角是否圆滑不停; "
              "翻 SMOOTH=False 再跑一次对比。")
    finally:
        ss.close_session(sdk)


if __name__ == "__main__":
    main()
