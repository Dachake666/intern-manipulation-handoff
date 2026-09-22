#!/usr/bin/env python3
"""捕捉真机"工具朝下"的世界位姿基准 —— 修正 world_grasp 导出 UVW 的关键一步。

背景: Mac 仿真导出的世界位姿里, UVW(姿态)由重建标定推算, 与真机约定对不上
(仿真算出 tool-down≈[U≈0,V≈0,W≈-33], 真机实际 tool-down 的 UVW 完全不同)。
真机才有正确答案: 把左臂摆成一个已知的 tool-down 姿态, 读 armGetWorlds, 记下
真实的 [X,Y,Z,U,V,W]。工具朝下时 UVW 近似恒定, 拿到它就能覆盖仿真导出的 UVW。

做法: 使能后(可选)移动到一个已验证的 tool-down 关节姿态(取自历史成功抓取轨迹),
      读世界位姿并保存。ENABLE_REAL_MOTION=False 时只读当前姿态不移动。

运行(Linux 容器内): python3 capture_tool_down_pose.py
安全: 移动用点到点 armMoveJoints, 低速, 人守急停。
"""
import json
import os

import sdk_session as ss

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1
GLOBAL_SPEED = 3.0
ENABLE_REAL_MOTION = False        # True: 移动到基准姿态再读; False: 只读当前姿态

# 已验证的 tool-down 关节姿态(SDK 度), 取自 20260721 成功抓取轨迹 PICK1 到底点。
# 该姿态实测 Y 轴与"朝下"夹角 0.1°(仿真侧核对), 是可靠的 tool-down 参考。
TOOL_DOWN_Q_SDK = [-82.7, 39.4, 127.8, -85.8, -72.4, -14.7, 45.7]
OUT_DIR = "uploads"


def main():
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    try:
        if ENABLE_REAL_MOTION:
            bad = ss.check_limits(ARM_ID, TOOL_DOWN_Q_SDK)
            if bad:
                raise SystemExit(f"基准姿态超限, 中止: {bad}")
            print("基准 tool-down 关节角(SDK 度):", TOOL_DOWN_Q_SDK)
            input("回车移动到基准 tool-down 姿态(人守急停): ")
            ss.move_joints_abs(sdk, ARM_ID, TOOL_DOWN_Q_SDK)
            ss.wait_until_joints(sdk, ARM_ID, TOOL_DOWN_Q_SDK)
        else:
            print("DRY: 只读当前姿态(如需移动到基准姿态, 改 ENABLE_REAL_MOTION=True)")
            print("提示: 也可手动把臂摆成工具朝下再跑本脚本读取。")

        q = ss.read_joints(sdk, ARM_ID)
        w = ss.read_worlds(sdk, ARM_ID)
        print("\n================ 捕捉结果 ================")
        print("当前关节角(SDK 度):", [round(v, 2) for v in q])
        print("当前世界位姿 [X,Y,Z(mm), U,V,W(deg)]:", [round(v, 3) for v in w])
        print("  -> tool-down 的真实 UVW =", [round(v, 3) for v in w[3:]])
        print("  (把这个 UVW 告诉 Mac 侧, 用它覆盖 world_grasp 导出的姿态)")

        os.makedirs(OUT_DIR, exist_ok=True)
        path = os.path.join(OUT_DIR, "tool_down_pose.json")
        with open(path, "w") as f:
            json.dump({"q_sdk_deg": q, "world_sdk": w,
                       "uvw_deg": w[3:], "xyz_mm": w[:3]}, f,
                      indent=2, ensure_ascii=False)
        print("已保存:", path)
    finally:
        ss.close_session(sdk)


if __name__ == "__main__":
    main()
