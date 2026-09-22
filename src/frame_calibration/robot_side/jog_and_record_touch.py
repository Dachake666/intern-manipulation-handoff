#!/usr/bin/env python3
"""问题2b 配套: 交互式点动(jog) + 触碰记录 —— 定点触碰法 TCP 标定的采数工具。

操作方法:
  1. 在工作区固定一个尖点参照物(顶针/螺栓尖/笔尖朝上夹稳, 期间绝不能碰动);
  2. 用 j 命令小步点动, 引导【夹爪指间中心】轻触尖点;
  3. 触到后输入 rec <名字> 记录; 换一个明显不同的腕部朝向(建议相互差 30deg 以上),
     再次触碰并记录; 至少 3 次, 建议 4~5 次;
  4. save 保存, 然后用 analysis/solve_tcp_from_touch.py 求解 TCP 偏移 r。

命令:
  j<N> <delta>   点动: SDK 关节 N 转 delta 度(单步 <= 5)。例: j5 2.5 / j6 -3
  read           读取并打印当前 关节角 + XYZ/UVW
  rec <name>     记录当前姿态为一次触碰样本
  undo           删除最后一条记录
  list           列出已记录样本
  save           保存 JSON
  q              退出(有未保存记录时自动保存)

运行(容器内):
    python3 jog_and_record_touch.py
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
GLOBAL_SPEED = 3.0             # 点动用更低速度
ENABLE_REAL_MOTION = False     # False 时 j 命令只打印不动; read/rec 仍可用
MAX_JOG_STEP_DEG = 5.0
OUT_DIR = "data"


def do_jog(sdk, joint_num, delta):
    if not 1 <= joint_num <= 7:
        print("关节号必须是 1~7")
        return
    if abs(delta) > MAX_JOG_STEP_DEG:
        print(f"单步限制 ±{MAX_JOG_STEP_DEG} deg, 收到 {delta}")
        return
    current = ss.read_joints(sdk, ARM_ID)
    target = current[:]
    target[joint_num - 1] += delta
    bad = ss.check_limits(ARM_ID, target)
    if bad:
        print("目标越限, 拒绝:", "; ".join(bad))
        return
    print(f"j{joint_num}: {current[joint_num - 1]:+.3f} -> {target[joint_num - 1]:+.3f} deg")
    if not ENABLE_REAL_MOTION:
        print("(DRY RUN: 未发送。改 ENABLE_REAL_MOTION=True 后才会真动)")
        return
    ss.check_soft_stop(sdk, "jog")
    ss.move_joints_abs(sdk, ARM_ID, target)
    ss.wait_until_joints(sdk, ARM_ID, target, tol_deg=0.4, timeout_s=15.0)


def print_state(sdk):
    joints = ss.read_joints(sdk, ARM_ID)
    worlds = ss.read_worlds(sdk, ARM_ID)
    print(f"q_deg  = {[round(v, 3) for v in joints]}")
    print(f"xyz_mm = {[round(v, 3) for v in worlds[:3]]}  uvw_deg = {[round(v, 3) for v in worlds[3:]]}")


def save(records):
    if records:
        ss.save_samples(OUT_DIR, "touch_records", {
            "arm_id": ARM_ID,
            "description": "定点触碰法 TCP 标定记录(夹爪指间中心触同一尖点)",
        }, records)


def main():
    print(__doc__)
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
    records = []
    saved = True
    try:
        print(f"\nENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")
        print_state(sdk)
        while True:
            try:
                line = input("\ntouch> ").strip()
            except EOFError:
                break
            if not line:
                continue
            parts = line.split()
            cmd = parts[0].lower()

            if cmd == "q":
                break
            elif cmd == "read":
                print_state(sdk)
            elif cmd == "rec":
                name = parts[1] if len(parts) > 1 else f"touch_{len(records) + 1:02d}"
                records.append(ss.snapshot(sdk, ARM_ID, name, "touch point sample"))
                saved = False
                print(f"已记录 {name} (共 {len(records)} 条; 建议 >=4 条且腕部朝向差 >=30deg)")
            elif cmd == "undo":
                if records:
                    gone = records.pop()
                    saved = False
                    print(f"已删除 {gone['name']}")
                else:
                    print("没有记录可删")
            elif cmd == "list":
                for r in records:
                    print(f"  {r['name']:20s} q={[round(v, 2) for v in r['q_sdk_deg']]}")
                print(f"共 {len(records)} 条")
            elif cmd == "save":
                save(records)
                saved = True
            elif cmd.startswith("j") and len(cmd) > 1 and cmd[1:].isdigit() and len(parts) == 2:
                try:
                    do_jog(sdk, int(cmd[1:]), float(parts[1]))
                except ValueError:
                    print("角度格式不对, 例: j5 2.5")
            else:
                print("未知命令。可用: j<N> <delta> / read / rec <name> / undo / list / save / q")
    finally:
        if records and not saved:
            print("\n有未保存记录, 自动保存:")
            save(records)
        ss.close_session(sdk)
        print("\n提醒: 请确认 ENABLE_REAL_MOTION 已恢复为 False。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
