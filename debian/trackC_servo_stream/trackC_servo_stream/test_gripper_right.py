#!/usr/bin/env python3
"""右夹爪开合测试 —— 只动夹爪, 机械臂一动不动。

右夹爪 = 1 号(20260709 单发实测确认 左=2 右=1, commit 68e871b;
20260806 本脚本再次真机验证通过), 与左夹爪共用 COM1。
帧格式与已在真机跑通数百次的完全一致(execute_trajectory / execute_servo_grasp):
    eb 90 <gid> <len> <cmd> <data...> <checksum>
    close = 0x10 + [速度, 力度]
    open  = 0x11 + [开口位置]

安全:
  * 用【夹爪专用会话】: 不清报警、不使能、不设速度、不读世界坐标 ——
    机械臂全程不动, 只开串口写夹爪。
  * 不发任何运动指令, 不需要机械臂互斥锁。
  * 默认只做一次 open; --action close 只闭合; --cycle 做开合循环。

用法(容器内):
    python3 test_gripper_right.py                 # 干跑, 只打印要发的帧
    python3 test_gripper_right.py --run           # 张开一次
    python3 test_gripper_right.py --run --action close       # 闭合一次
    python3 test_gripper_right.py --run --cycle 3 # 开合 3 轮
    python3 test_gripper_right.py --run --gid 1 --com COM2   # 换 ID / 串口试
"""
import argparse
import sys
import time

import sdk_session as ss

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080

RIGHT_GID = 1                  # 右夹爪; 左=2(两者共用 COM1, 靠帧里的 ID 区分)
COM = "COM1"
SERIAL = (115200, 8, 0, 1)
CLOSE_VALUES = [500, 1000]     # [速度, 力度] —— 与左夹爪真机成功参数一致
OPEN_VALUES = [500]            # [开口位置]


def build_cmd(gid, command, values):
    """与真机跑通的实现逐字节一致, 不要改。"""
    data = []
    for v in values:
        v = max(0, min(0xFFFF, int(v)))
        data += [v & 0xFF, (v >> 8) & 0xFF]
    payload = [int(gid) & 0xFF, 1 + len(data), int(command) & 0xFF] + data
    return " ".join(f"{b:02x}" for b in
                    [0xEB, 0x90] + payload + [sum(payload) & 0xFF])


def open_gripper_session(robot_ip, local_ip, arm_ip, arm_port):
    """只建立夹爪串口所需的最小 SDK 会话。

    不复用 sdk_session.open_session(enable=False): 那条路径专为 Worlds
    逆解预检设计, 会强制等待 armGetWorlds。夹爪只需通信链路
    和 COM, 不应因为世界坐标读不到而被拦下。
    """
    sdk = ss.pypilot.PilotSDK(robot_ip)
    started = False
    try:
        print("=" * 60)
        print("启动 PilotSDK(夹爪专用会话)")
        print("=" * 60)
        if not sdk.start():
            raise RuntimeError("PilotSDK 启动失败")
        started = True
        time.sleep(1.0)

        code, status = sdk.getSoftStopSwitch()
        print("soft_stop:", (code, status))
        if code != 0 or status != "CLOSE":
            raise RuntimeError(f"软急停状态异常: ({code}, {status}), 中止")

        code = sdk.armInitData(local_ip, arm_ip, arm_port)
        print("armInitData:", code)
        ss.require_code_zero(code, "机械臂通信初始化")

        code, linked = sdk.armGetLinkStatus()
        print("link:", (code, linked))
        ss.require_true_status(code, linked, "机械臂通信链路")
        print("夹爪专用会话: 不清报警 / 不使能 / 不设速度 / "
              "不读 armGetWorlds")
        return sdk
    except BaseException:
        if started:
            try:
                sdk.stop()
            except Exception:
                pass
        raise


def main(argv=None):
    ap = argparse.ArgumentParser(description="右夹爪开合测试(不动机械臂)")
    ap.add_argument("--run", action="store_true", help="真发(不加则只打印帧)")
    ap.add_argument("--action", choices=("open", "close", "cycle"),
                    help="动作; 默认 open, 也可只 close 或指定 cycle")
    ap.add_argument("--cycle", type=int, default=0,
                    help="开合循环轮数; >0 时等价于 --action cycle")
    ap.add_argument("--gid", type=int, default=RIGHT_GID, help="夹爪 ID, 右=1 左=2")
    ap.add_argument("--com", default=COM)
    ap.add_argument("--robot-ip", default=ROBOT_IP, help="PilotSDK 主机 IP")
    ap.add_argument("--local-ip", default=LOCAL_IP, help="Debian 机器人网卡 IP")
    ap.add_argument("--arm-ip", default=ARM_IP, help="机械臂通信 IP")
    ap.add_argument("--arm-port", type=int, default=ARM_PORT)
    ap.add_argument("--force", type=int, default=CLOSE_VALUES[1], help="夹持力度")
    ap.add_argument("--wait-close", type=float, default=5.0)
    ap.add_argument("--wait-open", type=float, default=2.5)
    a = ap.parse_args(argv)

    close_hex = build_cmd(a.gid, 0x10, [CLOSE_VALUES[0], a.force])
    open_hex = build_cmd(a.gid, 0x11, OPEN_VALUES)
    if a.cycle < 0:
        ap.error("--cycle 不能为负数")
    action = "cycle" if a.cycle > 0 else (a.action or "open")
    if action == "cycle" and a.cycle <= 0:
        ap.error("--action cycle 必须同时给出 --cycle N (N>=1)")
    plan = (["open"] if action == "open" else
            ["close"] if action == "close" else
            ["close", "open"] * a.cycle)
    print("=" * 62)
    print(f"右夹爪测试  ID={a.gid}  串口={a.com} {SERIAL}  力度={a.force}")
    print("=" * 62)
    print(f"  张开 hex = {open_hex}")
    print(f"  闭合 hex = {close_hex}")
    print(f"  计划: {' -> '.join(plan)}")
    print(f"  连接: robot={a.robot_ip}  local={a.local_ip}  "
          f"arm={a.arm_ip}:{a.arm_port}")
    if not a.run:
        print("\n干跑结束。加 --run 真发(人守急停)。")
        return 0

    sdk = None
    try:
        sdk = open_gripper_session(a.robot_ip, a.local_ip, a.arm_ip, a.arm_port)
        code = sdk.armOpenCom(a.com, *SERIAL)
        print(f"\narmOpenCom({a.com}, {SERIAL}) -> {code}")
        ss.require_code_zero(code, "armOpenCom")

        input("\n回车开始发夹爪指令(手离开夹爪): ")
        for i, act in enumerate(plan, 1):
            hexcmd = open_hex if act == "open" else close_hex
            code = sdk.armWriteCom(a.com, bytes.fromhex(hexcmd.replace(" ", "")))
            wait = a.wait_open if act == "open" else a.wait_close
            print(f"  [{i}/{len(plan)}] {act:5s} armWriteCom -> {code}   "
                  f"等 {wait}s")
            ss.require_code_zero(code, f"夹爪{a.gid} {act}")
            time.sleep(wait)

        print("\n" + "=" * 62)
        print("指令已全部发出。请目视确认右夹爪是否真的动了。")
        print("  动了            -> 右夹爪 ID/串口正确, 通路正常")
        print("  没动但返回码 0  -> 帧发出去了但没人响应: ID 可能不对(试 --gid 3 等),")
        print("                     或右夹爪在另一个串口(试 --com COM2),")
        print("                     或右夹爪供电/接线有问题")
        print("  左夹爪动了      -> ID 串了, 当前这个 ID 其实是左爪")
    finally:
        if sdk is not None:
            ss.close_session(sdk)
    return 0


if __name__ == "__main__":
    sys.exit(main())
