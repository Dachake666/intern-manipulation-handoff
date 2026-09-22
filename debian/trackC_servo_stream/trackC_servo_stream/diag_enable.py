#!/usr/bin/env python3
"""机械臂使能专项诊断：不发运动指令，不操作夹爪。

默认只打印配置；显式加 --run 后才会清报警并发送【一次】整体使能命令，随后只读
轮询整体使能、伺服 OP 和目标单臂使能。无论成功失败都会停止 SDK。
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
import time

import robot_lock
import sdk_session as ss

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="使能专项诊断（不发运动指令、不操作夹爪）")
    ap.add_argument("--run", action="store_true",
                    help="真的连接、清报警并使能；不加则只打印配置")
    ap.add_argument("--arm-id", type=int, choices=(1, 2), default=1,
                    help="1=左臂，2=右臂")
    ap.add_argument("--robot-ip", default=ROBOT_IP)
    ap.add_argument("--local-ip", default=LOCAL_IP)
    ap.add_argument("--arm-ip", default=ARM_IP)
    ap.add_argument("--arm-port", type=int, default=ARM_PORT)
    args = ap.parse_args(argv)

    here = os.path.dirname(os.path.abspath(__file__))
    session_path = os.path.join(here, "sdk_session.py")
    sdk_path = getattr(ss.pypilot, "__file__", "<unknown>")
    print("=" * 64)
    print("使能专项诊断（零运动、零夹爪命令）")
    print("=" * 64)
    print(f"目标: robot={args.robot_ip}  local={args.local_ip}  "
          f"arm={args.arm_ip}:{args.arm_port}  {ss.arm_name(args.arm_id)}")
    print(f"Python: {sys.version.split()[0]}")
    print(f"pypilot: {sdk_path}")
    print(f"diag_enable.py SHA-256: {sha256(__file__)}")
    print(f"sdk_session.py SHA-256: {sha256(session_path)}")
    if not args.run:
        print("DRY RUN：未连接、未清报警、未使能。确认急停有人值守后加 --run。")
        return 0

    sdk = None
    started = False
    with robot_lock.acquire(args.robot_ip, args.arm_id, "diag_enable.py"):
        try:
            sdk = ss.pypilot.PilotSDK(args.robot_ip)
            if not sdk.start():
                raise RuntimeError("PilotSDK 启动失败")
            started = True
            time.sleep(1.0)

            soft_stop = sdk.getSoftStopSwitch()
            print("soft_stop:", soft_stop)
            if soft_stop != (0, "CLOSE"):
                raise RuntimeError(f"软急停状态异常: {soft_stop}")

            code = sdk.armInitData(args.local_ip, args.arm_ip, args.arm_port)
            print("armInitData:", code)
            ss.require_code_zero(code, "机械臂初始化")

            link = sdk.armGetLinkStatus()
            print("link:", link)
            ss.require_true_status(*link, "机械臂链路")

            code = sdk.armClearAlarm()
            print("clear alarm:", code)
            ss.require_code_zero(code, "清除报警")

            # 唯一一次写使能；后面全部是只读 getter。
            code = sdk.armRobotEnableOrNot(True)
            print("enable all:", code)
            ss.require_code_zero(code, "整体使能")
            ss.wait_true_status(sdk.armGetRobotEnableStatus, "enable")
            ss.wait_true_status(sdk.armServoIsOpOrNot, "servo op")
            ss.wait_true_status(
                lambda: sdk.armGetSingleRobotEnableStatus(args.arm_id),
                f"{ss.arm_name(args.arm_id)} single enable")
            print("✅ 使能专项诊断通过：三层状态均为 True；未发送运动/夹爪命令。")
            return 0
        except BaseException:
            if sdk is not None and started:
                ss.print_enable_diagnostics(sdk, (args.arm_id,))
            raise
        finally:
            if sdk is not None and started:
                try:
                    sdk.stop()
                    print("SDK 已停止。")
                except Exception as exc:         # noqa: BLE001 收尾失败只补充记录
                    print("⚠ SDK 停止异常:", exc)


if __name__ == "__main__":
    raise SystemExit(main())
