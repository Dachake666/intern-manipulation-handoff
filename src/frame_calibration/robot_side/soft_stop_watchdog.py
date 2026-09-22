#!/usr/bin/env python3
"""独立只读软急停看门狗；退出即由视觉执行器触发停机。

本进程只启动 PilotSDK 并读取 getSoftStopSwitch，不初始化机械臂、不清报警、
不使能、不设速度，也不获取机器人运动互斥锁，因此可与持锁执行器并行。
"""
from __future__ import annotations

import argparse
import os
import signal
import sys
import time

import sdk_session as ss


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--robot-ip", default=os.environ.get("XIFENG_ROBOT_IP"))
    ap.add_argument("--period-ms", type=float, default=50.0)
    args = ap.parse_args(argv)
    if not args.robot_ip:
        ap.error("必须由 CLI/环境提供 robot-ip")
    if not 20 <= args.period_ms <= 1000:
        ap.error("period-ms 必须在 [20,1000]")

    ss.require_pypilot()
    sdk = ss.pypilot.PilotSDK(args.robot_ip)
    if not sdk.start():
        print("soft-stop watchdog: PilotSDK 启动失败", file=sys.stderr)
        return 2
    running = True

    def stop(_signum, _frame):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        while running:
            code, status = sdk.getSoftStopSwitch()
            if code != 0 or status != "CLOSE":
                print(f"soft-stop watchdog: unsafe code={code} status={status}",
                      file=sys.stderr, flush=True)
                return 3
            time.sleep(args.period_ms / 1000.0)
        return 0
    finally:
        try:
            sdk.stop()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
