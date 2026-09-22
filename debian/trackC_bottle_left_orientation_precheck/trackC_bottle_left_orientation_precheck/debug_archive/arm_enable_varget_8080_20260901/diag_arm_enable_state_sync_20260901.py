#!/usr/bin/env python3

import time
from datetime import datetime
import pypilot

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080

POLL_INTERVAL = 0.5
POLL_DURATION = 10.0


def now():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]


def main():
    print("=" * 72)
    print("armInitData + Enable 状态同步诊断")
    print("时间:", now())
    print("=" * 72)

    sdk = pypilot.PilotSDK(ROBOT_IP)

    try:
        print(f"[{now()}] sdk.start()")
        ok = sdk.start()
        print(f"[{now()}] sdk.start -> {ok}")
        if not ok:
            raise RuntimeError("PilotSDK 启动失败")

        time.sleep(1)

        print(f"[{now()}] soft_stop -> {sdk.getSoftStopSwitch()}")

        print()
        print(f"[{now()}] >>> armInitData")

        t0 = time.monotonic()
        code = sdk.armInitData(LOCAL_IP, ARM_IP, ARM_PORT)
        dt = time.monotonic() - t0

        print(
            f"[{now()}] <<< armInitData: "
            f"code={code}, 耗时={dt:.3f}s"
        )

        print(f"[{now()}] link(before enable)   -> {sdk.armGetLinkStatus()}")
        print(f"[{now()}] enable(before enable) -> {sdk.armGetRobotEnableStatus()}")
        print(f"[{now()}] servo(before enable)  -> {sdk.armServoIsOpOrNot()}")

        print()
        print(f"[{now()}] >>> armRobotEnableOrNot(True) 只发送一次")

        t_enable = time.monotonic()
        enable_code = sdk.armRobotEnableOrNot(True)

        print(
            f"[{now()}] <<< enable command return: "
            f"{enable_code}"
        )

        print()
        print("开始轮询状态")
        print("-" * 72)

        sample = 0

        while True:
            elapsed = time.monotonic() - t_enable

            if elapsed > POLL_DURATION:
                break

            link = sdk.armGetLinkStatus()
            enable = sdk.armGetRobotEnableStatus()
            servo = sdk.armServoIsOpOrNot()

            print(
                f"[{now()}] "
                f"t=+{elapsed:5.2f}s  "
                f"#{sample:02d}  "
                f"link={link}  "
                f"enable={enable}  "
                f"servo={servo}"
            )

            sample += 1
            time.sleep(POLL_INTERVAL)

        print("-" * 72)
        print(f"[{now()}] 诊断结束")

    finally:
        print(f"[{now()}] sdk.stop()")
        try:
            sdk.stop()
        except Exception as e:
            print("sdk.stop exception:", e)


if __name__ == "__main__":
    main()
