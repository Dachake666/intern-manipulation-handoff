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
    print("=" * 70)
    print("armInitData 状态同步诊断")
    print("时间:", now())
    print(f"robot={ROBOT_IP}")
    print(f"local={LOCAL_IP}")
    print(f"arm={ARM_IP}:{ARM_PORT}")
    print("=" * 70)

    sdk = pypilot.PilotSDK(ROBOT_IP)

    try:
        print(f"[{now()}] sdk.start()")
        ok = sdk.start()
        print(f"[{now()}] sdk.start -> {ok}")

        if not ok:
            raise RuntimeError("PilotSDK 启动失败")

        time.sleep(1.0)

        print(f"[{now()}] soft_stop -> {sdk.getSoftStopSwitch()}")

        print()
        print(f"[{now()}] >>> 调用 armInitData")

        t_init = time.monotonic()
        code = sdk.armInitData(LOCAL_IP, ARM_IP, ARM_PORT)
        init_elapsed = time.monotonic() - t_init

        print(
            f"[{now()}] <<< armInitData 返回: "
            f"code={code}, 耗时={init_elapsed:.3f}s"
        )

        if code != 0:
            raise RuntimeError(f"armInitData 失败: {code}")

        print()
        print("开始轮询，仅仅读状态，不发送 enable 命令")
        print("-" * 70)

        t0 = time.monotonic()
        sample = 0

        while True:
            elapsed = time.monotonic() - t0
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

        print("-" * 70)
        print(f"[{now()}] 诊断结束")

    finally:
        print(f"[{now()}] sdk.stop()")
        try:
            sdk.stop()
        except Exception as e:
            print("sdk.stop exception:", e)


if __name__ == "__main__":
    main()
