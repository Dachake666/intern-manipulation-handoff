#!/usr/bin/env python3
import statistics
import time

import sdk_session as ss
import robot_lock

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.147"
ARM_PORT = 8080
ARM_ID   = 1

N = 1500
PERIOD_S = 0.020


def main():
    sdk = None
    protection_disabled = False

    with robot_lock.acquire(
        ROBOT_IP,
        ARM_ID,
        "probe_pulse_latency_stationary.py",
    ):
        try:
            sdk = ss.open_session(
                ROBOT_IP,
                LOCAL_IP,
                ARM_IP,
                ARM_PORT,
                5.0,
                (ARM_ID,),
            )

            q = ss.read_joints(sdk, ARM_ID)

            print("固定目标 q =", [round(x, 3) for x in q])
            print(f"测试 {N} 帧，同一目标，周期 {PERIOD_S*1000:.0f} ms")

            vec = ss.make_float_vector(q)

            ss.check_soft_stop(sdk, "stationary pulse probe")

            ss.set_protect_status(sdk, ARM_ID, False)
            protection_disabled = True

            times = []
            late = []
            next_due = time.monotonic()

            input("保持机械臂当前位置不变；回车开始 stationary pulse latency probe，Ctrl-C 中止：")

            for i in range(N):
                now = time.monotonic()

                if now < next_due:
                    time.sleep(next_due - now)

                send_start = time.perf_counter()

                code = sdk.armPluseToServo(
                    ARM_ID,
                    vec,
                )
                ss.require_code_zero(
                    code,
                    "stationary armPluseToServo",
                )

                dt_ms = (
                    time.perf_counter() - send_start
                ) * 1000.0

                times.append(dt_ms)

                now = time.monotonic()
                lateness = max(
                    0.0,
                    now - next_due - PERIOD_S
                ) * 1000.0
                late.append(lateness)

                next_due = now + max(
                    0.0,
                    PERIOD_S - dt_ms / 1000.0
                )

            xs = sorted(times)

            def pct(p):
                idx = round((len(xs) - 1) * p)
                return xs[idx]

            print("\n=== armPluseToServo stationary latency ===")
            print(f"calls:       {len(times)}")
            print(f"mean:        {statistics.mean(times):.3f} ms")
            print(f"median:      {statistics.median(times):.3f} ms")
            print(f"p90:         {pct(0.90):.3f} ms")
            print(f"p95:         {pct(0.95):.3f} ms")
            print(f"p99:         {pct(0.99):.3f} ms")
            print(f"max:         {max(times):.3f} ms")
            print(f">5ms:        {sum(x > 5 for x in times)}")
            print(f">20ms:       {sum(x > 20 for x in times)}")
            print(f">40ms:       {sum(x > 40 for x in times)}")

        finally:
            if sdk is not None:
                if protection_disabled:
                    try:
                        ss.set_protect_status(
                            sdk,
                            ARM_ID,
                            True,
                        )
                    except Exception as exc:
                        print("恢复保护失败:", exc)

                try:
                    ss.close_session(sdk)
                except Exception as exc:
                    print("关闭 SDK 失败:", exc)


if __name__ == "__main__":
    main()
