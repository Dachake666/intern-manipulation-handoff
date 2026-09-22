#!/usr/bin/env python3
import statistics
import time
import os
import threading
import json

import sdk_session as ss
import robot_lock

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.244"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1

N = 300
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
            pulse_records = []
            next_due = time.monotonic()

            print(
                f"TRACE_ID pid={os.getpid()} "
                f"main_tid={threading.get_native_id()}",
                flush=True,
            )

            input("保持机械臂当前位置不变；回车开始 stationary pulse latency probe，Ctrl-C 中止：")

            for i in range(N):
                now = time.monotonic()

                if now < next_due:
                    time.sleep(next_due - now)

                wall_start_ns = time.time_ns()
                perf_start_ns = time.perf_counter_ns()

                code = sdk.armPluseToServo(
                    ARM_ID,
                    vec,
                )

                perf_end_ns = time.perf_counter_ns()
                wall_end_ns = time.time_ns()

                ss.require_code_zero(
                    code,
                    "stationary armPluseToServo",
                )

                dt_ms = (
                    perf_end_ns - perf_start_ns
                ) / 1_000_000.0

                pulse_records.append({
                    "frame": i,
                    "pid": os.getpid(),
                    "main_tid": threading.get_native_id(),
                    "wall_start_s": wall_start_ns / 1e9,
                    "wall_end_s": wall_end_ns / 1e9,
                    "latency_ms": dt_ms,
                })

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

            out = "pulse_stationary_correlation.jsonl"
            with open(out, "w", encoding="utf-8") as fh:
                for rec in pulse_records:
                    fh.write(json.dumps(rec) + "\n")

            print(f"correlation log: {out}")
            print("\n=== slow Pulse calls >=20ms ===")

            for rec in pulse_records:
                if rec["latency_ms"] >= 20.0:
                    print(
                        f"frame={rec['frame']:03d} "
                        f"start={rec['wall_start_s']:.6f} "
                        f"end={rec['wall_end_s']:.6f} "
                        f"latency={rec['latency_ms']:.3f}ms"
                    )

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
