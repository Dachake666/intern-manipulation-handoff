#!/usr/bin/env python3
"""在一个机器人级锁内按顺序执行多物/跨左右臂 mission；首版禁止并发。"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
if (HERE / "execute_servo_grasp.py").exists():  # Track V 扁平发布包
    WORK = HERE
    ROBOT_SIDE = HERE
else:                                             # 源码工作区
    WORK = HERE.parent
    ROBOT_SIDE = WORK / "frame_calibration" / "robot_side"
sys.path.insert(0, str(ROBOT_SIDE)); sys.path.insert(0, str(WORK))
import robot_lock
import execute_servo_grasp as executor


def load_plan(path):
    plan = json.loads(Path(path).read_text(encoding="utf-8"))
    if plan.get("schema_version") != "mission_plan.v1":
        raise ValueError("mission plan schema 必须是 mission_plan.v1")
    if plan.get("policy") != "strict_serial" or not plan.get("steps"):
        raise ValueError("mission plan 必须 strict_serial 且至少一步")
    required = {"trajectory", "preflight_report", "approval_token",
                "scene_recheck", "robot_config_identity"}
    for i, step in enumerate(plan["steps"], 1):
        missing = required - set(step)
        if missing:
            raise ValueError(f"step {i} 缺字段 {sorted(missing)}")
    return plan


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("plan"); ap.add_argument("--robot-ip", required=True)
    ap.add_argument("--local-ip", required=True); ap.add_argument("--arm-ip")
    ap.add_argument("--arm-port", type=int, default=8080)
    ap.add_argument("--vision-check-command", required=True)
    ap.add_argument("--scene-watchdog-command", required=True)
    ap.add_argument("--soft-stop-watchdog-command", required=True)
    ap.add_argument("--step-deg", type=float, default=.4)
    ap.add_argument("--period-ms", type=float, default=20)
    ap.add_argument("--speed", type=float, default=15)
    a = ap.parse_args(argv)
    plan = load_plan(a.plan)
    executor.EXTERNAL_TASK_LOCK = True
    with robot_lock.acquire_robot(a.robot_ip, "mission_runner.py"):
        for index, step in enumerate(plan["steps"], 1):
            print(f"\n===== mission step {index}/{len(plan['steps'])} =====")
            cmd = [step["trajectory"], "--run", "--robot-ip", a.robot_ip,
                   "--local-ip", a.local_ip, "--arm-ip", a.arm_ip or a.robot_ip,
                   "--arm-port", str(a.arm_port), "--step-deg", str(a.step_deg),
                   "--period-ms", str(a.period_ms), "--speed", str(a.speed),
                   "--preflight-report", step["preflight_report"],
                   "--approval-token", step["approval_token"],
                   "--scene-recheck", step["scene_recheck"],
                   "--robot-config-identity", step["robot_config_identity"],
                   "--vision-check-command", a.vision_check_command,
                   "--scene-watchdog-command", a.scene_watchdog_command,
                   "--soft-stop-watchdog-command", a.soft_stop_watchdog_command]
            rc = executor.main(cmd)
            if rc != 0:
                print(f"mission 在 step {index} 停止 rc={rc}")
                return rc
    print("mission 全部步骤串行完成")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
