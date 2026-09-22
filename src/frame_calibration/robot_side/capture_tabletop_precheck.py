#!/usr/bin/env python3
"""导出既有成功 Worlds JSON 的只读控制器 IK；无 --run、无运动入口。

与 Debian 现有 execute_tabletop_pick_place_worlds.py/sdk_session.py/robot_lock.py
放在同一目录。复用现场执行器的计划校验、密集采样、限位和连续性检查。
输出中的 IK 是受当前构型影响的预测，不是已执行到位关节；不代表混合路径通过。
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys


READ_SDK_METHODS = frozenset({
    "armGetWorlds", "armGetJoints", "armGetAxisParameter", "armTryWorlds",
    "armGetLinkStatus", "getSoftStopSwitch", "armGetMoveState",
})
READ_HELPERS = frozenset({
    "read_worlds", "read_joints", "read_axis_limits", "axis_limits_look_valid",
    "check_limits", "check_soft_stop", "read_move_state",
})


class ReadOnlySDK:
    def __init__(self, sdk):
        self._sdk = sdk

    def __getattr__(self, name):
        if name not in READ_SDK_METHODS:
            raise RuntimeError(f"只读预检拒绝 SDK 操作: {name}")
        return getattr(self._sdk, name)


class Recorder:
    def __init__(self, ss, rows):
        self._ss = ss
        self._rows = rows

    def __getattr__(self, name):
        if name not in READ_HELPERS:
            raise RuntimeError(f"只读预检拒绝辅助操作: {name}")
        return getattr(self._ss, name)

    def try_worlds(self, sdk, arm_id, pose):
        row = {"sample": len(self._rows) + 1,
               "worlds_xyzuvw": list(map(float, pose)), "q_sdk_deg": None}
        self._rows.append(row)  # 异常/限位失败也保留最后请求
        result = self._ss.try_worlds(sdk, arm_id, pose)
        if result is None:
            return None
        result = list(map(float, result))
        if len(result) != 7 or not all(math.isfinite(v) for v in result):
            row["invalid_joint_response"] = True
            raise ValueError("armTryWorlds 未返回有限的七关节角")
        row["q_sdk_deg"] = result
        return result


def capture(executor, ss, config, plan, report):
    """测试可注入 fake；仅使用 open_read_only_session 与 SDK stop 收尾。"""
    sdk = None
    interrupted = False
    report.update(status="PRECHECK_FAILED", dense_ik_samples=[],
                  real_motion_authorized=False, session_stopped=False,
                  scope="EXISTING_WORLDS_PLAN_IK_NOT_HYBRID_COLLISION_QUALIFICATION")
    try:
        sdk = ss.open_read_only_session(config["robot_ip"], config["local_ip"],
                                        config["arm_ip"], config["arm_port"],
                                        (executor.ARM_ID,))
        readonly = ReadOnlySDK(sdk)
        report["live_start"] = {
            "worlds": executor._finite_vector(ss.read_worlds(readonly, executor.ARM_ID), 6, "live worlds"),
            "joints": executor._finite_vector(ss.read_joints(readonly, executor.ARM_ID), 7, "live joints"),
        }
        recorder = Recorder(ss, report["dense_ik_samples"])
        summary = executor.precheck_full_path(recorder, readonly, plan,
                                              margin=config["limit_margin_deg"])
        report["precheck_summary"] = summary
        report["endpoint_ik"] = {
            item["stage"]: item["target_joints_deg"] for item in summary["segments"]}
        report["status"] = "WORLDS_IK_PRECHECK_PASS_NOT_MOTION_AUTHORIZED"
        return 0
    except KeyboardInterrupt:
        interrupted = True
        report["error"] = "KeyboardInterrupt"
        return 130
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
        return 2
    finally:
        if sdk is not None:
            if not interrupted:
                # 全路径失败也独立查询各端点，便于离线复现；绝不把端点逆解算作路径 PASS。
                report["standalone_endpoint_queries"] = {}
                try:
                    for name, pose in executor.movement_targets(plan):
                        rows = []
                        try:
                            Recorder(ss, rows).try_worlds(ReadOnlySDK(sdk), executor.ARM_ID, pose)
                        except Exception as exc:
                            rows.append({"error_type": type(exc).__name__})
                        report["standalone_endpoint_queries"][name] = {
                            "role": "SDK_IK_PREDICTED_UNSEEDED_NOT_PATH_PASS", "samples": rows}
                except KeyboardInterrupt:
                    report["endpoint_queries_interrupted"] = True
                except Exception as exc:
                    report["endpoint_queries_error_type"] = type(exc).__name__
            try:
                # 只读会话从未改速度/保护，直接 stop，不走含还原写操作的通用关闭器。
                sdk.stop()
                report["session_stopped"] = True
            except Exception as exc:
                report["status"] = "SESSION_STOP_FAILED"
                report["stop_error"] = f"{type(exc).__name__}: {exc}"


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_local(name):
    path = Path(__file__).resolve().parent / f"{name}.py"
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载同目录依赖: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path, help="Debian 已有成功 JSON，不用新建/修改")
    parser.add_argument("--output", type=Path, default=Path("precheck_worlds_report.json"))
    parser.add_argument("--robot-ip")
    parser.add_argument("--local-ip")
    parser.add_argument("--arm-ip")
    parser.add_argument("--arm-port", type=int)
    parser.add_argument("--limit-margin-deg", type=float)
    args = parser.parse_args(argv)
    # 即使终端残留真机环境开关，本工具也不继承它。
    os.environ["XIFENG_ALLOW_REAL_MOTION"] = "0"
    executor = load_local("execute_tabletop_pick_place_worlds")
    ss = load_local("sdk_session")
    lock = load_local("robot_lock")
    if args.output.resolve() == args.plan.resolve() or args.output.suffix.lower() != ".json":
        parser.error("--output 必须是不同于输入轨迹的 JSON 文件")
    args.limit_margin_deg = (executor.LIMIT_MARGIN_DEG if args.limit_margin_deg is None
                             else args.limit_margin_deg)
    args.speed, args.recovery_only = 1.0, False  # 仅供既有配置解析；从不设置控制器速度
    config = executor._runtime_config(args, os.environ)
    plan, plan_sha = executor.load_plan(args.plan)
    report = {
        "schema_version": "tabletop_readonly_ik_capture.v1",
        "captured_at": datetime.now(timezone.utc).isoformat(),
        "plan_sha256": plan_sha,
        "plan_path": str(args.plan.resolve()),
        "plan_content": plan,
        "runtime": {key: config[key] for key in
                    ("robot_ip", "local_ip", "arm_ip", "arm_port", "limit_margin_deg")},
        "source_hashes": {Path(module.__file__).name: digest(module.__file__)
                          for module in (executor, ss, lock)},
        "capture_script_sha256": digest(__file__),
        "warning": "控制器 IK 预测不是实测到位构型；不证明碰撞或混合转运可行",
        "ik_role": "SDK_IK_PREDICTED_NOT_MEASURED_ARRIVAL",
    }
    print(f"plan SHA-256: {plan_sha}")
    for name, sha in report["source_hashes"].items():
        print(f"{name} SHA-256: {sha}")
    print("READ ONLY: 零使能、零运动、零夹爪命令；仅预检并导出")
    with lock.acquire(config["robot_ip"], executor.ARM_ID, "capture_tabletop_precheck.py"):
        code = capture(executor, ss, config, plan, report)
    if report["status"] == "SESSION_STOP_FAILED":
        code = 2
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    print(f"{report['status']}: {args.output.resolve()}")
    if "error" in report:
        print(report["error"])
    for stage, joints in report.get("endpoint_ik", {}).items():
        print(f"{stage}: q_sdk_deg={joints}")
    return code


if __name__ == "__main__":
    sys.exit(main())
