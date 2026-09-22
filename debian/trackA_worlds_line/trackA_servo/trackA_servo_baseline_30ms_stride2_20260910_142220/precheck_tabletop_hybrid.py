#!/usr/bin/env python3
"""混合候选的 Debian 只读诊断：不使能、不运动、不操作夹爪，无 --run。

MoveWorlds: 密集 armTryWorlds/限位/解连续性与离线关节分支比较。
MoveJoints: 按候选密集关节做实时限位交集检查，另查询预期 EE 的 IK；
后者不验证所指定关节的 FK，也不证明该 MoveJ 路径可执行或无碰撞。
所有查询均受当前真实构型影响，没有把预测关节写回机器人作 seed。
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import sys

import execute_tabletop_pick_place_worlds as shared
import sdk_session as ss
import robot_lock
import scrub_log

SCHEMA = "tabletop_hybrid_candidate.v1"
SCHEMA_V2 = "tabletop_hybrid_candidate.v2"
STAGES = [
    ("START_ASSERT_SAFE", "ASSERT_ENDPOINT"), ("OPEN_BEFORE_PICK", "GRIPPER"),
    ("PICK_HOVER", "MOVE_WORLDS"), ("PICK_DESCEND", "MOVE_WORLDS"),
    ("CLOSE_AT_PICK", "GRIPPER"), ("PICK_ASCEND", "MOVE_WORLDS"),
    ("TRANSFER_MID", "MOVE_JOINTS"), ("PLACE_HOVER", "MOVE_JOINTS"),
    ("PLACE_DESCEND", "MOVE_WORLDS"), ("OPEN_AT_PLACE", "GRIPPER"),
    ("PLACE_ASCEND", "MOVE_WORLDS"), ("RETURN_SAFE", "MOVE_WORLDS"),
    ("END_ASSERT_SAFE", "ASSERT_ENDPOINT"),
]
READ_METHODS = frozenset({"armGetWorlds", "armGetJoints", "armGetAxisParameter", "armTryWorlds",
                          "getSoftStopSwitch", "armGetRobotMoveState", "armGetLinkStatus"})


class ReadOnlySDK:
    def __init__(self, sdk):
        self._sdk = sdk

    def __getattr__(self, name):
        if name not in READ_METHODS:
            raise RuntimeError(f"只读工具拒绝 SDK 操作: {name}")
        return getattr(self._sdk, name)


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vec(values, n):
    return shared._finite_vector(values, n, "hybrid numeric vector")


def pose6(pose):
    values = vec(pose["position_mm"], 3) + vec(pose["sdk_world_uvw_deg"], 3)
    quat = vec(pose["quaternion_xyzw"], 4)
    if abs(sum(x*x for x in quat) - 1) > .001:
        raise ValueError("四元数未归一化")
    return values


def limit_errors(q, limits):
    return shared._controller_limit_violations(vec(q, 7), limits, margin=ss.LIMIT_MARGIN_DEG)


def max_delta(a, b):
    return max(abs(x-y) for x, y in zip(a, b))


def validate(plan):
    """严格拒绝错臂、错顺序、非有限值及可执行伪装；只消费现有离线候选。"""
    if (plan.get("schema_version") not in (SCHEMA, SCHEMA_V2) or plan.get("status") != "OFFLINE_CANDIDATE_BLOCKED"
            or plan.get("real_motion_authorized") is not False or plan.get("debian_execution_allowed") is not False):
        raise ValueError("需要原样的混合离线候选，不能修改授权/状态开关")
    contract = plan["contract"]
    expected = {"arm": "left", "arm_id": shared.ARM_ID, "gripper_id": shared.GRIPPER_ID,
                "world_frame": "sdk_world", "length_unit": "millimeter",
                "dynamic_transport": "MOVE_WORLDS_AND_MOVE_JOINTS"}
    if any(contract.get(k) != v or isinstance(contract.get(k), bool) for k, v in expected.items()):
        raise ValueError("混合轨迹坐标/臂/夹爪契约不符")
    if plan.get("meta", {}).get("arm_id") != shared.ARM_ID or plan["meta"].get("gripper_id") != shared.GRIPPER_ID:
        raise ValueError("meta臂/夹爪与contract不一致")
    stages = list(STAGES)
    if plan["schema_version"] == SCHEMA_V2:
        stages = [(name, "MOVE_JOINTS" if name == "RETURN_SAFE" else kind) for name, kind in stages]
        if not any(s.get("name") == "TRANSFER_MID" for s in plan["stages"]):
            stages = [(name, kind) for name, kind in stages if name != "TRANSFER_MID"]
        home = vec(plan["home_joints_sdk_deg"], 7)
        if limit_errors(home, ss.LIMITS_DEG[shared.ARM_ID]):
            raise ValueError("Home关节违反共享限位")
        end = next(s for s in plan["stages"] if s["name"] == "END_ASSERT_SAFE")
        ret = next(s for s in plan["stages"] if s["name"] == "RETURN_SAFE")
        if max_delta(home, vec(ret["q_sdk_deg"], 7)) > 1e-6:
            raise ValueError("RETURN_SAFE与home_joints_sdk_deg不一致")
        if max_delta(pose6(ret["expected_endpoint"]), pose6(end["pose"])) > 1e-6:
            raise ValueError("RETURN_SAFE的预期末端与最终Home断言不一致")
    if [(s["name"], s["kind"]) for s in plan["stages"]] != stages:
        raise ValueError("段名称/顺序/动作类型不符")
    actions = [s.get("action") for s in plan["stages"] if s["kind"] == "GRIPPER"]
    if actions != ["open", "close", "open"]:
        raise ValueError("夹爪事件不完整")
    frames = plan["offline_replay"]["frames"]
    groups = {name: [] for name, kind in stages if kind.startswith("MOVE_")}
    previous, phase = None, -1
    order = list(groups)
    for frame in frames:
        if "q_sdk_deg" not in frame:
            continue
        q = vec(frame["q_sdk_deg"], 7)
        if limit_errors(q, ss.LIMITS_DEG[shared.ARM_ID]):
            raise ValueError(f"离线帧超出共享限位: {frame['stage']}")
        if previous is not None and max_delta(previous, q) > shared.MAX_JOINT_STEP_DEG:
            raise ValueError("离线帧存在关节跳变")
        previous = q
        name = frame["stage"]
        if name in groups:
            index = order.index(name)
            if index < phase or index > phase + 1:
                raise ValueError("离线帧顺序不完整")
            phase = index
            groups[name].append(q)
        else:
            initial_names = {"CAPTURED_LIVE_START_NOT_ASSERTED_SAFE"}
            if plan["schema_version"] == SCHEMA_V2:
                initial_names.add("RECORDED_FINAL_HOME_NOT_CURRENT_LIVE_START")
            if name not in initial_names or phase != -1:
                raise ValueError("未知离线运动帧")
    if any(not v for v in groups.values()):
        raise ValueError("缺少某段密集回放")
    for stage in plan["stages"]:
        kind = stage["kind"]
        if kind in ("MOVE_WORLDS", "ASSERT_ENDPOINT"):
            pose6(stage["pose"])
        elif kind == "MOVE_JOINTS":
            pose6(stage["expected_endpoint"])
            if max_delta(vec(stage["q_sdk_deg"], 7), groups[stage["name"]][-1]) > 1e-6:
                raise ValueError("MoveJ命令与回放终点不一致")
    if max_delta(vec(plan["expected_pick_ascent_joints_sdk_deg"], 7), groups["PICK_ASCEND"][-1]) > 1e-6:
        raise ValueError("抬升交接关节与回放不一致")
    return groups


def snapshot(helper, sdk):
    helper.check_soft_stop(sdk, "混合只读预检")
    if helper.read_move_state(sdk, shared.ARM_ID):
        raise RuntimeError("机械臂正在移动，请停止其他程序/示教移动后重试")
    return {"worlds": vec(helper.read_worlds(sdk, shared.ARM_ID), 6),
            "joints": vec(helper.read_joints(sdk, shared.ARM_ID), 7)}


def stable(helper, sdk, initial):
    current = snapshot(helper, sdk)
    if max_delta(current["joints"], initial["joints"]) > shared.ARRIVE_TOL_DEG:
        raise RuntimeError("只读查询期间机械臂关节已改变，当前报告失效")
    if max_delta(current["worlds"][:3], initial["worlds"][:3]) > shared.ARRIVE_TOL_MM:
        raise RuntimeError("只读查询期间机械臂位置已改变，当前报告失效")
    return current


def capture(helper, config, plan, report):
    groups = validate(plan)
    sdk = None
    code = 2
    report.update(status="READONLY_DIAGNOSTICS_FAILED", real_motion_authorized=False,
                  session_stopped=False, segments=[], scope="NOT_A_MIXED_PATH_MOTION_PASS")
    try:
        sdk = helper.open_read_only_session(config["robot_ip"], config["local_ip"], config["arm_ip"], config["arm_port"], (shared.ARM_ID,))
        ro = ReadOnlySDK(sdk)
        initial = snapshot(helper, ro)
        report["live_start"] = initial
        limits = [vec(pair, 2) for pair in helper.read_axis_limits(ro, shared.ARM_ID)]
        if len(limits) != 7 or not helper.axis_limits_look_valid(limits)[0]:
            raise ValueError("控制器限位读数无效")
        limits = helper.intersect_axis_limits(limits, helper.LIMITS_DEG[shared.ARM_ID])
        report["effective_limits_deg"] = limits
        safe = pose6(plan["stages"][0]["pose"])
        report["start_comparison"] = {"safe_xyz_delta_mm": [a-b for a,b in zip(initial["worlds"][:3], safe[:3])],
                                      "safe_uvw_delta_deg": [((a-b+180)%360)-180 for a,b in zip(initial["worlds"][3:], safe[3:])],
                                      "action": "REPORT_ONLY_NO_AUTOMATIC_HOME_OR_RECOVERY"}
        report["live_start_limit_errors"] = limit_errors(initial["joints"], limits)
        previous_pose, previous_ik = initial["worlds"], initial["joints"]
        failures = len(report["live_start_limit_errors"])
        for stage in plan["stages"]:
            name, kind = stage["name"], stage["kind"]
            if not kind.startswith("MOVE_"):
                continue
            stable(helper, ro, initial)
            segment = {"stage": name, "kind": kind, "queries": [], "issues": []}
            report["segments"].append(segment)
            planned = groups[name][-1]
            if kind == "MOVE_JOINTS":
                violations = [(i, limit_errors(q, limits)) for i,q in enumerate(groups[name])]
                segment["joint_limit_errors"] = [{"sample": i, "errors": bad} for i,bad in violations if bad]
                segment["dense_joint_frames_checked"] = len(groups[name])
                segment["specified_joints_sdk_deg"] = planned
                segment["move_j_verdict"] = "LIMITS_ONLY_NOT_FK_OR_CONTROLLER_PATH_VERIFIED"
                if segment["joint_limit_errors"]:
                    segment["issues"].append("DENSE_JOINT_LIMIT_VIOLATION")
                target = pose6(stage["expected_endpoint"])
                poses = [target]
                previous_ik = None  # 实际机器人没有到这里，不能把指定q伪装成SDK求解seed。
            else:
                target = pose6(stage["pose"])
                poses = shared.densify_line(previous_pose, target)
                if len(poses) > 4000:
                    raise ValueError("查询路径异常长，请核对当前位置与单位")
            for i, pose in enumerate(poses, 1):
                if i % 25 == 1:
                    stable(helper, ro, initial)
                row = {"sample": i, "worlds_xyzuvw": pose, "q_sdk_deg": None,
                       "role": "UNSEEDED_SDK_IK_NOT_ACTUAL_ARRIVAL"}
                segment["queries"].append(row)
                q = helper.try_worlds(ro, shared.ARM_ID, pose)
                if q is None:
                    row["error"] = "IK_UNREACHABLE_OR_SDK_ERROR"
                    segment["issues"].append(f"IK_QUERY_FAILED:{i}")
                    previous_ik = None
                    continue
                q = vec(q, 7)
                row["q_sdk_deg"] = q
                row["limit_errors"] = limit_errors(q, limits)
                if row["limit_errors"]:
                    segment["issues"].append(f"IK_LIMIT_VIOLATION:{i}")
                if previous_ik is not None:
                    row["adjacent_ik_delta_deg"] = max_delta(previous_ik, q)
                    if row["adjacent_ik_delta_deg"] > shared.MAX_JOINT_STEP_DEG:
                        segment["issues"].append(f"IK_BRANCH_JUMP:{i}")
                previous_ik = q
            endpoint = segment["queries"][-1]["q_sdk_deg"]
            if endpoint is not None:
                difference = [a-b for a,b in zip(endpoint, planned)]
                segment["endpoint_delta_from_planned_joints_deg"] = difference
                segment["endpoint_branch_matches_within_deg"] = shared.ARRIVE_TOL_DEG
                if max(abs(v) for v in difference) > shared.ARRIVE_TOL_DEG:
                    segment["issues"].append("ENDPOINT_IK_DIFFERS_FROM_OFFLINE_BRANCH")
            # 将尚未执行的MoveJ终点作为后段查询的几何起点，不声称它已真实到位。
            previous_pose = target
            if kind == "MOVE_JOINTS":
                previous_ik = None
            failures += len(segment["issues"])
            segment["diagnostic_status"] = "ISSUES_FOUND" if segment["issues"] else "QUERY_CHECKS_PASS_NOT_MOTION_PASS"
            print(f"{name}: {len(poses)} IK queries, {len(segment['issues'])} issues", flush=True)
        report["live_end"] = stable(helper, ro, initial)
        report["issue_count"] = failures
        report["status"] = "READONLY_DIAGNOSTICS_COMPLETE_WITH_ISSUES" if failures else "READONLY_DIAGNOSTICS_COMPLETE_NOT_MOTION_AUTHORIZED"
        code = 2 if failures else 0
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        code = 130
    except Exception as exc:
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        if sdk is not None:
            try:
                result = sdk.stop()  # 未改变速度/保护，无须通用close_session中的控制器写操作。
                report["session_stop_return"] = result
                if result is False or (result is not None and result is not True and result != 0):
                    raise RuntimeError(f"SDK stop rejected: {result!r}")
                report["session_stopped"] = True
            except Exception as exc:
                report.update(status="SESSION_STOP_FAILED", stop_error=type(exc).__name__)
                code = 2
    return code


def write_report(path, report):
    text = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    for pattern, replacement in scrub_log.PATTERNS:
        text = pattern.sub(replacement, text)
    with path.open("x", encoding="utf-8") as handle:
        handle.write(text)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", type=Path)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--dry-run", action="store_true", help="仅文件检查，不连接SDK")
    modes.add_argument("--precheck-only", action="store_true", help="连接SDK只读查询（默认）")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--robot-ip")
    parser.add_argument("--local-ip")
    parser.add_argument("--arm-ip")
    parser.add_argument("--arm-port", type=int)
    args = parser.parse_args(argv)
    os.environ["XIFENG_ALLOW_REAL_MOTION"] = "0"
    try:
        plan = json.loads(args.plan.read_text())
        groups = validate(plan)
        plan_sha = digest(args.plan)
        if args.output is None:
            args.output = Path("hybrid_precheck_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
        if args.output.exists() or args.output.suffix.lower() != ".json":
            raise ValueError("报告输出必须为尚不存在的.json，不覆盖已有轨迹/报告")
        report = {"schema_version": "tabletop_hybrid_readonly_report.v1", "captured_at": datetime.now(timezone.utc).isoformat(),
                  "plan_sha256": plan_sha, "plan_file": args.plan.name, "real_motion_authorized": False,
                  "source_hashes": {Path(m.__file__).name: digest(m.__file__) for m in (shared, ss, robot_lock, scrub_log)},
                  "precheck_script_sha256": digest(__file__), "candidate_blockers": plan["blockers"],
                  "notice": "MoveJ仅限位诊断；SDK逆解没有seed，不能证明整个混合路径或碰撞通过"}
        print("READ ONLY — 不清故障、不使能、不设速度、不运动、不打开夹爪串口")
        print("plan SHA-256:", plan_sha)
        for name, sha in report["source_hashes"].items():
            print(name, "SHA-256:", sha)
        print("precheck SHA-256:", report["precheck_script_sha256"])
        if args.dry_run:
            report.update(status="FILE_CHECKS_PASS_NOT_MOTION_AUTHORIZED", sdk_connected=False,
                          motion_stages=len(groups), offline_joint_frames=sum(len(v) for v in groups.values()))
            code = 0
        else:
            args.speed, args.limit_margin_deg, args.recovery_only = 1., ss.LIMIT_MARGIN_DEG, False
            config = shared._runtime_config(args, os.environ)
            for key in ("robot_ip", "arm_ip", "local_ip"):
                ipaddress.ip_address(config[key])
            report["runtime"] = {k: config[k] for k in ("robot_ip", "arm_ip", "local_ip", "arm_port", "limit_margin_deg")}
            with robot_lock.acquire(config["robot_ip"], shared.ARM_ID, "precheck_tabletop_hybrid.py"):
                code = capture(ss, config, plan, report)
        write_report(args.output, report)
        print(report["status"], "— 报告:", args.output.resolve())
        print("本工具无运动入口；无论结果如何，都不代表真机运动获准。")
        return code
    except (Exception, SystemExit) as exc:
        if isinstance(exc, SystemExit):
            print("只读会话未启动:", str(exc))
            return 2
        print(f"只读测试失败: {type(exc).__name__}: {exc}")
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
