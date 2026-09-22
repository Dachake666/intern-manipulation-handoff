#!/usr/bin/env python3
"""Debian 双臂候选只读限位诊断，无运动/使能/清警/夹爪入口。

--precheck-only 读真实控制器的两臂轴极限、当前 Joints/Worlds，对导出的全部
关节帧做比较。不把 URDF_WORLD 坐标送入 armTryWorlds，不证明 MoveJ/Worlds
的轨迹、双臂同步、碰撞或实物抓取。--dry-run 仅离线；默认也不连接 SDK。
"""
from __future__ import annotations

import argparse
import contextlib
import ctypes
from datetime import datetime, timezone
import hashlib
import ipaddress
import json
import math
import os
from pathlib import Path
import sys
import tempfile

HERE = Path(__file__).resolve().parent
CONFIG_ROOT = next((root for root in (HERE, HERE.parent.parent)
                    if (root / "arm_profiles.v1.json").is_file()), HERE)
sys.path.insert(0, str(CONFIG_ROOT))
import arm_profiles as ap
import robot_lock
import scrub_log

PROFILE_FILE = CONFIG_ROOT / "arm_profiles.v1.json"
PROFILES = {arm: ap.arm_profile(arm, PROFILE_FILE) for arm in ("left", "right")}
SHARED = ap.load_profiles(PROFILE_FILE)["shared"]
READ_METHODS = frozenset({"armGetWorlds", "armGetJoints", "armGetAxisParameter",
                          "getSoftStopSwitch", "armGetRobotMoveState", "armGetLinkStatus"})
RUNTIME_FILES = ("precheck_dual_arm.py", "sdk_session.py", "robot_lock.py", "scrub_log.py")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def scrub(text):
    for pattern, replacement in scrub_log.PATTERNS:
        text = pattern.sub(replacement, text)
    return text


def vector(value, size):
    if (not isinstance(value, (list, tuple)) or len(value) != size or
            any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in value)):
        raise ValueError(f"需要 {size} 个有限数值")
    return [float(v) for v in value]


def nominal_limits(arm):
    p = PROFILES[arm]
    return p.get("controller_limits_deg") or p["candidate_nominal_urdf_limits_deg"]


def validate(plan):
    if (plan.get("schema_version") != "dual_arm_simulation.v1" or
            plan.get("coordinate_frame") != "URDF_WORLD_SIM_ONLY" or
            plan.get("real_motion_authorized") is not False or
            plan.get("debian_execution_allowed") is not False):
        raise ValueError("只接受未获运动授权的双臂仿真原格式，不允许修改授权开关")
    if plan.get("units") != {"joint": "degree_sdk_sign", "position": "meter", "time": "second"}:
        raise ValueError("单位/SDK关节符号契约错误")
    playback = plan["playback"]
    step, period = vector([playback["joint_step_deg"], playback["period_s"]], 2)
    if not 0 < step <= SHARED["pulse_step_deg"]:
        raise ValueError("密集步长无效")
    if period != SHARED["pulse_period_ms"] / 1000:
        raise ValueError("回放周期不一致")
    frames = plan["frames"]
    if not isinstance(frames, list) or not 2 <= len(frames) <= 100000:
        raise ValueError("密集轨迹长度无效")
    for arm, pr in PROFILES.items():
        for key in ("arm_id", "gripper_id", "joint_ids", "ee_link_id"):
            value = plan["meta"][arm][key]
            if (value != pr[key] or type(value) is not type(pr[key]) or
                    (isinstance(value, list) and any(type(v) is not int for v in value))):
                raise ValueError(f"{arm} {key} 映射不一致")
    prior, held, events = None, False, []
    for index, row in enumerate(frames):
        q = {arm: vector(row["q_sdk_deg"][arm], 7) for arm in PROFILES}
        if not isinstance(row.get("stage"), str) or not row["stage"]:
            raise ValueError("缺少阶段名称")
        for arm in PROFILES:
            if prior and max(abs(a-b) for a, b in zip(q[arm], prior[arm])) > step + 1e-9:
                raise ValueError(f"{arm} 第{index}帧存在跳变")
        if "event" in row:
            if prior is None or row.get("event_arms") != ["left", "right"]:
                raise ValueError("夹爪事件臂/位置不正确")
            if any(max(abs(a-b) for a, b in zip(q[arm], prior[arm])) > 1e-9 for arm in PROFILES):
                raise ValueError("夹爪事件前未停稳")
            held = row["event"] == "close"
            events.append((row["stage"], row["event"]))
        if row.get("carrying") is not held:
            raise ValueError("持物状态与事件不一致")
        prior = q
    if events != [("OPEN_BEFORE_PICK", "open"), ("CLOSE_AT_PICK", "close"), ("OPEN_AT_PLACE", "open")]:
        raise ValueError("夹爪事件顺序不完整")
    return frames


def limit_metrics(frames, arm, limits, margin):
    bounds = [vector(pair, 2) for pair in limits]
    if len(bounds) != 7 or any(hi <= lo for lo, hi in bounds):
        raise ValueError("关节限位表无效或无交集")
    lows, highs, margins = [math.inf]*7, [-math.inf]*7, [math.inf]*7
    hard, reserved, first = [], [], None
    for index, row in enumerate(frames):
        q = vector(row["q_sdk_deg"][arm], 7)
        for j, (angle, (lo, hi)) in enumerate(zip(q, bounds)):
            lows[j], highs[j] = min(lows[j], angle), max(highs[j], angle)
            room = min(angle-lo, hi-angle)
            margins[j] = min(margins[j], room)
            item = {"frame": index, "stage": row["stage"], "joint": j+1,
                    "angle_deg": angle, "limits_deg": [lo, hi], "margin_deg": room}
            if room < -1e-9:
                hard.append(item)
            if room < margin - 1e-8:
                reserved.append(item)
            if first is None or room < first["margin_deg"]:
                first = item
    return {"limits_deg": bounds, "required_margin_deg": margin,
            "joint_ranges_deg": list(map(list, zip(lows, highs))),
            "minimum_margin_per_joint_deg": margins, "tightest": first,
            "hard_violation_count": len(hard), "margin_violation_count": len(reserved),
            "hard_violations_first_100": hard[:100], "margin_violations_first_100": reserved[:100],
            "frames_checked": len(frames),
            "status": "PASS_LIMIT_VALUES_ONLY" if not reserved else "FAIL_LIMIT_VALUES"}


class ReadOnlySDK:
    def __init__(self, sdk):
        self._sdk = sdk

    def __getattr__(self, name):
        if name not in READ_METHODS:
            raise RuntimeError(f"只读入口拒绝 SDK 操作：{name}")
        return getattr(self._sdk, name)


def snapshot(helper, ro):
    helper.check_soft_stop(ro, "双臂只读限位采集")
    result = {}
    for arm, pr in PROFILES.items():
        aid = pr["arm_id"]
        if helper.read_move_state(ro, aid):
            raise RuntimeError(f"{arm} 正在运动；先停止其它脚本或示教动作")
        result[arm] = {"joints_sdk_deg": vector(helper.read_joints(ro, aid), 7),
                       "worlds_xyzuvw_mm_deg": vector(helper.read_worlds(ro, aid), 6)}
    return result


def capture(helper, config, plan, report):
    """只调用现行只读会话及读数助手，不调用通用运动 close_session。"""
    frames = validate(plan)
    sdk = None
    report.update(status="READONLY_FAILED", session_stopped=False, arms={}, real_motion_authorized=False)
    code = 2
    try:
        sdk = helper.open_read_only_session(config["robot_ip"], config["local_ip"],
                                             config["arm_ip"], config["arm_port"],
                                             tuple(p["arm_id"] for p in PROFILES.values()))
        ro = ReadOnlySDK(sdk)
        report["sdk_connected"] = True
        report["live_start"] = snapshot(helper, ro)
        issues = 0
        for arm, pr in PROFILES.items():
            aid = pr["arm_id"]
            raw = [vector(pair, 2) for pair in helper.read_axis_limits(ro, aid)]
            if len(raw) != 7 or not helper.axis_limits_look_valid(raw)[0]:
                raise ValueError(f"{arm} 控制器限位读数无效（不能将全0或未就绪当真限位）")
            # 不因控制器放宽而取消左J7的76度物理上限；右保留nominal候选包络，身份仍未实测标定。
            effective = helper.intersect_axis_limits(raw, nominal_limits(arm))
            metrics = limit_metrics(frames, arm, effective, SHARED["minimum_joint_margin_deg"])
            current = report["live_start"][arm]["joints_sdk_deg"]
            first = frames[0]["q_sdk_deg"][arm]
            start_delta = [a-b for a, b in zip(current, first)]
            start_gap = max(abs(v) for v in start_delta)
            row = {"arm_id": aid, "raw_controller_limits_deg": raw,
                   "configured_envelope_deg": nominal_limits(arm),
                   "configured_envelope_source": pr["controller_limits_status"],
                   "dense_joint_limit_check": metrics,
                   "current_to_first_delta_deg": start_delta,
                   "current_to_first_max_abs_deg": start_gap,
                   "start_alignment_reference_deg": SHARED["pulse_step_deg"],
                   "start_aligned": start_gap <= SHARED["pulse_step_deg"],
                   "start_action": "REPORT_ONLY_NO_MOTION_NO_HOME_RECOVERY"}
            current_row = {"stage": "LIVE_START", "q_sdk_deg": {arm: current}}
            row["live_start_limit_check"] = limit_metrics([current_row], arm, effective, SHARED["minimum_joint_margin_deg"])
            report["arms"][arm] = row
            issues += int(metrics["margin_violation_count"] > 0) + int(not row["start_aligned"])
            issues += int(row["live_start_limit_check"]["margin_violation_count"] > 0)
            # 控制器参数重复回读，避免未就绪读数/查询过程配置漂移悄悄通过。
            again = [vector(pair, 2) for pair in helper.read_axis_limits(ro, aid)]
            if len(again) != 7 or any(abs(a-b) > 1e-6 for x, y in zip(raw, again) for a, b in zip(x, y)):
                raise RuntimeError(f"{arm} 两次控制器限位读数不一致")
            print(f"{arm}: {len(frames)}帧，限位={metrics['status']}，起点差={start_gap:.3f}deg", flush=True)
        report["live_end"] = snapshot(helper, ro)
        for arm in PROFILES:
            delta = max(abs(a-b) for a, b in zip(report["live_start"][arm]["joints_sdk_deg"],
                                                 report["live_end"][arm]["joints_sdk_deg"]))
            report["arms"][arm]["snapshot_drift_deg"] = delta
            if delta > SHARED["pulse_step_deg"]:
                raise RuntimeError(f"{arm} 查询期间实际关节发生变化，当前报告失效")
        report["issue_count"] = issues
        report["status"] = "READONLY_COMPLETE_WITH_ISSUES" if issues else "READONLY_LIMITS_AND_START_PASS_NOT_MOTION_AUTHORIZED"
        code = 2 if issues else 0
    except KeyboardInterrupt:
        report["status"] = "INTERRUPTED"
        code = 130
    except Exception as exc:
        report["error"] = scrub(f"{type(exc).__name__}: {exc}")
    finally:
        if sdk is not None:
            try:
                result = sdk.stop()
                if result is False or (result is not None and result is not True and result != 0):
                    raise RuntimeError("SDK stop 返回失败")
                report["session_stopped"] = True
            except Exception as exc:
                report.update(status="SESSION_STOP_FAILED", stop_error=type(exc).__name__)
                code = 2
    return code


def flush_all():
    sys.stdout.flush()
    sys.stderr.flush()
    try:
        ctypes.CDLL(None).fflush(None)
    except (AttributeError, OSError):
        pass


@contextlib.contextmanager
def captured_sdk_log(path):
    """SDK native C++ 输出也进入匿名临时文件；只保存脱敏后的持久日志。"""
    if path.exists():
        raise ValueError("SDK日志已存在，不覆盖")
    with tempfile.TemporaryFile(dir=path.parent) as stream:
        flush_all()
        saved = (os.dup(1), os.dup(2))
        try:
            os.dup2(stream.fileno(), 1)
            os.dup2(stream.fileno(), 2)
            yield
        finally:
            flush_all()
            os.dup2(saved[0], 1)
            os.dup2(saved[1], 2)
            os.close(saved[0])
            os.close(saved[1])
            stream.seek(0)
            text = scrub(stream.read().decode("utf-8", errors="replace"))
            with path.open("x", encoding="utf-8") as handle:
                handle.write(text)


def check_bundle_manifest():
    path = HERE / "SHA256SUMS"
    if not path.exists():
        return "SOURCE_TREE_NO_RELEASE_MANIFEST"
    required = set(RUNTIME_FILES) | {"arm_profiles.py", "arm_profiles.v1.json", "dual_arm_simulation.json", "validation_report.json"}
    seen = set()
    for line in path.read_text().splitlines():
        value, name = line.split(maxsplit=1)
        name = name.lstrip("*")
        if Path(name).name != name or name in seen or digest(HERE / name) != value:
            raise ValueError(f"发布清单错误或文件已改变：{name}")
        seen.add(name)
    if not required.issubset(seen):
        raise ValueError("发布清单缺少必需运行文件")
    return "MATCH_LOCAL_MANIFEST_NOT_A_SIGNATURE"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("plan", nargs="?", type=Path, default=HERE / "dual_arm_simulation.json")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--dry-run", action="store_true", help="仅文件检查；默认行为")
    group.add_argument("--precheck-only", action="store_true", help="连接两臂，只读取实际状态与限位")
    for name in ("robot-ip", "local-ip", "arm-ip"):
        parser.add_argument("--" + name)
    parser.add_argument("--arm-port", type=int)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    os.environ["XIFENG_ALLOW_REAL_MOTION"] = "0"
    output = args.output or Path("dual_arm_precheck_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
    sdk_log = output.with_name(output.stem + "_sdk.log")
    if (output.exists() or sdk_log.exists() or output.suffix.lower() != ".json" or
            not output.parent.is_dir()):
        print("输出目录必须已存在，JSON/日志必须为新文件；不会覆盖旧轨迹或报告。")
        return 2
    report = {"schema_version": "dual_arm_controller_limit_report.v1",
              "captured_at": datetime.now(timezone.utc).isoformat(), "status": "INPUT_FAILED",
              "real_motion_authorized": False, "sdk_connected": False,
              "notice": "Only controller limit readback / joint-value comparison. NOT a motion, collision, FK, IK, timing or SDK dual-concurrency PASS.",
              "identity_status": "IP_UNRESOLVED_NO_STABLE_ID_VERIFICATION",
              "scene_and_calibration": "UNREGISTERED_URDF_WORLD_NOT_CONVERTED_TO_SDK",
              "no_motion_reason": "Right physical/tool validation, complete collisions, current scene, GUI approval and SDK concurrency remain unresolved."}
    code = 2
    try:
        report["manifest_status"] = check_bundle_manifest()
        plan = json.loads(args.plan.read_text())
        validate(plan)
        report["plan_sha256"] = digest(args.plan)
        report["plan_file"] = args.plan.name
        validation_path = args.plan.parent / "validation_report.json"
        validation = json.loads(validation_path.read_text())
        if (validation.get("schema_version") != "dual_arm_simulation_validation.v1" or
                validation.get("plan_sha256") != report["plan_sha256"] or
                validation.get("dense_frames_checked") != len(plan["frames"])):
            raise ValueError("离线报告与轨迹SHA/帧数/格式不符")
        report["offline_qualification"] = validation.get("task_checks", {"status": validation.get("status", "UNKNOWN")})
        report["gui_review"] = plan["gui_review"]
        hashes = {name: digest(HERE / name) for name in RUNTIME_FILES}
        hashes.update({"arm_profiles.py": digest(ap.__file__), "arm_profiles.v1.json": digest(PROFILE_FILE),
                       "validation_report.json": digest(validation_path)})
        report["source_hashes"] = hashes
        if plan["provenance"]["profiles_sha256"] != hashes["arm_profiles.v1.json"]:
            raise ValueError("轨迹绑定的臂配置已经改变")
        report["offline_configured_limit_checks"] = {
            arm: limit_metrics(plan["frames"], arm, nominal_limits(arm), SHARED["minimum_joint_margin_deg"])
            for arm in PROFILES}
        print("READ ONLY：不使能、不清故障、不改速度/保护、不运动、不操作夹爪。")
        print("plan SHA-256:", report["plan_sha256"])
        if not args.precheck_only:
            report["status"] = "FILE_CHECKS_PASS_NOT_MOTION_AUTHORIZED"
            code = 0
            if any(v["margin_violation_count"] for v in report["offline_configured_limit_checks"].values()):
                report["status"], code = "FILE_LIMIT_ISSUES", 2
        else:
            config = {}
            for key in ("robot_ip", "local_ip", "arm_ip"):
                value = getattr(args, key) or os.environ.get("XIFENG_" + key.upper())
                if key == "arm_ip" and not value:
                    value = config["robot_ip"]
                if not value:
                    raise ValueError(f"需要现场的 --{key.replace('_', '-')} 或 XIFENG_{key.upper()}")
                config[key] = str(ipaddress.ip_address(value))
            config["arm_port"] = (args.arm_port if args.arm_port is not None else
                                  int(os.environ.get("XIFENG_ARM_PORT", "8080")))
            if not 1 <= config["arm_port"] <= 65535:
                raise ValueError("arm-port 无效")
            report["runtime"] = config
            print("正在读取两臂；SDK输出将自动脱敏保存到", sdk_log.name, flush=True)
            with robot_lock.acquire_robot(config["robot_ip"], "precheck_dual_arm.py"):
                with captured_sdk_log(sdk_log):
                    import sdk_session as helper  # 仅显式只读模式加载Debian SDK；离线不导入pypilot。
                    code = capture(helper, config, plan, report)
    except KeyboardInterrupt:
        report["status"], code = "INTERRUPTED", 130
    except (Exception, SystemExit) as exc:
        report["error"] = scrub(f"{type(exc).__name__}: {exc}")
        code = 2
    if sdk_log.is_file():
        # SDK 导入/启动出错也保留已脱敏日志的身份，避免只有成功报告能关联日志。
        report["sdk_log_file"] = sdk_log.name
        report["sdk_log_sha256"] = digest(sdk_log)
    text = scrub(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n")
    with output.open("x", encoding="utf-8") as handle:
        handle.write(text)
    print(report["status"], "报告：", output.resolve())
    if "error" in report:
        print(report["error"])
    print("任何退出码都不是运动授权。请回传此JSON及同名_sdk.log；不要覆盖原成功版本。")
    return code


if __name__ == "__main__":
    raise SystemExit(main())
