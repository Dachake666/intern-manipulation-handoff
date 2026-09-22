#!/usr/bin/env python3
"""[20260909] A 成功路线的 Pulse Servo 消费者；默认离线，不含自动 MoveJ。

复用 A 的只读/使能/收尾检查和 Track C 的关节透传辅助；不会执行父文件的
MoveWorlds/MoveJ。新路线必须有匹配 GUI + 独立完整资格报告才能人工 --run。
"""
from __future__ import annotations

import numpy as np
from controller import JointMPC, Settings

import argparse
import ipaddress
import json
import math
import os
from pathlib import Path
import time

import execute_tabletop_hybrid_trial_reviewfix_field as field
import robot_lock
import sdk_session as ss
import servo_common as sc
import tabletop_servo_contract as contract


def runtime_hashes():
    modules = (contract, sc, ss, field, field.hp, field.worlds, robot_lock,
               field.scrub_log, contract.arm_profiles)
    hashes = {Path(m.__file__).name: contract.digest(m.__file__) for m in modules}
    hashes["arm_profiles.v1.json"] = contract.digest(contract.arm_profiles._config_path())
    return hashes


def gap(a, b):
    return max(abs(x-y) for x,y in zip(contract.vector(a), contract.vector(b)))


def require_reviews(args, plan):
    if not args.gui_review or not args.qualification_report:
        raise ValueError("新 Servo 需 --gui-review 和 --qualification-report；A 的真机 PASS 不能代替")
    sha = contract.digest(args.trajectory)
    gui = json.loads(Path(args.gui_review).read_text())
    qualification = json.loads(Path(args.qualification_report).read_text())
    count = plan["kinematics"]["motion_frame_count"]
    if (gui.get("schema_version") != "pybullet_gui_review.v1" or gui.get("result") != "PASS"
            or gui.get("trajectory_sha256") != sha or gui.get("full_replay_completed") is not True
            or gui.get("frame_count") != count or gui.get("arm_id") != plan["meta"]["arm_id"]
            or gui.get("dense_step_deg") != plan["stream_policy"]["max_step_deg"]):
        raise ValueError("GUI 审核不是本次完整 Servo 轨迹")
    if (qualification.get("schema_version") != "tabletop_servo_qualification.v1"
            or qualification.get("trajectory_sha256") != sha
            or qualification.get("executor_sha256") != contract.digest(__file__)
            or qualification.get("servo_common_sha256") != contract.digest(sc.__file__)
            or qualification.get("contract_sha256") != contract.digest(contract.__file__)
            or qualification.get("runtime_hashes") != runtime_hashes()
            or qualification.get("verdict") != "PASS"
            or qualification.get("frame_count") != count
            or qualification.get("stream_policy") != plan["stream_policy"]
            or not qualification.get("scene_id")):
        raise ValueError("Servo 完整资格报告未通过或与实际文件/场景/时序不匹配")
    for key in ("fk_path", "self_collision", "robot_environment_collision",
                "held_and_released_object_collision", "entry_region", "servo_dynamics"):
        item = qualification.get("checks", {}).get(key, {})
        if item.get("status") != "PASS" or not item.get("evidence_files"):
            raise ValueError(f"缺少 {key} 的实质资格证据")
        for ref in item["evidence_files"]:
            evidence_path = (args.qualification_report.parent / ref["path"]).resolve()
            if contract.digest(evidence_path) != ref["sha256"]:
                raise ValueError(f"{key} 证据文件 SHA 不匹配")
            proof = json.loads(evidence_path.read_text())
            if (proof.get("schema_version") != f"tabletop_servo_{key}_evidence.v1"
                    or proof.get("status") != "PASS" or proof.get("blockers") != []
                    or proof.get("trajectory_sha256") != sha
                    or proof.get("arm_id") != plan["meta"]["arm_id"]
                    or proof.get("scene_id") != qualification["scene_id"]
                    or proof.get("frame_count") != count
                    or proof.get("stream_policy") != plan["stream_policy"]):
                raise ValueError(f"{key} 证据未完整通过或引用了其他轨迹/时序/场景")
    return {"gui_sha256": contract.digest(args.gui_review),
            "qualification_sha256": contract.digest(args.qualification_report)}


def live_precheck(sdk, plan, report=None):
    report = {} if report is None else report
    report["status"] = "CHECKING_NOT_MOTION_AUTHORIZED"
    before = field.snapshot(sdk)
    report["start"] = before
    field.require_protection(sdk)
    bounds = field.read_effective_limits(sdk)
    report["effective_limits_deg"] = bounds
    metrics = contract.validate(plan, bounds)
    after = field.snapshot(sdk)
    field.assert_unchanged(after, before, "Servo 只读预检期间")
    initial_gap = gap(after["joints"], plan["first_point_q_sdk_deg"])
    report.update(start=after, initial_gap_deg=initial_gap,
                  first_point_q_sdk_deg=plan["first_point_q_sdk_deg"], motion_metrics=metrics)
    if initial_gap > plan["stream_policy"]["max_initial_gap_deg"]:
        raise ValueError(
            f"首点不匹配：差 {initial_gap:.3f}°，允许 {plan['stream_policy']['max_initial_gap_deg']:.3f}°；"
            f"不自动 MoveJ。目标首点为 {plan['first_point_q_sdk_deg']}")
    report["status"] = "READONLY_PASS_NOT_COLLISION_PROOF"
    return report


def wait_settled(sdk, target):
    deadline, previous, stable = time.monotonic()+30.0, None, 0
    while time.monotonic() < deadline:
        state = field.snapshot(sdk, require_stopped=False)
        moving = ss.read_move_state(sdk, field.ARM_ID)
        if (moving is False and gap(state["joints"], target) <= field.JOINT_TOL_DEG
                and previous is not None and gap(state["joints"], previous) <= field.START_DRIFT_JOINT_DEG):
            stable += 1
            if stable >= 3: return state
        else:
            stable = 0
        previous = state["joints"]
        time.sleep(0.1)
    raise RuntimeError("Servo 端点未在容差内停稳")


def set_protection(sdk, value):
    ss.set_protect_status(sdk, field.ARM_ID, value)
    if ss.read_protect_status(sdk, field.ARM_ID) is not value:
        raise RuntimeError(f"保护状态回读不是 {value}")


def stream(sdk, plan, log):
    policy = plan["stream_policy"]
    pacer = sc.StrictPacer(policy["period_s"])
    previous = plan["first_point_q_sdk_deg"]
    source = {s["name"]: s for s in plan["source_stages"]}
    events = log["events"] = []
    last_stage = None
    sent = 0

    # --------------------------------------------------------
    # MPC SHADOW:
    # 真机仍发送原 reference；MPC 仅使用真实反馈计算候选命令。
    # 关节反馈固定约 40 ms 更新，因此每两个 20 ms motion frame 读一次。
    # --------------------------------------------------------
    mpc_dt = float(policy["period_s"])
    mpc_settings = Settings(
        dt=mpc_dt,
        horizon=15,
        response_alpha=0.4,

        # 只用于 Servo 开始前的真实状态 warm-up。
        # warm-up 后立即恢复为 10 ms。
        solver_time_limit_s=0.050,
    )

    mpc_margin = float(policy["margin_deg"])
    mpc_raw_limits = np.asarray(
        ss.LIMITS_DEG[field.ARM_ID],
        dtype=float,
    )
    mpc_limits = np.column_stack((
        mpc_raw_limits[:, 0] + mpc_margin,
        mpc_raw_limits[:, 1] - mpc_margin,
    ))

    # MPC 必须使用“实际 runtime stream”的速度包络。
    # field runtime 会对原 trajectory 做 stride/downsample，
    # 因此不能继续沿用原始 5 deg/s source policy。
    #
    # 这里只允许达到已经通过现有 Servo 安全检查并实际准备发送的
    # runtime reference 最大速度，不额外放宽。
    _runtime_peak_velocity = 0.0
    _prev_runtime_q = None

    for _item in plan["waypoints"]:
        if "gripper" in _item:
            _prev_runtime_q = None
            continue

        if "q_sdk_deg" not in _item:
            continue

        _q = np.asarray(
            _item["q_sdk_deg"],
            dtype=float,
        )

        if _prev_runtime_q is not None:
            _runtime_peak_velocity = max(
                _runtime_peak_velocity,
                float(
                    np.max(
                        np.abs(
                            _q - _prev_runtime_q
                        )
                    )
                    / mpc_dt
                ),
            )

        _prev_runtime_q = _q

    if _runtime_peak_velocity <= 0:
        raise RuntimeError(
            "无法从 runtime plan 得到有效速度包络"
        )

    mpc_vmax = _runtime_peak_velocity + 1e-6

    print(
        f"MPC runtime vmax="
        f"{mpc_vmax:.3f} deg/s",
        flush=True,
    )

    mpc_controller = JointMPC(
        mpc_limits,
        mpc_vmax,
        float(policy["max_acceleration_deg_s2"]),
        mpc_settings,
    )

    # 参考窗口不能跨过夹爪事件。
    mpc_windows = []
    _block = []

    def _flush_mpc_block():
        nonlocal _block
        if not _block:
            return
        arr = np.asarray(_block, dtype=float)
        for i in range(len(arr)):
            idx = np.minimum(
                np.arange(i, i + mpc_settings.horizon),
                len(arr) - 1,
            )
            mpc_windows.append(arr[idx].copy())
        _block = []

    for _item in plan["waypoints"]:
        if "q_sdk_deg" in _item:
            _block.append(
                np.asarray(_item["q_sdk_deg"], dtype=float)
            )
        elif "gripper" in _item:
            _flush_mpc_block()

    _flush_mpc_block()

    motion_count = sum(
        1 for _item in plan["waypoints"]
        if "q_sdk_deg" in _item
    )

    if len(mpc_windows) != motion_count:
        raise RuntimeError(
            f"MPC window count mismatch: "
            f"{len(mpc_windows)} != {motion_count}"
        )

    # 真机 MPC 初始状态直接读取控制器：
    # fb=True  = 实际关节位置
    # fb=False = 控制器当前目标位置
    mpc_measured = np.asarray(
        ss.read_joints(
            sdk,
            field.ARM_ID,
            fb=True,
        ),
        dtype=float,
    )

    mpc_previous_command = np.asarray(
        ss.read_joints(
            sdk,
            field.ARM_ID,
            fb=False,
        ),
        dtype=float,
    )

    mpc_previous_velocity = np.zeros(7, dtype=float)

    # OSQP 真机运行前预热：
    # 只求解，不向机械臂发送任何 MPC 命令。
    # 若当前真实状态与第一段 reference 无法在 10 ms 内可靠求解，
    # 则在 Servo 正式开始前直接失败。
    warmup_results = []

    for _ in range(3):
        _warm = mpc_controller.step(
            mpc_measured,
            mpc_previous_command,
            mpc_previous_velocity,
            mpc_windows[0],
        )
        warmup_results.append({
            "solve_wall_ms": float(_warm["solve_wall_ms"]),
            "iterations": int(_warm["solver_iterations"]),
            "status": _warm["status"],
        })

    print(
        "MPC WARMUP:",
        warmup_results,
        flush=True,
    )

    # 正式 20 ms Servo/shadow 阶段仍坚持 10 ms fail-fast。
    # 超时则 shadow 自动退出，绝不使用旧解。
    mpc_controller.solver.update_settings(
        time_limit=0.010,
    )

    mpc_shadow_enabled = True

    mpc_shadow = {
        "mode": "REAL_ROBOT_SHADOW_REFERENCE_STILL_SENT",
        "response_alpha": mpc_settings.response_alpha,
        "horizon": mpc_settings.horizon,
        "feedback_nominal_period_s": 0.040,
        "solve_calls": 0,
        "solve_total_ms": 0.0,
        "solve_max_ms": 0.0,
        "feedback_reads": 0,
        "feedback_total_ms": 0.0,
        "feedback_max_ms": 0.0,
        "candidate_max_deviation_deg": 0.0,
        "candidate_max_deviation_frame": None,
        "errors": [],
        "feedback_samples": [],
    }

    # 20ms Servo 热循环诊断。
    # 只写内存，热循环中绝不 print / flush / 写文件。
    latency_probe = {
        "soft_stop_calls": 0,
        "soft_stop_total_ms": 0.0,
        "soft_stop_max_ms": 0.0,
        "soft_stop_over_5ms": 0,
        "soft_stop_over_20ms": 0,
        "soft_stop_over_40ms": 0,

        "pulse_calls": 0,
        "pulse_total_ms": 0.0,
        "pulse_max_ms": 0.0,
        "pulse_over_5ms": 0,
        "pulse_over_20ms": 0,
        "pulse_over_40ms": 0,

        # 只记录明显慢调用，便于定位发生在哪一帧/哪一段。
        "slow_events": [],
    }

    # --------------------------------------------------------
    # Servo 热循环外一次性准备 Boost.Python FloatVector。
    # 原来每帧 ss.pulse_to_servo() 都会：
    # Python list -> FloatVector -> armPluseToServo
    #
    # 现在把 FloatVector 构造全部提前，只让热循环做 SDK 发送。
    # --------------------------------------------------------
    prepare_t0 = time.perf_counter()

    prepared_vectors = [
        (
            ss.make_float_vector(w["q_sdk_deg"])
            if "q_sdk_deg" in w
            else None
        )
        for w in plan["waypoints"]
    ]

    prepare_ms = (time.perf_counter() - prepare_t0) * 1000.0
    prepare_count = sum(v is not None for v in prepared_vectors)

    log["servo_vector_prepare"] = {
        "motion_vectors": prepare_count,
        "total_ms": prepare_ms,
        "mean_ms_per_vector": (
            prepare_ms / prepare_count
            if prepare_count else 0.0
        ),
        "mode": "PREBUILT_FLOATVECTOR_OUTSIDE_HOT_LOOP",
    }

    try:
        set_protection(sdk, False)
        log["protection_disabled_for_servo"] = True
        for w, prepared_q in zip(
            plan["waypoints"],
            prepared_vectors,
        ):
            if "gripper" in w:
                actual = wait_settled(sdk, previous)
                pose_stage = {"CLOSE_AT_PICK": "PICK_DESCEND", "OPEN_AT_PLACE": "PLACE_DESCEND"}.get(w["stage"])
                if pose_stage:
                    pose = source[pose_stage]["pose"]
                    error = field.world_error(actual["worlds"], pose["position_mm"]+pose["sdk_world_uvw_deg"])
                    if error[0] > field.WORLD_TOL_MM or error[1] > field.WORLD_TOL_DEG:
                        raise RuntimeError(f"{w['stage']} 实际 EE 偏差 {error}，不操作夹爪")
                set_protection(sdk, True)
                # 复用现行 A 参数编码；只会在完成只读预检和人工确认后开 COM。
                field.command_gripper(sdk, plan, w["gripper"])
                events.append({"stage": w["stage"], "action": w["gripper"],
                               "actual": actual, "time": field.utc_now()})
                set_protection(sdk, False)

                # 夹爪事件形成 MPC block 边界。
                mpc_previous_command = np.asarray(
                    previous,
                    dtype=float,
                )
                mpc_previous_velocity = np.zeros(
                    7,
                    dtype=float,
                )
                mpc_measured = np.asarray(
                    actual["joints"],
                    dtype=float,
                )

                pacer.reset()
                continue
            stage_changed = w["stage"] != last_stage

            if stage_changed:
                print("SERVO", w["stage"], flush=True)
                events.append({
                    "stage": w["stage"],
                    "first_frame": pacer.frames,
                    "time": field.utc_now(),
                })
                last_stage = w["stage"]

            # 避免每个20ms Servo 帧都塞一个阻塞 SDK read。
            # stage 首帧必查；随后每10帧（约200ms）再查一次。
            if stage_changed or sent % 10 == 0:
                t0 = time.perf_counter()
                ss.check_soft_stop(sdk, "tabletop Servo")
                dt_ms = (time.perf_counter() - t0) * 1000.0

                latency_probe["soft_stop_calls"] += 1
                latency_probe["soft_stop_total_ms"] += dt_ms
                latency_probe["soft_stop_max_ms"] = max(
                    latency_probe["soft_stop_max_ms"], dt_ms
                )
                latency_probe["soft_stop_over_5ms"] += int(dt_ms > 5.0)
                latency_probe["soft_stop_over_20ms"] += int(dt_ms > 20.0)
                latency_probe["soft_stop_over_40ms"] += int(dt_ms > 40.0)

                if dt_ms > 5.0:
                    latency_probe["slow_events"].append({
                        "type": "soft_stop",
                        "frame": sent,
                        "stage": w["stage"],
                        "duration_ms": dt_ms,
                    })

            # ------------------------------------------------
            # MPC shadow 真机反馈：
            # 每两个 20 ms motion frame 读取一次真实关节。
            # 读取发生在 pacer.before_send() 前，避免在计划发送时刻之后
            # 再插入阻塞 SDK read。
            # ------------------------------------------------
            if mpc_shadow_enabled:
                try:
                    ref_q = np.asarray(
                        w["q_sdk_deg"],
                        dtype=float,
                    )

                    if sent % 2 == 0:
                        # SDK 反馈约 40 ms 刷新一次：
                        # 有新反馈时用真实关节状态校正 estimator。
                        _read_t0 = time.perf_counter()

                        mpc_measured = np.asarray(
                            ss.read_joints(
                                sdk,
                                field.ARM_ID,
                                fb=True,
                            ),
                            dtype=float,
                        )

                        _read_ms = (
                            time.perf_counter() - _read_t0
                        ) * 1000.0

                        # 只有真正调用 armGetJoints 的帧才计为真实反馈。
                        mpc_shadow["feedback_reads"] += 1
                        mpc_shadow["feedback_total_ms"] += _read_ms
                        mpc_shadow["feedback_max_ms"] = max(
                            mpc_shadow["feedback_max_ms"],
                            _read_ms,
                        )

                        # 这里只保存真实反馈，不把模型预测值混进去。
                        mpc_shadow["feedback_samples"].append({
                            "frame": int(sent),
                            "measured_q_deg":
                                mpc_measured.tolist(),
                            "reference_q_deg":
                                ref_q.tolist(),
                        })

                    else:
                        # 中间的 20 ms 帧没有新反馈。
                        # 用上一帧实际发送的 command 和内部模型
                        # 预测当前关节状态，避免重复使用 40 ms 的旧反馈。
                        mpc_measured = (
                            (1.0 - mpc_settings.response_alpha)
                            * mpc_measured
                            + mpc_settings.response_alpha
                            * mpc_previous_command
                        )

                    _result = mpc_controller.step(
                        mpc_measured,
                        mpc_previous_command,
                        mpc_previous_velocity,
                        mpc_windows[sent],
                    )

                    _candidate = np.asarray(
                        _result["command_deg"],
                        dtype=float,
                    )

                    _dev = float(
                        np.max(np.abs(_candidate - ref_q))
                    )

                    mpc_shadow["solve_calls"] += 1
                    mpc_shadow["solve_total_ms"] += float(
                        _result["solve_wall_ms"]
                    )
                    mpc_shadow["solve_max_ms"] = max(
                        mpc_shadow["solve_max_ms"],
                        float(_result["solve_wall_ms"]),
                    )

                    if (
                        _dev
                        > mpc_shadow[
                            "candidate_max_deviation_deg"
                        ]
                    ):
                        mpc_shadow[
                            "candidate_max_deviation_deg"
                        ] = _dev
                        mpc_shadow[
                            "candidate_max_deviation_frame"
                        ] = int(sent)

                    # SHADOW 模式下真正发送的仍然是 ref_q，
                    # 因此下一次 MPC 初值必须跟真实已发送指令一致。
                    _prev_command = (
                        mpc_previous_command.copy()
                    )
                    mpc_previous_command = ref_q.copy()
                    mpc_previous_velocity = (
                        ref_q - _prev_command
                    ) / mpc_dt

                except Exception as exc:
                    # Shadow 出错不得改变已经验证的原 Servo 行为。
                    # 记录一次后关闭 shadow，原 reference 继续正常执行。
                    mpc_shadow_enabled = False
                    mpc_shadow["errors"].append(
                        f"frame={sent}: "
                        f"{type(exc).__name__}: {exc}"
                    )

            pacer.before_send()  # 迟到则 realign；绝不补发积压帧。

            if prepared_q is None:
                raise RuntimeError(
                    "Servo motion waypoint 缺少 prepared FloatVector"
                )

            t0 = time.perf_counter()

            code = sdk.armPluseToServo(
                field.ARM_ID,
                prepared_q,
            )
            ss.require_code_zero(
                code,
                "tabletop Servo armPluseToServo",
            )

            dt_ms = (time.perf_counter() - t0) * 1000.0

            latency_probe["pulse_calls"] += 1
            latency_probe["pulse_total_ms"] += dt_ms
            latency_probe["pulse_max_ms"] = max(
                latency_probe["pulse_max_ms"], dt_ms
            )
            latency_probe["pulse_over_5ms"] += int(dt_ms > 5.0)
            latency_probe["pulse_over_20ms"] += int(dt_ms > 20.0)
            latency_probe["pulse_over_40ms"] += int(dt_ms > 40.0)

            if dt_ms > 5.0:
                latency_probe["slow_events"].append({
                    "type": "pulse",
                    "frame": sent,
                    "stage": w["stage"],
                    "duration_ms": dt_ms,
                })

            sent += 1
            previous = w["q_sdk_deg"]

        if mpc_shadow["solve_calls"]:
            mpc_shadow["solve_mean_ms"] = (
                mpc_shadow["solve_total_ms"]
                / mpc_shadow["solve_calls"]
            )
        else:
            mpc_shadow["solve_mean_ms"] = None

        if mpc_shadow["feedback_reads"]:
            mpc_shadow["feedback_mean_ms"] = (
                mpc_shadow["feedback_total_ms"]
                / mpc_shadow["feedback_reads"]
            )
        else:
            mpc_shadow["feedback_mean_ms"] = None

        mpc_shadow["enabled_until_end"] = (
            mpc_shadow_enabled
        )
        log["mpc_shadow"] = mpc_shadow

        print(
            "MPC SHADOW: "
            f"solve_mean="
            f"{mpc_shadow['solve_mean_ms']} ms, "
            f"solve_max="
            f"{mpc_shadow['solve_max_ms']:.3f} ms, "
            f"read_mean="
            f"{mpc_shadow['feedback_mean_ms']} ms, "
            f"read_max="
            f"{mpc_shadow['feedback_max_ms']:.3f} ms, "
            f"max_candidate_dev="
            f"{mpc_shadow['candidate_max_deviation_deg']:.4f} deg, "
            f"errors={len(mpc_shadow['errors'])}",
            flush=True,
        )

        actual = wait_settled(sdk, previous)
        target = source["RETURN_SAFE"]["expected_endpoint"]
        error = field.world_error(actual["worlds"], target["position_mm"]+target["sdk_world_uvw_deg"])
        if error[0] > field.WORLD_TOL_MM or error[1] > field.WORLD_TOL_DEG:
            raise RuntimeError(f"Servo 返回 Home 的 EE 偏差 {error}")
        log["final_snapshot"] = actual
        set_protection(sdk, True)
    finally:
        log["timing"] = pacer.report()
        log["motion_commands_sent"] = sent

        for prefix in ("soft_stop", "pulse"):
            calls = latency_probe[f"{prefix}_calls"]
            total = latency_probe[f"{prefix}_total_ms"]
            latency_probe[f"{prefix}_mean_ms"] = (
                total / calls if calls else 0.0
            )

        # 按最慢调用排序，只留最严重的前50条。
        latency_probe["slow_events"] = sorted(
            latency_probe["slow_events"],
            key=lambda x: x["duration_ms"],
            reverse=True,
        )[:50]

        log["sdk_latency_probe"] = latency_probe


def main(argv=None, *, input_fn=input):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("trajectory", type=Path)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--precheck-only", action="store_true")
    mode.add_argument("--run", action="store_true")
    parser.add_argument("--gui-review", type=Path)
    parser.add_argument("--qualification-report", type=Path)
    parser.add_argument("--report", type=Path)
    parser.add_argument("--speed", type=float, default=3.0)
    parser.add_argument("--robot-ip", default=os.environ.get("XIFENG_ROBOT_IP"))
    parser.add_argument("--local-ip", default=os.environ.get("XIFENG_LOCAL_IP"))
    parser.add_argument("--arm-ip", default=os.environ.get("XIFENG_ARM_IP"))
    parser.add_argument("--arm-port", type=int, default=8080)
    args = parser.parse_args(argv)
    plan = json.loads(args.trajectory.read_text())
    metrics = contract.validate(plan)
    if not math.isfinite(args.speed) or not 0 < args.speed <= plan["stream_policy"]["max_global_speed_percent"]:
        raise ValueError("新 Servo 试验速度需 >0 且 <=5%；真实关节节拍由 JSON 决定")
    log = {"schema_version": "tabletop_servo_run.v1", "created_at": field.utc_now(),
           "trajectory_sha256": contract.digest(args.trajectory),
           "executor_sha256": contract.digest(__file__),
           "source_plan_sha256": plan["meta"]["source_plan_sha256"],
           "dependencies": runtime_hashes(),
           "metrics": metrics, "status": "DRY_RUN", "motion_commands_sent": 0,
           "robot_ip": args.robot_ip, "local_ip": args.local_ip,
           "arm_ip": args.arm_ip or args.robot_ip, "arm_port": args.arm_port,
           "speed_percent": args.speed, "arm_id": field.ARM_ID, "gripper_id": field.GRIPPER_ID}
    print(json.dumps({"status": "FILE_CHECKS_PASS", "trajectory_sha256": log["trajectory_sha256"],
                      "metrics": metrics, "first_point": plan["first_point_q_sdk_deg"],
                      "real_motion_authorized": False}, ensure_ascii=False, indent=2), flush=True)
    if not args.run and not args.precheck_only:
        if args.report: field.write_log(args.report, log)
        return 0
    output = args.report or Path(f"tabletop_servo_{time.strftime('%Y%m%d_%H%M%S')}_{time.time_ns()%1000000000}.json")
    if output.exists() or output.resolve() == args.trajectory.resolve():
        raise ValueError("报告不能覆盖已有文件或轨迹")
    sdk, enabled, completed, code = None, False, False, 2
    try:
        if args.run:
            if os.environ.get("XIFENG_ALLOW_REAL_MOTION") != "1":
                raise ValueError("未设置 XIFENG_ALLOW_REAL_MOTION=1")
            log["reviews"] = require_reviews(args, plan)
        if not args.robot_ip or not args.local_ip:
            raise ValueError("缺现场 --robot-ip / --local-ip")
        for address in (args.robot_ip, args.local_ip, args.arm_ip or args.robot_ip):
            ipaddress.ip_address(address)
        if not 1 <= args.arm_port <= 65535:
            raise ValueError("arm-port 需在 1..65535")
        with robot_lock.acquire_robot(args.robot_ip, "execute_tabletop_servo.py"):
            try:
                sdk = ss.open_read_only_session(args.robot_ip, args.local_ip,
                                                args.arm_ip or args.robot_ip, args.arm_port, (field.ARM_ID,))
                log["precheck"] = {}
                live_precheck(sdk, plan, log["precheck"])
                if args.precheck_only:
                    log["status"] = "READONLY_PASS_NOT_MOTION_AUTHORIZED"
                    code = 0
                else:
                    answer = input_fn("新 Servo 将关闭突变保护；确认空爪、首点/整臂/持物路径净空并手握急停。回车使能，q 中止：")
                    if answer.strip(): raise RuntimeError("操作者中止")
                    field.assert_unchanged(field.snapshot(sdk), log["precheck"]["start"], "人工确认后")
                    enabled = True  # 使能过程中异常也要收尾。
                    field.enable_checked_session(sdk, args.speed, log)
                    log["execution_precheck"] = {}
                    live_precheck(sdk, plan, log["execution_precheck"])
                    field.assert_unchanged(log["execution_precheck"]["start"], log["precheck"]["start"], "使能后")
                    ss.require_code_zero(sdk.armOpenCom(field.GRIPPER_COM, *field.GRIPPER_SERIAL), "open gripper COM")
                    stream(sdk, plan, log)
                    completed = True
                    log["status"] = "SERVO_COMPLETED_RETURNED_HOME"
                    code = 0
            finally:
                if sdk is not None:
                    ok = field.checked_cleanup(sdk, log, enabled=enabled, completed=completed)
                    log["cleanup"]["protection_disabled_by_executor"] = bool(log.get("protection_disabled_for_servo"))
                    if not ok:
                        code = 2
                        log["status"] = "CLEANUP_FAILED"
    except BaseException as exc:
        log["error"] = f"{type(exc).__name__}: {exc}"
        log["status"] = "FAILED"
        print(log["error"], flush=True)
        code = 2
    finally:
        log["finished_at"] = field.utc_now()
        field.write_log(output, log)
        print("报告：", output.resolve(), flush=True)
    return code


if __name__ == "__main__":
    raise SystemExit(main())
