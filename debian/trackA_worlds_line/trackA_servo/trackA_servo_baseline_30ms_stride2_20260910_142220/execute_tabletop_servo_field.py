#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import execute_tabletop_servo as base
import execute_tabletop_hybrid_trial_reviewfix_field as field
import sdk_session as ss
import tabletop_servo_contract as contract

EXPECTED_PLAN_SHA = "0a18545594212ecf428c36145de7525fcdd16753b89191ecc7e6730f1d1aaec1"
EXPECTED_GUI_SHA = "14176342f3ed8fda12bfe82fccd18255b748bad6d17c438576a95a7380f22589"

HOME_MATCH_DEG = 5.0

# Field trial only:
# 当前离 Servo 首点在该范围内时，先 MoveJ 吸到精确首点，再进入 Servo。
FIRST_ENTRY_MATCH_DEG = 5.0

# 每 2 个原始 Servo 点发送 1 个；保留每个 stage 的首尾和夹爪事件。
RUNTIME_STRIDE = 2
RUNTIME_MAX_STEP_DEG = 0.21

_orig_live_precheck = base.live_precheck
_orig_stream = base.stream


def field_reviews(args, plan):
    plan_sha = contract.digest(args.trajectory)

    if plan_sha != EXPECTED_PLAN_SHA:
        raise ValueError(
            f"field trial 只允许指定 Servo candidate: {plan_sha}"
        )

    if not args.gui_review:
        raise ValueError("field trial 仍要求 --gui-review")

    gui_path = Path(args.gui_review)
    gui_sha = contract.digest(gui_path)

    if gui_sha != EXPECTED_GUI_SHA:
        raise ValueError(f"GUI review SHA 不匹配: {gui_sha}")

    gui = json.loads(gui_path.read_text())
    count = plan["kinematics"]["motion_frame_count"]

    if (
        gui.get("schema_version") != "pybullet_gui_review.v1"
        or gui.get("result") != "PASS"
        or gui.get("trajectory_sha256") != plan_sha
        or gui.get("full_replay_completed") is not True
        or gui.get("frame_count") != count
        or gui.get("arm_id") != plan["meta"]["arm_id"]
        or gui.get("dense_step_deg")
            != plan["stream_policy"]["max_step_deg"]
    ):
        raise ValueError(
            "GUI review 不是当前 Servo candidate 的完整 PASS"
        )

    return {
        "mode": "OPERATOR_SUPERVISED_FIELD_TRIAL",
        "formal_servo_qualification_bypassed": True,
        "scope": "FIELD_TRIAL_ONLY_NOT_FORMAL_QUALIFICATION_PASS",
        "gui_sha256": gui_sha,
        "field_wrapper_sha256": contract.digest(__file__),
    }


def field_live_precheck(sdk, plan, report=None):
    report = {} if report is None else report
    report["status"] = "CHECKING_FIELD_START"

    before = field.snapshot(sdk)
    report["start"] = before

    field.require_protection(sdk)

    bounds = field.read_effective_limits(sdk)
    report["effective_limits_deg"] = bounds

    metrics = contract.validate(plan, bounds)

    after = field.snapshot(sdk)
    field.assert_unchanged(
        after, before, "Servo field 只读预检期间"
    )

    first_gap = base.gap(
        after["joints"], plan["first_point_q_sdk_deg"]
    )
    home_gap = base.gap(
        after["joints"], plan["home_joints_sdk_deg"]
    )

    report.update(
        start=after,
        initial_gap_deg=first_gap,
        home_gap_deg=home_gap,
        first_point_q_sdk_deg=plan["first_point_q_sdk_deg"],
        motion_metrics=metrics,
    )

    if first_gap <= plan["stream_policy"]["max_initial_gap_deg"]:
        report["status"] = "READONLY_PASS_AT_SERVO_START"
        return report

    if first_gap <= FIRST_ENTRY_MATCH_DEG:
        report["status"] = "READONLY_PASS_NEAR_SERVO_START_ENTRY_REQUIRED"
        return report

    if home_gap <= HOME_MATCH_DEG:
        report["status"] = "READONLY_PASS_HOME_ENTRY_REQUIRED"
        return report

    raise ValueError(
        f"当前既不在 Servo 首点附近也不在 canonical Home 附近: "
        f"first_gap={first_gap:.3f}°, "
        f"home_gap={home_gap:.3f}°"
    )


def make_runtime_plan(plan):
    runtime = copy.deepcopy(plan)
    src = plan["waypoints"]
    out = []

    i = 0
    while i < len(src):
        w = src[i]

        # gripper / 非运动事件原样保留
        if "q_sdk_deg" not in w:
            out.append(copy.deepcopy(w))
            i += 1
            continue

        # 同一个 stage 作为一个连续 block
        stage = w.get("stage")
        block = []

        while (
            i < len(src)
            and "q_sdk_deg" in src[i]
            and src[i].get("stage") == stage
        ):
            block.append(src[i])
            i += 1

        indexes = list(range(0, len(block), RUNTIME_STRIDE))

        # stage 最后一点必须保留
        if indexes[-1] != len(block) - 1:
            indexes.append(len(block) - 1)

        out.extend(copy.deepcopy(block[j]) for j in indexes)

    runtime["waypoints"] = out

    # Field A/B test: keep exact Servo path/frames unchanged,
    # only stretch runtime stream period from source 20 ms to 30 ms.
    runtime["stream_policy"]["period_s"] = 0.030

    metrics = contract.motion_metrics(
        out,
        runtime["stream_policy"]["period_s"],
    )

    if metrics["max_step_deg"] > RUNTIME_MAX_STEP_DEG:
        raise RuntimeError(
            f"stride runtime max step "
            f"{metrics['max_step_deg']:.4f}° > "
            f"{RUNTIME_MAX_STEP_DEG:.4f}°"
        )

    if (
        metrics["max_velocity_deg_s"]
        > plan["stream_policy"]["max_velocity_deg_s"]
    ):
        raise RuntimeError(
            "stride runtime velocity exceeds candidate envelope"
        )

    if (
        metrics["max_acceleration_deg_s2"]
        > plan["stream_policy"]["max_acceleration_deg_s2"]
    ):
        raise RuntimeError(
            "stride runtime acceleration exceeds candidate envelope"
        )

    runtime["kinematics"] = metrics
    return runtime, metrics



def field_stream(sdk, plan, log):
    current = field.snapshot(sdk)
    first = plan["first_point_q_sdk_deg"]

    first_gap = base.gap(current["joints"], first)

    if first_gap > plan["stream_policy"]["max_initial_gap_deg"]:

        home_gap = base.gap(
            current["joints"], plan["home_joints_sdk_deg"]
        )

        if (
            first_gap > FIRST_ENTRY_MATCH_DEG
            and home_gap > HOME_MATCH_DEG
        ):
            raise RuntimeError(
                f"Servo entry 拒绝: "
                f"first_gap={first_gap:.3f}°, "
                f"home_gap={home_gap:.3f}°"
            )

        field.require_protection(sdk)

        limits = field.read_effective_limits(sdk)

        frames = field.dense_joint_line_limit_precheck(
            current["joints"],
            first,
            limits,
            plan["stream_policy"]["margin_deg"],
            "FIELD_HOME_TO_SERVO_START",
        )

        print(
            f"FIELD ENTRY: canonical Home -> Servo 首点, "
            f"max joint delta {first_gap:.3f}°, "
            f"limit precheck {frames} pts",
            flush=True,
        )

        ss.move_joints_abs(
            sdk,
            field.ARM_ID,
            first,
        )

        ss.wait_until_joints(
            sdk,
            field.ARM_ID,
            first,
            tol_deg=field.JOINT_TOL_DEG,
            timeout_s=90.0,
            motion_timeout_s=10.0,
            label="FIELD_HOME_TO_SERVO_START",
        )

        field.wait_robot_idle(
            sdk,
            timeout_s=12.0,
            label="FIELD_HOME_TO_SERVO_START",
        )

        settled = field.snapshot(sdk)

        settled_gap = base.gap(
            settled["joints"], first
        )

        if (
            settled_gap
            > plan["stream_policy"]["max_initial_gap_deg"]
        ):
            raise RuntimeError(
                f"FIELD ENTRY 到首点误差仍过大: "
                f"{settled_gap:.3f}°"
            )

        strict = {}
        _orig_live_precheck(
            sdk,
            plan,
            strict,
        )

        log["field_entry"] = {
            "mode": "MOVEJ_HOME_TO_RECORDED_SERVO_START",
            "start": current,
            "arrival": settled,
            "limit_precheck_frames": frames,
            "arrival_gap_deg": settled_gap,
            "strict_precheck_after_entry": strict,
        }

        ans = input(
            "已到 Servo 首点。确认整臂/瓶子/框路径净空，"
            "回车开始透传，q 中止："
        )

        if ans.strip():
            raise RuntimeError(
                "操作者在 Servo 首点中止"
            )

    runtime_plan, runtime_metrics = make_runtime_plan(plan)

    log["field_runtime_servo"] = {
        "stride": RUNTIME_STRIDE,
        "source_motion_frames": plan["kinematics"]["motion_frame_count"],
        "runtime_motion_frames": runtime_metrics["motion_frame_count"],
        "source_period_s": plan["stream_policy"]["period_s"],
        "period_s": runtime_plan["stream_policy"]["period_s"],
        "runtime_duration_s": runtime_metrics["stream_duration_s"],
        "runtime_max_step_deg": runtime_metrics["max_step_deg"],
        "runtime_max_velocity_deg_s": runtime_metrics["max_velocity_deg_s"],
        "runtime_max_acceleration_deg_s2":
            runtime_metrics["max_acceleration_deg_s2"],
        "scope": "FIELD_TRIAL_RUNTIME_DOWNSAMPLE_OF_EXACT_A_DERIVED_PATH",
    }

    print(
        f"FIELD SERVO: stride={RUNTIME_STRIDE}, "
        f"frames={runtime_metrics['motion_frame_count']}, "
        f"duration≈{runtime_metrics['stream_duration_s']:.2f}s, "
        f"max_step={runtime_metrics['max_step_deg']:.3f}°, "
        f"max_vel={runtime_metrics['max_velocity_deg_s']:.2f}°/s",
        flush=True,
    )

    _orig_stream(sdk, runtime_plan, log)


def main():
    if "--field-trial" not in sys.argv:
        raise SystemExit(
            "必须显式给 --field-trial"
        )

    sys.argv.remove("--field-trial")

    base.require_reviews = field_reviews
    base.live_precheck = field_live_precheck
    base.stream = field_stream

    reports_before = set(Path(".").glob("tabletop_servo_*.json"))

    rc = base.main()

    if rc != 0:
        reports_after = set(Path(".").glob("tabletop_servo_*.json"))
        new_reports = sorted(reports_after - reports_before)

        for report in new_reports:
            try:
                report.unlink()
                print(f"failed run report removed: {report}")
            except OSError as exc:
                print(f"failed run report cleanup failed: {report}: {exc}")

    return rc


if __name__ == "__main__":
    raise SystemExit(main())
