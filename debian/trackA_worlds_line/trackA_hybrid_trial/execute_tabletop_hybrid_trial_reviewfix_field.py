#!/usr/bin/env python3
"""Operator-supervised mixed MoveWorlds/MoveJoints executor for v1/v2 candidates.

This consumer keeps the original BLOCKED candidate JSON unchanged. Real motion requires all of:
  * --run
  * --accept-blockers for the unchanged v1 field reference only
  * SHA-bound GUI review plus qualification evidence for any new v2 candidate
  * XIFENG_ALLOW_REAL_MOTION=1
  * a read-only live hard precheck with no IK/limit/jump failures
  * an explicit operator Enter confirmation before enabling

Review-fix revision adds reference-boundary/start/margin diagnostics only.
It does NOT seed armTryWorlds, alter commanded targets, or authorize a blocked v2.
Default current-reviewed startup validates the actual stopped configuration and first line;
operator confirmation does not assert a computed collision PASS. Joint margins remain 5 degrees.
It supports the intended mixed transport:
  MoveWorlds -> MoveJoints -> MoveWorlds.
The unseeded armTryWorlds branch mismatch found by the read-only report is treated as a warning,
not as proof that the commanded MoveJ target is invalid. During the real run it verifies the
actual PICK_ASCEND branch before the first MoveJ, waits for every MoveJ to reach the specified
7-axis target, reads the actual Worlds pose at the handoff, then performs a fresh live IK/limit
precheck from that actual pose before the next MoveWorlds segment.

First hardware trial deliberately pauses at two handoff points unless --auto-continue is given:
  1) after PICK_ASCEND, before transport;
  2) after PLACE_HOVER, before PLACE_DESCEND.

Any exception clears the controller route with emergency_stop=True. The gripper is never
automatically opened on an exception.
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
import time
from typing import Any, Mapping, Sequence

import execute_tabletop_pick_place_worlds as worlds
import precheck_tabletop_hybrid as hp
import robot_lock
import scrub_log
import sdk_session as ss

ARM_ID = worlds.ARM_ID
GRIPPER_ID = worlds.GRIPPER_ID
GRIPPER_COM = worlds.GRIPPER_COM
GRIPPER_SERIAL = worlds.GRIPPER_SERIAL
START_POS_TOL_MM = 80.0
START_UVW_TOL_DEG = 20.0
WORLD_TOL_MM = 5.0
WORLD_TOL_DEG = 3.0
JOINT_TOL_DEG = 1.0
PICK_BRANCH_TOL_DEG = 8.0
SAFE_HOME_JOINTS_DEG = [-17.711, 29.247, 6.936, -75.605, -13.196, -11.823, 12.235]  # canonical original Home joints
DEFAULT_HANDOFF_POS_TOL_MM = 60.0
DEFAULT_HANDOFF_UVW_TOL_DEG = 20.0
MAX_WORLD_IK_STEP_DEG = 6.0
RUNTIME_WORLD_PROBE_POINTS = 3
START_DRIFT_JOINT_DEG = 0.2
START_DRIFT_MM = 1.0
START_DRIFT_UVW_DEG = 0.2

# Only for stopped-state settling between live precheck and command.
# Startup/operator-review drift checks remain strict above.
PRECOMMAND_SETTLE_MM = 3.0
PRECOMMAND_SETTLE_UVW_DEG = 0.5
PRECOMMAND_SETTLE_JOINT_DEG = 0.5
MOVEJ_CHECK_STEP_DEG = 0.5
LEGACY_PLAN_SHA256 = "3f3e36f370521e4ab815e377ddb491a3cd2adb24d95dcf843451d03d7587c06e"


def sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def finite_vec(values: Sequence[Any], n: int, label: str) -> list[float]:
    return worlds._finite_vector(values, n, label)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def effective_stages(plan: Mapping[str, Any]) -> list[dict[str, Any]]:
    """[已核实] v1保留Debian现场覆盖；v2命令全部在JSON显式声明。"""
    result = []
    for raw in plan["stages"]:
        stage = dict(raw)
        if plan["schema_version"] == hp.SCHEMA and stage["name"] == "RETURN_SAFE":
            stage.update(kind="MOVE_JOINTS", q_sdk_deg=list(SAFE_HOME_JOINTS_DEG),
                         expected_endpoint=stage["pose"])
        result.append(stage)
    return result


def assert_unchanged(current: Mapping[str, Any], reviewed: Mapping[str, Any], label: str) -> None:
    dp, da = world_error(current["worlds"], reviewed["worlds"])
    dq = max_abs_delta(current["joints"], reviewed["joints"])
    if dp > START_DRIFT_MM or da > START_DRIFT_UVW_DEG or dq > START_DRIFT_JOINT_DEG:
        raise RuntimeError(f"{label}: reviewed state drifted ({dp:.3f}mm/{da:.3f}deg/{dq:.3f}joint deg); rerun precheck and confirmation")


def precommand_revalidate(
    sdk: Any,
    reference: Mapping[str, Any],
    *,
    label: str,
    limits: Sequence[Sequence[float]],
    margin: float,
    world_target: Sequence[float] | None = None,
    joint_target: Sequence[float] | None = None,
) -> dict[str, Any]:
    """
    A stopped servo can settle slightly while dense IK queries are running.

    Strict startup/review drift thresholds are NOT changed.
    For pre-command settling only:
      - tiny drift -> continue
      - modest drift -> refresh the live precheck from the newest real state
      - large drift -> abort
    """
    current = snapshot(sdk)

    dp, da = world_error(current["worlds"], reference["worlds"])
    dq = max_abs_delta(current["joints"], reference["joints"])

    result = {
        "initial_drift_mm": dp,
        "initial_drift_uvw_deg": da,
        "initial_drift_joint_deg": dq,
        "refreshed": False,
    }

    # Original strict gate still passes.
    if (
        dp <= START_DRIFT_MM
        and da <= START_DRIFT_UVW_DEG
        and dq <= START_DRIFT_JOINT_DEG
    ):
        result["status"] = "STRICT_STABLE"
        return result

    # Beyond normal stopped settling: still abort.
    if (
        dp > PRECOMMAND_SETTLE_MM
        or da > PRECOMMAND_SETTLE_UVW_DEG
        or dq > PRECOMMAND_SETTLE_JOINT_DEG
    ):
        raise RuntimeError(
            f"{label}: stopped state changed too much "
            f"({dp:.3f}mm/{da:.3f}deg/{dq:.3f}joint deg)"
        )

    print(
        f"  pre-command settling: "
        f"{dp:.3f}mm/{da:.3f}deg/{dq:.3f}joint deg; "
        f"refreshing live precheck"
    )

    # Revalidate from the newest ACTUAL robot state rather than using
    # the older precheck start.
    if world_target is not None:
        refreshed = quick_world_handoff_precheck(
            sdk,
            current["worlds"],
            world_target,
            limits,
            margin,
            f"{label}/REFRESH",
            start_joints=current["joints"],
        )
        result["refresh"] = {
            "kind": "MOVE_WORLDS",
            "dense_points": refreshed["dense_points"],
            "max_joint_step_deg": refreshed["max_joint_step_deg"],
        }

    elif joint_target is not None:
        frames = dense_joint_line_limit_precheck(
            current["joints"],
            joint_target,
            limits,
            margin,
            f"{label}/REFRESH",
        )
        result["refresh"] = {
            "kind": "MOVE_JOINTS",
            "dense_joint_frames": frames,
        }

    else:
        raise RuntimeError(f"{label}: no target supplied for revalidation")

    latest = snapshot(sdk)
    dp2, da2 = world_error(latest["worlds"], current["worlds"])
    dq2 = max_abs_delta(latest["joints"], current["joints"])

    result["post_refresh_drift_mm"] = dp2
    result["post_refresh_drift_uvw_deg"] = da2
    result["post_refresh_drift_joint_deg"] = dq2
    result["refreshed"] = True
    result["status"] = "REFRESHED_FROM_LIVE_STATE"

    if (
        dp2 > PRECOMMAND_SETTLE_MM
        or da2 > PRECOMMAND_SETTLE_UVW_DEG
        or dq2 > PRECOMMAND_SETTLE_JOINT_DEG
    ):
        raise RuntimeError(
            f"{label}: state still unstable after refreshed precheck "
            f"({dp2:.3f}mm/{da2:.3f}deg/{dq2:.3f}joint deg)"
        )

    return result


def require_protection(sdk: Any) -> bool:
    code, state = sdk.armGetRobotProtectStatus(ARM_ID)
    ss.require_code_zero(code, "read controller protection")
    if not (state is True or type(state) is int and state == 1):
        raise RuntimeError("controller protection is not enabled; no enable or motion permitted")
    return True


def pose6(pose: Mapping[str, Any]) -> list[float]:
    return finite_vec(pose["position_mm"], 3, "position_mm") + finite_vec(
        pose["sdk_world_uvw_deg"], 3, "sdk_world_uvw_deg")


def max_abs_delta(a: Sequence[float], b: Sequence[float]) -> float:
    return max(abs(float(x) - float(y)) for x, y in zip(a, b))


def angle_delta_deg(a: float, b: float) -> float:
    return ((float(a) - float(b) + 180.0) % 360.0) - 180.0


def world_error(actual: Sequence[float], target: Sequence[float]) -> tuple[float, float]:
    dp = max(abs(float(a) - float(b)) for a, b in zip(actual[:3], target[:3]))
    da = max(abs(angle_delta_deg(a, b)) for a, b in zip(actual[3:], target[3:]))
    return dp, da


def limit_errors(q: Sequence[float], limits: Sequence[Sequence[float]], margin: float) -> list[str]:
    bad = []
    for i, (value, pair) in enumerate(zip(q, limits), 1):
        lo, hi = float(pair[0]), float(pair[1])
        if not (lo + margin <= float(value) <= hi - margin):
            bad.append(
                f"J{i}={float(value):+.3f}° outside [{lo + margin:+.3f}, {hi - margin:+.3f}]°")
    return bad


def dense_joint_line_limit_precheck(
    q0: Sequence[float], q1: Sequence[float], limits: Sequence[Sequence[float]],
    margin: float, label: str, *, max_step_deg: float = MOVEJ_CHECK_STEP_DEG,
) -> int:
    """Limit-only precheck for actual live joints -> MoveJ target."""
    a = finite_vec(q0, 7, f"{label} live q")
    b = finite_vec(q1, 7, f"{label} target q")
    max_delta = max_abs_delta(a, b)
    n = max(1, int(math.ceil(max_delta / float(max_step_deg))))

    for i in range(n + 1):
        t = i / n
        q = [x + (y - x) * t for x, y in zip(a, b)]
        bad = limit_errors(q, limits, margin)
        if bad:
            raise RuntimeError(
                f"{label}: live->target MoveJ limit precheck sample {i}/{n} failed: "
                + "; ".join(bad)
            )
    return n


def read_effective_limits(sdk: Any) -> list[tuple[float, float]]:
    ctrl = ss.read_axis_limits(sdk, ARM_ID)
    valid, bad = ss.axis_limits_look_valid(ctrl)
    if not valid:
        raise RuntimeError(f"controller axis limits invalid: {bad}")
    return ss.intersect_axis_limits(ctrl, ss.LIMITS_DEG[ARM_ID])


def wait_robot_idle(
    sdk: Any, *, timeout_s: float = 8.0, poll_s: float = 0.10, label: str = ""
) -> None:
    """Wait for armGetRobotMoveState to fall false after an issued motion.

    wait_until_worlds()/wait_until_joints() are position-error based and may return a little
    before the controller clears its moving flag. Treat that short tail as normal settling,
    rather than misclassifying our own just-finished command as an external robot program.
    """
    deadline = time.time() + float(timeout_s)
    while True:
        ss.check_soft_stop(sdk, f"wait stopped/{label}")
        if not ss.read_move_state(sdk, ARM_ID):
            return
        if time.time() >= deadline:
            tag = f" {label}" if label else ""
            raise RuntimeError(
                f"controller still reports robot moving after{tag} for {timeout_s:.1f}s"
            )
        time.sleep(float(poll_s))


def snapshot(sdk: Any, *, require_stopped: bool = True) -> dict[str, list[float]]:
    ss.check_soft_stop(sdk, "hybrid trial snapshot")
    if require_stopped and ss.read_move_state(sdk, ARM_ID):
        raise RuntimeError("robot is moving; stop other robot programs/teaching first")
    return {
        "worlds": finite_vec(ss.read_worlds(sdk, ARM_ID), 6, "live worlds"),
        "joints": finite_vec(ss.read_joints(sdk, ARM_ID), 7, "live joints"),
    }


def assert_start_safe(current: Mapping[str, Sequence[float]], plan: Mapping[str, Any],
                      start_policy: str = "current-reviewed") -> None:
    safe = pose6(plan["stages"][0]["pose"])
    dp, da = world_error(current["worlds"], safe)
    if start_policy not in ("current-reviewed", "nominal-safe"):
        raise ValueError("unknown start policy")
    if start_policy == "nominal-safe":
        print(f"start safe error: position={dp:.2f} mm, orientation={da:.2f}°")
        if dp > START_POS_TOL_MM or da > START_UVW_TOL_DEG:
            raise RuntimeError(
                f"current pose is not within startup Safe tolerance: {dp:.1f}mm/{da:.1f}° > "
                f"{START_POS_TOL_MM:.1f}mm/{START_UVW_TOL_DEG:.1f}°")


def dense_world_precheck(
    sdk: Any,
    start_pose: Sequence[float],
    target_pose: Sequence[float],
    limits: Sequence[Sequence[float]],
    margin: float,
    label: str,
    *, start_joints: Sequence[float] | None = None,
) -> dict[str, Any]:
    poses = worlds.densify_line(start_pose, target_pose)
    if len(poses) > 4000:
        raise RuntimeError(f"{label}: first-line query exceeds 4000 samples; check scene and units")
    previous_q = finite_vec(start_joints, 7, "actual line start joints") if start_joints is not None else None
    max_step = 0.0
    samples = []
    for i, p in enumerate(poses, 1):
        if i % 25 == 1:
            ss.check_soft_stop(sdk, label)
            if ss.read_move_state(sdk, ARM_ID):
                raise RuntimeError(f"{label}: robot moved during IK precheck")
        q = ss.try_worlds(sdk, ARM_ID, p)
        if q is None:
            raise RuntimeError(f"{label}: armTryWorlds failed at dense sample {i}/{len(poses)}")
        q = finite_vec(q, 7, f"{label} IK")
        bad = limit_errors(q, limits, margin)
        if bad:
            raise RuntimeError(f"{label}: IK limit violation at sample {i}: {'; '.join(bad)}")
        if previous_q is not None:
            step = max_abs_delta(q, previous_q)
            max_step = max(max_step, step)
            if step > MAX_WORLD_IK_STEP_DEG:
                raise RuntimeError(
                    f"{label}: unseeded IK branch jump {step:.3f}° > {MAX_WORLD_IK_STEP_DEG:.3f}° "
                    f"at sample {i}")
        previous_q = q
        samples.append({"sample": i, "worlds_xyzuvw": list(p), "q_sdk_deg": q})
    return {"dense_points": len(poses), "max_joint_step_deg": max_step,
            "endpoint_ik_deg": previous_q, "queries": samples,
            "actual_start_joints_checked": start_joints is not None,
            "scope": "IK_LIMIT_CONTINUITY_ONLY_NOT_COLLISION_PASS"}


def quick_world_handoff_precheck(
    sdk: Any,
    start_pose: Sequence[float],
    target_pose: Sequence[float],
    limits: Sequence[Sequence[float]],
    margin: float,
    label: str,
    *,
    start_joints: Sequence[float] | None = None,
) -> dict[str, Any]:
    """
    Runtime-only local branch/limit check.

    The complete path has already been densely checked before motion.
    Here we only verify that the ACTUAL current joint configuration can
    enter the next MoveWorlds branch continuously.
    """
    poses = worlds.densify_line(start_pose, target_pose)

    if len(poses) > 4000:
        raise RuntimeError(
            f"{label}: path exceeds 4000 samples; check scene and units"
        )

    probe_count = min(RUNTIME_WORLD_PROBE_POINTS, len(poses))

    previous_q = (
        finite_vec(start_joints, 7, f"{label} actual start joints")
        if start_joints is not None
        else None
    )

    samples = []
    max_step = 0.0

    for i, pose in enumerate(poses[:probe_count], 1):
        ss.check_soft_stop(sdk, label)

        if ss.read_move_state(sdk, ARM_ID):
            raise RuntimeError(
                f"{label}: robot moved during runtime handoff probe"
            )

        q = ss.try_worlds(sdk, ARM_ID, pose)
        if q is None:
            raise RuntimeError(
                f"{label}: armTryWorlds failed at runtime probe "
                f"{i}/{probe_count}"
            )

        q = finite_vec(q, 7, f"{label} runtime IK")

        bad = limit_errors(q, limits, margin)
        if bad:
            raise RuntimeError(
                f"{label}: runtime IK limit violation: "
                + "; ".join(bad)
            )

        if previous_q is not None:
            step = max_abs_delta(q, previous_q)
            max_step = max(max_step, step)

            if step > MAX_WORLD_IK_STEP_DEG:
                raise RuntimeError(
                    f"{label}: runtime IK branch jump "
                    f"{step:.3f}° > {MAX_WORLD_IK_STEP_DEG:.3f}°"
                )

        previous_q = q

        samples.append({
            "sample": i,
            "worlds_xyzuvw": list(pose),
            "q_sdk_deg": q,
        })

    return {
        # Keep compatibility with existing callers.
        "dense_points": probe_count,
        "path_dense_points_total": len(poses),
        "max_joint_step_deg": max_step,
        "endpoint_ik_deg": previous_q,
        "queries": samples,
        "actual_start_joints_checked": start_joints is not None,
        "scope": "RUNTIME_LOCAL_HANDOFF_PROBE_AFTER_FULL_PRECHECK",
    }


def replay_group_map(plan: Mapping[str, Any]) -> dict[str, list[list[float]]]:
    # hp.validate performs the strict schema/order/replay consistency checks.
    return hp.validate(plan)


def joint_margin_summary(
    points: Sequence[Sequence[float]], limits: Sequence[Sequence[float]],
    margin: float, *, source: str,
) -> dict[str, Any]:
    """Numeric joint bounds only. Never label IK/replay samples as motion telemetry."""
    if len(limits) != 7 or not points:
        raise ValueError("joint margin summary requires seven limits and nonempty samples")
    bounds = [finite_vec(pair, 2, "limit pair") for pair in limits]
    if any(lo >= hi for lo, hi in bounds):
        raise ValueError("invalid limit pair")
    per_joint = [float("inf")] * 7
    for row in points:
        q = finite_vec(row, 7, "margin sample")
        for i, (lo, hi) in enumerate(bounds):
            per_joint[i] = min(per_joint[i], q[i] - lo, hi - q[i])
    minimum = min(per_joint)
    return {
        "sample_source": source,
        "sample_count": len(points),
        "minimum_hard_limit_margin_per_joint_deg": per_joint,
        "minimum_hard_limit_margin_deg": minimum,
        "limiting_joint": per_joint.index(minimum) + 1,
        "required_margin_deg": margin,
        "minimum_slack_beyond_required_margin_deg": minimum - margin,
        "scope": "NUMERIC_LIMITS_ONLY_NOT_MEASURED_FK_OR_COLLISION_PASS",
    }


def reference_boundary_diagnostic(
    reference_q: Sequence[float], query_q: Sequence[float], *,
    from_stage: str, to_stage: str, reference_basis: str,
) -> dict[str, Any]:
    """Compare two references; this does NOT seed the SDK or predict a physical jump."""
    a = finite_vec(reference_q, 7, "boundary reference")
    b = finite_vec(query_q, 7, "boundary first query")
    differences = [y-x for x,y in zip(a,b)]
    largest = max(abs(x) for x in differences)
    return {
        "from_stage": from_stage, "to_stage": to_stage,
        "reference_basis": reference_basis,
        "reference_joints_sdk_deg": a, "first_query_joints_sdk_deg": b,
        "delta_per_joint_deg": differences, "max_delta_deg": largest,
        "largest_delta_joint": max(range(7), key=lambda i: abs(differences[i])) + 1,
        "runtime_continuity_limit_deg": MAX_WORLD_IK_STEP_DEG,
        "exceeds_runtime_continuity_limit": largest > MAX_WORLD_IK_STEP_DEG,
        "reference_is_measured_live_start": reference_basis == "ACTUAL_LIVE_START",
        "ik_seed_supplied": False,
        "scope": "REFERENCE_TO_UNSEEDED_QUERY_ONLY_NOT_ACTUAL_HANDOFF_OR_MOTION_PASS",
    }


def recorded_start_diagnostic(plan: Mapping[str, Any], current: Mapping[str, Any]) -> dict[str, Any]:
    """Expose start mismatch during precheck, without changing recorded GUI/evidence."""
    if plan["schema_version"] != hp.SCHEMA_V2:
        return {"status": "NOT_REQUIRED_FOR_V1", "matches_recorded_state": None}
    recorded = plan.get("reviewed_start")
    if not isinstance(recorded, dict):
        return {"status": "MISSING_RECORDED_START", "matches_recorded_state": False}
    q = finite_vec(recorded.get("joints"), 7, "recorded start joints")
    w = finite_vec(recorded.get("worlds"), 6, "recorded start worlds")
    actual_q = finite_vec(current["joints"], 7, "actual start joints")
    actual_w = finite_vec(current["worlds"], 6, "actual start worlds")
    dp, da = world_error(actual_w, w)
    dq = max_abs_delta(actual_q, q)
    first = next(f for f in plan["offline_replay"]["frames"] if "q_sdk_deg" in f)
    frame_delta = max_abs_delta(q, finite_vec(first["q_sdk_deg"], 7, "replay start"))
    matches = dp <= START_DRIFT_MM and da <= START_DRIFT_UVW_DEG and dq <= START_DRIFT_JOINT_DEG and frame_delta <= 1e-6
    return {
        "status": "MATCHES_RECORDED_START" if matches else "DIFFERS_FROM_RECORDED_START",
        "matches_recorded_state": matches,
        "recorded_reference_status": plan.get("reviewed_start_status", "SEE_SEPARATE_GUI_EVIDENCE"),
        "position_max_axis_delta_mm": dp, "uvw_max_component_delta_deg": da,
        "joint_max_delta_deg": dq,
        "joint_delta_per_axis_deg": [x-y for x,y in zip(actual_q,q)],
        "recorded_vs_replay_initial_joint_delta_deg": frame_delta,
        "current_captured_state": {"worlds":actual_w,"joints":actual_q},
        "scope": "RECORDED_START_COMPARISON_NOT_GUI_APPROVAL_OR_CURRENT_SCENE_QUALIFICATION",
    }


def readonly_hard_precheck(
    sdk: Any,
    plan: Mapping[str, Any],
    margin: float,
    *, start_policy: str = "current-reviewed",
    report: dict[str, Any] | None = None,
) -> dict[str, Any]:
    groups = replay_group_map(plan)
    initial = snapshot(sdk)
    assert_start_safe(initial, plan, start_policy)
    require_protection(sdk)
    limits = read_effective_limits(sdk)
    start_bad = limit_errors(initial["joints"], limits, margin)
    if start_bad:
        raise RuntimeError("live start joint limit error: " + "; ".join(start_bad))

    if report is None:
        report = {}
    report.update({
        "live_start": initial,
        "effective_limits_deg": limits,
        "segments": [],
        "warnings": [],
        "start_policy": start_policy,
        "scope": "IK_LIMITS_ONLY_OPERATOR_REVIEW_IS_NOT_COLLISION_PASS",
        "protection_enabled": True,
    })
    report["boundary_checks"] = []
    report["actual_post_movej_handoffs_verified"] = False
    previous_pose = initial["worlds"]
    previous_reference_q = list(initial["joints"])
    previous_reference_basis = "ACTUAL_LIVE_START"
    previous_reference_stage = "LIVE_START"
    first_world = True
    for stage in effective_stages(plan):
        kind, name = stage["kind"], stage["name"]
        if name == "RETURN_SAFE":
            q_target = finite_vec(
                stage["q_sdk_deg"], 7, "RETURN_SAFE fixed home q"
            )

            bad = limit_errors(q_target, limits, margin)
            if bad:
                raise RuntimeError(
                    "RETURN_SAFE fixed Home target limit error: "
                    + "; ".join(bad)
                )
            if plan["schema_version"] == hp.SCHEMA_V2:
                for i, q in enumerate(groups[name]):
                    if limit_errors(q, limits, margin):
                        raise RuntimeError(f"RETURN_SAFE dense frame {i} violates live controller limits")

            for axis, value in enumerate(q_target):
                over_pos, over_neg = ss.joint_out_limit(
                    sdk, ARM_ID, axis, value
                )
                if over_pos or over_neg:
                    raise RuntimeError(
                        f"RETURN_SAFE fixed Home controller rejects "
                        f"J{axis+1}={value:.3f}°"
                    )

            expected = pose6(stage["expected_endpoint"])

            report["segments"].append({
                "stage": name,
                "kind": "MOVE_JOINTS_FIXED_HOME",
                "target_q_sdk_deg": q_target,
                "runtime_live_movej_precheck_required": True,
                "margin_summary": joint_margin_summary(
                    groups[name] if plan["schema_version"] == hp.SCHEMA_V2 else [q_target],
                    limits, margin,
                    source="OFFLINE_RETURN_REPLAY" if plan["schema_version"] == hp.SCHEMA_V2 else "FIXED_HOME_TARGET_ONLY",
                ),
            })

            previous_pose = expected
            previous_reference_q = q_target
            previous_reference_basis = "SPECIFIED_MOVEJ_TARGET_NOT_MEASURED_ARRIVAL"
            previous_reference_stage = name
            continue

        if kind == "MOVE_JOINTS":
            frames = groups[name]
            for i, q in enumerate(frames, 1):
                bad = limit_errors(q, limits, margin)
                if bad:
                    raise RuntimeError(f"{name}: dense MoveJ replay sample {i} violates limits: {'; '.join(bad)}")
            q_target = finite_vec(stage["q_sdk_deg"], 7, f"{name} target")
            for axis, value in enumerate(q_target):
                over_pos, over_neg = ss.joint_out_limit(sdk, ARM_ID, axis, value)
                if over_pos or over_neg:
                    raise RuntimeError(f"{name}: controller armJointOutLimit rejects J{axis+1}={value:.3f}°")
            expected = pose6(stage["expected_endpoint"])
            q_ik = ss.try_worlds(sdk, ARM_ID, expected)
            branch_delta = None
            if q_ik is not None:
                branch_delta = max_abs_delta(q_ik, q_target)
                if branch_delta > PICK_BRANCH_TOL_DEG:
                    report["warnings"].append(
                        f"{name}: unseeded endpoint IK differs from specified MoveJ branch by {branch_delta:.3f}°")
            report["segments"].append({
                "stage": name, "kind": kind, "dense_joint_frames": len(frames),
                "unseeded_endpoint_branch_delta_deg": branch_delta,
                "specified_joints_sdk_deg": q_target,
                "unseeded_endpoint_ik_deg": finite_vec(q_ik, 7, "endpoint query") if q_ik is not None else None,
                "margin_summary": joint_margin_summary(frames, limits, margin, source="OFFLINE_MOVEJ_REPLAY"),
                "runtime_actual_arrival_verified": False,
            })
            previous_pose = expected
            previous_reference_q = q_target
            previous_reference_basis = "SPECIFIED_MOVEJ_TARGET_NOT_MEASURED_ARRIVAL"
            previous_reference_stage = name
        elif kind == "MOVE_WORLDS":
            target = pose6(stage["pose"])
            seg = dense_world_precheck(sdk, previous_pose, target, limits, margin, name,
                                      start_joints=initial["joints"] if first_world else None)
            first_world = False
            seg.update(stage=name, kind=kind)
            seg["margin_summary"] = joint_margin_summary(
                [row["q_sdk_deg"] for row in seg["queries"]], limits, margin,
                source="RECORDED_UNSEEDED_WORLDS_IK_NOT_MEASURED_ARRIVAL",
            )
            if seg["queries"]:
                boundary = reference_boundary_diagnostic(
                    previous_reference_q, seg["queries"][0]["q_sdk_deg"],
                    from_stage=previous_reference_stage, to_stage=name,
                    reference_basis=previous_reference_basis,
                )
                seg["incoming_boundary"] = boundary
                report["boundary_checks"].append(boundary)
                if boundary["exceeds_runtime_continuity_limit"]:
                    report["warnings"].append(
                        f"{previous_reference_stage}->{name}: reference-to-first-query delta "
                        f"{boundary['max_delta_deg']:.3f}deg exceeds runtime "
                        f"{MAX_WORLD_IK_STEP_DEG:.3f}deg; NOT a measured jump, actual handoff unverified"
                    )
            report["segments"].append(seg)
            # Compare only as a warning; unseeded IK is not the commanded MoveJ branch.
            planned = groups[name][-1]
            endpoint = seg.get("endpoint_ik_deg")
            if endpoint is not None:
                delta = max_abs_delta(endpoint, planned)
                if delta > PICK_BRANCH_TOL_DEG:
                    report["warnings"].append(
                        f"{name}: unseeded endpoint IK differs from offline replay branch by {delta:.3f}°")
            previous_pose = target
            previous_reference_q = list(seg["endpoint_ik_deg"])
            previous_reference_basis = "PRECEDING_UNSEEDED_IK_PREDICTION_NOT_ARRIVAL"
            previous_reference_stage = name
    report["live_end"] = snapshot(sdk)
    assert_unchanged(report["live_end"], initial, "read-only precheck")
    return report


def build_gripper_frame(gripper_id: int, command: int, values: Sequence[int]) -> bytes:
    data: list[int] = []
    for v in values:
        n = int(v)
        if not 0 <= n <= 65535:
            raise ValueError(f"gripper uint16 out of range: {v}")
        data.extend((n & 0xFF, (n >> 8) & 0xFF))
    payload = [int(gripper_id) & 0xFF, 1 + len(data), int(command) & 0xFF] + data
    return bytes([0xEB, 0x90] + payload + [sum(payload) & 0xFF])


def open_gripper_com(sdk: Any) -> None:
    code = sdk.armOpenCom(GRIPPER_COM, *GRIPPER_SERIAL)
    ss.require_code_zero(code, "armOpenCom(COM1)")


def command_gripper(sdk: Any, plan: Mapping[str, Any], action: str) -> None:
    policy = plan["gripper_policy"]
    if action == "open":
        command = int(policy.get("open_command", 0x11))
        values = [int(policy["open"]["position"])]
        settle = float(policy["open"].get("settle_s", 0.0))
    elif action == "close":
        command = int(policy.get("close_command", 0x10))
        values = [int(policy["close"]["speed"]), int(policy["close"]["force"])]
        settle = float(policy["close"].get("settle_s", 0.0))
    else:
        raise ValueError(f"unknown gripper action: {action}")
    frame = build_gripper_frame(GRIPPER_ID, command, values)
    print(f"  gripper {action}: {frame.hex(' ')}")
    code = sdk.armWriteCom(GRIPPER_COM, frame)
    ss.require_code_zero(code, f"gripper {action} armWriteCom")
    if settle > 0:
        time.sleep(settle)


def handoff_world_check(
    actual_world: Sequence[float], expected_world: Sequence[float],
    pos_tol_mm: float, uvw_tol_deg: float, label: str,
) -> dict[str, float]:
    dp, da = world_error(actual_world, expected_world)
    print(f"  {label} actual-world vs expected: {dp:.2f} mm / {da:.2f}°")
    if dp > pos_tol_mm or da > uvw_tol_deg:
        raise RuntimeError(
            f"{label}: actual MoveJ FK differs too much from expected endpoint: "
            f"{dp:.1f}mm/{da:.1f}° > {pos_tol_mm:.1f}mm/{uvw_tol_deg:.1f}°")
    return {"position_error_mm": dp, "orientation_error_deg": da}


def operator_gate(prompt: str, input_fn: Any, auto_continue: bool) -> None:
    if auto_continue:
        print("[auto-continue] " + prompt)
        return
    answer = input_fn(prompt + " [Enter=continue, q=abort]: ").strip().lower()
    if answer != "":
        raise KeyboardInterrupt("operator aborted")


def execute_real(
    sdk: Any,
    plan: Mapping[str, Any],
    limits: Sequence[Sequence[float]],
    margin: float,
    *,
    input_fn: Any,
    auto_continue: bool,
    handoff_pos_tol_mm: float,
    handoff_uvw_tol_deg: float,
    run_log: dict[str, Any],
    start_policy: str = "current-reviewed",
) -> None:
    expected_pick_q = finite_vec(plan["expected_pick_ascent_joints_sdk_deg"], 7, "expected_pick_ascent")
    carrying = False
    for stage in effective_stages(plan):
        name, kind = stage["name"], stage["kind"]

        print(f"[{name}] {kind}")
        event: dict[str, Any] = {"stage": name, "kind": kind, "started_at": datetime.now(timezone.utc).isoformat()}
        run_log["events"].append(event)
        event["before"] = snapshot(sdk)
        require_protection(sdk)
        start_bad = limit_errors(event["before"]["joints"], limits, margin)
        if start_bad:
            raise RuntimeError(f"{name}: live start joint limit error: {start_bad}")

        if kind == "ASSERT_ENDPOINT":
            target = pose6(stage["pose"])
            dp, da = world_error(event["before"]["worlds"], target)
            event["assert_error"] = {"position_mm": dp, "orientation_deg": da}
            if name == "START_ASSERT_SAFE":
                assert_start_safe(event["before"], plan, start_policy)
                assert_unchanged(event["before"], run_log["operator_confirmation"]["reviewed_state"], "START_ASSERT_SAFE")
                event["start_policy"] = start_policy
            else:
                if dp > WORLD_TOL_MM or da > WORLD_TOL_DEG:
                    raise RuntimeError(f"END_ASSERT_SAFE failed: {dp:.1f}mm/{da:.1f}°")

        elif kind == "GRIPPER":
            event["command"] = {"action": stage["action"], "policy": plan["gripper_policy"]}
            event["command_issued_at"] = utc_now()
            command_gripper(sdk, plan, stage["action"])
            event["command_returned_at"] = utc_now()
            carrying = bool(stage.get("carrying_after", carrying))
            event["carrying_after"] = carrying

        elif kind == "MOVE_WORLDS":
            target = pose6(stage["pose"])
            # Crucial: precheck from the actual current FK, including after a MoveJ handoff.
            live = event["before"]["worlds"]
            dyn = quick_world_handoff_precheck(sdk, live, target, limits, margin, f"LIVE/{name}",
                                      start_joints=event["before"]["joints"])
            event["live_precheck"] = dyn
            print(
                f"  live handoff probe: "
                f"{dyn['dense_points']}/{dyn.get('path_dense_points_total', dyn['dense_points'])} pts, "
                f"max IK step {dyn['max_joint_step_deg']:.3f}°"
            )
            ss.check_soft_stop(sdk, name)
            require_protection(sdk)
            event["precommand_state_check"] = precommand_revalidate(
                sdk,
                event["before"],
                label=f"pre-command/{name}",
                limits=limits,
                margin=margin,
                world_target=target,
            )
            event["command"] = {"sdk_method": "armMoveWorlds", "worlds_xyzuvw": target,
                                "interpolation_en": False}
            event["command_issued_at"] = utc_now()
            ss.move_worlds(sdk, ARM_ID, target, interpolation_en=False)
            event["command_returned_at"] = utc_now()
            actual_world = ss.wait_until_worlds(
                sdk, ARM_ID, target, tol_mm=WORLD_TOL_MM, tol_deg=WORLD_TOL_DEG,
                timeout_s=60.0, motion_timeout_s=10.0, label=name)
            # Position tolerance can be met before the controller clears move_state.
            # Wait for that normal settling tail before taking the post-stage snapshot.
            event["arrival_tolerance_at"] = utc_now()
            wait_robot_idle(sdk, timeout_s=12.0, label=name)
            event["controller_stopped_at"] = utc_now()
            actual_world = finite_vec(ss.read_worlds(sdk, ARM_ID), 6, f"{name} settled worlds")
            dp_settle, da_settle = world_error(actual_world, target)
            if dp_settle > WORLD_TOL_MM or da_settle > WORLD_TOL_DEG:
                raise RuntimeError(
                    f"{name}: settled pose outside tolerance: {dp_settle:.2f}mm/{da_settle:.2f}°"
                )
            event["arrival_worlds"] = actual_world
            event["arrival_joints"] = finite_vec(ss.read_joints(sdk, ARM_ID), 7, f"{name} arrival joints")
            if name == "PICK_ASCEND":
                branch_delta = max_abs_delta(event["arrival_joints"], expected_pick_q)
                event["pick_branch_delta_deg"] = branch_delta
                print(f"  PICK_ASCEND actual branch delta: {branch_delta:.3f}°")
                if branch_delta > PICK_BRANCH_TOL_DEG:
                    print(
                        f"  WARNING: PICK_ASCEND branch delta {branch_delta:.3f}° > "
                        f"{PICK_BRANCH_TOL_DEG:.3f}°; live MoveJ bounds will be checked; this is NOT swept-clearance verification")
                print("  PICK_ASCEND reached; continuing automatically to MoveJ transport")
                assert_unchanged(
                    snapshot(sdk),
                    {"worlds": actual_world, "joints": event["arrival_joints"]},
                    "PICK_ASCEND post-arrival",
                )

        elif kind == "MOVE_JOINTS":
            target_q = finite_vec(stage["q_sdk_deg"], 7, f"{name} q")
            bad = limit_errors(target_q, limits, margin)
            if bad:
                raise RuntimeError(f"{name}: target limit error: {'; '.join(bad)}")
            for axis, value in enumerate(target_q):
                over_pos, over_neg = ss.joint_out_limit(sdk, ARM_ID, axis, value)
                if over_pos or over_neg:
                    raise RuntimeError(f"{name}: controller rejects J{axis+1}={value:.3f}°")

            live_q_before_movej = finite_vec(
                ss.read_joints(sdk, ARM_ID), 7, f"{name} live pre-MoveJ q"
            )
            live_frames = dense_joint_line_limit_precheck(
                live_q_before_movej,
                target_q,
                limits,
                margin,
                f"LIVE/{name}",
            )
            event["live_movej_limit_precheck_frames"] = live_frames
            event["live_movej_start_delta_deg"] = max_abs_delta(
                live_q_before_movej, target_q
            )
            print(
                f"  live MoveJ limit precheck: {live_frames} pts, "
                f"start->target max joint delta "
                f"{event['live_movej_start_delta_deg']:.3f}°"
            )

            ss.check_soft_stop(sdk, name)
            require_protection(sdk)
            event["precommand_state_check"] = precommand_revalidate(
                sdk,
                event["before"],
                label=f"pre-command/{name}",
                limits=limits,
                margin=margin,
                joint_target=target_q,
            )
            event["command"] = {"sdk_method": "armMoveJoints", "q_sdk_deg": target_q,
                                "need_callback": False, "delay_ms": 0}
            event["command_issued_at"] = utc_now()
            ss.move_joints_abs(sdk, ARM_ID, target_q)
            event["command_returned_at"] = utc_now()
            actual_q = ss.wait_until_joints(
                sdk, ARM_ID, target_q, tol_deg=JOINT_TOL_DEG,
                timeout_s=90.0, motion_timeout_s=10.0, label=name)
            event["arrival_tolerance_at"] = utc_now()
            wait_robot_idle(sdk, timeout_s=12.0, label=name)
            event["controller_stopped_at"] = utc_now()
            actual_q = finite_vec(ss.read_joints(sdk, ARM_ID), 7, f"{name} settled q")
            settled_joint_error = max_abs_delta(actual_q, target_q)
            if settled_joint_error > JOINT_TOL_DEG:
                raise RuntimeError(
                    f"{name}: settled joint error {settled_joint_error:.3f}° > {JOINT_TOL_DEG:.3f}°"
                )
            actual_world = finite_vec(ss.read_worlds(sdk, ARM_ID), 6, f"{name} arrival worlds")
            event["arrival_joints"] = actual_q
            event["arrival_worlds"] = actual_world
            event["joint_error_deg"] = max_abs_delta(actual_q, target_q)
            expected_world = pose6(stage["expected_endpoint"])
            event["handoff_world_check"] = handoff_world_check(
                actual_world, expected_world, handoff_pos_tol_mm, handoff_uvw_tol_deg, name)
            print("  actual joints:", [round(v, 3) for v in actual_q])
            print("  actual worlds:", [round(v, 3) for v in actual_world])
            if name == "PLACE_HOVER":
                # [重建] 人工确认后，在下一段发令前从实际状态检查一次，不重复计算。
                print("  PLACE_HOVER reached; continuing automatically to descent")
                assert_unchanged(
                    snapshot(sdk),
                    {"worlds": actual_world, "joints": actual_q},
                    "PLACE_HOVER post-arrival",
                )
        else:
            raise RuntimeError(f"unsupported stage kind: {kind}")

        event["finished_at"] = datetime.now(timezone.utc).isoformat()
        event["after"] = snapshot(sdk)


def write_log(path: Path, payload: Mapping[str, Any]) -> None:
    text = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    for pattern, replacement in scrub_log.PATTERNS:
        text = pattern.sub(replacement, text)
    with path.open("x", encoding="utf-8") as handle:
        handle.write(text)


def runtime_config(args: argparse.Namespace) -> dict[str, Any]:
    robot_ip = args.robot_ip or os.environ.get("XIFENG_ROBOT_IP")
    local_ip = args.local_ip or os.environ.get("XIFENG_LOCAL_IP")
    arm_ip = args.arm_ip or os.environ.get("XIFENG_ARM_IP") or robot_ip
    if not robot_ip or not local_ip or not arm_ip:
        raise ValueError("robot/local/arm IP are required")
    for value in (robot_ip, local_ip, arm_ip):
        ipaddress.ip_address(value)
    speed = float(args.speed)
    if not math.isfinite(speed) or not 0.1 <= speed <= 20.0:
        raise ValueError("--speed must be in [0.1,20.0]")
    if args.limit_margin_deg != ss.LIMIT_MARGIN_DEG:
        raise ValueError(f"limit margin is fixed at shared {ss.LIMIT_MARGIN_DEG} degrees; no bypass")
    if not 1 <= args.arm_port <= 65535:
        raise ValueError("arm port out of range")
    for val, upper, name in ((args.handoff_pos_tol_mm, DEFAULT_HANDOFF_POS_TOL_MM, "handoff-pos"),
                             (args.handoff_uvw_tol_deg, DEFAULT_HANDOFF_UVW_TOL_DEG, "handoff-uvw")):
        if not math.isfinite(val) or not 0 < val <= upper:
            raise ValueError(f"{name} tolerance invalid or wider than field reference")
    return {
        "robot_ip": robot_ip, "local_ip": local_ip, "arm_ip": arm_ip,
        "arm_port": int(args.arm_port), "speed": speed,
        "limit_margin_deg": ss.LIMIT_MARGIN_DEG, "start_policy": args.start_policy,
        "auto_continue": args.auto_continue, "handoff_pos_tol_mm": args.handoff_pos_tol_mm,
        "handoff_uvw_tol_deg": args.handoff_uvw_tol_deg,
    }


def load_candidate(path: Path) -> dict[str, Any]:
    plan = json.loads(path.read_text(encoding="utf-8"))
    hp.validate(plan)
    policy = plan["gripper_policy"]
    if policy.get("gripper_id") != GRIPPER_ID:
        raise ValueError("gripper policy ID mismatch")
    for action, expected_command in (("open", 0x11), ("close", 0x10)):
        if policy.get(action + "_command") != expected_command:
            raise ValueError("unexpected gripper protocol command")
        worlds._settle_seconds(policy[action].get("settle_s"), f"{action}.settle_s")
    for action, keys in (("open", ("position",)), ("close", ("speed", "force"))):
        for key in keys:
            value = policy[action][key]
            if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 1000:
                raise ValueError(f"gripper {action}.{key} must be integer [0,1000]")
    return plan


def validate_qualification_evidence(key: str, evidence: Mapping[str, Any],
                                    plan: Mapping[str, Any], plan_hash: str) -> None:
    """校验报告语义；文件存在和SHA匹配本身不能把BLOCKED/局部检查提升为PASS。"""
    if (evidence.get("schema_version") != f"tabletop_hybrid_{key}_evidence.v1"
            or evidence.get("result") != "PASS" or evidence.get("trajectory_sha256") != plan_hash
            or evidence.get("arm_id") != ARM_ID or evidence.get("blockers") != []):
        raise RuntimeError(f"{key} evidence schema/result/trajectory/arm/blockers invalid")
    if key == "geometry":
        components = evidence.get("verified_components", {})
        if any(components.get(k) is not True for k in ("table", "bin", "object", "gripper", "wrist", "full_robot")):
            raise RuntimeError("geometry evidence lacks confirmed full robot/gripper/object/environment geometry")
    elif key == "calibration":
        session = plan.get("meta", {}).get("t_session_used")
        if session is None or evidence.get("t_session_used") != session:
            raise RuntimeError("calibration evidence session does not match plan meta.t_session_used")
        captured = datetime.fromisoformat(evidence["sample_captured_at"])
        if captured.tzinfo is None:
            raise RuntimeError("calibration evidence sample needs timezone")
        age = (datetime.now(timezone.utc) - captured).total_seconds()
        for name, bound in (("max_axis_delta_mm", 5.0), ("euclidean_delta_mm", 8.0)):
            value = evidence.get(name)
            if (isinstance(value, bool) or not isinstance(value, (int, float))
                    or not math.isfinite(value) or not 0 <= value < bound):
                raise RuntimeError("calibration residual missing or exceeds validity gate")
        if not 0 <= age <= 7 * 24 * 3600 or evidence.get("sample_after_last_chassis_or_waist_motion") is not True:
            raise RuntimeError("calibration evidence is stale or predates waist/chassis motion")
    else:
        expected = [s["name"] for s in effective_stages(plan) if s["kind"].startswith("MOVE_")]
        if evidence.get("reviewed_motion_stages") != expected:
            raise RuntimeError(f"{key} evidence omits motion stages")
        for field, upper in (("dense_step_deg", MOVEJ_CHECK_STEP_DEG), ("world_step_mm", 2.0), ("world_step_deg", 1.0)):
            value = evidence.get(field)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not 0 < value <= upper:
                raise RuntimeError(f"{key} evidence insufficient interpolation density")
        if key == "dense_collision":
            checks = evidence.get("coverage", {})
            if any(checks.get(k) != "PASS" for k in ("self_collision", "environment", "held_object", "open_gripper", "wrist")):
                raise RuntimeError("dense collision evidence is incomplete")
        elif evidence.get("matches_sdk_moveworlds_and_movejoints") is not True:
            raise RuntimeError("controller interpolation semantics not verified")


def qualification_gate(plan: Mapping[str, Any], plan_hash: str, args: argparse.Namespace) -> dict[str, Any]:
    """新轨迹不可把人工现场审查/accept-blockers当作标定和密集碰撞PASS。"""
    if plan["schema_version"] == hp.SCHEMA:
        if plan_hash != LEGACY_PLAN_SHA256:
            raise RuntimeError("v1 operator-trial exception only covers the byte-identical field plan; regenerate changed motion as v2")
        return {"mode": "V1_FIELD_REFERENCE_OPERATOR_TRIAL",
                "notice": "Original trajectory only; changed executor/start still unverified",
                "blockers_acknowledged": bool(args.accept_blockers)}
    required = ("geometry", "calibration", "dense_collision", "controller_interpolation")
    mq = plan.get("motion_qualification", {})
    if mq.get("status") != "STAGED_TEST_READY":
        raise RuntimeError("v2 motion qualification BLOCKED: require geometry/calibration/dense collision/"
                           "controller interpolation evidence; --accept-blockers cannot override")
    if args.qualification_report is None:
        raise RuntimeError("v2 requires --qualification-report with SHA-bound evidence files")
    evidence_path = args.qualification_report.resolve()
    evidence = json.loads(evidence_path.read_text())
    if (evidence.get("schema_version") != "tabletop_hybrid_qualification.v1"
            or evidence.get("result") != "STAGED_TEST_READY"
            or evidence.get("trajectory_sha256") != plan_hash):
        raise RuntimeError("qualification report schema/result/trajectory SHA mismatch")
    for key in required:
        check = evidence.get("checks", {}).get(key, {})
        refs = check.get("evidence_files", [])
        if check.get("status") != "PASS" or not refs:
            raise RuntimeError(f"missing qualified {key} evidence")
        for ref in refs:
            source = (evidence_path.parent / ref["path"]).resolve()
            if not source.is_file() or sha256(source) != ref["sha256"]:
                raise RuntimeError(f"{key} evidence missing or SHA mismatch: {source.name}")
            validate_qualification_evidence(key, json.loads(source.read_text()), plan, plan_hash)
    if args.gui_review is None:
        raise RuntimeError("v2 requires --gui-review; offline headless PASS is not a human GUI review")
    gui = json.loads(args.gui_review.read_text())
    step = gui.get("dense_step_deg")
    if (gui.get("schema_version") != "pybullet_gui_review.v1"
            or gui.get("result") != "PASS" or gui.get("trajectory_sha256") != plan_hash
            or gui.get("arm_id") != ARM_ID or not str(gui.get("reviewer", "")).strip()
            or isinstance(step, bool) or not isinstance(step, (int, float))
            or not math.isfinite(step) or not 0 < step <= MOVEJ_CHECK_STEP_DEG):
        raise RuntimeError("GUI review is missing, not PASS, hash/arm mismatched, or insufficient density")
    expected = [s["name"] for s in effective_stages(plan) if s["kind"].startswith("MOVE_")]
    if gui.get("reviewed_motion_stages") != expected or gui.get("full_replay_completed") is not True:
        raise RuntimeError("GUI review must cover every motion stage including fixed-joint return")
    return {"mode": "V2_SHA_BOUND_STAGED_TEST", "gui_review": gui,
            "gui_review_sha256": sha256(args.gui_review), "qualification_report": evidence,
            "qualification_report_sha256": sha256(evidence_path)}


def require_qualified_start(plan: Mapping[str, Any], current: Mapping[str, Any]) -> None:
    """v2不能把已审核的固定回放偷换成未知起点路径；v1仍按现场人工审查流程。"""
    if plan["schema_version"] != hp.SCHEMA_V2:
        return
    replay_start = next(f for f in plan["offline_replay"]["frames"] if "q_sdk_deg" in f)
    recorded = plan.get("reviewed_start")
    if not isinstance(recorded, dict):
        raise RuntimeError("v2 lacks reviewed_start actual joints/worlds evidence; requalify first segment")
    reference = {"joints": finite_vec(recorded.get("joints"), 7, "reviewed_start.joints"),
                 "worlds": finite_vec(recorded.get("worlds"), 6, "reviewed_start.worlds")}
    if max_abs_delta(reference["joints"], replay_start["q_sdk_deg"]) > 1e-6:
        raise RuntimeError("v2 reviewed_start q does not match GUI first frame; requalify first segment")
    try:
        assert_unchanged(current, reference, "v2 qualified start")
    except RuntimeError as exc:
        raise RuntimeError(f"{exc}; record current pose and requalify first segment") from exc


def read_speed(sdk: Any) -> float:
    code, value = sdk.armGetGlobalSpeed()
    ss.require_code_zero(code, "read global speed")
    if isinstance(value, bool):
        raise RuntimeError("global speed must be numeric")
    speed = float(value)
    if not math.isfinite(speed) or not 0 < speed <= 100:
        raise RuntimeError("cannot verify previous global speed; no enable permitted")
    return speed


def enable_checked_session(sdk: Any, speed: float, log: dict[str, Any]) -> None:
    """[重建] 沿用现场初始化API，但复用已确认的只读会话，异常时始终保有SDK句柄。"""
    require_protection(sdk)
    ss.check_soft_stop(sdk, "before enable")
    state = log["enable"] = {"started_at": utc_now(), "speed_before": read_speed(sdk)}
    state["clear_alarm_code"] = sdk.armClearAlarm()
    ss.require_code_zero(state["clear_alarm_code"], "clear alarm")
    state["enable_code"] = sdk.armRobotEnableOrNot(True)
    ss.require_code_zero(state["enable_code"], "enable all")
    time.sleep(1.0)
    code, enabled = sdk.armGetRobotEnableStatus()
    state["enable_readback"] = [code, enabled]
    ss.require_true_status(code, enabled, "enable readback")
    code, servo = sdk.armServoIsOpOrNot()
    state["servo_readback"] = [code, servo]
    ss.require_true_status(code, servo, "servo readback")
    state["speed_set_code"] = sdk.armSetGlobalSpeed(speed)
    ss.require_code_zero(state["speed_set_code"], "set trial speed")
    state["speed_readback"] = read_speed(sdk)
    if abs(state["speed_readback"] - speed) > 0.01:
        raise RuntimeError("trial speed readback mismatch")
    code, single = sdk.armGetSingleRobotEnableStatus(ARM_ID)
    state["arm_enable_readback"] = [code, single]
    ss.require_true_status(code, single, "single arm enable")
    state["protection_enabled"] = require_protection(sdk)
    state["finished_at"] = utc_now()


def checked_cleanup(sdk: Any, log: dict[str, Any], *, enabled: bool, completed: bool) -> bool:
    """成功状态只能在实际速度恢复回读/保护回读/SDK停止后写出。"""
    cleanup: dict[str, Any] = {"started_at": utc_now(), "errors": [], "protection_disabled_by_executor": False}
    log["cleanup"] = cleanup
    route_stop_confirmed = completed
    if enabled and not completed:
        try:
            ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
            cleanup["emergency_route_clear"] = "RETURNED_WITHOUT_ERROR"
            route_stop_confirmed = True
        except BaseException as exc:
            cleanup["errors"].append(f"emergency route clear: {type(exc).__name__}: {exc}")
        cleanup["gripper_auto_opened"] = False
    if enabled:
        stopped_confirmed = False
        try:
            log["final_snapshot"] = snapshot(sdk)
            stopped_confirmed = True
        except BaseException as exc:
            cleanup["errors"].append(f"final stopped snapshot: {type(exc).__name__}: {exc}")
        try:
            if not stopped_confirmed or not route_stop_confirmed:
                cleanup["speed_restore_status"] = "DEFERRED_STOP_UNCONFIRMED"
                raise RuntimeError("stop/route clearing is unconfirmed; keep trial speed, use physical E-stop before restoring speed")
            before = log.get("enable", {}).get("speed_before")
            if before is None:
                raise RuntimeError("previous speed was not recorded")
            code = sdk.armSetGlobalSpeed(before)
            cleanup["speed_restore_code"] = code
            ss.require_code_zero(code, "restore global speed")
            current = read_speed(sdk)
            cleanup["speed_restored_readback"] = current
            cleanup["speed_expected"] = before
            if abs(current - before) > 0.01:
                raise RuntimeError("speed restoration readback mismatch")
            cleanup["speed_restore_status"] = "PASS"
        except BaseException as exc:
            cleanup["errors"].append(f"restore speed: {type(exc).__name__}: {exc}")
        try:
            # 本执行器从不关闭保护；若外部改变，则仅恢复到True并回读。
            protected = ss.read_protect_status(sdk, ARM_ID)
            if not protected:
                ss.set_protect_status(sdk, ARM_ID, True)
                cleanup["protection_restoration_attempted"] = True
            cleanup["protection_enabled"] = require_protection(sdk)
        except BaseException as exc:
            cleanup["errors"].append(f"protection: {type(exc).__name__}: {exc}")
    try:
        result = sdk.stop()
        cleanup["sdk_stop_return"] = result
        # 厂商void返回None也正常；明确False/非零返回码不能写成功。
        if result is False or (result is not None and result is not True and result != 0):
            raise RuntimeError(f"sdk.stop rejected: {result!r}")
        cleanup["sdk_stopped"] = True
    except BaseException as exc:
        cleanup["sdk_stopped"] = False
        cleanup["errors"].append(f"sdk.stop: {type(exc).__name__}: {exc}")
    cleanup["finished_at"] = utc_now()
    cleanup["status"] = "PASS" if not cleanup["errors"] else "FAILED"
    return not cleanup["errors"]


def main(argv: Sequence[str] | None = None, *, input_fn: Any = input) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("plan", type=Path)
    modes = ap.add_mutually_exclusive_group()
    modes.add_argument("--run", action="store_true")
    modes.add_argument("--precheck-only", action="store_true")
    modes.add_argument("--dry-run", action="store_true")
    ap.add_argument("--accept-blockers", action="store_true", help="v1 field reference ONLY; does not authorize v2")
    ap.add_argument("--start-policy", choices=("current-reviewed", "nominal-safe"), default="current-reviewed")
    ap.add_argument("--gui-review", type=Path)
    ap.add_argument("--qualification-report", type=Path)
    ap.add_argument("--robot-ip")
    ap.add_argument("--local-ip")
    ap.add_argument("--arm-ip")
    ap.add_argument("--arm-port", type=int, default=8080)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--limit-margin-deg", type=float, default=ss.LIMIT_MARGIN_DEG,
                    help="must equal shared 5 degree margin; retained for command compatibility")
    ap.add_argument("--auto-continue", action="store_true", help="skip in-run handoff prompts, never startup review")
    ap.add_argument("--handoff-pos-tol-mm", type=float, default=DEFAULT_HANDOFF_POS_TOL_MM)
    ap.add_argument("--handoff-uvw-tol-deg", type=float, default=DEFAULT_HANDOFF_UVW_TOL_DEG)
    ap.add_argument("--log", type=Path)
    args = ap.parse_args(argv)
    log: dict[str, Any] = {"schema_version": "tabletop_hybrid_trial_run.v2", "started_at": utc_now(),
                           "events": [], "status": "VALIDATING",
                           "arguments": {k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()}}
    code = 2
    try:
        if args.log is None:
            args.log = Path("hybrid_trial_" + ("run_" if args.run else "precheck_") +
                            datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
        if args.log.exists() or args.log.suffix.lower() != ".json" or args.log.resolve() == args.plan.resolve():
            raise ValueError("log must be a new .json; never overwrite plan/evidence")
        plan = load_candidate(args.plan)
        groups = replay_group_map(plan)
        plan_hash = sha256(args.plan)
        log.update(plan_file=args.plan.name, plan_sha256=plan_hash, executor_sha256=sha256(__file__),
                   source_hashes={Path(mod.__file__).name: sha256(mod.__file__)
                                  for mod in (worlds, hp, ss, robot_lock, scrub_log)},
                   candidate_blockers=plan.get("blockers", []),
                   effective_motion_commands=[s for s in effective_stages(plan) if s["kind"].startswith("MOVE_")])
        print("plan SHA-256:", plan_hash)
        print("executor SHA-256:", log["executor_sha256"])
        print("start policy:", args.start_policy, "— operator review is NOT collision PASS")
        log["validation_mode"] = {
            "mode": "LIVE_RUNTIME_VALIDATION",
            "trajectory_sha256": plan_hash,
            "historical_qualification_gate_used": False,
        }
        if args.dry_run:
            log.update(status="FILE_CHECKS_PASS_NOT_MOTION_AUTHORIZED", sdk_connected=False,
                       motion_stages=len(groups), dense_frames=sum(len(q) for q in groups.values()))
            code = 0
        else:
            if plan["schema_version"] == hp.SCHEMA_V2:
                args.handoff_pos_tol_mm = min(args.handoff_pos_tol_mm, WORLD_TOL_MM)
                args.handoff_uvw_tol_deg = min(args.handoff_uvw_tol_deg, WORLD_TOL_DEG)
            config = runtime_config(args)
            log["config"] = config
            if args.run:
                if os.environ.get("XIFENG_ALLOW_REAL_MOTION") != "1":
                    raise RuntimeError("real motion requires XIFENG_ALLOW_REAL_MOTION=1")
                if plan["schema_version"] == hp.SCHEMA and not args.accept_blockers:
                    raise RuntimeError("v1 real trial requires --accept-blockers; original candidate remains unchanged")
            with robot_lock.acquire(config["robot_ip"], ARM_ID, "execute_tabletop_hybrid_trial.py"):
                sdk, enabled, completed = None, False, False
                try:
                    sdk = ss.open_read_only_session(config["robot_ip"], config["local_ip"], config["arm_ip"],
                                                    config["arm_port"], (ARM_ID,))
                    log["sdk_connected"] = True
                    log["readonly_precheck"] = {}
                    pre = readonly_hard_precheck(sdk, plan, ss.LIMIT_MARGIN_DEG, start_policy=args.start_policy,
                                                 report=log["readonly_precheck"])
                    log["status"] = (
                        "READONLY_IK_LIMIT_PASS_WITH_DIAGNOSTIC_WARNINGS_NOT_MOTION_AUTHORIZED"
                        if pre["warnings"] else "READONLY_HARD_PRECHECK_PASS_NOT_MOTION_AUTHORIZED"
                    )
                    print(f"READ-ONLY IK/LIMIT CHECKS COMPLETE: {len(pre['segments'])} motion segments")
                    for segment in pre["segments"]:
                        detail = segment.get("margin_summary")
                        if detail:
                            print(
                                f"  {segment['stage']}: {detail['sample_source']}; "
                                f"min hard margin {detail['minimum_hard_limit_margin_deg']:.3f}deg, "
                                f"beyond required margin {detail['minimum_slack_beyond_required_margin_deg']:.3f}deg"
                            )
                    for warning in pre["warnings"]:
                        print("  WARNING:", warning)
                    print("Live IK/limit precheck complete.")
                    if args.run:
                        # 记录确认的实际构型而不是名义Safe；此门不允许auto-continue跳过。
                        require_protection(sdk)
                        speed_before = read_speed(sdk)
                        reviewed = snapshot(sdk)
                        assert_unchanged(reviewed, pre["live_start"], "before operator review")
                        log["runtime_start"] = {
                            "mode": "CURRENT_LIVE_START",
                            "worlds": list(reviewed["worlds"]),
                            "joints": list(reviewed["joints"]),
                        }
                        print("REVIEWED CURRENT STATE:", json.dumps(reviewed))
                        print("FIRST TARGET:", pose6(next(s for s in plan["stages"] if s["kind"] == "MOVE_WORLDS")["pose"]))
                        answer = input_fn(
                            "Confirm EMPTY gripper; robot/table/object/bin and entire current-to-first-target path clear; "
                            "E-stop held. This is manual clearance review, NOT collision PASS. "
                            "Enter to clear alarm, enable and run; any text aborts: ").strip()
                        if answer:
                            raise KeyboardInterrupt("operator aborted before enable")
                        log["operator_confirmation"] = {
                            "confirmed_at": utc_now(), "reviewed_state": reviewed, "empty_gripper_confirmed": True,
                            "table_object_bin_and_first_path_reviewed": True, "emergency_stop_ready": True,
                            "scope": "MANUAL_ONLY_NOT_COMPUTED_COLLISION_PASS",
                            "previous_speed_readback": speed_before}
                        assert_unchanged(snapshot(sdk), reviewed, "before enable")
                        require_protection(sdk)
                        if sha256(args.plan) != plan_hash:
                            raise RuntimeError("plan changed after review")
                        # 重读速度失败发生在任何使能写前；不能吞掉并继续。
                        log["enable"] = {"speed_before": read_speed(sdk)}
                        enabled = True  # 持有句柄，初始化任一步失败仍做可审计收尾。
                        enable_checked_session(sdk, config["speed"], log)
                        current = snapshot(sdk)
                        assert_unchanged(current, reviewed, "after enable")
                        limits = read_effective_limits(sdk)
                        bad = limit_errors(current["joints"], limits, ss.LIMIT_MARGIN_DEG)
                        if bad:
                            raise RuntimeError(f"enabled start limit violation: {bad}")
                        log["enabled_start"] = current
                        first = next(s for s in plan["stages"] if s["kind"] == "MOVE_WORLDS")
                        log["enabled_first_world_precheck"] = quick_world_handoff_precheck(
                            sdk, current["worlds"], pose6(first["pose"]), limits, ss.LIMIT_MARGIN_DEG,
                            "ENABLED/FIRST_WORLD", start_joints=current["joints"])
                        assert_unchanged(snapshot(sdk), reviewed, "before gripper COM")
                        require_protection(sdk)
                        open_gripper_com(sdk)
                        execute_real(sdk, plan, limits, ss.LIMIT_MARGIN_DEG, input_fn=input_fn,
                                     auto_continue=args.auto_continue,
                                     handoff_pos_tol_mm=config["handoff_pos_tol_mm"],
                                     handoff_uvw_tol_deg=config["handoff_uvw_tol_deg"],
                                     run_log=log, start_policy=args.start_policy)
                        completed = True
                        log["motion_completed_at"] = utc_now()
                        log["status"] = "MOTION_RETURNED_SAFE_PENDING_CLEANUP"
                    code = 0
                except KeyboardInterrupt as exc:
                    log.update(status="OPERATOR_ABORT_OR_INTERRUPT", error=str(exc))
                    code = 130
                except Exception as exc:
                    log.update(status="FAILED", error=f"{type(exc).__name__}: {exc}")
                    code = 2
                finally:
                    if sdk is not None:
                        cleanup_ok = checked_cleanup(sdk, log, enabled=enabled, completed=completed)
                        if not cleanup_ok:
                            log["motion_completed"] = completed
                            log["status"] = "CLEANUP_FAILED"
                            code = 2
                        elif completed:
                            log["status"] = "PASS_RETURNED_SAFE"
    except KeyboardInterrupt as exc:
        log.update(status="OPERATOR_ABORT_OR_INTERRUPT", error=str(exc))
        code = 130
    except Exception as exc:
        log.update(status="FAILED", error=f"{type(exc).__name__}: {exc}")
        code = 2
    finally:
        log["finished_at"] = utc_now()
        should_save_log = (
            args.log is not None
            and not args.log.exists()
            and (
                not args.run
                or log.get("status") == "PASS_RETURNED_SAFE"
            )
        )

        if should_save_log:
            try:
                write_log(args.log, log)
                print("log:", args.log.resolve())
            except Exception as exc:
                print("LOG WRITE FAILED:", type(exc).__name__, str(exc))
                log["status_before_log_failure"] = log["status"]
                log["status"] = "LOG_WRITE_FAILED"
                code = 2
        elif args.run and log.get("status") != "PASS_RETURNED_SAFE":
            print("run log not saved: motion did not complete and return Home successfully")
        print(log["status"])
        if log.get("error"):
            print(log["error"])
    return code


if __name__ == "__main__":
    sys.exit(main())
