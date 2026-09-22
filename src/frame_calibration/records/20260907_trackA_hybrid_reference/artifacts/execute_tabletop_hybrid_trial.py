#!/usr/bin/env python3
"""Operator-supervised real-hardware trial executor for tabletop_hybrid_candidate.v1.

This consumer keeps the original BLOCKED candidate JSON unchanged. Real motion requires all of:
  * --run
  * --accept-blockers
  * XIFENG_ALLOW_REAL_MOTION=1
  * a read-only live hard precheck with no IK/limit/jump failures
  * an explicit operator Enter confirmation before enabling

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


def sha256(path: str | Path) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def finite_vec(values: Sequence[Any], n: int, label: str) -> list[float]:
    if len(values) != n:
        raise ValueError(f"{label}: expected {n} values, got {len(values)}")
    out = [float(v) for v in values]
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f"{label}: non-finite value")
    return out


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
    margin: float, label: str, *, max_step_deg: float = 0.5,
) -> int:
    """Limit-only precheck for actual live joints -> MoveJ target."""
    a = finite_vec(q0, 7, f"{label} live q")
    b = finite_vec(q1, 7, f"{label} target q")
    max_delta = max_abs_delta(a, b)
    n = max(1, int(math.ceil(max_delta / float(max_step_deg))))

    for i in range(1, n + 1):
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


def assert_start_safe(current: Mapping[str, Sequence[float]], plan: Mapping[str, Any]) -> None:
    safe = pose6(plan["stages"][0]["pose"])
    dp, da = world_error(current["worlds"], safe)
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
) -> dict[str, Any]:
    poses = worlds.densify_line(start_pose, target_pose)
    previous_q = None
    max_step = 0.0
    for i, p in enumerate(poses, 1):
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
    return {"dense_points": len(poses), "max_joint_step_deg": max_step,
            "endpoint_ik_deg": previous_q}


def replay_group_map(plan: Mapping[str, Any]) -> dict[str, list[list[float]]]:
    # hp.validate performs the strict schema/order/replay consistency checks.
    return hp.validate(plan)


def readonly_hard_precheck(
    sdk: Any,
    plan: Mapping[str, Any],
    margin: float,
) -> dict[str, Any]:
    groups = replay_group_map(plan)
    initial = snapshot(sdk)
    assert_start_safe(initial, plan)
    limits = read_effective_limits(sdk)
    start_bad = limit_errors(initial["joints"], limits, margin)
    if start_bad:
        raise RuntimeError("live start joint limit error: " + "; ".join(start_bad))

    report: dict[str, Any] = {
        "live_start": initial,
        "effective_limits_deg": limits,
        "segments": [],
        "warnings": [],
    }
    previous_pose = initial["worlds"]
    for stage in plan["stages"]:
        kind, name = stage["kind"], stage["name"]
        if name == "RETURN_SAFE" and kind == "MOVE_WORLDS":
            q_target = finite_vec(
                SAFE_HOME_JOINTS_DEG, 7, "RETURN_SAFE fixed home q"
            )

            bad = limit_errors(q_target, limits, margin)
            if bad:
                raise RuntimeError(
                    "RETURN_SAFE fixed Home target limit error: "
                    + "; ".join(bad)
                )

            for axis, value in enumerate(q_target):
                over_pos, over_neg = ss.joint_out_limit(
                    sdk, ARM_ID, axis, value
                )
                if over_pos or over_neg:
                    raise RuntimeError(
                        f"RETURN_SAFE fixed Home controller rejects "
                        f"J{axis+1}={value:.3f}°"
                    )

            expected = pose6(stage["pose"])

            report["segments"].append({
                "stage": name,
                "kind": "MOVE_JOINTS_FIXED_HOME",
                "target_q_sdk_deg": q_target,
                "runtime_live_movej_precheck_required": True,
            })

            previous_pose = expected
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
            })
            previous_pose = expected
        elif kind == "MOVE_WORLDS":
            target = pose6(stage["pose"])
            seg = dense_world_precheck(sdk, previous_pose, target, limits, margin, name)
            seg.update(stage=name, kind=kind)
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
    report["live_end"] = snapshot(sdk)
    if max_abs_delta(report["live_start"]["joints"], report["live_end"]["joints"]) > 0.2:
        raise RuntimeError("robot joints changed during read-only precheck")
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
) -> None:
    expected_pick_q = finite_vec(plan["expected_pick_ascent_joints_sdk_deg"], 7, "expected_pick_ascent")
    carrying = False
    for stage in plan["stages"]:
        name, kind = stage["name"], stage["kind"]

        if name == "RETURN_SAFE" and kind == "MOVE_WORLDS":
            stage = dict(stage)
            stage["kind"] = "MOVE_JOINTS"
            stage["q_sdk_deg"] = list(SAFE_HOME_JOINTS_DEG)
            stage["expected_endpoint"] = stage["pose"]
            kind = "MOVE_JOINTS"

            print(
                "  RETURN_SAFE override: fixed Home via MoveJoints "
                "(candidate JSON unchanged)"
            )

        print(f"[{name}] {kind}")
        event: dict[str, Any] = {"stage": name, "kind": kind, "started_at": datetime.now(timezone.utc).isoformat()}
        run_log["events"].append(event)
        event["before"] = snapshot(sdk)

        if kind == "ASSERT_ENDPOINT":
            target = pose6(stage["pose"])
            dp, da = world_error(event["before"]["worlds"], target)
            event["assert_error"] = {"position_mm": dp, "orientation_deg": da}
            if name == "START_ASSERT_SAFE":
                if dp > START_POS_TOL_MM or da > START_UVW_TOL_DEG:
                    raise RuntimeError(f"START_ASSERT_SAFE failed: {dp:.1f}mm/{da:.1f}°")
            else:
                if dp > WORLD_TOL_MM or da > WORLD_TOL_DEG:
                    raise RuntimeError(f"END_ASSERT_SAFE failed: {dp:.1f}mm/{da:.1f}°")

        elif kind == "GRIPPER":
            command_gripper(sdk, plan, stage["action"])
            carrying = bool(stage.get("carrying_after", carrying))
            event["carrying_after"] = carrying

        elif kind == "MOVE_WORLDS":
            target = pose6(stage["pose"])
            # Crucial: precheck from the actual current FK, including after a MoveJ handoff.
            live = event["before"]["worlds"]
            dyn = dense_world_precheck(sdk, live, target, limits, margin, f"LIVE/{name}")
            event["live_precheck"] = dyn
            print(f"  live line precheck: {dyn['dense_points']} pts, max IK step {dyn['max_joint_step_deg']:.3f}°")
            ss.check_soft_stop(sdk, name)
            ss.move_worlds(sdk, ARM_ID, target, interpolation_en=False)
            actual_world = ss.wait_until_worlds(
                sdk, ARM_ID, target, tol_mm=WORLD_TOL_MM, tol_deg=WORLD_TOL_DEG,
                timeout_s=60.0, motion_timeout_s=10.0, label=name)
            # Position tolerance can be met before the controller clears move_state.
            # Wait for that normal settling tail before taking the post-stage snapshot.
            wait_robot_idle(sdk, timeout_s=12.0, label=name)
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
                        f"{PICK_BRANCH_TOL_DEG:.3f}°; continuing because live MoveJ limit precheck is authoritative")
                operator_gate("Bottle should now be lifted. Inspect grip/clearance before MoveJ transport",
                              input_fn, auto_continue)

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
            ss.move_joints_abs(sdk, ARM_ID, target_q)
            actual_q = ss.wait_until_joints(
                sdk, ARM_ID, target_q, tol_deg=JOINT_TOL_DEG,
                timeout_s=90.0, motion_timeout_s=10.0, label=name)
            wait_robot_idle(sdk, timeout_s=12.0, label=name)
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
                # Pre-probe the next descent from the actual MoveJ FK before operator continuation.
                next_stage = next(s for s in plan["stages"] if s["name"] == "PLACE_DESCEND")
                next_target = pose6(next_stage["pose"])
                probe = dense_world_precheck(
                    sdk, actual_world, next_target, limits, margin, "LIVE/PLACE_HOVER->PLACE_DESCEND")
                event["next_descent_live_precheck"] = probe
                print(f"  descent live precheck: {probe['dense_points']} pts, max IK step {probe['max_joint_step_deg']:.3f}°")
                operator_gate("At PLACE_HOVER. Inspect bottle/table/frame clearance before descent",
                              input_fn, auto_continue)
        else:
            raise RuntimeError(f"unsupported stage kind: {kind}")

        event["finished_at"] = datetime.now(timezone.utc).isoformat()
        event["after"] = snapshot(sdk)


def write_log(path: Path, payload: Mapping[str, Any]) -> None:
    text = json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    for pattern, replacement in scrub_log.PATTERNS:
        text = pattern.sub(replacement, text)
    path.write_text(text, encoding="utf-8")


def runtime_config(args: argparse.Namespace) -> dict[str, Any]:
    robot_ip = args.robot_ip or os.environ.get("XIFENG_ROBOT_IP")
    local_ip = args.local_ip or os.environ.get("XIFENG_LOCAL_IP")
    arm_ip = args.arm_ip or os.environ.get("XIFENG_ARM_IP") or robot_ip
    if not robot_ip or not local_ip or not arm_ip:
        raise ValueError("robot/local/arm IP are required")
    speed = float(args.speed)
    if not 0.1 <= speed <= 20.0:
        raise ValueError("--speed must be in [0.1,20.0]")
    margin = float(args.limit_margin_deg)
    if not 0.0 <= margin <= ss.LIMIT_MARGIN_DEG:
        raise ValueError(f"--limit-margin-deg must be in [0,{ss.LIMIT_MARGIN_DEG}]")
    return {
        "robot_ip": robot_ip, "local_ip": local_ip, "arm_ip": arm_ip,
        "arm_port": int(args.arm_port), "speed": speed, "limit_margin_deg": margin,
    }


def load_candidate(path: Path) -> dict[str, Any]:
    plan = json.loads(path.read_text(encoding="utf-8"))
    hp.validate(plan)
    return plan


def main(argv: Sequence[str] | None = None, *, input_fn: Any = input) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("plan", type=Path)
    modes = ap.add_mutually_exclusive_group()
    modes.add_argument("--run", action="store_true")
    modes.add_argument("--precheck-only", action="store_true")
    modes.add_argument("--dry-run", action="store_true")
    ap.add_argument("--accept-blockers", action="store_true",
                    help="explicitly acknowledge candidate blockers for an operator-supervised hardware trial")
    ap.add_argument("--robot-ip")
    ap.add_argument("--local-ip")
    ap.add_argument("--arm-ip")
    ap.add_argument("--arm-port", type=int, default=8080)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--limit-margin-deg", type=float, default=5.0)
    ap.add_argument("--auto-continue", action="store_true",
                    help="skip the two in-run visual handoff prompts; not recommended on the first trial")
    ap.add_argument("--handoff-pos-tol-mm", type=float, default=DEFAULT_HANDOFF_POS_TOL_MM)
    ap.add_argument("--handoff-uvw-tol-deg", type=float, default=DEFAULT_HANDOFF_UVW_TOL_DEG)
    ap.add_argument("--log", type=Path)
    args = ap.parse_args(argv)

    try:
        plan = load_candidate(args.plan)
        groups = replay_group_map(plan)
        plan_hash = sha256(args.plan)
        print("plan SHA-256:", plan_hash)
        print("executor SHA-256:", sha256(__file__))
        print("sdk_session SHA-256:", sha256(ss.__file__))
        print("candidate remains unchanged: status=OFFLINE_CANDIDATE_BLOCKED, real_motion_authorized=false")
        print("candidate blockers:")
        for b in plan.get("blockers", []):
            print("  -", b)
        if args.dry_run:
            motion_frames = sum(len(v) for v in groups.values())
            q_frames = sum(1 for f in plan['offline_replay']['frames'] if 'q_sdk_deg' in f)
            print(f"DRY RUN PASS: {len(groups)} motion stages, {motion_frames} commanded motion frames; "
                  f"{q_frames} q-bearing frames including captured live start")
            return 0
        config = runtime_config(args)
        if args.run:
            if os.environ.get("XIFENG_ALLOW_REAL_MOTION") != "1":
                raise RuntimeError("real motion requires XIFENG_ALLOW_REAL_MOTION=1")
            if not args.accept_blockers:
                raise RuntimeError("real trial requires --accept-blockers; original candidate blockers are not removed")
        print(f"transport: robot={config['robot_ip']} local={config['local_ip']} arm={config['arm_ip']}:{config['arm_port']}")
        print(f"mode: {'REAL OPERATOR-SUPERVISED TRIAL' if args.run else 'READ-ONLY HARD PRECHECK'}")

        with robot_lock.acquire(config["robot_ip"], ARM_ID, "execute_tabletop_hybrid_trial.py"):
            # Phase A: read-only hard precheck. Branch differences are warnings only.
            ro = None
            try:
                ro = ss.open_read_only_session(
                    config["robot_ip"], config["local_ip"], config["arm_ip"], config["arm_port"], (ARM_ID,))
                pre = readonly_hard_precheck(ro, plan, config["limit_margin_deg"])
                print(f"READ-ONLY HARD PRECHECK PASS: {len(pre['segments'])} motion segments")
                for warning in pre["warnings"]:
                    print("  WARNING:", warning)
            finally:
                if ro is not None:
                    try:
                        ro.stop()
                    except Exception:
                        pass
            if not args.run:
                print("No enable / no motion / no gripper command.")
                return 0

            answer = input_fn(
                "Read-only hard precheck passed. Confirm path clear + E-stop in hand; press Enter to ENABLE and run, q to abort: "
            ).strip().lower()
            if answer != "":
                print("operator aborted before enable")
                return 2

            # Phase B: enabled session; recheck effective limits/start before opening gripper COM.
            sdk = None
            if args.log is None:
                args.log = Path("hybrid_trial_run_" + datetime.now().strftime("%Y%m%d_%H%M%S_%f") + ".json")
            if args.log.exists():
                raise RuntimeError(f"log file already exists: {args.log}")
            run_log: dict[str, Any] = {
                "schema_version": "tabletop_hybrid_trial_run.v1",
                "started_at": datetime.now(timezone.utc).isoformat(),
                "plan_file": args.plan.name,
                "plan_sha256": plan_hash,
                "executor_sha256": sha256(__file__),
                "config": config,
                "accepted_candidate_blockers": True,
                "candidate_blockers": plan.get("blockers", []),
                "readonly_precheck_warnings": pre["warnings"],
                "events": [],
                "status": "STARTING",
            }
            try:
                sdk = ss.open_session(
                    config["robot_ip"], config["local_ip"], config["arm_ip"], config["arm_port"],
                    config["speed"], (ARM_ID,), enable=True)
                current = snapshot(sdk)
                assert_start_safe(current, plan)
                limits = read_effective_limits(sdk)
                bad = limit_errors(current["joints"], limits, config["limit_margin_deg"])
                if bad:
                    raise RuntimeError("enabled-session live start limit error: " + "; ".join(bad))
                # Recheck first Worlds line from the exact enabled-session start.
                first_world = next(s for s in plan["stages"] if s["kind"] == "MOVE_WORLDS")
                first_probe = dense_world_precheck(
                    sdk, current["worlds"], pose6(first_world["pose"]), limits,
                    config["limit_margin_deg"], "ENABLED/PICK_HOVER")
                run_log["enabled_start"] = current
                run_log["enabled_first_world_precheck"] = first_probe
                print(f"enabled-session first line precheck PASS: {first_probe['dense_points']} pts")
                open_gripper_com(sdk)
                execute_real(
                    sdk, plan, limits, config["limit_margin_deg"], input_fn=input_fn,
                    auto_continue=args.auto_continue,
                    handoff_pos_tol_mm=float(args.handoff_pos_tol_mm),
                    handoff_uvw_tol_deg=float(args.handoff_uvw_tol_deg),
                    run_log=run_log)
                run_log["status"] = "PASS_RETURNED_SAFE"
                run_log["finished_at"] = datetime.now(timezone.utc).isoformat()
                print("HYBRID REAL TRIAL PASS: returned to Safe.")
                return 0
            except KeyboardInterrupt as exc:
                run_log["status"] = "OPERATOR_ABORT_OR_INTERRUPT"
                run_log["error"] = str(exc)
                raise
            except BaseException as exc:
                run_log["status"] = "FAILED"
                run_log["error"] = f"{type(exc).__name__}: {exc}"
                raise
            finally:
                if sdk is not None:
                    if run_log.get("status") != "PASS_RETURNED_SAFE":
                        try:
                            ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
                            print("abnormal exit: controller route cleared with emergency_stop=True")
                        except Exception as stop_exc:
                            print("WARNING: failed to clear route; use physical E-stop:", stop_exc)
                        print("gripper was NOT automatically opened; inspect held object manually")
                    try:
                        run_log["final_snapshot"] = {
                            "worlds": finite_vec(ss.read_worlds(sdk, ARM_ID), 6, "final worlds"),
                            "joints": finite_vec(ss.read_joints(sdk, ARM_ID), 7, "final joints"),
                        }
                    except Exception as snap_exc:
                        run_log["final_snapshot_error"] = str(snap_exc)
                    ss.close_session(sdk)
                run_log["finished_at"] = datetime.now(timezone.utc).isoformat()
                try:
                    write_log(args.log, run_log)
                    print("run log:", args.log.resolve())
                except Exception as log_exc:
                    print("WARNING: could not write run log:", log_exc)
    except KeyboardInterrupt:
        print("operator interrupted; motion route stop requested if session was enabled")
        return 130
    except Exception as exc:
        print(f"HYBRID TRIAL ABORTED: {type(exc).__name__}: {exc}")
        return 2


if __name__ == "__main__":
    sys.exit(main())
