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
# 当前在 canonical Home 附近时，运行时生成 Servo bridge 连续接入原首点。
FIRST_ENTRY_MATCH_DEG = 5.0

# 每 2 个原始 Servo 点发送 1 个；保留每个 stage 的首尾和夹爪事件。
RUNTIME_STRIDE = 4
RUNTIME_MAX_STEP_DEG = 0.41

# VAJ V2:
# 对已验证 q-polyline 的路径进度 s(t) 做 jerk-limited S-curve。
# 不强制原 polyline 每一个微小折点满足 joint-space jerk。
VAJ2_VMAX_DEG_S = 20.0
VAJ2_AMAX_DEG_S2 = 200.0
VAJ2_JMAX_DEG_S3 = 2000.0

# 防止再次出现 V1 的数百秒异常轨迹。
VAJ2_MAX_RUNTIME_S = 30.0


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
    runtime["stream_policy"]["period_s"] = 0.020

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




def _vaj2_gap(a, b):
    """该路径坐标使用 joint infinity-norm。"""
    return max(
        abs(float(x) - float(y))
        for x, y in zip(a, b)
    )


def _vaj2_scurve(distance, vmax, amax, jmax):
    """静止->静止，对称 jerk-limited 7-phase S-curve。"""
    import math

    L = float(distance)
    V = float(vmax)
    A = float(amax)
    J = float(jmax)

    if L <= 0.0:
        return [], 0.0

    if min(V, A, J) <= 0.0:
        raise RuntimeError("VAJ2 limits must be positive")

    # 达到 Vmax 所需的加速参数。
    if V <= A * A / J:
        tj_v = math.sqrt(V / J)
        ta_v = 0.0
        apeak_v = J * tj_v
    else:
        tj_v = A / J
        ta_v = V / A - tj_v
        apeak_v = A

    xacc_v = apeak_v * (
        tj_v * tj_v
        + 1.5 * tj_v * ta_v
        + 0.5 * ta_v * ta_v
    )

    if L >= 2.0 * xacc_v:
        # 可以达到 Vmax。
        tj = tj_v
        ta = ta_v
        tv = (L - 2.0 * xacc_v) / V

    else:
        # 达不到 Vmax。
        tj_a = A / J
        l_reach_amax = 2.0 * A * tj_a * tj_a

        if L >= l_reach_amax:
            # 达到 Amax，但没有巡航。
            tj = tj_a
            ta = (
                -3.0 * tj
                + math.sqrt(
                    tj * tj + 4.0 * L / A
                )
            ) / 2.0
            ta = max(0.0, ta)

        else:
            # 连 Amax 都达不到：纯 triangular-jerk。
            tj = (L / (2.0 * J)) ** (1.0 / 3.0)
            ta = 0.0

        tv = 0.0

    phases = [
        (+J, tj),
        (0.0, ta),
        (-J, tj),
        (0.0, tv),
        (-J, tj),
        (0.0, ta),
        (+J, tj),
    ]

    return phases, sum(dt for _, dt in phases)


def _vaj2_state(phases, t):
    """积分 piecewise-constant jerk -> (s, v, a)。"""
    x = 0.0
    v = 0.0
    a = 0.0
    elapsed = 0.0

    for jerk, duration in phases:
        if duration <= 0.0:
            continue

        tau = min(
            duration,
            max(0.0, t - elapsed),
        )

        if tau > 0.0:
            x2 = (
                x
                + v * tau
                + 0.5 * a * tau * tau
                + jerk * tau**3 / 6.0
            )
            v2 = (
                v
                + a * tau
                + 0.5 * jerk * tau * tau
            )
            a2 = a + jerk * tau

            x, v, a = x2, v2, a2

        if t <= elapsed + duration:
            return x, v, a

        elapsed += duration

    return x, v, a


def _vaj2_diff_metrics(values, dt):
    """对标量序列做离散 v/a/j，仅用于记录。"""
    if not values:
        return {
            "v": 0.0,
            "a": 0.0,
            "j": 0.0,
        }

    vals = [
        values[0],
        values[0],
        values[0],
        *values,
        values[-1],
        values[-1],
        values[-1],
    ]

    vel = [
        (b - a) / dt
        for a, b in zip(vals[:-1], vals[1:])
    ]

    acc = [
        (b - a) / dt
        for a, b in zip(vel[:-1], vel[1:])
    ]

    jerk = [
        (b - a) / dt
        for a, b in zip(acc[:-1], acc[1:])
    ]

    return {
        "v": max((abs(x) for x in vel), default=0.0),
        "a": max((abs(x) for x in acc), default=0.0),
        "j": max((abs(x) for x in jerk), default=0.0),
    }


def _vaj2_joint_diagnostics(start_q, items, dt):
    """joint-space v/a/j 只记录、不作为 V2 stretch 条件。"""
    if not items:
        return {
            "joint_v": 0.0,
            "joint_a": 0.0,
            "joint_j": 0.0,
        }

    qs = [
        list(start_q),
        list(start_q),
        list(start_q),
    ]

    qs.extend(
        list(w["q_sdk_deg"])
        for w in items
    )

    qs.extend([
        list(qs[-1]),
        list(qs[-1]),
        list(qs[-1]),
    ])

    vel = [
        [
            (b - a) / dt
            for a, b in zip(q0, q1)
        ]
        for q0, q1 in zip(qs[:-1], qs[1:])
    ]

    acc = [
        [
            (b - a) / dt
            for a, b in zip(v0, v1)
        ]
        for v0, v1 in zip(vel[:-1], vel[1:])
    ]

    jerk = [
        [
            (b - a) / dt
            for a, b in zip(a0, a1)
        ]
        for a0, a1 in zip(acc[:-1], acc[1:])
    ]

    def peak(rows):
        return max(
            (
                abs(x)
                for row in rows
                for x in row
            ),
            default=0.0,
        )

    return {
        "joint_v": peak(vel),
        "joint_a": peak(acc),
        "joint_j": peak(jerk),
    }


def _vaj2_retime_block(
    start_q,
    block,
    period_s,
    vmax,
    amax,
    jmax,
):
    """沿原 q-polyline 做 progress S-curve，不改变几何路径。"""
    import bisect
    import copy
    import math

    if not block:
        return [], {}

    qnodes = [list(map(float, start_q))]
    inode = [copy.deepcopy(block[0])]
    cumulative = [0.0]

    for item in block:
        q = list(map(float, item["q_sdk_deg"]))
        ds = _vaj2_gap(qnodes[-1], q)

        if ds <= 1e-12:
            continue

        qnodes.append(q)
        inode.append(copy.deepcopy(item))
        cumulative.append(cumulative[-1] + ds)

    L = cumulative[-1]

    if L <= 1e-12:
        item = copy.deepcopy(block[-1])
        item["q_sdk_deg"] = list(map(
            float, block[-1]["q_sdk_deg"]
        ))
        item["vaj2_generated"] = True

        return [item], {
            "path_length_deg": 0.0,
            "frames": 1,
            "duration_s": period_s,
            "scalar_v_deg_s": 0.0,
            "scalar_a_deg_s2": 0.0,
            "scalar_j_deg_s3": 0.0,
        }

    phases, nominal_T = _vaj2_scurve(
        L, vmax, amax, jmax
    )

    nominal_end, _, _ = _vaj2_state(
        phases, nominal_T
    )

    if nominal_T <= 0.0 or nominal_end <= 0.0:
        raise RuntimeError(
            f"VAJ2 invalid profile L={L}, T={nominal_T}"
        )

    # 把总时间对齐到整数个 Servo slot。
    n = max(
        1,
        math.ceil(nominal_T / period_s),
    )

    actual_T = n * period_s
    stretch = actual_T / nominal_T

    distance_scale = L / nominal_end

    def interpolate(spos):
        if spos >= L - 1e-10:
            item = copy.deepcopy(inode[-1])
            item["q_sdk_deg"] = list(qnodes[-1])
            item["vaj2_generated"] = True
            return item

        idx = bisect.bisect_right(
            cumulative, spos
        ) - 1

        idx = max(
            0,
            min(idx, len(qnodes) - 2),
        )

        ds = (
            cumulative[idx + 1]
            - cumulative[idx]
        )

        alpha = (
            (spos - cumulative[idx]) / ds
            if ds > 0.0
            else 1.0
        )

        q = [
            a + alpha * (b - a)
            for a, b in zip(
                qnodes[idx],
                qnodes[idx + 1],
            )
        ]

        item = copy.deepcopy(inode[idx + 1])
        item["q_sdk_deg"] = q
        item["vaj2_generated"] = True
        return item

    out = []
    spos_samples = []

    for k in range(1, n + 1):
        actual_t = k * period_s

        # 时间拉伸只用于对齐 Servo slot；
        # 会让 v/a/j 比理论限制更小，不会更大。
        nominal_t = min(
            nominal_T,
            actual_t / stretch,
        )

        spos, _, _ = _vaj2_state(
            phases, nominal_t
        )

        spos *= distance_scale
        spos = min(L, max(0.0, spos))

        out.append(interpolate(spos))
        spos_samples.append(spos)

    # endpoint 强制精确一致。
    out[-1]["q_sdk_deg"] = list(qnodes[-1])
    spos_samples[-1] = L

    scalar = _vaj2_diff_metrics(
        [0.0] + spos_samples,
        period_s,
    )

    joint = _vaj2_joint_diagnostics(
        start_q,
        out,
        period_s,
    )

    return out, {
        "path_length_deg": L,
        "frames": len(out),
        "duration_s": len(out) * period_s,
        "nominal_profile_s": nominal_T,
        "slot_stretch": stretch,
        "scalar_v_deg_s": scalar["v"],
        "scalar_a_deg_s2": scalar["a"],
        "scalar_j_deg_s3": scalar["j"],
        "diagnostic_joint_v_deg_s":
            joint["joint_v"],
        "diagnostic_joint_a_deg_s2":
            joint["joint_a"],
        "diagnostic_joint_j_deg_s3":
            joint["joint_j"],
    }


def vaj2_retime_plan(
    runtime_plan,
    initial_q,
    period_s,
):
    """只在夹爪/等待事件处要求完全停稳。"""
    import copy

    result = copy.deepcopy(runtime_plan)

    out = []
    block = []
    stats = []

    current_q = list(map(float, initial_q))

    def flush():
        nonlocal block, current_q

        if not block:
            return

        generated, info = _vaj2_retime_block(
            current_q,
            block,
            period_s,
            VAJ2_VMAX_DEG_S,
            VAJ2_AMAX_DEG_S2,
            VAJ2_JMAX_DEG_S3,
        )

        out.extend(generated)

        if generated:
            current_q = list(
                generated[-1]["q_sdk_deg"]
            )

        info["block_index"] = len(stats)
        info["source_anchor_count"] = len(block)
        stats.append(info)

        block = []

    for item in runtime_plan["waypoints"]:
        if "q_sdk_deg" in item:
            block.append(item)
        else:
            flush()
            out.append(copy.deepcopy(item))

    flush()

    result["waypoints"] = out

    motion_frames = sum(
        x.get("frames", 0)
        for x in stats
    )

    duration_s = motion_frames * period_s

    return result, {
        "blocks": stats,
        "block_count": len(stats),
        "motion_frames": motion_frames,
        "duration_s": duration_s,
        "max_scalar_v_deg_s": max(
            (
                x.get("scalar_v_deg_s", 0.0)
                for x in stats
            ),
            default=0.0,
        ),
        "max_scalar_a_deg_s2": max(
            (
                x.get("scalar_a_deg_s2", 0.0)
                for x in stats
            ),
            default=0.0,
        ),
        "max_scalar_j_deg_s3": max(
            (
                x.get("scalar_j_deg_s3", 0.0)
                for x in stats
            ),
            default=0.0,
        ),
        "diagnostic_joint_v_deg_s": max(
            (
                x.get("diagnostic_joint_v_deg_s", 0.0)
                for x in stats
            ),
            default=0.0,
        ),
        "diagnostic_joint_a_deg_s2": max(
            (
                x.get("diagnostic_joint_a_deg_s2", 0.0)
                for x in stats
            ),
            default=0.0,
        ),
        "diagnostic_joint_j_deg_s3": max(
            (
                x.get("diagnostic_joint_j_deg_s3", 0.0)
                for x in stats
            ),
            default=0.0,
        ),
    }


def field_stream(sdk, plan, log):
    entry_bridge_start = None
    entry_bridge_precheck_frames = 0
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

        entry_bridge_start = list(current["joints"])
        entry_bridge_precheck_frames = frames

        print(
            f"FIELD ENTRY SERVO: canonical Home -> Servo 首点, "
            f"max joint delta {first_gap:.3f}°, "
            f"limit precheck {frames} pts; "
            f"将并入同一 Servo stream",
            flush=True,
        )

        log["field_entry"] = {
            "mode": "SERVO_BRIDGE_HOME_TO_RECORDED_SERVO_START",
            "start": current,
            "planned_target": first,
            "limit_precheck_frames": frames,
            "arrival": None,
            "arrival_gap_deg": None,
            "strict_precheck_after_entry":
                "NOT_APPLICABLE_UNIFIED_SERVO_STREAM",
        }

    runtime_plan, runtime_metrics = make_runtime_plan(plan)

    entry_bridge = []
    entry_max_step_deg = 0.0

    if entry_bridge_start is not None:
        import math

        period_s = float(
            runtime_plan["stream_policy"]["period_s"]
        )
        body_step_deg = float(
            runtime_metrics["max_step_deg"]
        )

        if body_step_deg <= 0.0:
            raise RuntimeError(
                f"非法 runtime max_step: {body_step_deg}"
            )

        delta = [
            b - a
            for a, b in zip(entry_bridge_start, first)
        ]
        max_delta = max(abs(x) for x in delta)

        # n_intervals 个等距区间。
        # 最终 first 点由原主体的第一个 waypoint 下发，
        # 所以这里只生成 1..n-1，避免首点重复停一帧。
        n_intervals = max(
            1,
            math.ceil(max_delta / body_step_deg),
        )

        for i in range(1, n_intervals):
            alpha = i / n_intervals
            q = [
                a + alpha * d
                for a, d in zip(entry_bridge_start, delta)
            ]

            entry_bridge.append({
                "stage": "FIELD_HOME_TO_SERVO_START",
                "q_sdk_deg": q,
                "field_runtime_generated": True,
            })

        entry_max_step_deg = (
            max_delta / n_intervals
            if n_intervals
            else 0.0
        )

        runtime_plan["waypoints"] = (
            entry_bridge
            + runtime_plan["waypoints"]
        )

        log["field_entry"].update({
            "servo_bridge_intervals": n_intervals,
            "servo_bridge_generated_frames":
                len(entry_bridge),
            "servo_bridge_period_s": period_s,
            "servo_bridge_max_step_deg":
                entry_max_step_deg,
            "servo_bridge_extra_duration_s":
                len(entry_bridge) * period_s,
            "body_first_point_completes_bridge": True,
        })

    body_motion_frames = int(
        runtime_metrics["motion_frame_count"]
    )

    pre_vaj_motion_frames = (
        body_motion_frames + len(entry_bridge)
    )

    period_s = float(
        runtime_plan["stream_policy"]["period_s"]
    )

    runtime_plan, vaj2_stats = vaj2_retime_plan(
        runtime_plan,
        initial_q=list(current["joints"]),
        period_s=period_s,
    )

    final_metrics = contract.motion_metrics(
        runtime_plan["waypoints"],
        period_s,
    )

    total_motion_frames = int(
        final_metrics["motion_frame_count"]
    )

    total_duration_s = float(
        final_metrics["stream_duration_s"]
    )

    full_max_step_deg = float(
        final_metrics["max_step_deg"]
    )

    full_max_velocity_deg_s = float(
        vaj2_stats["diagnostic_joint_v_deg_s"]
    )

    if full_max_step_deg > RUNTIME_MAX_STEP_DEG + 1e-9:
        raise RuntimeError(
            f"VAJ2 max step {full_max_step_deg:.4f}° > "
            f"{RUNTIME_MAX_STEP_DEG:.4f}°"
        )

    if total_duration_s > VAJ2_MAX_RUNTIME_S:
        raise RuntimeError(
            f"VAJ2 runtime {total_duration_s:.2f}s > "
            f"safety guard {VAJ2_MAX_RUNTIME_S:.2f}s; "
            "拒绝执行"
        )

    log["field_runtime_servo"] = {
        "mode": "VAJ2_PATH_PROGRESS_SCURVE",
        "stride_seed": RUNTIME_STRIDE,
        "source_motion_frames":
            plan["kinematics"]["motion_frame_count"],
        "pre_vaj_runtime_motion_frames":
            pre_vaj_motion_frames,
        "runtime_motion_frames":
            total_motion_frames,
        "source_period_s":
            plan["stream_policy"]["period_s"],
        "period_s":
            runtime_plan["stream_policy"]["period_s"],
        "runtime_duration_s":
            total_duration_s,
        "runtime_max_step_deg":
            full_max_step_deg,

        "path_progress_limits": {
            "vmax_deg_s": VAJ2_VMAX_DEG_S,
            "amax_deg_s2": VAJ2_AMAX_DEG_S2,
            "jmax_deg_s3": VAJ2_JMAX_DEG_S3,
        },

        "measured_path_progress": {
            "max_v_deg_s":
                vaj2_stats["max_scalar_v_deg_s"],
            "max_a_deg_s2":
                vaj2_stats["max_scalar_a_deg_s2"],
            "max_j_deg_s3":
                vaj2_stats["max_scalar_j_deg_s3"],
        },

        # 只诊断，不作为 V2 的硬限制。
        "joint_space_diagnostics": {
            "max_v_deg_s":
                vaj2_stats["diagnostic_joint_v_deg_s"],
            "max_a_deg_s2":
                vaj2_stats["diagnostic_joint_a_deg_s2"],
            "max_j_deg_s3":
                vaj2_stats["diagnostic_joint_j_deg_s3"],
        },

        "vaj_block_count":
            vaj2_stats["block_count"],
        "vaj_blocks":
            vaj2_stats["blocks"],

        "scope":
            "VAJ2_SCURVE_PROGRESS_OVER_VERIFIED_Q_POLYLINE",
    }

    print(
        f"FIELD SERVO VAJ2: "
        f"dt={period_s*1000:.0f}ms, "
        f"blocks={vaj2_stats['block_count']}, "
        f"frames={total_motion_frames}, "
        f"duration≈{total_duration_s:.2f}s, "
        f"step≤{full_max_step_deg:.3f}°, "
        f"path-v={vaj2_stats['max_scalar_v_deg_s']:.2f}/"
        f"{VAJ2_VMAX_DEG_S:.1f}, "
        f"path-a={vaj2_stats['max_scalar_a_deg_s2']:.1f}/"
        f"{VAJ2_AMAX_DEG_S2:.1f}, "
        f"path-j={vaj2_stats['max_scalar_j_deg_s3']:.1f}/"
        f"{VAJ2_JMAX_DEG_S3:.1f}",
        flush=True,
    )

    print(
        f"  joint diagnostic only: "
        f"v={vaj2_stats['diagnostic_joint_v_deg_s']:.2f}°/s, "
        f"a={vaj2_stats['diagnostic_joint_a_deg_s2']:.1f}°/s², "
        f"j={vaj2_stats['diagnostic_joint_j_deg_s3']:.1f}°/s³",
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
