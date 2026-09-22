#!/usr/bin/env python3
"""执行 ``tabletop_plan.v1`` 的左臂桌上抓放 MoveWorlds 第一版。

默认模式会连接当前控制器，但使用 ``open_session(enable=False)``，只读取当前
Worlds/关节/轴限位并对整条路径调用密集 ``armTryWorlds``；不会清报警、使能、
打开夹爪串口或发送运动。真机执行必须同时满足：

* 命令行显式给 ``--run``；
* 环境变量 ``XIFENG_ALLOW_REAL_MOTION=1``；
* plan 为 ``OFFLINE_CANDIDATE_PRECHECK_REQUIRED`` 且没有 input blocker；
* 只读全路径预检通过后，现场操作员再按回车确认。

执行阶段会重新建立使能会话并再次完成同一套全路径预检。所有运动只使用
``armMoveWorlds(..., interpolation_en=False)`` 并逐段等待到位。任何异常只清除
控制器残留路径，不会擅自松开夹爪；由现场人员根据物体状态决定后续处理。

Debian 示例（IP 必须在运行时注入）：

  python3 execute_tabletop_pick_place_worlds.py tabletop_plan.json
  XIFENG_ALLOW_REAL_MOTION=1 python3 execute_tabletop_pick_place_worlds.py \
      tabletop_plan.json --run
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping, Sequence


SCHEMA_VERSION = "tabletop_plan.v1"
RUNNABLE_STATUS = "OFFLINE_CANDIDATE_PRECHECK_REQUIRED"
BLOCKED_STATUS = "OFFLINE_CANDIDATE_BLOCKED"
ARM_ID = 1
GRIPPER_ID = 2
GRIPPER_COM = "COM1"
GRIPPER_SERIAL = (115200, 8, 0, 1)

GLOBAL_SPEED = 1.0
SAMPLE_MM = 2.0
SAMPLE_DEG = 1.0
LIMIT_MARGIN_DEG = 5.0
MAX_JOINT_STEP_DEG = 4.0
SAFE_POSITION_TOL_MM = 50.0
SAFE_ORIENTATION_TOL_DEG = 15.0
ARRIVE_TOL_MM = 3.0
ARRIVE_TOL_DEG = 1.5
ARRIVE_TIMEOUT_S = 60.0


STAGE_CONTRACT = (
    ("START_ASSERT_SAFE", "ASSERT_ENDPOINT", None, "carrying", False),
    ("OPEN_BEFORE_PICK", "GRIPPER", "open", "carrying_after", False),
    ("PICK_HOVER", "MOVE_WORLDS", None, "carrying", False),
    ("PICK_DESCEND", "MOVE_WORLDS", None, "carrying", False),
    ("CLOSE_AT_PICK", "GRIPPER", "close", "carrying_after", True),
    ("PICK_ASCEND", "MOVE_WORLDS", None, "carrying", True),
    ("PLACE_HOVER", "MOVE_WORLDS", None, "carrying", True),
    ("PLACE_DESCEND", "MOVE_WORLDS", None, "carrying", True),
    ("OPEN_AT_PLACE", "GRIPPER", "open", "carrying_after", False),
    ("PLACE_ASCEND", "MOVE_WORLDS", None, "carrying", False),
    ("RETURN_SAFE", "MOVE_WORLDS", None, "carrying", False),
    ("END_ASSERT_SAFE", "ASSERT_ENDPOINT", None, "carrying", False),
)

RESOLVED_STAGE_KEYS = {
    "START_ASSERT_SAFE": "safe_endpoint",
    "PICK_HOVER": "pick_hover_endpoint",
    "PICK_DESCEND": "pick_endpoint",
    "PICK_ASCEND": "pick_hover_endpoint",
    "PLACE_HOVER": "place_hover_endpoint",
    "PLACE_DESCEND": "place_endpoint",
    "PLACE_ASCEND": "place_hover_endpoint",
    "RETURN_SAFE": "safe_endpoint",
    "END_ASSERT_SAFE": "safe_endpoint",
}


class PlanError(ValueError):
    """计划文件契约不成立。"""


class PrecheckError(RuntimeError):
    """live 无运动预检未通过。"""


def _finite_vector(values: Any, length: int, label: str) -> list[float]:
    if not isinstance(values, (list, tuple)) or len(values) != length:
        raise PlanError(f"{label} 必须是 {length} 维数组")
    try:
        out = [float(value) for value in values]
    except (TypeError, ValueError) as exc:
        raise PlanError(f"{label} 必须全部是数值") from exc
    if not all(math.isfinite(value) for value in out):
        raise PlanError(f"{label} 含非有限数值")
    return out


def _pose6(item: Any, label: str) -> list[float]:
    if not isinstance(item, Mapping):
        raise PlanError(f"{label} 必须是 pose 对象")
    xyz = _finite_vector(item.get("position_mm"), 3, f"{label}.position_mm")
    uvw = _finite_vector(item.get("sdk_world_uvw_deg"), 3,
                         f"{label}.sdk_world_uvw_deg")
    _finite_vector(item.get("quaternion_xyzw"), 4,
                   f"{label}.quaternion_xyzw")
    return xyz + uvw


def _canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def _sha256_path(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1 << 16), b""):
            digest.update(block)
    return digest.hexdigest()


def load_plan(path: str | Path) -> tuple[dict[str, Any], str]:
    plan_path = Path(path)
    raw = plan_path.read_bytes()
    try:
        plan = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise PlanError(f"无法读取计划 JSON: {exc}") from exc
    if not isinstance(plan, dict):
        raise PlanError("计划根节点必须是对象")
    validate_plan(plan)
    return plan, hashlib.sha256(raw).hexdigest()


def _same_pose(a: Sequence[float], b: Sequence[float], tol: float = 1e-6) -> bool:
    return len(a) == len(b) and all(abs(x - y) <= tol for x, y in zip(a, b))


def _uint16(value: Any, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= 65535:
        raise PlanError(f"{label} 必须是 [0,65535] 的整数")
    return value


def _settle_seconds(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise PlanError(f"{label} 必须是非负有限秒数")
    try:
        out = float(value)
    except (TypeError, ValueError) as exc:
        raise PlanError(f"{label} 必须是非负有限秒数") from exc
    if not math.isfinite(out) or out < 0:
        raise PlanError(f"{label} 必须是非负有限秒数")
    return out


def validate_plan(plan: Mapping[str, Any]) -> None:
    """执行器内置的最小严格契约，避免 Debian 运行包依赖 jsonschema/numpy。"""
    if plan.get("schema_version") != SCHEMA_VERSION:
        raise PlanError(
            f"schema_version={plan.get('schema_version')!r}，预期 {SCHEMA_VERSION!r}")
    if plan.get("status") not in (BLOCKED_STATUS, RUNNABLE_STATUS):
        raise PlanError(f"未知 plan.status={plan.get('status')!r}")
    if plan.get("real_motion_authorized") is not False:
        raise PlanError("计划 JSON 不得自行授权真机运动")

    contract = plan.get("contract")
    if not isinstance(contract, Mapping):
        raise PlanError("plan.contract 缺失")
    expected_contract = {
        "arm": "left", "arm_id": ARM_ID, "gripper_id": GRIPPER_ID,
        "world_frame": "sdk_world", "length_unit": "millimeter",
        "dynamic_transport": "MOVE_WORLDS_ONLY",
    }
    for key, expected in expected_contract.items():
        if contract.get(key) != expected:
            raise PlanError(f"contract.{key}={contract.get(key)!r}，预期 {expected!r}")

    safety = plan.get("safety")
    if not isinstance(safety, Mapping):
        raise PlanError("plan.safety 缺失")
    blockers = safety.get("input_blockers")
    if not isinstance(blockers, list) or any(
            not isinstance(value, str) or not value.strip() for value in blockers):
        raise PlanError("safety.input_blockers 必须是非空字符串数组（可为空数组）")
    expected_status = BLOCKED_STATUS if blockers else RUNNABLE_STATUS
    if plan.get("status") != expected_status:
        raise PlanError("plan.status 与 safety.input_blockers 不一致")

    resolved = plan.get("resolved")
    if not isinstance(resolved, Mapping):
        raise PlanError("plan.resolved 缺失")
    required_resolved = (
        "pick_endpoint", "pick_hover_endpoint", "place_endpoint",
        "place_hover_endpoint", "safe_endpoint",
    )
    resolved_poses = {
        key: _pose6(resolved.get(key), f"resolved.{key}")
        for key in required_resolved
    }

    stages = plan.get("stages")
    if not isinstance(stages, list) or len(stages) != len(STAGE_CONTRACT):
        raise PlanError(f"stages 必须严格包含 {len(STAGE_CONTRACT)} 个阶段")
    policy = plan.get("gripper_policy")
    if not isinstance(policy, Mapping):
        raise PlanError("plan.gripper_policy 缺失")
    if policy.get("gripper_id") != GRIPPER_ID:
        raise PlanError(f"gripper_policy.gripper_id 必须为 {GRIPPER_ID}")
    if policy.get("protocol_encoding") != "EB90_UINT16_LITTLE_ENDIAN":
        raise PlanError("只允许已经验证的 EB90 uint16 little-endian 夹爪协议")
    if policy.get("executor_binding") != "execute_tabletop_pick_place_worlds.py/v1":
        raise PlanError("gripper_policy.executor_binding 与当前执行器不匹配")
    if policy.get("open_command") != 0x11 or policy.get("close_command") != 0x10:
        raise PlanError("夹爪 open/close 命令必须分别为 0x11/0x10")
    policy_id = policy.get("policy_id")
    if not isinstance(policy_id, str) or not policy_id:
        raise PlanError("gripper_policy.policy_id 不能为空")
    policy_without_hash = dict(policy)
    policy_hash = policy_without_hash.pop("source_sha256", None)
    if not isinstance(policy_hash, str) or hashlib.sha256(
            _canonical_bytes(policy_without_hash)).hexdigest() != policy_hash:
        raise PlanError("gripper_policy.source_sha256 不匹配")
    open_policy = policy.get("open")
    close_policy = policy.get("close")
    if not isinstance(open_policy, Mapping) or not isinstance(close_policy, Mapping):
        raise PlanError("gripper_policy.open/close 缺失")
    _uint16(open_policy.get("position"), "gripper_policy.open.position")
    _settle_seconds(open_policy.get("settle_s"), "gripper_policy.open.settle_s")
    _uint16(close_policy.get("speed"), "gripper_policy.close.speed")
    _uint16(close_policy.get("force"), "gripper_policy.close.force")
    _settle_seconds(close_policy.get("settle_s"), "gripper_policy.close.settle_s")

    reference_orientation: list[float] | None = None
    for stage, expected in zip(stages, STAGE_CONTRACT):
        if not isinstance(stage, Mapping):
            raise PlanError("每个 stage 必须是对象")
        name, kind, action, state_key, state_value = expected
        if stage.get("name") != name or stage.get("kind") != kind:
            raise PlanError(f"阶段顺序/类型不匹配，预期 {name}/{kind}")
        if action is not None and stage.get("action") != action:
            raise PlanError(f"{name}.action 必须为 {action}")
        if stage.get(state_key) is not state_value:
            raise PlanError(f"{name}.{state_key} 必须为 {state_value}")
        if kind == "GRIPPER":
            if stage.get("policy_id") != policy_id or stage.get("wait_until_done") is not True:
                raise PlanError(f"{name} 未严格绑定夹爪策略或未等待动作完成")
            continue
        pose = _pose6(stage.get("pose"), f"stages.{name}.pose")
        expected_pose = resolved_poses[RESOLVED_STAGE_KEYS[name]]
        if not _same_pose(pose, expected_pose):
            raise PlanError(f"{name}.pose 与 resolved.{RESOLVED_STAGE_KEYS[name]} 不一致")
        if reference_orientation is None:
            reference_orientation = pose[3:]
        elif not _same_pose(pose[3:], reference_orientation):
            raise PlanError("MoveWorlds 第一版要求所有阶段固定同一 endpoint UVW")
        if kind == "MOVE_WORLDS" and (
                stage.get("interpolation_en") is not False or
                stage.get("wait_until_worlds") is not True or
                stage.get("critical") is not True):
            raise PlanError(f"{name} 必须是非混合、逐段等到位的关键 MoveWorlds")


def run_gate_error(plan: Mapping[str, Any], run_requested: bool,
                   allow_real_motion: bool) -> str | None:
    if not run_requested:
        return None
    blockers = plan["safety"]["input_blockers"]
    if not allow_real_motion:
        return "--run 已给出，但 XIFENG_ALLOW_REAL_MOTION!=1"
    if plan.get("status") != RUNNABLE_STATUS or blockers:
        return ("计划仍被阻断：--run 只接受 OFFLINE_CANDIDATE_PRECHECK_REQUIRED "
                "且 safety.input_blockers 为空")
    return None


def _angular_error_deg(a: float, b: float) -> float:
    return abs((float(a) - float(b) + 180.0) % 360.0 - 180.0)


def assert_current_safe(current: Sequence[float], safe: Sequence[float]) -> None:
    current_pose = _finite_vector(current, 6, "current worlds")
    safe_pose = _finite_vector(safe, 6, "safe endpoint")
    position_error = max(abs(a - b) for a, b in zip(current_pose[:3], safe_pose[:3]))
    orientation_error = max(
        _angular_error_deg(a, b) for a, b in zip(current_pose[3:], safe_pose[3:]))
    if position_error > SAFE_POSITION_TOL_MM or orientation_error > SAFE_ORIENTATION_TOL_DEG:
        raise PrecheckError(
            "当前 endpoint 不在计划 safe 点："
            f"位置最大差 {position_error:.2f}mm（允许 {SAFE_POSITION_TOL_MM:.1f}mm），"
            f"姿态最大差 {orientation_error:.2f}°（允许 {SAFE_ORIENTATION_TOL_DEG:.1f}°）")


def densify_line(start: Sequence[float], end: Sequence[float]) -> list[list[float]]:
    a = _finite_vector(start, 6, "segment start")
    b = _finite_vector(end, 6, "segment end")
    distance = math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))
    angle = max(_angular_error_deg(x, y) for x, y in zip(a[3:], b[3:]))
    count = max(1, math.ceil(max(distance / SAMPLE_MM, angle / SAMPLE_DEG)))
    # 首版契约要求姿态固定；保留通用角度最短路径插值，避免跨 +/-180° 绕远。
    angular_delta = [((y - x + 180.0) % 360.0 - 180.0)
                     for x, y in zip(a[3:], b[3:])]
    out = []
    for index in range(1, count + 1):
        ratio = index / count
        out.append(
            [x + (y - x) * ratio for x, y in zip(a[:3], b[:3])] +
            [x + delta * ratio for x, delta in zip(a[3:], angular_delta)])
    return out


def _controller_limit_violations(
        joints: Sequence[float], limits: Sequence[Sequence[float]]) -> list[str]:
    if len(joints) != 7 or len(limits) != 7:
        return [f"关节/控制器限位维度异常: joints={len(joints)} limits={len(limits)}"]
    bad = []
    for axis, (q, pair) in enumerate(zip(joints, limits), 1):
        if len(pair) != 2:
            bad.append(f"j{axis} 控制器限位不是 [lo,hi]")
            continue
        lo, hi = float(pair[0]), float(pair[1])
        if not lo + LIMIT_MARGIN_DEG <= float(q) <= hi - LIMIT_MARGIN_DEG:
            bad.append(
                f"j{axis}={float(q):+.2f}° 不在控制器安全区间 "
                f"[{lo + LIMIT_MARGIN_DEG:+.1f},{hi - LIMIT_MARGIN_DEG:+.1f}]°")
    return bad


def movement_targets(plan: Mapping[str, Any]) -> list[tuple[str, list[float]]]:
    return [
        (stage["name"], _pose6(stage["pose"], f"stages.{stage['name']}.pose"))
        for stage in plan["stages"] if stage["kind"] == "MOVE_WORLDS"
    ]


def precheck_full_path(ss: Any, sdk: Any, plan: Mapping[str, Any]) -> dict[str, Any]:
    """检查 current≈safe、双重限位及每条 MoveWorlds 直线的密集 IK 连续性。"""
    safe = _pose6(plan["resolved"]["safe_endpoint"], "resolved.safe_endpoint")
    current_world = [float(value) for value in ss.read_worlds(sdk, ARM_ID)]
    assert_current_safe(current_world, safe)
    current_joints = [float(value) for value in ss.read_joints(sdk, ARM_ID)]
    if len(current_joints) != 7:
        raise PrecheckError(f"当前关节反馈应为 7 维，实际 {len(current_joints)}")
    try:
        controller_limits = ss.read_axis_limits(sdk, ARM_ID)
    except Exception as exc:  # noqa: BLE001 - 无真限位必须阻断
        raise PrecheckError(f"无法读取控制器真实轴限位: {exc}") from exc
    valid, details = ss.axis_limits_look_valid(controller_limits)
    if not valid:
        raise PrecheckError(f"控制器轴限位读数无效: {details}")

    bad = list(ss.check_limits(
        ARM_ID, current_joints, margin=LIMIT_MARGIN_DEG))
    bad += _controller_limit_violations(current_joints, controller_limits)
    if bad:
        raise PrecheckError("当前关节姿态未通过双重限位检查:\n  " + "\n  ".join(bad))

    previous_pose = current_world
    previous_joints = current_joints
    reports: list[dict[str, Any]] = []
    total_dense = 0
    for segment_name, target in movement_targets(plan):
        dense = densify_line(previous_pose, target)
        maximum_jump = 0.0
        for sample_index, pose in enumerate(dense, 1):
            joints = ss.try_worlds(sdk, ARM_ID, pose)
            if joints is None or len(joints) != 7:
                raise PrecheckError(
                    f"{segment_name} 密集点 {sample_index}/{len(dense)} 不可达或奇异")
            joints = [float(value) for value in joints]
            bad = list(ss.check_limits(
                ARM_ID, joints, margin=LIMIT_MARGIN_DEG))
            bad += _controller_limit_violations(joints, controller_limits)
            if bad:
                raise PrecheckError(
                    f"{segment_name} 密集点 {sample_index}/{len(dense)} 限位失败:\n  " +
                    "\n  ".join(bad))
            joint_deltas = [abs(a - b) for a, b in zip(joints, previous_joints)]
            jump = max(joint_deltas)
            jump_joint = joint_deltas.index(jump)
            maximum_jump = max(maximum_jump, jump)
            if jump > MAX_JOINT_STEP_DEG:
                raise PrecheckError(
                    f"{segment_name} 密集点 {sample_index}/{len(dense)} "
                    f"IK 跳变 J{jump_joint + 1}: "
                    f"{previous_joints[jump_joint]:.3f}° -> "
                    f"{joints[jump_joint]:.3f}° "
                    f"(Δ={jump:.3f}° > {MAX_JOINT_STEP_DEG:.2f}°)\\n"
                    f"  pose={pose}\\n"
                    f"  previous_joints={previous_joints}\\n"
                    f"  current_joints={joints}")
            previous_joints = joints
        total_dense += len(dense)
        reports.append({
            "stage": segment_name,
            "dense_points": len(dense),
            "max_adjacent_joint_step_deg": maximum_jump,
            "target_joints_deg": previous_joints,
        })
        previous_pose = target
    return {
        "current_world": current_world,
        "current_joints_deg": current_joints,
        "controller_limits_deg": [[float(lo), float(hi)]
                                  for lo, hi in controller_limits],
        "dense_points": total_dense,
        "segments": reports,
    }


def build_gripper_frame(gripper_id: int, command: int,
                        values: Sequence[int]) -> bytes:
    data: list[int] = []
    for index, value in enumerate(values):
        number = _uint16(value, f"gripper value #{index + 1}")
        data.extend((number & 0xFF, (number >> 8) & 0xFF))
    payload = [int(gripper_id) & 0xFF, 1 + len(data), int(command) & 0xFF] + data
    checksum = sum(payload) & 0xFF
    return bytes([0xEB, 0x90] + payload + [checksum])


def open_gripper_com(ss: Any, sdk: Any) -> None:
    code = sdk.armOpenCom(GRIPPER_COM, *GRIPPER_SERIAL)
    ss.require_code_zero(code, "armOpenCom(COM1)")


def command_gripper(ss: Any, sdk: Any, plan: Mapping[str, Any], action: str) -> None:
    policy = plan["gripper_policy"]
    if action == "open":
        values = [_uint16(policy["open"]["position"], "open.position")]
        command = 0x11
        settle_s = _settle_seconds(policy["open"]["settle_s"], "open.settle_s")
    elif action == "close":
        values = [
            _uint16(policy["close"]["speed"], "close.speed"),
            _uint16(policy["close"]["force"], "close.force"),
        ]
        command = 0x10
        settle_s = _settle_seconds(policy["close"]["settle_s"], "close.settle_s")
    else:
        raise PlanError(f"未知夹爪动作 {action!r}")
    frame = build_gripper_frame(GRIPPER_ID, command, values)
    print(f"  夹爪 {action}: {frame.hex(' ')}")
    code = sdk.armWriteCom(GRIPPER_COM, frame)
    ss.require_code_zero(code, f"夹爪 {action} armWriteCom")
    if settle_s:
        time.sleep(settle_s)


def execute_stages(ss: Any, sdk: Any, plan: Mapping[str, Any]) -> None:
    """按严格状态机执行；调用者必须已在同一使能会话中完成全路径预检。"""
    safe = _pose6(plan["resolved"]["safe_endpoint"], "resolved.safe_endpoint")
    for stage in plan["stages"]:
        name = stage["name"]
        kind = stage["kind"]
        print(f"[{name}] {kind}")
        if kind == "ASSERT_ENDPOINT":
            assert_current_safe(ss.read_worlds(sdk, ARM_ID), safe)
        elif kind == "GRIPPER":
            command_gripper(ss, sdk, plan, stage["action"])
        elif kind == "MOVE_WORLDS":
            target = _pose6(stage["pose"], f"stages.{name}.pose")
            ss.check_soft_stop(sdk, name)
            ss.move_worlds(sdk, ARM_ID, target, interpolation_en=False)
            ss.wait_until_worlds(
                sdk, ARM_ID, target,
                tol_mm=ARRIVE_TOL_MM,
                tol_deg=ARRIVE_TOL_DEG,
                timeout_s=ARRIVE_TIMEOUT_S,
                label=name)
        else:  # validate_plan 已封死；保留 fail-closed 防未来误改。
            raise PlanError(f"未知 stage kind={kind!r}")


def _open_session(ss: Any, config: Mapping[str, Any], *, enable: bool) -> Any:
    return ss.open_session(
        config["robot_ip"], config["local_ip"], config["arm_ip"],
        config["arm_port"], config["global_speed"], (ARM_ID,), enable=enable)


def run_with_sdk(ss: Any, plan: Mapping[str, Any], config: Mapping[str, Any], *,
                 real_motion: bool, input_fn: Any = input) -> int:
    """可注入 FakeSS 的核心流程；不负责 CLI/env 授权门。"""
    validate_plan(plan)

    # 第一遍永远是只读 live 预检。失败时从未使能、未开串口、未运动。
    readonly_sdk = None
    try:
        readonly_sdk = _open_session(ss, config, enable=False)
        report = precheck_full_path(ss, readonly_sdk, plan)
        print(
            f"只读预检通过：{len(report['segments'])} 条 MoveWorlds 直线，"
            f"{report['dense_points']} 个 armTryWorlds 密集点。")
    finally:
        if readonly_sdk is not None:
            ss.close_session(readonly_sdk)

    if not real_motion:
        print("DRY RUN：只读会话已关闭；零使能、零运动、零夹爪命令。")
        return 0
    answer = input_fn(
        "只读全路径预检已通过。确认现场路径清空并手握急停后，直接回车使能执行；"
        "输入 q 中止: ").strip().lower()
    if answer != "":
        print("操作员未按空回车确认，本次不使能、不执行。")
        return 2

    # 第二遍在实际执行会话中复检，防止两次会话之间机器人位置或控制器状态漂移。
    sdk = None
    try:
        sdk = _open_session(ss, config, enable=True)
        report = precheck_full_path(ss, sdk, plan)
        print(
            f"执行前复检通过：{report['dense_points']} 个密集 IK 点；"
            "现在才打开 COM1。")
        open_gripper_com(ss, sdk)
        execute_stages(ss, sdk, plan)
        print("桌上抓放状态机执行完成，endpoint 已回到计划 safe 点。")
        return 0
    except BaseException:
        if sdk is not None:
            try:
                ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
                print("异常中止：已请求立即清除控制器残留路径。")
            except Exception as stop_exc:  # noqa: BLE001 - 不掩盖原异常
                print(f"警告：清除控制器路径失败，请人工急停确认: {stop_exc}")
        print("异常后未自动松开夹爪；请现场确认物体状态后人工处理。")
        raise
    finally:
        if sdk is not None:
            ss.close_session(sdk)


def _runtime_config(args: argparse.Namespace, environ: Mapping[str, str]) -> dict[str, Any]:
    robot_ip = args.robot_ip or environ.get("XIFENG_ROBOT_IP")
    local_ip = args.local_ip or environ.get("XIFENG_LOCAL_IP")
    arm_ip = args.arm_ip or environ.get("XIFENG_ARM_IP") or robot_ip
    port_raw: Any = args.arm_port
    if port_raw is None:
        port_raw = environ.get("XIFENG_ARM_PORT", "8080")
    try:
        arm_port = int(port_raw)
    except (TypeError, ValueError) as exc:
        raise PlanError("XIFENG_ARM_PORT/--arm-port 必须是整数") from exc
    if not robot_ip or not local_ip or not arm_ip:
        raise PlanError(
            "必须通过环境或 CLI 提供 XIFENG_ROBOT_IP、XIFENG_LOCAL_IP；"
            "XIFENG_ARM_IP 未给时跟随 ROBOT_IP")
    if not 1 <= arm_port <= 65535:
        raise PlanError("ARM_PORT 必须在 [1,65535]")
    speed = float(args.speed)
    if not math.isfinite(speed) or not 0.1 <= speed <= 20.0:
        raise PlanError("--speed 必须在 [0.1,20.0]%")
    return {
        "robot_ip": robot_ip,
        "local_ip": local_ip,
        "arm_ip": arm_ip,
        "arm_port": arm_port,
        "global_speed": speed,
    }


def _print_hashes(plan_path: Path, plan_sha256: str, ss: Any) -> None:
    executor_path = Path(__file__).resolve()
    sdk_path_raw = getattr(ss, "__file__", None)
    print(f"executor SHA-256: {_sha256_path(executor_path)}  {executor_path}")
    print(f"plan SHA-256:     {plan_sha256}  {plan_path.resolve()}")
    if sdk_path_raw:
        sdk_path = Path(sdk_path_raw).resolve()
        print(f"sdk_session SHA-256: {_sha256_path(sdk_path)}  {sdk_path}")
    else:
        print("sdk_session SHA-256: <injected test double>")


def main(argv: Sequence[str] | None = None, *, ss_module: Any = None,
         lock_module: Any = None, environ: Mapping[str, str] | None = None,
         input_fn: Any = input) -> int:
    parser = argparse.ArgumentParser(
        description="tabletop_plan.v1 左臂 MoveWorlds 执行器（默认只读 live 预检）")
    parser.add_argument("plan", help="tabletop_plan.v1 JSON")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--run", action="store_true",
                      help="请求真机执行；仍需环境门和人工确认")
    mode.add_argument("--precheck-only", action="store_true",
                      help="显式只读 live 预检（默认模式；不使能、不运动、不操作夹爪）")
    parser.add_argument("--robot-ip", help="覆盖 XIFENG_ROBOT_IP")
    parser.add_argument("--local-ip", help="覆盖 XIFENG_LOCAL_IP")
    parser.add_argument("--arm-ip", help="覆盖 XIFENG_ARM_IP；默认跟随 ROBOT_IP")
    parser.add_argument("--arm-port", type=int, help="覆盖 XIFENG_ARM_PORT（默认 8080）")
    parser.add_argument("--speed", type=float, default=GLOBAL_SPEED,
                        help="真机全局速度百分比，范围 [0.1,5.0]，默认 1.0")
    args = parser.parse_args(argv)
    env = os.environ if environ is None else environ

    try:
        plan, plan_sha256 = load_plan(args.plan)
        gate_error = run_gate_error(
            plan, args.run, env.get("XIFENG_ALLOW_REAL_MOTION") == "1")
        if gate_error:
            print(f"拒绝真机执行：{gate_error}")
            return 2
        config = _runtime_config(args, env)
    except (OSError, PlanError) as exc:
        print(f"计划/配置错误：{exc}")
        return 2

    if ss_module is None:
        import sdk_session as ss_module
    if lock_module is None:
        import robot_lock as lock_module
    _print_hashes(Path(args.plan), plan_sha256, ss_module)
    print(
        f"transport: robot={config['robot_ip']} local={config['local_ip']} "
        f"arm={config['arm_ip']}:{config['arm_port']}  ARM_ID={ARM_ID}")
    print(f"mode: {'REAL MOVE REQUESTED' if args.run else 'READ-ONLY LIVE PRECHECK'}")
    with lock_module.acquire(
            config["robot_ip"], ARM_ID,
            "execute_tabletop_pick_place_worlds.py"):
        return run_with_sdk(
            ss_module, plan, config, real_motion=args.run, input_fn=input_fn)


if __name__ == "__main__":
    sys.exit(main())
