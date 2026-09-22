"""[20260909] Track A -> Pulse Servo 契约；纯离线计算，不连接 SDK。

关节符号不再转换、EE 不再补偿。沿父文件的关节折线细分，不拟合新空间曲线。
安全范围取共享配置与 sdk_session 的交集，Home 取父轨迹，不用另一线路 HOME。
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import arm_profiles
import sdk_session as ss

SCHEMA = "tabletop_servo_candidate.v1"
SOURCE_SHA = "e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59"
MOTIONS = ("PICK_HOVER", "PICK_DESCEND", "PICK_ASCEND", "PLACE_HOVER",
           "PLACE_DESCEND", "PLACE_ASCEND", "RETURN_SAFE")
EVENTS = (("OPEN_BEFORE_PICK", "open"), ("CLOSE_AT_PICK", "close"),
          ("OPEN_AT_PLACE", "open"))


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 7 or
            any(isinstance(v, bool) or not isinstance(v, (int, float)) or
                not math.isfinite(v) for v in values)):
        raise ValueError("需要恰好七个有限关节角，单位 SDK deg")
    return [float(v) for v in values]


def policy():
    shared = arm_profiles.arm_profile("left")["shared"]
    period = shared["pulse_period_ms"] / 1000.0
    return {"period_s": period,
            "max_step_deg": shared["pulse_step_deg"] / 4.0,
            "max_velocity_deg_s": shared["pulse_step_deg"] / period,
            "max_acceleration_deg_s2": shared["pulse_step_deg"] / period**2,
            "max_initial_gap_deg": shared["pulse_step_deg"],
            "max_global_speed_percent": 5.0,
            "margin_deg": max(shared["minimum_joint_margin_deg"], ss.LIMIT_MARGIN_DEG),
            "limits_basis": "SHARED_COMMAND_ENVELOPE_NOT_ROBOT_ACCELERATION_RATING"}


def limits():
    return ss.intersect_axis_limits(
        arm_profiles.arm_profile("left")["controller_limits_deg"], ss.LIMITS_DEG[1])


def motion_metrics(waypoints, period):
    motions = [w for w in waypoints if "q_sdk_deg" in w]
    qs = [vector(w["q_sdk_deg"]) for w in motions]
    velocities, accelerations = [], []
    prev_q, prev_v = qs[0], [0.0] * 7
    for w in waypoints[1:]:
        q = vector(w["q_sdk_deg"]) if "q_sdk_deg" in w else prev_q
        v = [(b-a)/period for a,b in zip(prev_q, q)]
        velocities.append(v)
        accelerations.append([(b-a)/period for a,b in zip(prev_v, v)])
        prev_q, prev_v = q, v
    accelerations.append([-v/period for v in prev_v])
    ranges = [[min(q[j] for q in qs), max(q[j] for q in qs)] for j in range(7)]
    margins = [min(r[0]-lo, hi-r[1]) for r, (lo,hi) in zip(ranges, limits())]
    return {"motion_frame_count": len(qs),
            "stream_duration_s": len(qs)*period,
            "max_step_deg": max((max(abs(a-b) for a,b in zip(x,y))
                                 for x,y in zip(qs, qs[1:])), default=0.0),
            "max_velocity_deg_s": max((max(map(abs,v)) for v in velocities), default=0.0),
            "max_acceleration_deg_s2": max((max(map(abs,a)) for a in accelerations), default=0.0),
            "joint_used_ranges_deg": ranges, "minimum_margin_per_joint_deg": margins,
            "timing_note": "离散指令差分，不是实际反馈速度/加速度；停稳等待另计"}


def build(parent_path):
    if digest(parent_path) != SOURCE_SHA:
        raise ValueError("父轨迹不是已收到的 SDKALIGNED 实跑文件；不隐式更换来源")
    source = json.loads(Path(parent_path).read_text())
    if source["contract"]["arm_id"] != 1 or source["contract"]["gripper_id"] != 2:
        raise ValueError("只支持本次已确认的左臂/夹爪2")
    conf = policy()
    output, prev, held = [], None, False
    for f in source["offline_replay"]["frames"]:
        if "action" in f:
            held = f["action"] == "close"
            output.append({"stage": f["stage"], "gripper": f["action"],
                           "action": f["action"], "carrying_after": held,
                           "settle_s": source["gripper_policy"][f["action"]]["settle_s"]})
            continue
        q = vector(f["q_sdk_deg"])
        count = (max(1, math.ceil(max(abs(a-b) for a,b in zip(prev,q))/conf["max_step_deg"]))
                 if prev is not None else 1)
        for k in range(1,count+1):
            sample = q[:] if k == count else [a+(b-a)*k/count for a,b in zip(prev,q)]
            output.append({"stage": f["stage"], "q_sdk_deg": sample, "carrying": held})
        prev = q
    reference = {s["name"]: s for s in source["stages"]}
    for name in MOTIONS:
        rows = [w for w in output if w["stage"] == name and "q_sdk_deg" in w]
        if not rows:
            raise ValueError(f"缺少来源阶段 {name}")
        if "q_sdk_deg" in reference[name] and rows[-1]["q_sdk_deg"] != reference[name]["q_sdk_deg"]:
            # 最后一项使用原目标，消除线性计算的浮点舍入，禁止改关节构型。
            if max(abs(a-b) for a,b in zip(rows[-1]["q_sdk_deg"],reference[name]["q_sdk_deg"])) > 1e-9:
                raise ValueError(f"{name} 回放与实际 MoveJ 目标不一致")
            rows[-1]["q_sdk_deg"] = list(reference[name]["q_sdk_deg"])
    result = {"schema_version": SCHEMA, "status": "CANDIDATE_REQUIRES_SERVO_QUALIFICATION",
              "real_motion_authorized": False,
              "meta": {"arm_id": 1, "gripper_id": 2, "arm": "left", "j6_flipped": True,
                       "source_plan_sha256": SOURCE_SHA, "source_filename": Path(parent_path).name,
                       "generator": "gen_tabletop_hybrid_candidate.py --servo-parent",
                       "coordinate_semantics": "ALREADY_COMPENSATED_SDK_EE_NO_SECOND_TCP_TRANSFORM",
                       "path_basis": "SDK_QUERY_KNOTS_NOT_MEASURED_SERVO_MOTION",
                       "arm_profiles_sha256": digest(arm_profiles._config_path()),
                       "sdk_session_sha256": digest(ss.__file__)},
              "stream_policy": conf, "gripper_policy": copy.deepcopy(source["gripper_policy"]),
              "first_point_q_sdk_deg": output[0]["q_sdk_deg"],
              "home_joints_sdk_deg": list(source["home_joints_sdk_deg"]),
              "startup_policy": "REQUIRE_MATCHED_STOPPED_START_NO_AUTOMATIC_MOVEJ",
              "source_stages": copy.deepcopy(source["stages"]),
              "waypoints": output,
              "qualification": {"verdict": "BLOCKED", "gui_review": "PENDING",
                                "swept_collision": "NOT_QUALIFIED_FOR_SERVO",
                                "fk_path": "SDK_REFERENCE_NOT_PHYSICAL_SCENE_CALIBRATION",
                                "source_qualification": copy.deepcopy(source["qualification"]),
                                "blockers": ["NEW_SERVO_GUI_REVIEW_REQUIRED",
                                             "DENSE_FULL_SCENE_AND_HELD_OBJECT_QUALIFICATION_REQUIRED",
                                             "SERVO_DYNAMIC_LIMITS_AND_TIMING_QUALIFICATION_REQUIRED",
                                             "MATCHED_LIVE_START_AND_RUNTIME_PRECHECK_REQUIRED"]}}
    result["kinematics"] = validate(result)
    return result


def validate(plan, live_limits=None):
    if plan.get("schema_version") != SCHEMA or plan.get("real_motion_authorized") is not False:
        raise ValueError("需要 Servo 候选契约；运动资格由独立 SHA 绑定报告给出")
    meta = plan["meta"]
    if (type(meta.get("arm_id")) is not int or type(meta.get("gripper_id")) is not int
            or (meta.get("arm_id"), meta.get("gripper_id")) != (1,2)
            or meta.get("j6_flipped") is not True):
        raise ValueError("arm/gripper/SDK关节符号契约不一致")
    if (meta.get("coordinate_semantics") != "ALREADY_COMPENSATED_SDK_EE_NO_SECOND_TCP_TRANSFORM"
            or plan.get("startup_policy") != "REQUIRE_MATCHED_STOPPED_START_NO_AUTOMATIC_MOVEJ"):
        raise ValueError("坐标或启动契约不一致")
    if meta.get("source_plan_sha256") != SOURCE_SHA:
        raise ValueError("父轨迹身份不一致")
    if meta.get("arm_profiles_sha256") != digest(arm_profiles._config_path()):
        raise ValueError("共享手臂配置改变，需重建并重新审阅")
    if meta.get("sdk_session_sha256") != digest(ss.__file__):
        raise ValueError("SDK辅助依赖改变，需重新审阅")
    conf = plan["stream_policy"]
    if conf != policy():
        raise ValueError("下发策略与共享策略不一致，不接受运行时加速/改步长")
    wps = plan["waypoints"]
    if not isinstance(wps,list) or not wps or "q_sdk_deg" not in wps[0]:
        raise ValueError("缺少起始关节点")
    phases = []
    timeline = []
    events = []
    carrying = False
    for w in wps:
        if ("q_sdk_deg" in w) == ("gripper" in w):
            raise ValueError("每项只能是关节点或夹爪事件")
        if "q_sdk_deg" in w:
            vector(w["q_sdk_deg"])
            if w.get("carrying") is not carrying:
                raise ValueError("持物状态与开闭顺序不一致")
            if not phases or phases[-1] != w["stage"]: phases.append(w["stage"])
        else:
            events.append((w["stage"], w["gripper"]))
            if w.get("action") != w["gripper"] or w.get("settle_s") != plan["gripper_policy"][w["gripper"]]["settle_s"]:
                raise ValueError("夹爪事件/等待与策略不一致")
            carrying = w["gripper"] == "close"
            if w.get("carrying_after") is not carrying:
                raise ValueError("夹爪动作后的持物状态不一致")
        if not timeline or timeline[-1] != w["stage"]: timeline.append(w["stage"])
    if tuple(phases[1:]) != MOTIONS or tuple(events) != EVENTS:
        raise ValueError("运动阶段或夹爪开闭顺序不完整")
    expected = [phases[0], "OPEN_BEFORE_PICK", "PICK_HOVER", "PICK_DESCEND",
                "CLOSE_AT_PICK", "PICK_ASCEND", "PLACE_HOVER", "PLACE_DESCEND",
                "OPEN_AT_PLACE", "PLACE_ASCEND", "RETURN_SAFE"]
    if timeline != expected or len([w for w in wps if w["stage"] == phases[0]]) != 1:
        raise ValueError("夹爪事件未绑定到正确运动阶段")
    grip = plan["gripper_policy"]
    if (grip.get("gripper_id") != 2 or grip.get("open_command") != 0x11
            or grip.get("close_command") != 0x10):
        raise ValueError("夹爪协议/设备标识不一致")
    for action, keys in (("open", ("position",)), ("close", ("speed", "force"))):
        for key in keys:
            value = grip[action][key]
            if isinstance(value,bool) or not isinstance(value,int) or not 0 <= value <= 65535:
                raise ValueError("夹爪原生值必须为 uint16")
        settle = grip[action]["settle_s"]
        if isinstance(settle,bool) or not isinstance(settle,(float,int)) or not math.isfinite(settle) or not 0 <= settle <= 30:
            raise ValueError("夹爪等待时间无效")
    if vector(wps[0]["q_sdk_deg"]) != vector(plan["first_point_q_sdk_deg"]):
        raise ValueError("首点契约不一致")
    if vector(wps[-1]["q_sdk_deg"]) != vector(plan["home_joints_sdk_deg"]):
        raise ValueError("终点不是原固定 Home")
    effective = limits() if live_limits is None else ss.intersect_axis_limits(limits(), live_limits)
    for w in wps:
        if "q_sdk_deg" not in w: continue
        if any(q < lo+conf["margin_deg"]-1e-8 or q > hi-conf["margin_deg"]+1e-8
               for q,(lo,hi) in zip(w["q_sdk_deg"],effective)):
            raise ValueError(f"{w['stage']} 超出有效关节限位/内缩余量")
    result = motion_metrics(wps,conf["period_s"])
    if "kinematics" in plan and plan["kinematics"] != result:
        raise ValueError("关节数组与已记录的运动指标不一致；需重新生成")
    for key in ("max_step_deg", "max_velocity_deg_s", "max_acceleration_deg_s2"):
        if result[key] > conf[key]+1e-8: raise ValueError(f"{key} 超限")
    return result
