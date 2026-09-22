#!/usr/bin/env python3
from __future__ import annotations

import argparse
import copy
import json
import math
import sys
from pathlib import Path

WORK = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORK))
import arm_profiles
from .contracts import sha256_file
from .preflight import (collision_check, expanded_frames, limit_check,
                        grasp_event_qs, load_trajectory)


def summarize_grasp_precheck(collision: dict) -> dict:
    """把闭爪姿态的几何结果拆成可操作的失败原因。

    这只是无运动的离线几何预检，不替代 GUI、控制器限位回读或真机分段试跑。
    """
    details = collision.get("details", {})
    captures = details.get("grasp_capture_checks", [])
    if not captures:
        return {"status": "UNRESOLVED", "failure_mode": "MISSING_CLOSE_CAPTURE_CHECK",
                "captures": []}

    results = []
    overall = "PASS"
    modes = set()
    for index, capture in enumerate(captures):
        capture_status = capture.get("status", "UNRESOLVED")
        environment = capture.get("environment_clearance_at_close", {})
        environment_status = environment.get("status", "UNRESOLVED")
        reasons = []
        if capture.get("grasp_point_inside_object") is False:
            reasons.append("GRASP_POINT_OUTSIDE_BOTTLE")
        axial = capture.get("axis_offset_m")
        axial_range = capture.get("axis_range_m")
        lateral = capture.get("lateral_error_m")
        lateral_limit = capture.get("lateral_tolerance_m")
        if (isinstance(axial_range, list) and len(axial_range) == 2 and
                isinstance(axial, (int, float))):
            if axial < axial_range[0]:
                reasons.append("BOTTLE_BEFORE_CAPTURE_ZONE")
            elif axial > axial_range[1]:
                reasons.append("BOTTLE_PAST_CAPTURE_ZONE")
        if (isinstance(lateral, (int, float)) and
                isinstance(lateral_limit, (int, float)) and lateral > lateral_limit):
            reasons.append("LATERAL_MISS")
        if capture_status != "PASS":
            modes.add("BOTTLE_NOT_IN_GRASP_ZONE")
        if environment_status == "BLOCKED":
            modes.add("ARM_OR_GRIPPER_TOO_LOW_AT_CLOSE")
        if capture_status == "UNRESOLVED" or environment_status == "UNRESOLVED":
            overall = "UNRESOLVED"
        elif capture_status != "PASS" or environment_status != "PASS":
            overall = "BLOCKED"
        results.append({
            "grasp_index": index,
            "capture_status": capture_status,
            "axis_offset_mm": None if axial is None else float(axial) * 1000.0,
            "allowed_axis_range_mm": (None if not isinstance(axial_range, list)
                                       else [float(v) * 1000.0 for v in axial_range]),
            "lateral_error_mm": None if lateral is None else float(lateral) * 1000.0,
            "allowed_lateral_error_mm": (None if lateral_limit is None
                                          else float(lateral_limit) * 1000.0),
            "capture_failure_reasons": reasons,
            "environment_at_close": environment,
        })
    failure_mode = "NONE" if not modes else "+".join(sorted(modes))
    return {"status": overall, "failure_mode": failure_mode, "captures": results}


def main(argv=None):
    ap = argparse.ArgumentParser(description="轨迹候选离线资格门；绝不提升 REAL_VERIFIED")
    ap.add_argument("trajectory"); ap.add_argument("--task", required=True)
    ap.add_argument("--left-q", type=float, nargs=7, required=True)
    ap.add_argument("--right-q", type=float, nargs=7, required=True)
    ap.add_argument("--gui-review",
                    help="PyBullet GUI 人工回放记录；候选要求 GUI 时必须提供 PASS 记录")
    ap.add_argument("--geometry-precheck-only", action="store_true",
                    help="只返回无运动几何预检状态；不因缺 GUI 记录而返回失败")
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    meta, waypoints, schema = load_trajectory(a.trajectory)
    profile = arm_profiles.arm_profile(meta["arm"])
    inactive = arm_profiles.arm_profile("right" if meta["arm"] == "left" else "left")
    current = {"left": a.left_q, "right": a.right_q}
    first, frames = expanded_frames(waypoints, current[meta["arm"]], .4)
    sequence = [x["q_sdk_deg"] for x in frames]
    path_length_rad = sum(math.sqrt(sum(math.radians(y-x) ** 2 for x, y in zip(a0, b0)))
                          for a0, b0 in zip(sequence, sequence[1:]))
    task = json.loads(Path(a.task).read_text(encoding="utf-8"))
    dimensions = [pair.get("dimensions_m", [.05, .05, .05]) for pair in task["pairs"]]
    offline_profile = copy.deepcopy(profile)
    limits_source = profile["controller_limits_status"]
    limit_profile_id = meta.get("controller_limit_profile")
    if limit_profile_id:
        evidence = arm_profiles.controller_limit_profile(limit_profile_id)
        if (evidence["arm"], evidence["arm_id"]) != (meta["arm"], meta["arm_id"]):
            raise ValueError("轨迹 controller_limit_profile 与 arm 契约不一致")
        offline_profile["controller_limits_deg"] = evidence["planning_limits_deg"]
        limits_source = f"controller_limit_profiles.{limit_profile_id}.planning_limits_deg"
    elif offline_profile.get("controller_limits_deg") is None:
        offline_profile["controller_limits_deg"] = offline_profile["candidate_nominal_urdf_limits_deg"]
        limits_source = "NOMINAL_URDF_CANDIDATE_ONLY"
    limits = limit_check(offline_profile, first, frames, 5.0)
    collision = collision_check(offline_profile, inactive,
                                current[inactive["arm"]], first, frames, task, dimensions,
                                grasp_event_qs(waypoints))
    identity = (meta["arm_id"] == profile["arm_id"] and
                meta["gripper_id"] == profile["gripper_id"])
    gui_required = bool(meta.get("gui_review", {}).get("required"))
    gui_status, gui_details = ("PASS", {"required": False})
    if gui_required:
        gui_status, gui_details = "BLOCKED", {"required": True, "reason": "record_missing"}
        if a.gui_review:
            review = json.loads(Path(a.gui_review).read_text(encoding="utf-8"))
            expected_sha = sha256_file(a.trajectory)
            valid = (review.get("schema_version") == "pybullet_gui_review.v1" and
                     review.get("trajectory_sha256") == expected_sha and
                     review.get("qualification_task_sha256") == sha256_file(a.task) and
                     review.get("dense_step_deg") == .4 and
                     review.get("result") == "PASS")
            gui_status = "PASS" if valid else "BLOCKED"
            gui_details = {"required": True, "record": str(Path(a.gui_review).resolve()),
                           "record_sha256": sha256_file(a.gui_review),
                           "trajectory_hash_match": review.get("trajectory_sha256") == expected_sha,
                           "result": review.get("result")}
    gates = {"arm_gripper_contract": "PASS" if identity else "BLOCKED",
             "limits": limits["status"], "collision": collision["status"],
             "pybullet_gui_review": gui_status}
    geometry_gates = {key: gates[key]
                      for key in ("arm_gripper_contract", "limits", "collision")}
    geometry_verdict = ("PASS" if all(value == "PASS"
                                      for value in geometry_gates.values()) else "BLOCKED")
    grasp_precheck = summarize_grasp_precheck(collision)
    verdict = "PASS" if all(x == "PASS" for x in gates.values()) else "BLOCKED"
    report = {"schema_version": "trajectory_qualification.v1",
              "qualification": "CANDIDATE" if verdict == "PASS" else "BLOCKED",
              "verdict": verdict, "trajectory_sha256": sha256_file(a.trajectory),
              "trajectory_path": str(Path(a.trajectory).resolve()),
              "task_sha256": sha256_file(a.task), "trajectory_schema": schema,
              "arm": meta["arm"], "arm_id": meta["arm_id"],
              "gripper_id": meta["gripper_id"], "limits_source": limits_source,
              "gui_review": gui_details,
              "dense_frames": len(frames), "first_point_frames": len(first),
              "path_length_rad": path_length_rad,
              "gates": gates, "limit_report": limits["details"],
              "collision_report": collision["details"],
              "geometry_precheck_verdict": geometry_verdict,
              "grasp_precheck": grasp_precheck,
              "hardware_ready": "BLOCKED" not in profile["status"]}
    Path(a.out).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if a.geometry_precheck_only:
        console_report = {
            "mode": "GEOMETRY_PRECHECK_ONLY_NO_ROBOT_MOTION",
            "geometry_precheck_verdict": geometry_verdict,
            "arm": report["arm"],
            "dense_frames": report["dense_frames"],
            "geometry_gates": geometry_gates,
            "grasp_precheck": grasp_precheck,
            "note": "GUI review is deliberately outside this no-motion precheck",
        }
    else:
        console_report = {k: report[k] for k in (
            "qualification", "verdict", "geometry_precheck_verdict", "arm",
            "dense_frames", "gates", "grasp_precheck")}
    print(json.dumps(console_report, ensure_ascii=False, indent=2))
    effective_verdict = geometry_verdict if a.geometry_precheck_only else verdict
    return 0 if effective_verdict == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
