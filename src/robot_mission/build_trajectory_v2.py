#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import uuid
from pathlib import Path

import arm_profiles
from .contracts import (canonical_bytes, load_and_validate, sha256_bytes,
                        sha256_file, validate_document)
from .preflight import expanded_frames, load_trajectory

WORK = Path(__file__).resolve().parent.parent


def main(argv=None):
    ap = argparse.ArgumentParser(description="把通过离线资格门的轨迹封装成 trajectory.v2")
    ap.add_argument("legacy_trajectory"); ap.add_argument("--observation", required=True)
    ap.add_argument("--task", required=True, help="原始 task_request.v1")
    ap.add_argument("--adapted-task", required=True,
                    help="实际送入规划器的 pairs+obstacles JSON")
    ap.add_argument("--calibration", required=True)
    ap.add_argument("--qualification", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--robot-config-identity", required=True)
    a = ap.parse_args(argv)
    obs = load_and_validate(a.observation, "vision_observation.v1")
    task = load_and_validate(a.task, "task_request.v1")
    calibration = load_and_validate(a.calibration, "eye_to_hand_calibration.v1")
    qualification = json.loads(Path(a.qualification).read_text(encoding="utf-8"))
    if qualification.get("verdict") != "PASS" or qualification.get("qualification") != "CANDIDATE":
        raise SystemExit("轨迹资格报告不是 PASS/CANDIDATE")
    if qualification["trajectory_sha256"] != sha256_file(a.legacy_trajectory):
        raise SystemExit("资格报告未绑定当前 legacy trajectory")
    if qualification.get("task_sha256") != sha256_file(a.adapted_task):
        raise SystemExit("资格报告未绑定当前适配后的 pairs+obstacles 输入")
    if task["observation"]["sha256"] != sha256_file(a.observation):
        raise SystemExit("task_request 未绑定当前 observation")
    meta, waypoints, _ = load_trajectory(a.legacy_trajectory)
    profile = arm_profiles.arm_profile(meta["arm"])
    if calibration["arm"] != meta["arm"]:
        raise SystemExit("手眼标定 arm 与轨迹 arm 不一致")
    _, dense = expanded_frames(waypoints, profile["home_deg"], .4)
    object_ids = [x["object_id"] for x in task["actions"]]
    enhanced = []
    for index, waypoint in enumerate(waypoints):
        enhanced.append(waypoint)
        if "q_sdk_deg" not in waypoint:
            continue
        seg = waypoint.get("seg", "")
        next_seg = (waypoints[index + 1].get("seg", "")
                    if index + 1 < len(waypoints) else "")
        if next_seg == seg:
            continue
        for n, object_id in enumerate(object_ids, 1):
            if seg == f"PICK{n}_ASCEND":
                enhanced.append({"seg": f"PICK{n}_VISION_VERIFY",
                                 "vision_check": {"kind": "grasp_follow",
                                                  "object_id": object_id,
                                                  "max_error_m": .02}})
            if seg == f"PLACE{n}_ASCEND":
                enhanced.append({"seg": f"PLACE{n}_VISION_VERIFY",
                                 "vision_check": {"kind": "placement",
                                                  "object_id": object_id,
                                                  "max_error_m": .02}})
    dependencies = [WORK / "arm_profiles.v1.json",
                    WORK / "pick_place_coord/pick_place_coord.py",
                    WORK / "pick_place_coord/left_arm_ik.py"]
    dep_hash = sha256_bytes(canonical_bytes({str(p.relative_to(WORK)): sha256_file(p)
                                             for p in dependencies}))
    collision_hash = sha256_file(a.qualification)
    status = ("REAL_VERIFIED_PARENT" if profile["status"] == "REAL_VERIFIED_PARENT" and
              sha256_file(a.legacy_trajectory) == profile.get("track_c_parent_sha256")
              else "CANDIDATE")
    payload = {"schema_version": "trajectory.v2", "trajectory_id": str(uuid.uuid4()),
        "contract": {"arm": meta["arm"], "arm_id": profile["arm_id"],
                     "gripper_id": profile["gripper_id"], "ee_link_id": profile["ee_link_id"],
                     "tcp_id": calibration["input_conventions"]["tcp_id"],
                     "joint_angle_unit": "degree",
                     "sdk_from_urdf_sign": profile["sdk_from_urdf_sign"],
                     "executor": "track_c_armPluseToServo",
                     "robot_config_identity": a.robot_config_identity,
                     "qualification_status": status},
        "provenance": {"observation_sha256": sha256_file(a.observation),
                       "task_sha256": sha256_file(a.task),
                       "adapted_task_sha256": sha256_file(a.adapted_task),
                       "planner_id": "pick_place_coord+trajectory_qualification.v1",
                       "calibration_id": calibration["calibration_id"],
                       "dependencies_sha256": dep_hash,
                       "derived_trajectory_sha256": sha256_bytes(canonical_bytes(enhanced))},
        "pulse": {"step_deg": .4, "period_ms": 20, "dense_frame_count": len(dense)},
        "safety": {"minimum_joint_margin_deg": qualification["limit_report"]["minimum_margin_deg"],
                   "collision_report_sha256": collision_hash,
                   "first_point_checked": True, "carried_object_checked": True},
        "waypoints": enhanced}
    validate_document(payload, "trajectory.v2")
    Path(a.out).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"trajectory.v2 {status} dense={len(dense)} -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
