#!/usr/bin/env python3
from __future__ import annotations

import json
import argparse
from pathlib import Path

import numpy as np

from .contracts import load_and_validate, validate_task_links
from .grasp_pose import matrix_to_quaternion_xyzw, quaternion_xyzw_to_matrix


def _transform_point(T: np.ndarray, p) -> np.ndarray:
    return (T @ np.r_[np.asarray(p, float), 1.0])[:3]


def _sdk_to_pb(p_sdk_m, t_session_mm) -> np.ndarray:
    return np.asarray(p_sdk_m, float) - np.asarray(t_session_mm, float) / 1000.0


def _entity_pose_in_sdk_world(entity: dict, T_sdk_world_camera: np.ndarray,
                              camera_frame_id: str) -> tuple[np.ndarray, list[float], str]:
    """把 entity 整姿态变到 SDK world，且保证最多只做一次外参变换。"""
    local = np.eye(4)
    local[:3, :3] = quaternion_xyzw_to_matrix(
        entity["pose"]["quaternion_xyzw"])
    local[:3, 3] = np.asarray(entity["pose"]["position_m"], float)
    frame_id = entity["frame_id"]
    if frame_id == "sdk_world":
        world = local
        route = "SDK_WORLD_BYPASS_EXTRINSICS"
    elif frame_id == camera_frame_id:
        world = np.asarray(T_sdk_world_camera, float) @ local
        route = "CAMERA_TO_SDK_WORLD_ONCE"
    else:
        raise ValueError(
            f"entity.frame_id={frame_id!r} 既不是 sdk_world，也不是相机帧 "
            f"{camera_frame_id!r}")
    return (world[:3, 3],
            [float(v) for v in matrix_to_quaternion_xyzw(world[:3, :3])], route)


def _oriented_aabb_dimensions(dimensions, world_quaternion_xyzw):
    dims = np.asarray(dimensions, dtype=float)
    if dims.shape != (3,) or not np.all(np.isfinite(dims)) or np.any(dims <= 0):
        raise ValueError("AABB dimensions 必须是 3 个正有限数值")
    return np.abs(quaternion_xyzw_to_matrix(world_quaternion_xyzw)) @ dims


def adapt_legacy_file(path: str | Path) -> tuple[dict, dict]:
    raw = json.loads(Path(path).read_text(encoding="utf-8"))
    pairs = [{"pick": [float(v) for v in x["pick"]],
              "place": [float(v) for v in x["place"]]} for x in raw["pairs"]]
    obstacles = [{"center": [float(v) for v in x["center"]],
                  "half": [float(v) for v in x["half"]]} for x in raw.get("obstacles", [])]
    return {"pairs": pairs, "obstacles": obstacles}, {"adapter": "legacy_pairs_obstacles.v1"}


def observation_task_to_legacy(observation_path: str | Path, task_path: str | Path,
                               calibration_path: str | Path,
                               t_session_mm: list[float]) -> tuple[dict, dict]:
    obs = load_and_validate(observation_path, "vision_observation.v1")
    task = load_and_validate(task_path, "task_request.v1")
    calib = load_and_validate(calibration_path, "eye_to_hand_calibration.v1")
    validate_task_links(task, obs, observation_path)
    if obs["calibration"]["extrinsics_id"] != calib["calibration_id"]:
        raise ValueError("observation.extrinsics_id 与手眼标定不一致")
    T = np.asarray(calib["T_sdk_world_camera"], float)
    camera_frame_id = obs["camera"]["frame_id"]
    by_id = {o["object_id"]: o for o in obs["objects"]}
    pairs = []
    routes = []
    for action in task["actions"]:
        obj = by_id[action["object_id"]]
        pick_sdk, pick_quat, route = _entity_pose_in_sdk_world(
            obj, T, camera_frame_id)
        routes.append(route)
        place = action["place"]
        if place["frame_id"] == "sdk_world":
            place_pb = _sdk_to_pb(place["position_m"], t_session_mm)
        elif place["frame_id"] == "pb_world":
            place_pb = np.asarray(place["position_m"], float)
        else:
            raise ValueError(f"首版 place.frame_id 只接受 sdk_world/pb_world: {place['frame_id']}")
        pairs.append({"pick": _sdk_to_pb(pick_sdk, t_session_mm).tolist(),
                      "place": place_pb.tolist(), "object_id": obj["object_id"],
                      "dimensions_m": obj["dimensions_m"],
                      "object_pose_sdk_world": {
                          "position_m": [float(v) for v in pick_sdk],
                          "quaternion_xyzw": pick_quat,
                          "pose_role": obj.get("pose_role", "OBJECT_POSE")}})
    obstacles = []
    for item in obs["obstacles"]:
        center_sdk, world_quat, route = _entity_pose_in_sdk_world(
            item, T, camera_frame_id)
        routes.append(route)
        sigma = max(float(x) for x in item["uncertainty_1sigma_m"])
        inflation = max(task["safety_policy"]["obstacle_inflation_min_m"],
                        task["safety_policy"]["obstacle_sigma_multiplier"] * sigma)
        # 旋转盒在 PB 入口处取保守轴对齐包围盒；必须包含实体自身世界姿态。
        aabb_dims = _oriented_aabb_dimensions(item["dimensions_m"], world_quat)
        obstacles.append({"center": _sdk_to_pb(center_sdk, t_session_mm).tolist(),
                          "half": (aabb_dims / 2.0 + inflation).tolist(),
                          "source_obstacle_id": item["obstacle_id"],
                          "inflation_m": inflation})
    trace = {"adapter": "vision_task_to_legacy.v1", "calibration_id": calib["calibration_id"],
             "t_session_mm": list(map(float, t_session_mm)), "arm_preference": task["arm_preference"],
             "execution_policy": task["execution_policy"],
             "pose_routes": {name: routes.count(name) for name in sorted(set(routes))},
             "warning": "legacy pick 仍是物体中心；桌上抓取必须再走 grasp_pose endpoint 补偿"}
    return {"pairs": pairs, "obstacles": obstacles}, trace


def main(argv=None):
    ap = argparse.ArgumentParser(description="视觉契约入口适配为现有 pairs+obstacles")
    ap.add_argument("--observation", required=True); ap.add_argument("--task", required=True)
    ap.add_argument("--calibration", required=True); ap.add_argument("--frame-gate", required=True)
    ap.add_argument("--out", required=True); ap.add_argument("--trace-out", required=True)
    a = ap.parse_args(argv)
    frame = json.loads(Path(a.frame_gate).read_text(encoding="utf-8"))
    if frame.get("verdict") != "PASS":
        raise SystemExit("PB↔SDK frame gate 不是 PASS，拒绝适配视觉坐标")
    t_session = frame["candidate"]["t_session_mm"]
    legacy, trace = observation_task_to_legacy(a.observation, a.task,
                                                a.calibration, t_session)
    Path(a.out).write_text(json.dumps(legacy, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    Path(a.trace_out).write_text(json.dumps(trace, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"pairs={len(legacy['pairs'])} obstacles={len(legacy['obstacles'])} -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
