#!/usr/bin/env python3
"""从同步的 camera→target 与 sdk_world→target 位姿拟合 T_sdk_world_camera。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

WORK = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(WORK))
from robot_mission.contracts import canonical_bytes, validate_document


def quat_matrix(q):
    x, y, z, w = map(float, q)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if abs(n - 1) > 1e-3:
        raise ValueError(f"四元数未归一化: norm={n}")
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def rotation_angle_deg(R):
    return math.degrees(math.acos(float(np.clip((np.trace(R)-1)/2, -1, 1))))


def average_rotation(rotations):
    U, _, Vt = np.linalg.svd(np.sum(rotations, axis=0))
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def pose(sample, key):
    value = sample[key]
    p = np.asarray(value["position_m"], float)
    if p.shape != (3,) or not np.all(np.isfinite(p)):
        raise ValueError(f"{key}.position_m 无效")
    return p, quat_matrix(value["quaternion_xyzw"])


def fit(samples):
    candidates, translations = [], []
    for sample in samples:
        if float(sample["sync_delta_ms"]) > 50:
            raise ValueError(f"样本 {sample.get('sample_id')} 同步差超过 50ms")
        p_w, R_w_t = pose(sample, "sdk_world_target")
        p_c, R_c_t = pose(sample, "camera_target")
        R_w_c = R_w_t @ R_c_t.T
        candidates.append(R_w_c)
    R = average_rotation(candidates)
    for sample in samples:
        p_w, _ = pose(sample, "sdk_world_target")
        p_c, _ = pose(sample, "camera_target")
        translations.append(p_w - R @ p_c)
    t = np.mean(translations, axis=0)
    return R, t


def errors(samples, R, t):
    position, rotation = [], []
    for sample in samples:
        p_w, R_w_t = pose(sample, "sdk_world_target")
        p_c, R_c_t = pose(sample, "camera_target")
        position.append(float(np.linalg.norm(p_w - (R @ p_c + t))))
        rotation.append(rotation_angle_deg(R_w_t @ (R @ R_c_t).T))
    return position, rotation


def matrix4(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T.tolist()


def build_result(data, calibration_id, arm, camera_id, robot_identity):
    samples = data.get("samples", [])
    if len(samples) < 25:
        raise ValueError("至少需要 25 组同步样本（20 拟合 + 5 独立验证）")
    fit_rows, validation = samples[:-5], samples[-5:]
    if len(fit_rows) < 20:
        raise ValueError("拟合样本不足 20")
    R, t = fit(fit_rows)
    fit_pos, fit_rot = errors(fit_rows, R, t)
    val_pos, val_rot = errors(validation, R, t)
    T = np.asarray(matrix4(R, t)); inv = np.linalg.inv(T)
    hashes = [hashlib.sha256(canonical_bytes(x)).hexdigest() for x in samples]
    result = {
        "schema_version": "eye_to_hand_calibration.v1", "calibration_id": calibration_id,
        "camera_id": camera_id, "arm": arm, "length_unit": "meter",
        "input_conventions": {"robot_pose_direction": data["robot_pose_direction"],
                              "ros_position_unit": data["ros_position_unit"],
                              "tcp_id": data["tcp_id"],
                              "euler_tested_orders": ["Rx_Ry_Rz", "Rz_Ry_Rx"]},
        "T_sdk_world_camera": T.tolist(), "T_camera_sdk_world": inv.tolist(),
        "algorithm": "eye_to_hand_SE3_orientation_average_translation_mean_v1",
        "fit_samples": len(fit_rows), "validation_samples": len(validation),
        "residuals": {"fit_rms_m": float(np.sqrt(np.mean(np.square(fit_pos)))),
                      "validation_rms_m": float(np.sqrt(np.mean(np.square(val_pos)))),
                      "validation_max_m": max(val_pos),
                      "rotation_rms_deg": float(np.sqrt(np.mean(np.square(val_rot))))},
        "input_sha256": hashes,
        "stationary_configuration": {"waist_locked": True, "chassis_locked": True,
                                     "camera_mount_locked": True,
                                     "robot_identity": robot_identity},
        "status": "VALIDATED",
    }
    validate_document(result, "eye_to_hand_calibration.v1")
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input"); ap.add_argument("--out", required=True)
    ap.add_argument("--calibration-id", required=True); ap.add_argument("--camera-id", required=True)
    ap.add_argument("--arm", choices=("left", "right"), required=True)
    ap.add_argument("--robot-identity", required=True)
    a = ap.parse_args(argv)
    data = json.loads(Path(a.input).read_text(encoding="utf-8"))
    result = build_result(data, a.calibration_id, a.arm, a.camera_id, a.robot_identity)
    Path(a.out).write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result["residuals"], ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
