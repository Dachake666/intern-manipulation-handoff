#!/usr/bin/env python3
import json
import math
from pathlib import Path

import numpy as np
import pybullet as p


URDF_PATH = Path("/home/dev/workspace/pybullet_tests/urdf/XF0112048/robot_pybullet.urdf")

FIT_JSON = Path("diagnostics/left_arm_world_samples_latest.json")
VALIDATION_JSON = Path("diagnostics/sample_006_validation.json")

# 当前你已经验证 link11 最接近 SDK world
EE_LINK_ID = 11

# 左臂 7 个关节在 PyBullet 里的 joint id
# 如果你之前 FK 脚本用的不是 5~11，就只改这里
LEFT_ARM_JOINT_IDS = [5, 6, 7, 8, 9, 10, 11]


def load_samples():
    fit_data = json.loads(FIT_JSON.read_text(encoding="utf-8"))
    val_data = json.loads(VALIDATION_JSON.read_text(encoding="utf-8"))

    fit_samples = fit_data["samples"]
    val_samples = val_data["samples"]

    return fit_samples, val_samples


def q_deg_to_rad(q_deg):
    return [math.radians(float(v)) for v in q_deg]


def quat_to_rot_matrix(quat):
    mat = p.getMatrixFromQuaternion(quat)
    return np.array(mat, dtype=float).reshape(3, 3)


def reset_left_arm(robot, q_rad):
    if len(q_rad) != 7:
        raise ValueError(f"Expected 7 joint values, got {len(q_rad)}")

    for joint_id, q in zip(LEFT_ARM_JOINT_IDS, q_rad):
        p.resetJointState(robot, joint_id, float(q))


def get_link11_pose_mm(robot):
    state = p.getLinkState(robot, EE_LINK_ID, computeForwardKinematics=True)

    # index 4/5 是 worldLinkFramePosition / worldLinkFrameOrientation
    pos_m = np.array(state[4], dtype=float)
    orn = state[5]
    rot = quat_to_rot_matrix(orn)

    pos_mm = pos_m * 1000.0
    return pos_mm, rot


def compute_pb_for_samples(robot, samples):
    rows = []

    for sample in samples:
        name = sample["sample_name"]
        q_deg = sample["joints_feedback_true_deg"]
        q_rad = q_deg_to_rad(q_deg)

        sdk_xyz_mm = np.array(sample["sdk_world_xyz_mm"], dtype=float)

        reset_left_arm(robot, q_rad)
        pb_xyz_mm, link_rot = get_link11_pose_mm(robot)

        rows.append({
            "name": name,
            "q_deg": q_deg,
            "sdk_xyz_mm": sdk_xyz_mm,
            "pb_xyz_mm": pb_xyz_mm,
            "link_rot": link_rot,
        })

    return rows


def stats_from_residuals(residuals):
    residuals = np.array(residuals, dtype=float)
    norms = np.linalg.norm(residuals, axis=1)

    return {
        "rms": float(np.sqrt(np.mean(norms ** 2))),
        "max": float(np.max(norms)),
        "mean_abs_xyz": np.mean(np.abs(residuals), axis=0),
        "norms": norms,
        "residuals": residuals,
    }


def fit_translation_only(rows):
    sdk = np.array([r["sdk_xyz_mm"] for r in rows])
    pb = np.array([r["pb_xyz_mm"] for r in rows])

    t = np.mean(sdk - pb, axis=0)
    pred = pb + t
    residuals = sdk - pred

    return t, stats_from_residuals(residuals)


def fit_translation_plus_tcp(rows):
    # model:
    # sdk_i = pb_i + t + R_i * r_tcp
    #
    # unknown:
    # t: SDK/PB world fixed translation, 3x1
    # r_tcp: offset from link11 origin to SDK point, expressed in link11 local frame, 3x1

    A_blocks = []
    b_blocks = []

    for row in rows:
        sdk = row["sdk_xyz_mm"]
        pb = row["pb_xyz_mm"]
        R = row["link_rot"]

        # sdk - pb = I * t + R * r_tcp
        A_i = np.zeros((3, 6), dtype=float)
        A_i[:, 0:3] = np.eye(3)
        A_i[:, 3:6] = R

        b_i = sdk - pb

        A_blocks.append(A_i)
        b_blocks.append(b_i)

    A = np.vstack(A_blocks)
    b = np.concatenate(b_blocks)

    x, residual_sum, rank, singular_values = np.linalg.lstsq(A, b, rcond=None)

    t = x[0:3]
    r_tcp = x[3:6]

    residuals = []

    for row in rows:
        sdk = row["sdk_xyz_mm"]
        pb = row["pb_xyz_mm"]
        R = row["link_rot"]

        pred = pb + t + R @ r_tcp
        residuals.append(sdk - pred)

    if singular_values[-1] > 1e-12:
        cond = singular_values[0] / singular_values[-1]
    else:
        cond = float("inf")

    return t, r_tcp, rank, cond, singular_values, stats_from_residuals(residuals)


def eval_translation_only(rows, t):
    residuals = []

    for row in rows:
        pred = row["pb_xyz_mm"] + t
        residuals.append(row["sdk_xyz_mm"] - pred)

    return stats_from_residuals(residuals)


def eval_translation_plus_tcp(rows, t, r_tcp):
    residuals = []

    for row in rows:
        pred = row["pb_xyz_mm"] + t + row["link_rot"] @ r_tcp
        residuals.append(row["sdk_xyz_mm"] - pred)

    return stats_from_residuals(residuals)


def print_rows(title, rows, t=None, r_tcp=None):
    print()
    print("=" * 80)
    print(title)
    print("=" * 80)

    for row in rows:
        sdk = row["sdk_xyz_mm"]
        pb = row["pb_xyz_mm"]

        if r_tcp is None:
            pred = pb + t
        else:
            pred = pb + t + row["link_rot"] @ r_tcp

        residual = sdk - pred
        norm = np.linalg.norm(residual)

        print()
        print(row["name"])
        print("sdk_xyz_mm =", np.round(sdk, 3).tolist())
        print("pb_link11_world_xyz_mm =", np.round(pb, 3).tolist())
        print("pred_sdk_xyz_mm =", np.round(pred, 3).tolist())
        print("residual_mm =", np.round(residual, 3).tolist())
        print("residual_norm_mm =", round(float(norm), 3))


def print_stats(title, stats):
    print()
    print(title)
    print("rms_mm =", round(stats["rms"], 3))
    print("max_mm =", round(stats["max"], 3))
    print("mean_abs_xyz_mm =", np.round(stats["mean_abs_xyz"], 3).tolist())


def main():
    if not URDF_PATH.exists():
        raise FileNotFoundError(f"URDF not found: {URDF_PATH}")

    if not FIT_JSON.exists():
        raise FileNotFoundError(f"Fit json not found: {FIT_JSON}")

    if not VALIDATION_JSON.exists():
        raise FileNotFoundError(f"Validation json not found: {VALIDATION_JSON}")

    fit_samples, val_samples = load_samples()

    physics_client = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(
            str(URDF_PATH),
            basePosition=[0, 0, 0],
            baseOrientation=[0, 0, 0, 1],
            useFixedBase=True,
        )

        print("=" * 80)
        print("Joint mapping check")
        print("=" * 80)
        print("EE_LINK_ID =", EE_LINK_ID)
        print("LEFT_ARM_JOINT_IDS =", LEFT_ARM_JOINT_IDS)

        for jid in LEFT_ARM_JOINT_IDS:
            info = p.getJointInfo(robot, jid)
            print(
                f"joint {jid}: "
                f"name={info[1].decode(errors='ignore')}, "
                f"link={info[12].decode(errors='ignore')}"
            )

        fit_rows = compute_pb_for_samples(robot, fit_samples)
        val_rows = compute_pb_for_samples(robot, val_samples)

        # Model A: wrist/link11 origin + fixed translation only
        t_a, fit_stats_a = fit_translation_only(fit_rows)
        val_stats_a = eval_translation_only(val_rows, t_a)

        # Model B: link11 origin + local TCP offset + fixed translation
        t_b, r_tcp, rank, cond, singular_values, fit_stats_b = fit_translation_plus_tcp(fit_rows)
        val_stats_b = eval_translation_plus_tcp(val_rows, t_b, r_tcp)

        print()
        print("=" * 80)
        print("MODEL A: translation only")
        print("=" * 80)
        print("p_sdk = p_pb_link11 + t")
        print("t_mm =", np.round(t_a, 3).tolist())
        print_stats("fit samples stats", fit_stats_a)
        print_stats("validation samples stats", val_stats_a)

        print_rows("MODEL A validation residuals", val_rows, t=t_a, r_tcp=None)

        print()
        print("=" * 80)
        print("MODEL B: translation + link11-local TCP offset")
        print("=" * 80)
        print("p_sdk = p_pb_link11 + t + R_link11 * r_tcp")
        print("t_mm =", np.round(t_b, 3).tolist())
        print("r_tcp_in_link11_local_mm =", np.round(r_tcp, 3).tolist())
        print("r_tcp_norm_mm =", round(float(np.linalg.norm(r_tcp)), 3))
        print("least_squares_rank =", rank)
        print("condition_number =", round(float(cond), 3))
        print("singular_values =", np.round(singular_values, 6).tolist())
        print_stats("fit samples stats", fit_stats_b)
        print_stats("validation samples stats", val_stats_b)

        print_rows("MODEL B validation residuals", val_rows, t=t_b, r_tcp=r_tcp)

        print()
        print("=" * 80)
        print("DECISION_HINT")
        print("=" * 80)
        print("如果 MODEL B 的 validation rms/max 明显小于 MODEL A，且 r_tcp_norm 是几十毫米以上，则 SDK 更像 TCP。")
        print("如果 MODEL B 改善很小，或者 r_tcp_norm 很小/数值不稳定，则 SDK 更像 wrist/link11 原点。")
        print("如果 condition_number 很大，说明当前样本姿态变化不足，r_tcp 不够可靠，需要再采 wrist 方向变化更大的样本。")

    finally:
        p.disconnect(physics_client)


if __name__ == "__main__":
    main()
