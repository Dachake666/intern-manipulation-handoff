#!/usr/bin/env python3
import json
import math
import time
from datetime import datetime
from pathlib import Path

import pypilot


ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080

ARM_ID = 1  # 1 = left arm

ENABLE_REAL_MOTION = False

GLOBAL_SPEED = 3.0
WAIT_AFTER_ENABLE_SECONDS = 1.0
WAIT_AFTER_MOVE_SECONDS = 0.5
STEP_TIMEOUT_SECONDS = 20.0
POLL_INTERVAL_SECONDS = 0.2
TARGET_TOLERANCE_DEG = 0.3
MAX_ABS_DELTA_DEG = 3.0

# 采样动作。这里是累计小步运动。
# 如果厂家关节定义与你确认的不一致，只改这里即可。
MOVES = [
    {
        "sample_name": "sample_002_joint1_plus3deg",
        "joint": 1,
        "delta_deg": 3.0,
        "description": "left arm joint 1 +3 deg, expected shoulder pitch if mapping is correct",
    },
    {
        "sample_name": "sample_003_joint2_plus3deg",
        "joint": 2,
        "delta_deg": 3.0,
        "description": "left arm joint 2 +3 deg, expected shoulder roll if mapping is correct",
    },
    {
        "sample_name": "sample_004_joint4_plus3deg",
        "joint": 4,
        "delta_deg": 3.0,
        "description": "left arm joint 4 +3 deg, expected elbow pitch if mapping is correct",
    },
    {
        "sample_name": "sample_005_joint6_plus3deg",
        "joint": 6,
        "delta_deg": 3.0,
        "description": "left arm joint 6 +3 deg, expected wrist pitch if mapping is correct",
    },
]


def make_float_vector(values):
    vector = pypilot.FloatVector()
    for value in values:
        vector.append(float(value))
    return vector


def require_code_zero(code, operation):
    if code != 0:
        raise RuntimeError(f"{operation} failed, return code: {code}")


def require_true_status(code, status, operation):
    if code != 0 or status is not True:
        raise RuntimeError(
            f"{operation} status abnormal: code={code}, status={status}"
        )


def arm_name(arm_id):
    if arm_id == 1:
        return "left arm"
    if arm_id == 2:
        return "right arm"
    return f"unknown arm({arm_id})"


def read_joints(sdk):
    code, joints = sdk.armGetJoints(ARM_ID, True)
    require_code_zero(code, "armGetJoints feedback=True")
    joints_list = list(joints)

    if len(joints_list) != 7:
        raise RuntimeError(
            f"Expected 7 joints, got {len(joints_list)}: {joints_list}"
        )

    return joints_list


def read_worlds(sdk):
    code, worlds = sdk.armGetWorlds(ARM_ID, True)
    require_code_zero(code, "armGetWorlds feedback=True")
    worlds_list = list(worlds)

    if len(worlds_list) != 6:
        raise RuntimeError(
            f"Expected 6 world values, got {len(worlds_list)}: {worlds_list}"
        )

    return worlds_list


def sample_once(sdk, sample_name, description):
    joints_deg = read_joints(sdk)
    worlds = read_worlds(sdk)

    xyz_mm = worlds[:3]
    uvw_deg = worlds[3:]

    xyz_m = [v / 1000.0 for v in xyz_mm]
    joints_rad = [math.radians(v) for v in joints_deg]
    uvw_rad = [math.radians(v) for v in uvw_deg]

    sample = {
        "sample_name": sample_name,
        "description": description,
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "arm_id": ARM_ID,
        "joints_feedback_true_deg": joints_deg,
        "joints_feedback_true_rad": joints_rad,
        "worlds_feedback_true": worlds,
        "sdk_world_xyz_mm": xyz_mm,
        "sdk_world_xyz_m": xyz_m,
        "sdk_world_uvw_deg": uvw_deg,
        "sdk_world_uvw_rad": uvw_rad,
    }

    print()
    print("=" * 60)
    print(sample_name)
    print("=" * 60)
    print("description:", description)
    print("q_sdk_deg:", joints_deg)
    print("q_sdk_rad:", joints_rad)
    print("sdk_world_xyz_mm:", xyz_mm)
    print("sdk_world_xyz_m:", xyz_m)
    print("sdk_world_uvw_deg:", uvw_deg)

    return sample


def wait_until_joint_target(sdk, joint_number, target_angle):
    joint_index = joint_number - 1
    deadline = time.time() + STEP_TIMEOUT_SECONDS

    while time.time() < deadline:
        joints = read_joints(sdk)
        actual = joints[joint_index]
        error = abs(actual - target_angle)

        print(
            f"wait joint {joint_number}: "
            f"target={target_angle:.6f} deg, "
            f"actual={actual:.6f} deg, "
            f"error={error:.6f} deg"
        )

        if error <= TARGET_TOLERANCE_DEG:
            return joints

        time.sleep(POLL_INTERVAL_SECONDS)

    joints = read_joints(sdk)
    actual = joints[joint_index]
    error = abs(actual - target_angle)
    raise RuntimeError(
        f"joint {joint_number} did not reach target within "
        f"{STEP_TIMEOUT_SECONDS}s: target={target_angle}, "
        f"actual={actual}, error={error}"
    )


def move_one_joint_delta(sdk, joint_number, delta_deg):
    if joint_number not in range(1, 8):
        raise ValueError(f"joint_number must be 1 to 7, got {joint_number}")

    if abs(float(delta_deg)) > MAX_ABS_DELTA_DEG:
        raise ValueError(
            f"delta_deg {delta_deg} exceeds limit +/-{MAX_ABS_DELTA_DEG}"
        )

    current = read_joints(sdk)
    target = current[:]

    joint_index = joint_number - 1
    before = current[joint_index]
    target_angle = before + float(delta_deg)
    target[joint_index] = target_angle

    print()
    print("=" * 60)
    print(f"Move {arm_name(ARM_ID)} joint {joint_number} by {delta_deg:+.6f} deg")
    print("=" * 60)
    print("current joints:", current)
    print(
        f"joint {joint_number}: "
        f"{before:.6f} deg -> {target_angle:.6f} deg"
    )
    print("target joints:", target)

    if not ENABLE_REAL_MOTION:
        print("DRY RUN: no armMoveJoints command sent.")
        return

    target_vector = make_float_vector(target)

    move_code = sdk.armMoveJoints(
        ARM_ID,
        target_vector,
        False,
        0,
    )
    print("armMoveJoints return:", move_code)
    require_code_zero(move_code, "armMoveJoints")

    time.sleep(WAIT_AFTER_MOVE_SECONDS)
    wait_until_joint_target(sdk, joint_number, target_angle)


def wait_until_arm_enabled(sdk):
    deadline = time.time() + 10.0

    while time.time() < deadline:
        enable_code, enabled = sdk.armGetRobotEnableStatus()
        servo_code, servo_enabled = sdk.armServoIsOpOrNot()

        print(
            "enable/servo:",
            (enable_code, enabled),
            (servo_code, servo_enabled),
        )

        if (
            enable_code == 0
            and enabled is True
            and servo_code == 0
            and servo_enabled is True
        ):
            return

        time.sleep(0.5)

    raise RuntimeError("arm enable/servo did not become True within 10 seconds")


def start_sdk_and_arm(sdk):
    print("=" * 60)
    print("Start SDK and initialize left arm")
    print("=" * 60)

    start_result = sdk.start()
    print("start:", start_result)
    if not start_result:
        raise RuntimeError("PilotSDK start failed")

    time.sleep(1)

    soft_stop_code, soft_stop_status = sdk.getSoftStopSwitch()
    print("soft_stop:", (soft_stop_code, soft_stop_status))
    if soft_stop_code != 0 or soft_stop_status != "CLOSE":
        raise RuntimeError("soft stop is not CLOSE")

    init_code = sdk.armInitData(LOCAL_IP, ARM_IP, ARM_PORT)
    print("armInitData:", init_code)
    require_code_zero(init_code, "armInitData")

    link_code, linked = sdk.armGetLinkStatus()
    print("arm link:", (link_code, linked))
    require_true_status(link_code, linked, "arm link")

    clear_alarm_code = sdk.armClearAlarm()
    print("armClearAlarm:", clear_alarm_code)
    require_code_zero(clear_alarm_code, "armClearAlarm")

    enable_code = sdk.armRobotEnableOrNot(True)
    print("armRobotEnableOrNot:", enable_code)
    require_code_zero(enable_code, "armRobotEnableOrNot")

    time.sleep(WAIT_AFTER_ENABLE_SECONDS)
    wait_until_arm_enabled(sdk)

    single_code, single_enabled = sdk.armGetSingleRobotEnableStatus(ARM_ID)
    print("single enable:", (single_code, single_enabled))
    require_true_status(single_code, single_enabled, "single arm enable")

    speed_code = sdk.armSetGlobalSpeed(GLOBAL_SPEED)
    print("armSetGlobalSpeed:", speed_code)
    require_code_zero(speed_code, "armSetGlobalSpeed")


def main():
    sdk = pypilot.PilotSDK(ROBOT_IP)
    samples = []

    metadata = {
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "robot_ip": ROBOT_IP,
        "local_ip": LOCAL_IP,
        "arm_ip": ARM_IP,
        "arm_port": ARM_PORT,
        "arm_id": ARM_ID,
        "enable_real_motion": ENABLE_REAL_MOTION,
        "global_speed": GLOBAL_SPEED,
        "target_tolerance_deg": TARGET_TOLERANCE_DEG,
        "max_abs_delta_deg": MAX_ABS_DELTA_DEG,
        "note": (
            "SDK samples for fitting SDK arm world frame against "
            "PyBullet FK/local frames."
        ),
    }

    try:
        start_sdk_and_arm(sdk)

        samples.append(
            sample_once(
                sdk,
                "sample_001_initial",
                "initial pose before additional sample moves",
            )
        )

        for move in MOVES:
            move_one_joint_delta(
                sdk,
                int(move["joint"]),
                float(move["delta_deg"]),
            )

            samples.append(
                sample_once(
                    sdk,
                    move["sample_name"],
                    move["description"],
                )
            )

        result = {
            "metadata": metadata,
            "moves": MOVES,
            "samples": samples,
        }

        output_dir = Path("diagnostics")
        output_dir.mkdir(parents=True, exist_ok=True)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = output_dir / f"left_arm_world_samples_{timestamp}.json"

        output_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        latest_path = output_dir / "left_arm_world_samples_latest.json"
        latest_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        print()
        print("=" * 60)
        print("SAMPLES_SAVED")
        print("=" * 60)
        print("saved:", output_path)
        print("latest:", latest_path)

    finally:
        sdk.stop()


if __name__ == "__main__":
    main()
