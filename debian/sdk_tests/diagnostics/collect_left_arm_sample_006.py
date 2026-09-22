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

ARM_ID = 1

ENABLE_REAL_MOTION = False

GLOBAL_SPEED = 3.0
MOVE_JOINT = 5
MOVE_DELTA_DEG = 2.0

TARGET_TOLERANCE_DEG = 0.3
STEP_TIMEOUT_SECONDS = 20.0
POLL_INTERVAL_SECONDS = 0.2


def make_float_vector(values):
    vector = pypilot.FloatVector()
    for v in values:
        vector.append(float(v))
    return vector


def require_code_zero(code, name):
    if code != 0:
        raise RuntimeError(f"{name} failed, code={code}")


def require_true_status(code, status, name):
    if code != 0 or status is not True:
        raise RuntimeError(f"{name} abnormal, code={code}, status={status}")


def read_joints(sdk):
    code, joints = sdk.armGetJoints(ARM_ID, True)
    require_code_zero(code, "armGetJoints")
    return list(joints)


def read_worlds(sdk):
    code, worlds = sdk.armGetWorlds(ARM_ID, True)
    require_code_zero(code, "armGetWorlds")
    return list(worlds)


def sample_once(sdk, name, desc):
    q_deg = read_joints(sdk)
    worlds = read_worlds(sdk)

    sample = {
        "sample_name": name,
        "description": desc,
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "arm_id": ARM_ID,
        "joints_feedback_true_deg": q_deg,
        "joints_feedback_true_rad": [math.radians(v) for v in q_deg],
        "worlds_feedback_true": worlds,
        "sdk_world_xyz_mm": worlds[:3],
        "sdk_world_xyz_m": [v / 1000.0 for v in worlds[:3]],
        "sdk_world_uvw_deg": worlds[3:],
        "sdk_world_uvw_rad": [math.radians(v) for v in worlds[3:]],
    }

    print()
    print("=" * 60)
    print(name)
    print("=" * 60)
    print("description:", desc)
    print("q_sdk_deg =", q_deg)
    print("sdk_world_xyz_mm =", worlds[:3])
    print("sdk_world_uvw_deg =", worlds[3:])

    return sample


def wait_until_enabled(sdk):
    deadline = time.time() + 10.0

    while time.time() < deadline:
        enable_status = sdk.armGetRobotEnableStatus()
        servo_status = sdk.armServoIsOpOrNot()
        print("enable/servo:", enable_status, servo_status)

        if (
            enable_status[0] == 0
            and enable_status[1] is True
            and servo_status[0] == 0
            and servo_status[1] is True
        ):
            return

        time.sleep(0.5)

    raise RuntimeError("enable/servo did not become True")


def wait_joint(sdk, joint_number, target_angle):
    idx = joint_number - 1
    deadline = time.time() + STEP_TIMEOUT_SECONDS

    while time.time() < deadline:
        q = read_joints(sdk)
        actual = q[idx]
        err = abs(actual - target_angle)

        print(
            f"wait joint {joint_number}: "
            f"target={target_angle:.6f}, actual={actual:.6f}, err={err:.6f}"
        )

        if err <= TARGET_TOLERANCE_DEG:
            return

        time.sleep(POLL_INTERVAL_SECONDS)

    raise RuntimeError("joint did not reach target")


def main():
    sdk = pypilot.PilotSDK(ROBOT_IP)

    result = {
        "metadata": {
            "created_at": datetime.now().isoformat(timespec="seconds"),
            "purpose": "sample_006 validation sample, not used in previous 5-sample fitting",
            "robot_ip": ROBOT_IP,
            "local_ip": LOCAL_IP,
            "arm_ip": ARM_IP,
            "arm_port": ARM_PORT,
            "arm_id": ARM_ID,
            "move_joint": MOVE_JOINT,
            "move_delta_deg": MOVE_DELTA_DEG,
            "enable_real_motion": ENABLE_REAL_MOTION,
        },
        "samples": [],
    }

    try:
        print("=" * 60)
        print("Start SDK for sample_006 validation")
        print("=" * 60)

        start = sdk.start()
        print("start:", start)
        if not start:
            raise RuntimeError("sdk.start failed")

        time.sleep(1)

        soft_stop = sdk.getSoftStopSwitch()
        print("soft_stop:", soft_stop)
        if soft_stop[0] != 0 or soft_stop[1] != "CLOSE":
            raise RuntimeError("soft stop is not CLOSE")

        init_code = sdk.armInitData(LOCAL_IP, ARM_IP, ARM_PORT)
        print("armInitData:", init_code)
        require_code_zero(init_code, "armInitData")

        link = sdk.armGetLinkStatus()
        print("arm link:", link)
        require_true_status(link[0], link[1], "arm link")

        clear_code = sdk.armClearAlarm()
        print("armClearAlarm:", clear_code)
        require_code_zero(clear_code, "armClearAlarm")

        enable_code = sdk.armRobotEnableOrNot(True)
        print("armRobotEnableOrNot:", enable_code)
        require_code_zero(enable_code, "armRobotEnableOrNot")

        wait_until_enabled(sdk)

        single = sdk.armGetSingleRobotEnableStatus(ARM_ID)
        print("single enable:", single)
        require_true_status(single[0], single[1], "single enable")

        speed_code = sdk.armSetGlobalSpeed(GLOBAL_SPEED)
        print("armSetGlobalSpeed:", speed_code)
        require_code_zero(speed_code, "armSetGlobalSpeed")

        result["samples"].append(
            sample_once(
                sdk,
                "sample_006_before_move",
                "current pose before validation move",
            )
        )

        q_current = read_joints(sdk)
        q_target = q_current[:]
        idx = MOVE_JOINT - 1
        before = q_current[idx]
        target = before + MOVE_DELTA_DEG
        q_target[idx] = target

        print()
        print("=" * 60)
        print(f"Move joint {MOVE_JOINT} by {MOVE_DELTA_DEG:+.6f} deg")
        print("=" * 60)
        print("current joints:", q_current)
        print(f"joint {MOVE_JOINT}: {before:.6f} -> {target:.6f}")
        print("target joints:", q_target)

        if ENABLE_REAL_MOTION:
            move_code = sdk.armMoveJoints(
                ARM_ID,
                make_float_vector(q_target),
                False,
                0,
            )
            print("armMoveJoints return:", move_code)
            require_code_zero(move_code, "armMoveJoints")

            wait_joint(sdk, MOVE_JOINT, target)
        else:
            print("DRY RUN: no real motion sent")

        result["samples"].append(
            sample_once(
                sdk,
                "sample_006_joint5_plus2deg",
                "validation sample after left arm joint 5 +2 deg",
            )
        )

        out = Path("diagnostics/sample_006_validation.json")
        out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")

        print()
        print("=" * 60)
        print("SAMPLE_006_SAVED")
        print("=" * 60)
        print("saved:", out)

    finally:
        sdk.stop()


if __name__ == "__main__":
    main()
