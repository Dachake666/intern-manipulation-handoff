#!/usr/bin/env python3
import json
import time
from datetime import datetime

import pypilot


ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080

# 1 = left arm, 2 = right arm, 3 = waist
ARM_ID = 1


def safe_call(name, func):
    print()
    print("=" * 60)
    print(name)
    print("=" * 60)

    try:
        ret = func()
        print(ret)
        return ret
    except Exception as e:
        print("EXCEPTION:", repr(e))
        return None


def to_list(value):
    try:
        return list(value)
    except Exception:
        return value


def main():
    sdk = pypilot.PilotSDK(ROBOT_IP)

    result = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "robot_ip": ROBOT_IP,
        "local_ip": LOCAL_IP,
        "arm_ip": ARM_IP,
        "arm_port": ARM_PORT,
        "arm_id": ARM_ID,
    }

    try:
        print("=" * 60)
        print("Read left arm SDK pose / joints")
        print("=" * 60)

        start_ret = sdk.start()
        print("start:", start_ret)
        result["sdk_start"] = start_ret

        time.sleep(1)

        soft_stop = safe_call("getSoftStopSwitch", sdk.getSoftStopSwitch)
        result["soft_stop"] = soft_stop

        init_code = safe_call(
            "armInitData",
            lambda: sdk.armInitData(LOCAL_IP, ARM_IP, ARM_PORT),
        )
        result["armInitData"] = init_code

        link_status = safe_call("armGetLinkStatus", sdk.armGetLinkStatus)
        result["arm_link"] = link_status

        single_enable = safe_call(
            "armGetSingleRobotEnableStatus",
            lambda: sdk.armGetSingleRobotEnableStatus(ARM_ID),
        )
        result["single_enable"] = single_enable

        joints_feedback_true = safe_call(
            "armGetJoints feedback=True",
            lambda: sdk.armGetJoints(ARM_ID, True),
        )

        joints_feedback_false = safe_call(
            "armGetJoints feedback=False",
            lambda: sdk.armGetJoints(ARM_ID, False),
        )

        worlds_feedback_true = safe_call(
            "armGetWorlds feedback=True",
            lambda: sdk.armGetWorlds(ARM_ID, True),
        )

        worlds_feedback_false = safe_call(
            "armGetWorlds feedback=False",
            lambda: sdk.armGetWorlds(ARM_ID, False),
        )

        if joints_feedback_true and joints_feedback_true[0] == 0:
            result["joints_feedback_true"] = to_list(joints_feedback_true[1])

        if joints_feedback_false and joints_feedback_false[0] == 0:
            result["joints_feedback_false"] = to_list(joints_feedback_false[1])

        if worlds_feedback_true and worlds_feedback_true[0] == 0:
            result["worlds_feedback_true"] = to_list(worlds_feedback_true[1])

        if worlds_feedback_false and worlds_feedback_false[0] == 0:
            result["worlds_feedback_false"] = to_list(worlds_feedback_false[1])

        print()
        print("=" * 60)
        print("JSON_RESULT")
        print("=" * 60)
        print(json.dumps(result, ensure_ascii=False, indent=2))

    finally:
        sdk.stop()


if __name__ == "__main__":
    main()
