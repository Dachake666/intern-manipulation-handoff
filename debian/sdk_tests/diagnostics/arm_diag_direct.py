#!/usr/bin/env python3
import time
import pypilot

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080

# 1 = left arm, 2 = right arm, 3 = waist
ARM_ID = 1

def show(name, fn):
    print(f"\n===== {name} =====")
    try:
        ret = fn()
        print(ret)
        return ret
    except Exception as e:
        print("EXCEPTION:", repr(e))
        return None

sdk = pypilot.PilotSDK(ROBOT_IP)

try:
    print("start:", sdk.start())
    time.sleep(1)

    show("soft stop", sdk.getSoftStopSwitch)

    print("\n===== armInitData =====")
    print(sdk.armInitData(LOCAL_IP, ARM_IP, ARM_PORT))

    show("armGetLinkStatus", sdk.armGetLinkStatus)
    show("armServoIsOpOrNot", sdk.armServoIsOpOrNot)
    show("armIsAlarming", sdk.armIsAlarming)

    print("\n===== armClearAlarm =====")
    print(sdk.armClearAlarm())
    time.sleep(0.5)

    print("\n===== armRobotEnableOrNot(True) =====")
    print(sdk.armRobotEnableOrNot(True))
    time.sleep(1.0)

    show("armGetRobotEnableStatus", sdk.armGetRobotEnableStatus)
    show("armGetSingleRobotEnableStatus", lambda: sdk.armGetSingleRobotEnableStatus(ARM_ID))
    show("armSetGlobalSpeed(5.0)", lambda: sdk.armSetGlobalSpeed(5.0))
    show("armGetGlobalSpeed", sdk.armGetGlobalSpeed)

    show("armGetJoints feedback=True", lambda: sdk.armGetJoints(ARM_ID, True))
    show("armGetJoints feedback=False", lambda: sdk.armGetJoints(ARM_ID, False))

    show("armGetWorlds feedback=True", lambda: sdk.armGetWorlds(ARM_ID, True))
    show("armGetWorlds feedback=False", lambda: sdk.armGetWorlds(ARM_ID, False))

finally:
    sdk.stop()
