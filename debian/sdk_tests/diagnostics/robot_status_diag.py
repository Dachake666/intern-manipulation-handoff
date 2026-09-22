#!/usr/bin/env python3
import time
import json
import pypilot

ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"

def safe_call(name, func):
    print(f"\n===== {name} =====")
    try:
        ret = func()
        print(ret)
        return ret
    except Exception as e:
        print("EXCEPTION:", repr(e))
        return None

def print_obj(obj):
    for key in dir(obj):
        if key.startswith("_"):
            continue
        try:
            value = getattr(obj, key)
            if not callable(value):
                print(f"{key}: {value}")
        except Exception:
            pass

def main():
    sdk = pypilot.PilotSDK(ROBOT_IP)

    try:
        print("start:", sdk.start())
        time.sleep(1)

        print("requestConnection:", sdk.requestConnection(LOCAL_IP))
        time.sleep(3)

        safe_call("getConnectionStatus", sdk.getConnectionStatus)
        safe_call("getAgvStatus", sdk.getAgvStatus)
        safe_call("getOperatingMode", sdk.getOperatingMode)
        safe_call("getSoftStopSwitch", sdk.getSoftStopSwitch)
        safe_call("getDriving", sdk.getDriving)
        safe_call("getButtonStatus", sdk.getButtonStatus)

        print("\n===== queryAgvInfo =====")
        ret = safe_call("queryAgvInfo raw", sdk.queryAgvInfo)
        if ret and ret[0] == 0:
            print_obj(ret[1])

        print("\n===== queryIndustrialPcInfo =====")
        ret = safe_call("queryIndustrialPcInfo raw", sdk.queryIndustrialPcInfo)
        if ret and ret[0] == 0:
            print_obj(ret[1])

        print("\n===== getExtendedParameters =====")
        ret = safe_call("getExtendedParameters raw", sdk.getExtendedParameters)
        if ret and ret[0] == 0:
            params = ret[1]
            print("params count:", len(params))
            for p in params:
                try:
                    print_obj(p)
                except Exception:
                    print(p)

        print("\n===== Battery repeated test =====")
        for n in range(10):
            result_code, batteries = sdk.getBatterys()
            print(f"\nRound {n+1}, result_code={result_code}, battery_count={len(batteries)}")
            for i, battery in enumerate(batteries):
                print(f"--- Battery {i+1} ---")
                print("batteryId:", getattr(battery, "batteryId", None))
                print("batteryLevel:", getattr(battery, "batteryLevel", None))
                print("batteryVoltage:", getattr(battery, "batteryVoltage", None))
                print("batteryHealth:", getattr(battery, "batteryHealth", None))
                print("charging:", getattr(battery, "charging", None))
            time.sleep(2)

    finally:
        sdk.stop()

if __name__ == "__main__":
    main()
