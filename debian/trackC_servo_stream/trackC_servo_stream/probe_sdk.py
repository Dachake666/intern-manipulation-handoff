#!/usr/bin/env python3
"""探测【这台机器上装的 pypilot 轮子】到底有哪些接口、什么签名。

为什么需要: 20260731 真机实测发现手册(V260526.1)比容器里的轮子【新】——
按手册写的 4 个接口在真机上不存在或签名不同:
    armGetPosReFreshMS   本机没有这个属性
    armUnsetLoad         本机没有这个属性
    armGetAxisParameter  第三参要枚举 AxisParameterIndex, 不是 int
    armSetRobotLoad      本机签名是 (int, int), 不是 (int, LoadParameter)
    armTryWorlds         要 std::vector<double>, 给 FloatVector 直接 ArgumentError
按手册写代码 = 按一份还没实现的规格写。以后新接口一律【先探再用】。

本脚本【不连机器人、不发任何运动指令】—— 只 import pypilot 看类型和属性,
所以随时可以跑, 没有任何风险。

用法(容器内): python3 probe_sdk.py
把输出贴回来, 就能按真实接口改代码, 而不是照手册猜。
"""
import inspect
import sys

try:
    import pypilot
except Exception as exc:                       # noqa: BLE001
    print("✗ import pypilot 失败:", exc)
    sys.exit(1)

# 我们用到或想用的接口。分组是为了让输出一眼能看出"哪条线受影响"。
GROUPS = {
    "已在用(必须存在)": [
        "armInitData", "armGetLinkStatus", "armClearAlarm", "armRobotEnableOrNot",
        "armGetRobotEnableStatus", "armServoIsOpOrNot", "armSetGlobalSpeed",
        "armGetSingleRobotEnableStatus", "armGetRobotMoveState",
        "armClearRobotRoute", "armGetJoints", "armGetWorlds", "armMoveJoints",
        "armMoveWorlds", "armIsAlarming", "armOpenCom", "armWriteCom",
        "getSoftStopSwitch",
    ],
    "透传核心": [
        "armSetRobotProtectStatus", "armPluseToServo", "armWorldsToServo",
    ],
    "20260731 新用(已知有坑)": [
        "armTryWorlds", "armGetAxisParameter", "armSetAxisParameter",
        "armJointOutLimit", "armSetRobotLoad", "armUnsetLoad",
        "armGetPosReFreshMS", "armSetPosReFreshMS", "armGetRobotProtectStatus",
        "armGetSpeedFeedback", "armGetRobotPaused", "armGetRobotAxisMoveState",
        "armGetGlobalSpeed", "armSetEnableTolerance",
    ],
    "暂未用(备查)": [
        "armMoveWorldsCurve", "armMoveWorldsRound", "armTorqueToServo",
        "armSpeedToServo", "armPauseRobot", "armGetRobotPaused",
        "armGetTorqueFeedbackVector", "armActivateCollision",
        "armRobotSoftEmergencyStop", "armGetVelocity", "armGetAcceleration",
    ],
}


def sig_of(fn):
    """尽力取签名。Boost.Python 的函数 inspect 拿不到, 但 __doc__ 里通常带 C++ 签名。"""
    try:
        return str(inspect.signature(fn))
    except Exception:                          # noqa: BLE001
        doc = (getattr(fn, "__doc__", "") or "").strip().splitlines()
        for line in doc:
            if "(" in line and ")" in line:
                return line.strip()
        return "(签名未知)"


def main():
    print("=" * 74)
    print("pypilot 模块层")
    print("=" * 74)
    print("  版本:", getattr(pypilot, "__version__", "(无 __version__)"))
    print("  文件:", getattr(pypilot, "__file__", "?"))
    vecs = [n for n in dir(pypilot) if "Vector" in n]
    print("  向量类型:", vecs or "(无)")
    for n in ("FloatVector", "DoubleVector", "Int32Vector", "StringVector"):
        print(f"    {n:14s} {'✓ 有' if hasattr(pypilot, n) else '✗ 无'}")
    others = [n for n in dir(pypilot)
              if not n.startswith("_") and n not in vecs and n[0].isupper()]
    print("  其它导出类型:", others or "(无)")

    cls = getattr(pypilot, "PilotSDK", None)
    if cls is None:
        print("\n✗ 没有 PilotSDK 类, 后面没法查")
        return 1

    print("\n" + "=" * 74)
    print("PilotSDK 接口")
    print("=" * 74)
    missing_total = []
    for group, names in GROUPS.items():
        print(f"\n--- {group} ---")
        for n in names:
            fn = getattr(cls, n, None)
            if fn is None:
                print(f"  ✗ {n:34s} 本机没有")
                missing_total.append(n)
            else:
                print(f"  ✓ {n:34s} {sig_of(fn)[:110]}")

    have = {n for n in dir(cls) if n.startswith("arm") or n.startswith("get")}
    known = {n for names in GROUPS.values() for n in names}
    extra = sorted(have - known)
    if extra:
        print(f"\n--- 本机还有、我们没登记过的 {len(extra)} 个 ---")
        for n in extra:
            print(f"    {n}")

    print("\n" + "=" * 74)
    if missing_total:
        print(f"缺失 {len(missing_total)} 个: " + ", ".join(sorted(set(missing_total))))
        print("代码里对应的功能应当按能力门控跳过, 而不是每次运行刷警告。")
    else:
        print("登记的接口本机全部存在(签名是否匹配仍需实际调用验证)。")
    print("=" * 74)
    return 0


if __name__ == "__main__":
    sys.exit(main())
