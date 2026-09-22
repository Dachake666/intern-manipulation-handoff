#!/usr/bin/env python3
"""底盘(AGV)状态诊断 —— 只读，一条运动指令都不发。

用途: 底盘不动了, 先搞清楚是"被什么挡住了"还是"真坏了"。

⚠ SDK 不需要新增任何东西。这些接口本来就在, 我们只是从没用过 ——
   20260803 probe 时它们出现在"本机还有、我们没登记过的 52 个"里。
   本脚本按能力门控: 1.0 轮子缺的那几个(getPaused / getResettingStatus /
   getRcsConnectionState / getChargeStatus / getDrivingWheels)会自动跳过,
   换 2.0 轮子后会自动多出来。

安全: 全程只调 get* 读接口。不 armInitData、不使能、不设速度、不发任何动作。
      因此也【不需要】机械臂互斥锁 —— 它不碰机械臂, 跟手臂脚本可以同时跑。

用法(容器内): python3 diag_chassis.py
"""
import json
import sys

try:
    import pypilot
except Exception as exc:                       # noqa: BLE001
    print("✗ import pypilot 失败:", exc)
    sys.exit(1)

ROBOT_IP = "192.168.8.148"

# (接口名, 显示名, 怎么读结果, 对"底盘不动"意味着什么)
# 读法: "code_value" = (code, value); "raw" = 直接返回值; "list" = (code, [obj])
CHECKS = [
    ("getConnectionStatus", "连接状态",      "raw",
     "断了就什么都做不了"),
    ("getSoftStopSwitch",   "软急停",        "code_value",
     "不是 CLOSE 就是急停被按下/未复位 —— 这是最常见的原因"),
    ("getButtonStatus",     "按钮状态",      "button",
     "powerButton/resetButton 若为 1 表示被按住"),
    ("getOperatingMode",    "操作模式",      "code_value",
     "AUTO 才接受任务; MANUAL/MAINTENANCE 下不会自己走"),
    ("getAgvStatus",        "AGV 状态",      "code_value",
     "IDLE=空闲 RUNNING=在跑 OFFLINE=离线 ERROR=报错"),
    ("getDriving",          "是否在行驶",    "code_value",
     "True=正在走(那问题可能在别处) False=静止"),
    ("getPaused",           "暂停状态",      "code_value",
     "True 表示被暂停了, 要先恢复"),
    ("getResettingStatus",  "复位状态",      "code_value",
     "复位中通常不接受运动"),
    ("getRcsConnectionState", "RCS 连接",    "code_value",
     "OFFLINE 时上层调度下不来任务"),
    ("getChargeStatus",     "充电状态",      "code_value",
     "在充电桩上通常禁止行走"),
    ("getBatterys",         "电池",          "battery",
     "电量过低会限制或禁止行走"),
    ("getDrivingWheels",    "驱动轮",        "wheels",
     "某个轮报错/抱闸没松, 底盘就不动"),
    ("getAgvPosition",      "当前位置",      "position",
     "定位丢失(未定位)时不会执行移动任务"),
    ("getVelocity",         "当前速度",      "velocity",
     "全 0 = 确实没在动"),
    ("getTaskId",           "当前任务 ID",   "code_value",
     "空 = 根本没收到任务, 那就不是底盘的问题"),
    ("getActionStates",     "动作状态",      "list",
     "看有没有卡住的动作"),
    ("getController",       "控制权",        "code_value",
     "控制权不在我们这边时, 指令会被忽略"),
]


def fmt(kind, val):
    """把不同类型的返回值统一转成可读文本。字段名按对象实际有什么来取,
    不硬编码 —— 1.0/2.0 轮子的字段可能不同。"""
    if val is None:
        return "(None)"
    if kind == "button":
        return (f"powerButton={getattr(val, 'powerButton', '?')} "
                f"resetButton={getattr(val, 'resetButton', '?')}")
    if kind in ("battery", "wheels", "list"):
        try:
            items = list(val)
        except Exception:                      # noqa: BLE001
            return repr(val)[:120]
        if not items:
            return "(空列表)"
        out = []
        for it in items[:4]:
            attrs = {k: getattr(it, k) for k in dir(it)
                     if not k.startswith("_") and not callable(getattr(it, k))}
            out.append(", ".join(f"{k}={v}" for k, v in list(attrs.items())[:6]))
        return " | ".join(out)
    if kind in ("position", "velocity"):
        attrs = {k: getattr(val, k) for k in dir(val)
                 if not k.startswith("_") and not callable(getattr(val, k))}
        return ", ".join(f"{k}={v}" for k, v in list(attrs.items())[:8]) or repr(val)[:120]
    if kind == "raw":
        s = str(val)
        try:                                   # getConnectionStatus 返回 JSON 串
            return json.dumps(json.loads(s), ensure_ascii=False)[:200]
        except Exception:                      # noqa: BLE001
            return s[:200]
    return str(val)


def main():
    print("=" * 70)
    print("底盘(AGV)状态诊断 —— 只读, 不发任何运动指令")
    print("=" * 70)
    print(f"pypilot 版本: {getattr(pypilot, '__version__', '(无)')}")
    print(f"机器人 IP   : {ROBOT_IP}")

    have = [c for c in CHECKS if hasattr(pypilot.PilotSDK, c[0])]
    miss = [c[0] for c in CHECKS if not hasattr(pypilot.PilotSDK, c[0])]
    print(f"\n本机可用 {len(have)}/{len(CHECKS)} 项")
    if miss:
        print("  ✗ 本轮子没有: " + ", ".join(miss))
        print("    (手册是 2.0 版的; 换 2.0 轮子后这些会自动出现)")

    sdk = pypilot.PilotSDK(ROBOT_IP)
    if not sdk.start():
        print("\n✗ PilotSDK 启动失败 —— 连不上机器人, 先查网络/电源")
        return 1
    import time
    time.sleep(1.0)                            # 给数据管道一点建立时间

    print("\n" + "=" * 70)
    results = {}
    for name, label, kind, meaning in have:
        fn = getattr(sdk, name)
        try:
            ret = fn()
        except Exception as exc:               # noqa: BLE001
            print(f"  {label:12s} ✗ 调用失败: {exc}")
            continue
        if kind == "raw":
            code, val = 0, ret
        elif isinstance(ret, tuple) and len(ret) >= 2:
            code, val = ret[0], ret[1]
        else:
            code, val = 0, ret
        results[name] = (code, val)
        mark = "  " if code == 0 else "✗ "
        print(f"  {mark}{label:12s} {fmt(kind, val)}")
        print(f"      └ {meaning}")

    # ---- 汇总判读 ----
    print("\n" + "=" * 70)
    print("判读")
    print("=" * 70)
    hits = []

    def got(n):
        return results.get(n, (None, None))[1]

    if got("getSoftStopSwitch") not in (None, "CLOSE"):
        hits.append(f"软急停不是 CLOSE (={got('getSoftStopSwitch')}) "
                    f"-> 松开急停并复位")
    mode = got("getOperatingMode")
    if mode is not None and str(mode).upper() != "AUTO":
        hits.append(f"操作模式是 {mode}, 不是 AUTO -> 切到自动模式才接受任务")
    st = got("getAgvStatus")
    if st is not None and str(st).upper() in ("ERROR", "OFFLINE"):
        hits.append(f"AGV 状态 = {st} -> 先清报警/查连接")
    if got("getPaused") is True:
        hits.append("底盘处于【暂停】状态 -> 先恢复")
    if got("getResettingStatus") is True:
        hits.append("底盘正在【复位】 -> 等复位完成")
    if str(got("getRcsConnectionState") or "").upper() == "OFFLINE":
        hits.append("RCS 离线 -> 上层调度下不来任务")
    bs = got("getButtonStatus")
    if bs is not None:
        for b in ("powerButton", "resetButton"):
            if getattr(bs, b, 0):
                hits.append(f"{b} 处于按下状态")
    tid = got("getTaskId")
    if tid is not None and not str(tid).strip():
        hits.append("当前没有任务 ID -> 底盘根本没收到指令, 不是底盘故障")

    if hits:
        print("发现以下可能原因(按可能性排序):")
        for i, h in enumerate(hits, 1):
            print(f"  {i}. {h}")
    else:
        print("以上读到的状态里没有明显阻塞项。")
        print("接下来可查: 示教器/上位机是否有报警; 驱动轮抱闸是否释放;")
        print("           底盘急停回路(有些急停不体现在 getSoftStopSwitch);")
        print("           以及本轮子缺失的那几个接口(换 2.0 轮子再看)。")

    print("\n把完整输出贴回来, 我按实际字段再细化判读。")
    try:
        sdk.stop()
    except Exception:                          # noqa: BLE001
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
