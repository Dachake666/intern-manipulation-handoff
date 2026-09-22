#!/usr/bin/env python3
"""空爪诊断: armMoveJoints 大行程转运到底是"走得慢"还是"中途停住"。

背景(20260729): hybrid 模式的转运段连续两次 25s 未到位(残差 23.720° / 18.533°),
而 armMoveWorlds 在同一次运行里条条到位。现有日志只有一个总误差数字, 分不清:
  ① 还在慢慢走, 只是 25s 不够(那就该加时间);
  ② 中途停住/被拒/进保护(那加多久都没用)。
这两种情况的处置完全相反, 所以必须先测出来, 而不是带着物体反复试。

本脚本【不抓东西、不碰夹爪】, 只做一件事:
  armMoveWorlds 到 A 点 -> 读关节角 -> armMoveWorlds 到 B 点 -> 读关节角
  -> 回到 A -> 用一条 armMoveJoints 走到 B 的关节角, 全程 5Hz 记录:
       时间 / 逐关节实际角 / 逐关节误差 / move_state / 报警
最后打印误差随时间的曲线摘要, 直接回答"慢还是停"。

A/B 取自与真机任务同一几何: A = 抓取点上方悬停, B = 放置点上方悬停。
先把臂摆成工具朝下、夹爪中心在抓取点, 和正式任务取 anchor 的方式完全一致。

安全: 空爪运行; ENABLE_REAL_MOTION=False 时只打印计划。真跑人守急停。
用法(容器内): python3 diag_joint_transfer.py <轨迹.json>
"""
import json
import sys
import time

import sdk_session as ss

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1
GLOBAL_SPEED = 10.0            # 与真机现场一致(真机日志显示 set speed(10.0%))
ENABLE_REAL_MOTION = False

SAMPLE_HZ = 5.0
JOINT_TIMEOUT_S = 90.0         # 故意放长: 目的是看它到底要多久, 而不是卡在超时
ARRIVE_TOL_DEG = 0.5


def abs_pose(anchor, off_mm):
    return [anchor[0] + off_mm[0], anchor[1] + off_mm[1], anchor[2] + off_mm[2],
            anchor[3], anchor[4], anchor[5]]


def pick_ab(wps):
    """从轨迹里取 A=PICK_ASCEND 段末, B=转运段末(TRANSFER* 或 ARC12)。"""
    last = {}
    for w in wps:
        if "off_mm" in w:
            last[w["seg"]] = w["off_mm"]
    a = last.get("PICK_ASCEND")
    b = next((last[k] for k in ("TRANSFER_END", "TRANSFER_Y", "TRANSFER_X",
                                "TRANSFER", "ARC12") if k in last), None)
    if a is None or b is None:
        raise SystemExit(f"轨迹里找不到 A/B 段(有 {sorted(last)})")
    return a, b


def goto_world(sdk, pose, tag):
    ss.check_soft_stop(sdk, tag)
    ss.move_worlds(sdk, ARM_ID, pose, interpolation_en=False)
    ss.wait_until_worlds(sdk, ARM_ID, pose, label=f"({tag})")
    q = ss.read_joints(sdk, ARM_ID)
    print(f"  {tag} 关节角 = {[round(v, 2) for v in q]}")
    return q


def trace_joint_move(sdk, target):
    """下发一条 armMoveJoints, 全程采样, 返回采样表。不抛错 —— 诊断要看完过程。"""
    q0 = ss.read_joints(sdk, ARM_ID)
    span = [abs(a - b) for a, b in zip(q0, target)]
    print(f"\n  起点 = {[round(v, 2) for v in q0]}")
    print(f"  目标 = {[round(v, 2) for v in target]}")
    print(f"  逐关节行程 = {[round(v, 1) for v in span]}  最大 "
          f"j{span.index(max(span)) + 1}={max(span):.1f}°")
    ss.move_joints_abs(sdk, ARM_ID, target)
    t0 = time.time()
    rows = []
    while time.time() - t0 < JOINT_TIMEOUT_S:
        q = ss.read_joints(sdk, ARM_ID)
        err = [a - b for a, b in zip(q, target)]
        emax = max(abs(v) for v in err)
        try:
            busy = ss.read_move_state(sdk, ARM_ID)
        except Exception:                      # noqa: BLE001
            busy = None
        alarms = None
        try:
            _, alarms = sdk.armIsAlarming()
        except Exception:                      # noqa: BLE001
            pass
        rows.append({"t": time.time() - t0, "q": q, "err": err, "emax": emax,
                     "moving": busy, "alarms": alarms})
        print(f"    t={rows[-1]['t']:5.1f}s  最大误差 {emax:7.3f}°  "
              f"move_state={busy}  alarms={alarms}", end="\r")
        if emax <= ARRIVE_TOL_DEG:
            print()
            print(f"  ✓ 到位: {rows[-1]['t']:.1f}s, 最大误差 {emax:.3f}°")
            return rows
        time.sleep(1.0 / SAMPLE_HZ)
    print()
    print(f"  ✗ {JOINT_TIMEOUT_S:.0f}s 仍未到位, 最后误差 {rows[-1]['emax']:.3f}°")
    return rows


def verdict(rows):
    """给出判读: 慢 / 停 / 报警。"""
    if not rows:
        return
    print("\n" + "=" * 62)
    print("误差随时间(每 2s 取一个):")
    step = max(1, int(2.0 * SAMPLE_HZ))
    for r in rows[::step]:
        print(f"  t={r['t']:5.1f}s  {r['emax']:7.3f}°  moving={r['moving']}")
    last = rows[-1]
    print(f"  t={last['t']:5.1f}s  {last['emax']:7.3f}°  moving={last['moving']}  (末帧)")

    # 最后 4s 有没有实质改善
    tail = [r for r in rows if r["t"] >= last["t"] - 4.0]
    improved = tail[0]["emax"] - last["emax"] if len(tail) > 1 else 0.0
    moved = rows[0]["emax"] - last["emax"]
    rate = moved / last["t"] if last["t"] > 0 else 0.0
    print(f"\n总计: {last['t']:.1f}s 内误差从 {rows[0]['emax']:.1f}° 降到 "
          f"{last['emax']:.1f}° (平均 {rate:.2f}°/s)")
    print(f"最后 4s 改善 {improved:.2f}°")
    if any(r["alarms"] for r in rows):
        print("\n判读: 【有报警】—— 见上面的 alarms 字段, 先解报警。")
    elif last["emax"] <= ARRIVE_TOL_DEG:
        need = last["t"]
        print(f"\n判读: 【能到, 只是慢】走完要 {need:.0f}s。"
              f"把 wait_until_joints 的 timeout_s 提到 {need * 1.5:.0f}s 以上即可,"
              f" hybrid 方案本身没问题。")
    elif improved < 0.3 and last["moving"] is False:
        print("\n判读: 【真停了】控制器报未在运动且误差不再改善 —— 加时间没用。"
              "查该关节位形是否触发内部软限位/保护, 或换 anchor 姿态。")
    elif improved >= 0.3:
        print(f"\n判读: 【还在走, 只是慢】{JOINT_TIMEOUT_S:.0f}s 都没走完, "
              f"按当前速率还需约 {last['emax'] / max(rate, 1e-6):.0f}s。"
              f"这个速度不适合做转运, 建议维持 armMoveWorlds 直线。")
    else:
        print("\n判读: 介于两者之间, 把上面的表贴回来一起看。")


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("用法: diag_joint_transfer.py <轨迹.json>")
        return 2
    wps = json.load(open(argv[0], encoding="utf-8"))["waypoints"]
    off_a, off_b = pick_ab(wps)
    print(f"轨迹: {argv[0]}\nA(抓取点上方) off_mm={off_a}\nB(放置点上方) off_mm={off_b}")
    print("⚠ 本脚本【空爪运行】, 不会碰夹爪, 也不抓取任何物体。")
    if not ENABLE_REAL_MOTION:
        print("\nDRY RUN: 未下发。确认后改 ENABLE_REAL_MOTION=True 重跑。")
        return 0

    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED,
                          (ARM_ID,))
    try:
        anchor = ss.read_worlds(sdk, ARM_ID)
        print(f"\nanchor = {[round(v, 2) for v in anchor]}")
        pa, pb = abs_pose(anchor, off_a), abs_pose(anchor, off_b)
        input("回车开始(空爪, 人守急停): ")
        print("\n[1/3] armMoveWorlds 走到 A")
        goto_world(sdk, pa, "A")
        print("\n[2/3] armMoveWorlds 走到 B, 记下 B 的关节角")
        qb = goto_world(sdk, pb, "B")
        print("\n[3/3] 回 A, 然后用【一条 armMoveJoints】走到 B 的关节角")
        goto_world(sdk, pa, "A(回)")
        rows = trace_joint_move(sdk, qb)
        verdict(rows)
    finally:
        ss.close_session(sdk)
    return 0


if __name__ == "__main__":
    sys.exit(main())
