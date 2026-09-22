#!/usr/bin/env python3
"""armWorldsToServo 最小验证 —— 局部笛卡尔精修能力的验收工具。

20260804 真机结论(5 轮全过, 日志见
records/20260804_worlds_to_servo_minimal_runs/):
  armWorldsToServo 在当前机器人、当前 anchor 邻域内可用。
  已覆盖 30~50mm 平移、10~20° W 轴旋转、25~50mm/s 指令速度。
  最大位置误差 0.48mm / 姿态误差 0.17° / 实际关节与 armTryWorlds 终点预测
  最大差 0.96°(门限 5°)。未出现逆解分支跳变。

  ⚠ 已证明的只是"当前局部工作空间中平移和单轴小旋转可用"。
     没证明: 所有中间帧都选同一 IK 分支 / 不同 anchor / 带负载 / 复杂旋转 /
     完整抓放。这条线的定位是【局部精修】, 不是抓放主线。

⚠ 参数全部走命令行, 不要再手改常量。
   20260804 复查发现: 最强的几轮结果来自现场手改(50mm/20°/1.0mm), 而仓库里
   still 是 30mm/10°/0.5mm —— 仓库版本【复现不出】那几轮。而且日志里段名还写着
   "上升 30mm" 实际走了 50mm, 归档时会误导。现在段名由参数自动生成。

用法:
    python3 test_worlds_servo_minimal.py                      # 干跑
    python3 test_worlds_servo_minimal.py --run                # 默认 30mm/10°
    python3 test_worlds_servo_minimal.py --run \\
        --z-mm 50 --y-mm 50 --w-deg 20 --step-mm 1.0          # 复现 20260804 最强那轮
"""
import argparse
import hashlib
import signal
import sys
import time

import robot_lock
import sdk_session as ss
import servo_common as sc

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1

# ---- 实验验收门限(20260804 五轮实测全部满足, 以此为基线) ----
ACCEPT_POS_MM = 1.0            # 段末位置误差
ACCEPT_ORI_DEG = 0.5           # 段末姿态误差
ACCEPT_PRED_DEG = 2.0          # 实际关节 vs armTryWorlds 终点预测
JUMP_ABORT_DEG = 5.0           # 逐点预检: 相邻点关节跳变
MAX_JOINT_SPEED_DEG_S = 60.0   # 逐点预检: 由跳变/周期折算的关节速度


def build_moves(a):
    """按参数生成段落, 段名【直接反映实际数值】—— 不再出现"上升30mm 实走50mm"。"""
    moves = []
    for axis, val, name in ((0, a.x_mm, "X"), (1, a.y_mm, "Y"), (2, a.z_mm, "Z")):
        if val:
            off = [0.0] * 6
            off[axis] = float(val)
            moves.append((f"{name} 向 {val:+g}mm", off))
            moves.append((f"回原位(自{name})", [0.0] * 6))
    if a.w_deg:
        off = [0.0] * 6
        off[5] = float(a.w_deg)
        moves.append((f"绕 W 转 {a.w_deg:+g}°", off))
        moves.append(("回原位(自W)", [0.0] * 6))
    return moves


def densify(a, b, step_mm, step_deg):
    dist = sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])) ** 0.5
    dori = max(abs(x - y) for x, y in zip(a[3:], b[3:]))
    n = max(1, int(max(dist / step_mm, dori / step_deg) + 0.999))
    return [[x + (y - x) * k / n for x, y in zip(a, b)] for k in range(1, n + 1)]


def precheck_all(sdk, pts, q_now, seg, period_s):
    """逐点逆解预检 —— 不是只检段末。

    20260804 复查指出: 只检终点等于放过了中间所有帧。这里把整段每一个点都解一遍,
    检查 ①可达 ②不超限 ③相邻跳变 ④关节速度, 而且【第一个点要与当前实际关节角
    比】—— 否则起手那一跳查不出来(完整版 CW 就是栽在起手 119mm 阶跃上)。"""
    prev, worst_jump, worst_v = q_now, 0.0, 0.0
    sols = []
    for i, p in enumerate(pts):
        q = ss.try_worlds(sdk, ARM_ID, p)
        if q is None:
            raise SystemExit(f"✗ {seg} 第 {i} 点不可达/奇异: "
                             f"{[round(v, 1) for v in p[:3]]}")
        bad = ss.check_limits(ARM_ID, q)
        if bad:
            raise SystemExit(f"✗ {seg} 第 {i} 点逆解超限: {bad}")
        jump = max(abs(x - y) for x, y in zip(q, prev))
        v = jump / period_s
        worst_jump, worst_v = max(worst_jump, jump), max(worst_v, v)
        if jump > JUMP_ABORT_DEG:
            raise SystemExit(
                f"✗ {seg} 第 {i} 点关节跳变 {jump:.1f}° > {JUMP_ABORT_DEG}°\n"
                f"  {'起手跳变(当前位姿 -> 首帧)' if i == 0 else '逆解分支跳变'}")
        if v > MAX_JOINT_SPEED_DEG_S:
            raise SystemExit(f"✗ {seg} 第 {i} 点关节速度 {v:.0f}°/s > "
                             f"{MAX_JOINT_SPEED_DEG_S:.0f}°/s")
        sols.append(q)
        prev = q
    return sols, worst_jump, worst_v


def main(argv=None):
    ap = argparse.ArgumentParser(description="armWorldsToServo 最小验证")
    ap.add_argument("--run", action="store_true", help="真跑(不加则只打印计划)")
    ap.add_argument("--x-mm", type=float, default=0.0)
    ap.add_argument("--y-mm", type=float, default=30.0)
    ap.add_argument("--z-mm", type=float, default=30.0)
    ap.add_argument("--w-deg", type=float, default=0.0, help="绕 W 转多少度; 0=不测")
    ap.add_argument("--step-mm", type=float, default=0.5)
    ap.add_argument("--step-deg", type=float, default=0.25)
    ap.add_argument("--period-ms", type=float, default=20.0)
    ap.add_argument("--speed", type=float, default=3.0, help="全局速度 %%")
    a = ap.parse_args(argv)
    period = a.period_ms / 1000.0

    sha = hashlib.sha256(open(__file__, "rb").read()).hexdigest()[:16]
    moves = build_moves(a)
    print("=" * 68)
    print("armWorldsToServo 最小验证")
    print("=" * 68)
    print(f"脚本 SHA-256(前16) : {sha}")
    print(f"参数 : x={a.x_mm} y={a.y_mm} z={a.z_mm} w={a.w_deg}° "
          f"step={a.step_mm}mm/{a.step_deg}° period={a.period_ms}ms speed={a.speed}%")
    print(f"速度 : {a.step_mm/period:.0f}mm/s, {a.step_deg/period:.0f}°/s")
    print(f"验收门限 : 位置≤{ACCEPT_POS_MM}mm 姿态≤{ACCEPT_ORI_DEG}° "
          f"预测差≤{ACCEPT_PRED_DEG}°")
    for name, off in moves:
        print(f"  {name}")
    if not moves:
        print("\n没有任何段落 —— 至少给一个非零的 --x/y/z-mm 或 --w-deg")
        return 2
    if not a.run:
        print("\n干跑结束。加 --run 真跑(人守急停)。")
        return 0

    sdk, protect_off, results = None, False, []
    with robot_lock.acquire(ROBOT_IP, ARM_ID, "test_worlds_servo_minimal.py"):
      try:
        sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, a.speed, (ARM_ID,))
        start = ss.read_worlds(sdk, ARM_ID)
        q_now = ss.read_joints(sdk, ARM_ID)
        print(f"\n起始世界位姿 = {[round(v, 2) for v in start]}")
        print(f"起始关节角   = {[round(v, 2) for v in q_now]}")

        # 展开 + 逐点预检。流从【当前位姿】起算, 首帧不是阶跃。
        segs, cur, q_pred = [], list(start), q_now
        print("\n[逐点逆解预检]")
        for name, off in moves:
            tgt = [start[i] + off[i] for i in range(6)]
            pts = densify(cur, tgt, a.step_mm, a.step_deg)
            sols, jump, v = precheck_all(sdk, pts, q_pred, name, period)
            print(f"  {name:18s} {len(pts):4d} 点  最大跳变 {jump:.2f}°  "
                  f"最大关节速度 {v:5.1f}°/s  ✓")
            segs.append((name, pts, tgt, sols[-1]))
            cur, q_pred = tgt, sols[-1]
        total = sum(len(s[1]) for s in segs)
        print(f"  合计 {total} 点 x {a.period_ms:.0f}ms = {total*period:.1f}s")

        input("\n回车开始【armWorldsToServo 末端透传】(人守急停): ")
        print("关闭保护状态...")
        ss.set_protect_status(sdk, ARM_ID, False)
        protect_off = True

        pacer = sc.Pacer(period)
        for name, pts, tgt, q_expect in segs:
            print(f"\n>>> {name}  ({len(pts)} 点)")
            pacer.reset()
            for p in pts:
                ss.check_soft_stop(sdk, name)
                ss.worlds_to_servo(sdk, ARM_ID, p)
                pacer.tick()
            time.sleep(0.5)                       # 让伺服追完最后几个点

            w = ss.read_worlds(sdk, ARM_ID)
            q = ss.read_joints(sdk, ARM_ID)
            dp = max(abs(x - y) for x, y in zip(w[:3], tgt[:3]))
            da = max(abs(x - y) for x, y in zip(w[3:], tgt[3:]))
            dq = max(abs(x - y) for x, y in zip(q, q_expect))
            ok = dp <= ACCEPT_POS_MM and da <= ACCEPT_ORI_DEG and dq <= ACCEPT_PRED_DEG
            print(f"    位置差 {dp:5.2f}mm  姿态差 {da:5.2f}°  "
                  f"与预测差 {dq:5.2f}°  {'✓' if ok else '✗ 未达验收门限'}")
            results.append((name, dp, da, dq, ok))
            if not ok:
                raise SystemExit(
                    f"✗ {name} 未达验收门限 "
                    f"(位置≤{ACCEPT_POS_MM}mm 姿态≤{ACCEPT_ORI_DEG}° "
                    f"预测差≤{ACCEPT_PRED_DEG}°)")

        pacer.print_report("worlds 透传")
        print("\n" + "=" * 68)
        print(f"✅ {len(results)} 段全部通过验收门限。")
        print(f"   最大位置差 {max(r[1] for r in results):.2f}mm  "
              f"最大姿态差 {max(r[2] for r in results):.2f}°  "
              f"最大预测差 {max(r[3] for r in results):.2f}°")
      finally:
        # 清理期间屏蔽第二次 Ctrl-C —— 现场只按一次, 让恢复保护走完
        old = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        try:
            if sdk is None:
                print("会话未建立")
            else:
                if protect_off:
                    print("\n恢复保护状态(清理中, Ctrl-C 已屏蔽)...")
                    try:
                        ss.set_protect_status(sdk, ARM_ID, True)
                    except Exception as exc:      # noqa: BLE001
                        print("⚠ 恢复保护失败, 请手动确认:", exc)
                try:
                    st = ss.read_protect_status(sdk, ARM_ID)
                    print(f"保护状态回读: {st}  {'✓' if st else '✗ 请手动处理!'}")
                except Exception as exc:          # noqa: BLE001
                    print("⚠ 保护状态回读失败:", exc)
                ss.close_session(sdk)
        finally:
            signal.signal(signal.SIGINT, old)
    return 0


if __name__ == "__main__":
    sys.exit(main())
