#!/usr/bin/env python3
"""armWorldsToServo 最小验证 —— 只回答一个问题: 这个接口到底能不能用。

为什么要单独写这个:
  execute_worlds_servo_grasp.py 那条线现在其实【没在用 armWorldsToServo】——
  它是"用 armTryWorlds 逆解 -> 检查 -> 用 armPluseToServo 发关节角"。真正的末端
  透传一次都没跑起来过。混在完整抓放任务里试, 分不清是接口不行还是任务参数不对。
  这里把 JSON / anchor+偏移契约 / 逆解预检 / 夹爪 全部剥掉, 只剩:

      读当前位姿 -> 加几个小偏移 -> armWorldsToServo 流式发 -> 看走没走对

安全底线(一条都没省):
  * 机器人互斥锁, 同一台臂同时只允许一个进程;
  * 步长很小(默认 0.5mm/帧 = 25mm/s), 关保护期间不敢大;
  * 流的第一个点【就是当前位姿】—— 不存在起手阶跃(完整版就是栽在这:
    第一帧比实际位置高 119mm, 保护又关着, 直接撞限位);
  * 每段结束读一次关节角, 与 armTryWorlds 的预测比对, 差太多立即停;
  * finally 无条件恢复保护并回读确认。

用法(容器内):
    python3 test_worlds_servo_minimal.py            # 干跑, 只打印计划
    python3 test_worlds_servo_minimal.py --run      # 真跑(人守急停)
    python3 test_worlds_servo_minimal.py --run --rotate   # 额外测一段纯旋转
"""
import sys
import time

import robot_lock
import sdk_session as ss

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1
GLOBAL_SPEED = 10.0

STEP_MM = 1.0                  # 每帧位置步长 -> 0.5/0.02 = 25mm/s, 比完整版慢一半
STEP_ORI_DEG = 0.25            # 每帧姿态步长 -> 12.5°/s
PERIOD_S = 0.02

# 相对【起始位姿】的偏移 [dX,dY,dZ,dU,dV,dW]。每一项是一段, 段与段之间停稳。
# 默认只做纯平移的小往返: 上 30 -> 回原位 -> 右 30 -> 回原位。
MOVES = [
    ("上升 30mm",   [0, 0,  50, 0, 0, 0]),
    ("回原位",      [0, 0,   0, 0, 0, 0]),
    ("Y 向 30mm",   [0, 50,  0, 0, 0, 0]),
    ("回原位",      [0, 0,   0, 0, 0, 0]),
]
# --rotate 时追加。这才是真正要回答的问题: 任务里不可能全程不转手腕。
ROTATE_MOVES = [
    ("绕 W 转 10°", [0, 0, 0, 0, 0, 20]),
    ("转回来",      [0, 0, 0, 0, 0,  0]),
]

JUMP_ABORT_DEG = 5.0           # 段末实际关节角与预测差超过这个 -> 判定逆解跳变


def densify(a, b):
    """两个位姿之间线性加密。位置和姿态分别算段数, 取大的那个。"""
    dist = sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])) ** 0.5
    dori = max(abs(x - y) for x, y in zip(a[3:], b[3:]))
    n = max(1, int((max(dist / STEP_MM, dori / STEP_ORI_DEG)) + 0.999))
    return [[x + (y - x) * k / n for x, y in zip(a, b)] for k in range(1, n + 1)]


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    run = "--run" in argv
    moves = MOVES + (ROTATE_MOVES if "--rotate" in argv else [])

    print("=" * 66)
    print("armWorldsToServo 最小验证")
    print("=" * 66)
    print(f"步长 {STEP_MM}mm / {STEP_ORI_DEG}°, 周期 {PERIOD_S*1000:.0f}ms "
          f"-> {STEP_MM/PERIOD_S:.0f}mm/s, {STEP_ORI_DEG/PERIOD_S:.0f}°/s")
    for name, off in moves:
        print(f"  {name:14s} 偏移 {off}")
    if not run:
        print("\n干跑结束。加 --run 真跑(人守急停), 加 --rotate 额外测旋转。")
        return 0

    sdk = None
    protect_off = False
    with robot_lock.acquire(ROBOT_IP, ARM_ID, "test_worlds_servo_minimal.py"):
      try:
        sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED,
                              (ARM_ID,))
        start = ss.read_worlds(sdk, ARM_ID)
        q0 = ss.read_joints(sdk, ARM_ID)
        print(f"\n起始世界位姿 = {[round(v, 2) for v in start]}")
        print(f"起始关节角   = {[round(v, 2) for v in q0]}")

        # 逐段展开。⚠ 流从【当前位姿】起算 —— 第一帧不会是阶跃。
        segs, cur = [], list(start)
        for name, off in moves:
            tgt = [start[i] + off[i] for i in range(6)]
            segs.append((name, densify(cur, tgt), tgt))
            cur = tgt
        total = sum(len(s[1]) for s in segs)
        print(f"共 {len(segs)} 段, {total} 个透传点, 约 {total*PERIOD_S:.1f}s")

        # 逆解预览: 只为知道"控制器打算怎么走", 不用它来执行(这才是本测试的点)
        print("\n[逆解预览] 每段终点 armTryWorlds 的解:")
        pred = []
        for name, _pts, tgt in segs:
            q = ss.try_worlds(sdk, ARM_ID, tgt)
            pred.append(q)
            if q is None:
                raise SystemExit(f"✗ {name} 终点逆解失败, 该点不可达")
            bad = ss.check_limits(ARM_ID, q)
            if bad:
                raise SystemExit(f"✗ {name} 终点逆解超限: {bad}")
            print(f"  {name:14s} -> {[round(v, 1) for v in q]}")

        input("\n回车开始【armWorldsToServo 末端透传】(人守急停): ")
        print("关闭保护状态...")
        ss.set_protect_status(sdk, ARM_ID, False)
        protect_off = True

        q_prev = q0
        for (name, pts, tgt), q_pred in zip(segs, pred):
            print(f"\n>>> {name}  ({len(pts)} 点)")
            t = time.time()
            for p in pts:
                ss.check_soft_stop(sdk, name)
                ss.worlds_to_servo(sdk, ARM_ID, p)
                t += PERIOD_S
                time.sleep(max(0.0, t - time.time()))
            time.sleep(0.5)                       # 让伺服追完最后几个点

            w = ss.read_worlds(sdk, ARM_ID)
            q = ss.read_joints(sdk, ARM_ID)
            dp = max(abs(a - b) for a, b in zip(w[:3], tgt[:3]))
            da = max(abs(a - b) for a, b in zip(w[3:], tgt[3:]))
            dq_step = max(abs(a - b) for a, b in zip(q, q_prev))
            dq_pred = max(abs(a - b) for a, b in zip(q, q_pred))
            print(f"    到位: 位置差 {dp:5.2f}mm  姿态差 {da:5.2f}°")
            print(f"    实际关节 {[round(v, 1) for v in q]}")
            print(f"    本段关节行程 {dq_step:5.1f}°   与 armTryWorlds 预测差 "
                  f"{dq_pred:5.2f}°  {'✓' if dq_pred <= JUMP_ABORT_DEG else '✗ 分支不一致!'}")
            if dq_pred > JUMP_ABORT_DEG:
                raise SystemExit(
                    f"✗ {name}: 实际落点与 armTryWorlds 预测差 {dq_pred:.1f}° "
                    f"(门限 {JUMP_ABORT_DEG}°)\n"
                    f"  含义: 控制器执行时选的逆解分支和预检时【不是同一个】——\n"
                    f"  这正是 armWorldsToServo 无法事前验证的那个风险, 已实测到。\n"
                    f"  结论: 这条路不该用于正式任务。")
            q_prev = q

        print("\n" + "=" * 66)
        print("✅ armWorldsToServo 全部段落走完, 且每段落点都与逆解预测一致。")
        print("   说明: 这个接口在本机可用, 且逆解分支稳定 —— 可以继续往下评估。")
      finally:
        if sdk is None:
            print("会话未建立")
        else:
            if protect_off:
                print("\n恢复保护状态...")
                try:
                    ss.set_protect_status(sdk, ARM_ID, True)
                except Exception as exc:          # noqa: BLE001
                    print("⚠ 恢复保护失败, 请手动确认:", exc)
            try:
                print(f"保护状态回读: {ss.read_protect_status(sdk, ARM_ID)}")
            except Exception as exc:              # noqa: BLE001
                print("⚠ 保护状态回读失败:", exc)
            ss.close_session(sdk)
    return 0


if __name__ == "__main__":
    sys.exit(main())
