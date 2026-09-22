#!/usr/bin/env python3
"""Track C-worlds: 末端伺服透传(armWorldsToServo)执行世界坐标抓放。

与 execute_servo_grasp.py(关节透传)是【对称的一对】—— 同样的关保护范式、同样的
节拍器、同样的夹爪四步，区别只有一个：喂给伺服的是 [X,Y,Z,U,V,W] 而不是 7 个关节
角，逆解由控制器内部完成(手册 2.7.32：“配合 setRobotProtectStatus 使用，用于末端
摇操透传”)。

为什么值得做: 抓放任务天生是笛卡尔描述的(“下探 120mm”“横移 140mm”)。关节透传要先
把世界点逆解成关节角才能喂, 中间隔了一层; 末端透传直接喂世界点, 路径就是你画的
那条, 不会因为关节插值而鼓/塌。

⚠ 这条线独有的风险: 【逆解分支跳变】
  关节透传的路径是我们自己插的, 相邻两点必然连续。末端透传是控制器【每帧独立
  逆解】—— 同一个世界位姿可能有多组关节解, 万一相邻两帧解到了不同分支, 关节角
  会瞬间跳几十度, 而此时保护是关的。这是关节透传【不存在】的失效模式。
  对策: 下发前用 armTryWorlds(手册 2.7.53 逆解接口)把每个密集点先解一遍,
  逐点检查 ①可达 ②不超限 ③与上一点的关节跳变在门限内。三条全过才允许下发。
  (armTryWorlds 是 20260731 重读手册才发现的 —— 没有它这条线不该上机。)

安全: 与关节透传一致 —— 关保护期间失去突变保护, 故步长必须小、全轨迹预检、
夹爪期间临时恢复保护、finally 无条件恢复并回读确认。
ENABLE_REAL_MOTION=False 时只打印计划。真跑人守急停。

用法(容器内): python3 execute_worlds_servo_grasp.py <世界轨迹.json>
"""
import hashlib
import json
import math
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
GLOBAL_SPEED = 10.0
ENABLE_REAL_MOTION = True

# ---- 透传参数: 速度 = 步长 / 周期, 和关节透传同一个道理 ----
STEP_MM = 1.0                  # 相邻透传点的位置步长(mm) -> 1.0/0.02 = 50mm/s
STEP_ORI_DEG = 0.5             # 相邻透传点的姿态步长(度); 本轨迹 UVW 恒定, 用不上
PERIOD_S = 0.02                # 下发周期(50Hz)

# ---- 执行通道(20260731 复查后的结构性修改) ----
#   "joints" 【默认/推荐】用 armTryWorlds 把世界点逐个逆解成关节序列, 检查通过后
#            用 armPluseToServo 执行【这条完全相同的关节序列】。
#            为什么这样才对: 原来的做法是"用 armTryWorlds 检查, 然后把关节解丢掉,
#            让 armWorldsToServo 重新逆解一遍"—— 两次可能选不同分支, 于是
#            【检查的不是执行的】, 那个预检等于白做。改成执行同一条序列后,
#            "逆解分支跳变"这个失效模式从根上不存在了, 而且复用的是已经真机跑通
#            过的 pulse 通道。
#   "worlds" [实验] 真·末端透传 armWorldsToServo。控制器每帧独立逆解, 我们无法
#            保证它选的分支和预检时一致。在有更强证据之前不要用它跑完整任务。
EXEC_VIA = "joints"

# ---- 逆解预检(本线的命门) ----
IK_PRECHECK = True             # 关掉等于裸奔, 除非你在诊断
IK_MAX_JOINT_SPEED_DEG_S = 60.0  # 由相邻跳变/周期折算的关节速度上限
IK_JUMP_ABORT_DEG = 5.0        # 相邻密集点的关节跳变超过此值 -> 判定为分支跳变, 拒绝
IK_JUMP_WARN_DEG = 2.0

# ---- 停稳判据(与关节透传一致的三条件) ----
SETTLE_TOL_MM = 3.0            # worlds 通道: 位置容差
SETTLE_TOL_DEG = 0.8           # joints 通道: 关节容差(与 pulse 线一致)
SETTLE_DELTA_DEG = 0.05        # joints 通道: 判"不动了"的帧间变化上限
SETTLE_TOL_ORI_DEG = 1.5       # worlds 通道: 姿态容差
SETTLE_DELTA_MM = 0.3
SETTLE_FRAMES = 3
SETTLE_TIMEOUT_S = 15.0

# ---- 观测(与关节透传共用 servo_common) ----
FOLLOW_SAMPLE_EVERY = 0        # 0 = 关闭(默认)。⚠ 20260731 教训: 开着它会往
                               # 透传热循环里塞阻塞 SDK 读, 实测造成 5Hz 周期性微顿,
                               # 观感明显不如未优化版。只在诊断时开。
FOLLOW_WARN_DEG = 3.0

# ---- 负载 / 限位 ----
PAYLOAD_KG = 0.2
PAYLOAD_COM_M = (0.0, 0.0, 0.0)
USE_PAYLOAD = False            # ⚠ 20260803 probe: 本机 armSetRobotLoad 签名是
                               # (int, int), 第 3 参那个整数代表什么【未知】;
                               # armUnsetLoad 本机不存在, 连"卸载"都没手段。
                               # 探明语义前不要打开 —— 传错会让动力学补偿更离谱。
CONTROLLER_LIMITS_MODE = "enforce"      # report | enforce | off
# 20260803 现场决定: 先只 report 一轮, 人工核对那 7 组数字无误后再手动改 enforce。
# 理由: 采纳控制器限位是【改变安全判定依据】的动作, 不该由我根据一次日志替你决定。
#
# ⚠ 后果要知道: report 模式下 worlds 的逆解预检【仍会在 TRANSFER_Y 被拦】——
#    j4=-115.01 落在本地 URDF 表 [-120,0] 收 5° 后的 [-115,-5] 之外, 但它在
#    控制器真值 [-128,+103] 里完全合法。那是本地表过紧造成的假超限, 不是真问题。
#    核对完数字, 把这里改成 "enforce" 就能过。
# 20260803 真机对比后改为 enforce: 本地 URDF 表与控制器真值差得很远, 而且【两个
# 方向都错】—— j1/j4 本地过紧(造成假超限, worlds 预检就是被这个卡死: j4=-115.01
# 明明在控制器 [-128,+103] 之内却被本地 [-120,0] 判超), j6 本地过松(45 vs 真值 40,
# 会放行实际撞限位的点)。控制器才是权威。
# 采纳前必过 axis_limits_look_valid(): 数据未就绪时会读到全 0, 那种值一旦采纳
# 会把任何轨迹全判超限。      # report | enforce | off

# 手册里有、但【本机 SDK 轮子不一定有】的接口。20260731 真机实测: 4 个手册接口在
# 这台机器上不存在或签名不同。开机探一次, 没有就整段跳过, 不再每次运行刷警告。
# 20260803 probe 基线(记录见 records/20260803_sdk_capability_baseline/):
#   本机【有】: armGetAxisParameter(第4参要 AxisParameterIndex 枚举) / armTryWorlds
#              (DoubleVector) / armJointOutLimit / armGetSpeedFeedback /
#              armGetRobotPaused / armGetRobotProtectStatus / armGetGlobalSpeed /
#              armGetRobotAxisMoveState / armSetEnableTolerance
#   本机【无】: armUnsetLoad / armGetPosReFreshMS / armSetPosReFreshMS /
#              armRobotSoftEmergencyStop
#   签名对但【语义未知】: armSetRobotLoad(int, int)
OPTIONAL_CAPS = ("armGetAxisParameter", "armSetRobotLoad", "armUnsetLoad",
                 "armGetSpeedFeedback", "armGetRobotPaused",
                 "armGetRobotProtectStatus", "armGetGlobalSpeed", "armTryWorlds",
                 "armJointOutLimit")

GRIPPER_ID = 2
GRIPPER_COM = "COM1"
GRIPPER_SERIAL = (115200, 8, 0, 1)
GRIPPER_CLOSE = [500, 1000]
GRIPPER_OPEN = [500]


# ---------------------------------------------------------------- 夹爪(已验证接口)
def build_gripper_cmd(gid, command, values):
    data = []
    for v in values:
        v = max(0, min(0xFFFF, int(v)))
        data += [v & 0xFF, (v >> 8) & 0xFF]
    payload = [int(gid) & 0xFF, 1 + len(data), int(command) & 0xFF] + data
    return " ".join(f"{b:02x}" for b in
                    [0xEB, 0x90] + payload + [sum(payload) & 0xFF])


def open_gripper_com(sdk):
    code = sdk.armOpenCom(GRIPPER_COM, *GRIPPER_SERIAL)
    print("armOpenCom:", (code, GRIPPER_COM, *GRIPPER_SERIAL))
    ss.require_code_zero(code, "armOpenCom")


def gripper(sdk, action):
    hexcmd = (build_gripper_cmd(GRIPPER_ID, 0x11, GRIPPER_OPEN) if action == "open"
              else build_gripper_cmd(GRIPPER_ID, 0x10, GRIPPER_CLOSE))
    print(f"  [夹爪{GRIPPER_ID}] {action} hex={hexcmd}")
    if ENABLE_REAL_MOTION:
        code = sdk.armWriteCom(GRIPPER_COM, bytes.fromhex(hexcmd.replace(" ", "")))
        ss.require_code_zero(code, f"夹爪{GRIPPER_ID} {action}")
        time.sleep(5.0 if action == "close" else 2.5)


# ---------------------------------------------------------------- 轨迹
def load_traj(path):
    raw = open(path, "rb").read()
    print(f"轨迹 SHA-256: {hashlib.sha256(raw).hexdigest()}")
    d = json.loads(raw.decode("utf-8"))
    if "single_relative" not in d["meta"].get("task", ""):
        raise SystemExit("非锚点+相对偏移轨迹(task 应含 single_relative), 拒绝")
    return d["meta"], d["waypoints"]


def abs_pose(anchor, off_mm):
    """每点 world = anchor.xyz + off_mm, 姿态全程锁 anchor 的 UVW(与 Track A 同契约)。"""
    return [anchor[0] + off_mm[0], anchor[1] + off_mm[1], anchor[2] + off_mm[2],
            anchor[3], anchor[4], anchor[5]]


def corners_of(wps):
    """段末压缩: 同名段只保留最后一点(它才是拐角), 夹爪事件原样保留。
    生成侧为了仿真按 8mm 加密过, 那个密度不是我们透传要的密度 —— 这里先压回
    拐角, 再按 STEP_MM 重新加密。"""
    out = []
    for i, w in enumerate(wps):
        if "gripper" in w:
            out.append(dict(w))
            continue
        nxt = wps[i + 1] if i + 1 < len(wps) else None
        if nxt is None or "gripper" in nxt or nxt["seg"] != w["seg"]:
            out.append(dict(w))
    return out


def densify_pose(a, b, step_mm=STEP_MM, step_ori=STEP_ORI_DEG):
    """两个世界位姿之间线性加密, 返回不含 a 的点列。

    位置和姿态【分别】算需要多少段, 取大的那个 —— 位置走得远就按位置切, 姿态转
    得多就按姿态切, 保证两者的单步都不超门限。这是与关节透传 densify 的唯一结构
    性差异(那边只有一种单位)。"""
    dist = math.dist(a[:3], b[:3])
    dori = max(abs(x - y) for x, y in zip(a[3:], b[3:]))
    n = max(1, int(math.ceil(max(dist / step_mm, dori / step_ori))))
    return [[x + (y - x) * k / n for x, y in zip(a, b)] for k in range(1, n + 1)]


# ---------------------------------------------------------------- 逆解预检
def ik_precheck(sdk, dense_stream):
    """把每个密集点逆解一遍, 检查 可达 / 限位 / 分支连续性。

    这是末端透传【必须】做而关节透传不必做的一步。关节透传的路径是我们自己插的,
    相邻点天然连续; 末端透传由控制器每帧独立逆解, 相邻帧解到不同分支时关节角会
    瞬间跳变几十度 —— 而那时保护是关的。这里提前把整条流解一遍就能发现。

    返回 (逐点关节解, 最大相邻跳变, 出问题的位置)。"""
    print(f"\n[逆解预检] armTryWorlds 逐点求解 {len(dense_stream)} 个点...")
    joints, prev, worst_jump, worst_at = [], None, 0.0, None
    for i, (pose, seg) in enumerate(dense_stream):
        q = ss.try_worlds(sdk, ARM_ID, pose)
        if q is None:
            raise SystemExit(
                f"✗ 逆解失败: 第 {i} 点({seg}) world="
                f"{[round(v,1) for v in pose[:3]]} 不可达或奇异。\n"
                f"  对策: 把抓放点挪近机器人, 或换个 anchor 姿态。")
        bad = ss.check_limits(ARM_ID, q)
        if bad:
            raise SystemExit(
                f"✗ 第 {i} 点({seg}) 的逆解超限:\n    " + "\n    ".join(bad) +
                f"\n  world={[round(v,1) for v in pose[:3]]}")
        if prev is not None:
            jump = max(abs(a - b) for a, b in zip(q, prev))
            if jump > worst_jump:
                worst_jump, worst_at = jump, (i, seg)
            if jump > IK_JUMP_ABORT_DEG:
                per = " ".join(f"j{k+1}:{a-b:+.1f}"
                               for k, (a, b) in enumerate(zip(q, prev)))
                raise SystemExit(
                    f"✗ 逆解分支跳变: 第 {i} 点({seg}) 相邻关节跳 {jump:.1f}° "
                    f"(门限 {IK_JUMP_ABORT_DEG}°)\n    逐关节: {per}\n"
                    f"  含义: 控制器在这两帧解到了【不同的逆解分支】。透传时保护是关的,\n"
                    f"  这一跳会让手臂猛甩。必须改路径(绕开奇异/换 anchor 姿态)后再跑。")
        joints.append(q)
        prev = q
    flag = "⚠ 偏大" if worst_jump > IK_JUMP_WARN_DEG else "✓"
    print(f"  全部可达且不超限。相邻最大关节跳变 {worst_jump:.2f}° "
          f"@ 第{worst_at[0] if worst_at else 0}点"
          f"{f'({worst_at[1]})' if worst_at else ''}  {flag}")
    return joints, worst_jump


def check_joint_speed(joints, period_s, limit_deg_s):
    """由相邻关节解的差值折算关节速度, 超限就拒绝。

    逆解预检只看"跳变有没有超门限"还不够: 一串每步 4° 的解在 5° 门限下全都合格,
    但 4°/20ms = 200°/s, 远超伺服能跟的速度。位置连续 != 速度可行。"""
    worst, at = 0.0, 0
    for i, (a, b) in enumerate(zip(joints, joints[1:]), start=1):
        v = max(abs(x - y) for x, y in zip(a, b)) / period_s
        if v > worst:
            worst, at = v, i
    if worst > limit_deg_s:
        raise SystemExit(
            f"✗ 关节速度超限: 第 {at} 点处 {worst:.0f}°/s > 门限 {limit_deg_s:.0f}°/s\n"
            f"  位置是连续的, 但走这么快伺服跟不上。把 STEP_MM 调小或 PERIOD_S 调大。")
    return worst


# ---------------------------------------------------------------- 停稳
def wait_settled_joints(sdk, target_q, tag):
    """EXEC_VIA="joints" 时的停稳判据 —— 在【关节空间】判。

    为什么不沿用世界位姿判据: 这条通道下发的是关节角, 停稳目标就是预检算出的段末
    关节角, 直接比对最准; 而世界位姿反馈是 50ms 刷新(本机改不了), 还要多绕一层
    正解。"检查的/执行的/验收的"三者用同一组量, 才叫闭环。"""
    deadline = time.time() + SETTLE_TIMEOUT_S
    prev, stable = None, 0
    err = delta = float("inf")
    moving = None
    while time.time() < deadline:
        q = ss.read_joints(sdk, ARM_ID)
        err = max(abs(a - b) for a, b in zip(q, target_q))
        try:
            moving = ss.read_move_state(sdk, ARM_ID)
        except Exception:                      # noqa: BLE001
            moving = None
        delta = (max(abs(a - b) for a, b in zip(q, prev))
                 if prev is not None else float("inf"))
        prev = q
        if moving is False and delta <= SETTLE_DELTA_DEG and err <= SETTLE_TOL_DEG:
            stable += 1
            if stable >= SETTLE_FRAMES:
                print(f"    {tag} 停稳: 关节误差 {err:.3f}° (连续{SETTLE_FRAMES}帧)")
                return q
        else:
            stable = 0
        time.sleep(0.1)
    speeds = None
    try:
        speeds = ss.read_speed_feedback(sdk, ARM_ID)
    except Exception:                          # noqa: BLE001
        pass
    per = " ".join(f"j{k+1}:{a-b:+.2f}" for k, (a, b) in enumerate(zip(prev, target_q)))
    spd = ("\n  逐关节速度: " + " ".join(f"j{k+1}:{v:g}"
           for k, v in enumerate(speeds))) if speeds else ""
    raise RuntimeError(f"{tag} {SETTLE_TIMEOUT_S}s 未达停稳判据(关节空间)\n"
                       f"  最大误差 {err:.3f}° 帧间变化 {delta:.3f}° "
                       f"move_state={moving}\n  逐关节: {per}{spd}")


def wait_settled(sdk, target6, tag):
    """三条件停稳(与关节透传同一套思路, 只是量的是世界位姿):
    ①控制器报已停 ②帧间位置变化极小 ③与目标的位置/姿态误差在容差内, 连续 N 帧。"""
    deadline = time.time() + SETTLE_TIMEOUT_S
    prev, stable = None, 0
    dp = da = delta = float("inf")
    moving = None
    while time.time() < deadline:
        w = ss.read_worlds(sdk, ARM_ID)
        dp = max(abs(a - b) for a, b in zip(w[:3], target6[:3]))
        da = max(abs(a - b) for a, b in zip(w[3:], target6[3:]))
        try:
            moving = ss.read_move_state(sdk, ARM_ID)
        except Exception:                      # noqa: BLE001
            moving = None
        delta = (max(abs(a - b) for a, b in zip(w[:3], prev[:3]))
                 if prev is not None else float("inf"))
        prev = w
        if (moving is False and delta <= SETTLE_DELTA_MM
                and dp <= SETTLE_TOL_MM and da <= SETTLE_TOL_ORI_DEG):
            stable += 1
            if stable >= SETTLE_FRAMES:
                print(f"    {tag} 停稳: 位置差 {dp:.2f}mm 姿态差 {da:.2f}° "
                      f"(连续{SETTLE_FRAMES}帧)")
                return w
        else:
            stable = 0
        time.sleep(0.1)
    speeds = None
    try:
        speeds = ss.read_speed_feedback(sdk, ARM_ID)
    except Exception:                          # noqa: BLE001
        pass
    spd = ("\n  逐关节速度: " + " ".join(f"j{k+1}:{v:g}"
                                     for k, v in enumerate(speeds))) if speeds else ""
    verdict = ("判读: 速度全 0 = 真停住了" if speeds and
               not any(abs(v) > 1e-6 for v in speeds) else "判读: 还在动 = 走得慢")
    raise RuntimeError(f"{tag} {SETTLE_TIMEOUT_S}s 未达停稳判据\n"
                       f"  位置差 {dp:.2f}mm 姿态差 {da:.2f}° 帧间变化 {delta:.2f}mm "
                       f"move_state={moving}{spd}\n  {verdict}")


# ---------------------------------------------------------------- 主流程
def build_stream(anchor, wps):
    """把轨迹展开成 [("move", 点列, 段末目标) | ("grip", 动作, 段末目标)]。"""
    plan, prev = [], None
    for c in corners_of(wps):
        if "gripper" in c:
            plan.append(("grip", c["gripper"], prev))
            continue
        pose = abs_pose(anchor, c["off_mm"])
        if prev is not None:
            plan.append(("move", densify_pose(prev, pose), pose, c["seg"]))
        prev = pose
    return plan


def main(argv=None):
    argv = list(argv if argv is not None else sys.argv[1:])
    precheck_only = "--precheck-only" in argv
    if precheck_only:
        argv.remove("--precheck-only")
    if not argv:
        print("用法: execute_worlds_servo_grasp.py <世界轨迹.json> [--precheck-only]")
        return 2
    meta, wps = load_traj(argv[0])
    print(f"轨迹: {argv[0]}  执行通道: {EXEC_VIA}"
          f"{'  【只做逆解预检, 机械臂不动】' if precheck_only else ''}")
    print(f"步长 {STEP_MM}mm / {STEP_ORI_DEG}° , 周期 {PERIOD_S*1000:.0f}ms "
          f"-> 约 {STEP_MM/PERIOD_S:.0f} mm/s")
    print(f"契约: {meta.get('exec_contract', '')}")

    if not ENABLE_REAL_MOTION and not precheck_only:
        plan = build_stream([0.0] * 6, wps)
        n = sum(len(x[1]) for x in plan if x[0] == "move")
        print(f"\n透传密集点: {n} 个, 预计时长 {n*PERIOD_S:.1f}s")
        for item in plan:
            if item[0] == "grip":
                print(f"  [夹爪 {item[1]}]")
            else:
                print(f"  {item[3]:16s} {len(item[1]):4d} 点 -> "
                      f"偏移 {[round(v,1) for v in item[2][:3]]}")
        print("\nDRY RUN: 未下发。确认后改 ENABLE_REAL_MOTION=True 重跑;"
              " 或加 --precheck-only 连上机器人只算逆解(机械臂不动)。")
        return 0

    sdk = None
    protect_off = False
    pacer = sc.Pacer(PERIOD_S)
    follow = sc.FollowMonitor(ARM_ID, FOLLOW_SAMPLE_EVERY, FOLLOW_WARN_DEG)
    with robot_lock.acquire(ROBOT_IP, ARM_ID, "execute_worlds_servo_grasp.py"):
      try:
        # --precheck-only = 【只读会话】: 不清报警、不使能、不设速度、不开夹爪、
        # 不关保护。armTryWorlds 是纯运动学计算, 不需要机械臂上电 —— 更不该
        # 顺手把它使能起来。这样"只想看看这条路径行不行"就完全无风险。
        sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED,
                              (ARM_ID,), enable=not precheck_only)
        caps = ss.probe_caps(sdk, OPTIONAL_CAPS)
        ss.print_caps(caps)
        if not caps.get("armTryWorlds"):
            raise SystemExit(
                "✗ 本机没有 armTryWorlds —— 没有逆解就无法做分支跳变预检, "
                "这条线不该上机。")
        if CONTROLLER_LIMITS_MODE != "off" and caps.get("armGetAxisParameter"):
            try:
                real = ss.read_axis_limits(sdk, ARM_ID)
                ok, bad_ax = ss.axis_limits_look_valid(real)
                if not ok:
                    print(f"⚠ 读回的轴极限不像真值({', '.join(bad_ax)}), "
                          f"多半是数据未就绪 —— 忽略, 沿用本地表")
                else:
                    adopt = (CONTROLLER_LIMITS_MODE == "enforce")
                    sc.describe_axis_limits(real, ss.LIMITS_DEG[ARM_ID], adopted=adopt)
                    if adopt:
                        ss.LIMITS_DEG[ARM_ID] = real
            except Exception as exc:          # noqa: BLE001
                print(f"⚠ 读取控制器轴极限失败({exc}), 沿用本地表")

        anchor = ss.read_worlds(sdk, ARM_ID)
        print(f"\nanchor 世界位姿 = {[round(v, 2) for v in anchor]}")
        print(f"anchor 关节角   = "
              f"{[round(v, 2) for v in ss.read_joints(sdk, ARM_ID)]}")
        print(f"姿态: 全程锁定 anchor 的 UVW=[{anchor[3]:.1f},{anchor[4]:.1f},"
              f"{anchor[5]:.1f}]")

        plan = build_stream(anchor, wps)
        n_dense = sum(len(x[1]) for x in plan if x[0] == "move")
        print(f"透传密集点: {n_dense} 个, 预计时长 {n_dense*PERIOD_S:.1f}s")

        # ---- 逆解预检: 逐段求解, 【把关节解留下来】 ----
        seg_joints = {}
        if IK_PRECHECK:
            worst_all = 0.0
            for idx, item in enumerate(plan):
                if item[0] != "move":
                    continue
                js, jump = ik_precheck(sdk, [(p, item[3]) for p in item[1]])
                v = check_joint_speed(js, PERIOD_S, IK_MAX_JOINT_SPEED_DEG_S)
                print(f"    {item[3]:16s} 最大关节速度 {v:5.1f}°/s "
                      f"(门限 {IK_MAX_JOINT_SPEED_DEG_S:.0f})")
                seg_joints[idx] = js
                worst_all = max(worst_all, jump)
            print(f"\n[逆解预检] 全部通过。全程最大相邻跳变 {worst_all:.2f}°")
        elif EXEC_VIA == "joints":
            raise SystemExit("✗ EXEC_VIA='joints' 需要逆解结果, 不能关 IK_PRECHECK")

        if precheck_only:
            print("\n✅ --precheck-only 完成: 只做了逆解与校验, 机械臂全程未动,"
                  "\n   未使能、未开夹爪、未关保护。")
            return 0

        open_gripper_com(sdk)
        gripper(sdk, "open")
        input("\n回车开始执行(人守急停): ")
        print("关闭保护状态(透传前置条件)...")
        ss.set_protect_status(sdk, ARM_ID, False)
        protect_off = True

        pacer.reset()
        last_target = None
        last_seg_q = None
        for idx, item in enumerate(plan):
            if item[0] == "grip":
                if item[2] is not None:
                    if EXEC_VIA == "joints" and last_seg_q is not None:
                        wait_settled_joints(sdk, last_seg_q, "夹爪前")
                    else:
                        wait_settled(sdk, item[2], "夹爪前")
                ss.set_protect_status(sdk, ARM_ID, True)
                protect_off = False
                gripper(sdk, item[1])
                ss.set_protect_status(sdk, ARM_ID, False)
                protect_off = True
                pacer.reset()      # ⚠ 必须: 否则会把夹爪那几秒"追回来"
                continue
            if EXEC_VIA == "joints":
                # 执行【预检时算出并检查过的那条关节序列】, 不让控制器重新逆解。
                for q in seg_joints[idx]:
                    ss.check_soft_stop(sdk, "servo(joints)")
                    ss.pulse_to_servo(sdk, ARM_ID, q)
                    follow.maybe_sample(sdk, commanded=q, label=item[3])
                    pacer.tick()
                last_seg_q = seg_joints[idx][-1]
            else:
                for pose in item[1]:
                    ss.check_soft_stop(sdk, "worlds servo")
                    ss.worlds_to_servo(sdk, ARM_ID, pose)
                    follow.maybe_sample(sdk, label=item[3])
                    pacer.tick()
            last_target = item[2]

        if EXEC_VIA == "joints" and last_seg_q is not None:
            fq = wait_settled_joints(sdk, last_seg_q, "终点")
            err = max(abs(a - b) for a, b in zip(fq, last_seg_q))
            w = ss.read_worlds(sdk, ARM_ID)
            print(f"\n✅ 执行完成(joints 通道): 终点关节误差 {err:.3f}° (已确认停稳)")
            print(f"   对应世界位置 {[round(v,1) for v in w[:3]]} "
                  f"(目标 {[round(v,1) for v in last_target[:3]]})")
        else:
            final = wait_settled(sdk, last_target, "终点")
            dp = max(abs(a - b) for a, b in zip(final[:3], last_target[:3]))
            print(f"\n✅ 执行完成(worlds 通道): 终点位置差 {dp:.2f}mm (已确认停稳)")
        pacer.print_report(f"worlds 规划 / {EXEC_VIA} 执行")
        follow.print_report()
      finally:
        # ⚠ 不能在 finally 里 return —— 会吞掉正在传播的异常。
        if sdk is None:
            print("会话未建立(初始化阶段就失败), 无需收尾")
        else:
          if protect_off:
              print("恢复保护状态...")
              try:
                  ss.set_protect_status(sdk, ARM_ID, True)
              except Exception as exc:      # noqa: BLE001
                  print("⚠ 恢复保护失败, 请手动确认:", exc)
          try:
              st = ss.read_protect_status(sdk, ARM_ID)
              print(f"保护状态回读: {st}  "
                    f"{'✓' if st else '✗ 仍未开启, 请手动处理!'}")
          except Exception as exc:          # noqa: BLE001
              print("⚠ 保护状态回读失败, 请手动确认:", exc)
          ss.close_session(sdk)
    return 0


if __name__ == "__main__":
    sys.exit(main())
