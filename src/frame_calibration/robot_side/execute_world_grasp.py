#!/usr/bin/env python3
"""真机执行 world_grasp 的锚点+相对偏移抓取轨迹(move worlds 主线, 平滑第一优先级)。

契约(见 world_grasp_latest.json meta): 把左臂摆成"工具竖直朝下、夹爪中心位于物体
实际抓取点"读一次 armGetWorlds 得 anchor[X,Y,Z,U,V,W]; 每个路点 world =
anchor.xyz + off_mm, 姿态全程用 anchor 的 tool-down UVW(固定)。偏移免标定。

执行模式(20260729 晚修订: hybrid 真机三连败, 回退到全直线):
  MODE="point"   【默认/主线】逐拐角 armMoveWorlds 直线 + 每条等到位。几何完全
                 确定。20260729 用这个模式 + 12 点弧轨迹真机连成 3 次(到位误差
                 0.8~3.9mm)。配 TRANSFER_MODE="line" 的轨迹后, 转运并成 1~2 条
                 单轴长直线 —— 命令类型和已成功那版完全一样, 只是步子大了。
  MODE="hybrid"  ✗【暂停使用, 未验证】转运用一条 armMoveJoints(关节空间)。原理上
                 更快更连贯, 但 20260729 真机连续三次失败:
                   run1  预检与执行前的限位标准冲突而中止(已修, 见 preflight);
                   run2  armMoveJoints 25s 未到位, 残差 23.720°;
                   run3  同上, 残差 18.533°(j4 行程约 53°, 25s 走了约 34°)。
                 两次都发生在【夹爪已闭合、物体在手】之后。现有日志分不清是
                 "走得慢没走完"还是"中途停住"(wait_until_joints 当时没有停滞检测,
                 现已补上)。要继续研究请先跑 diag_joint_transfer.py 空爪诊断,
                 不要带物重复试。
  MODE="curve"   ✗ 不可用留档。armMoveWorldsCurve 返回 0 但控制器不执行(两次
                 实测手臂原地未动、位置差=整段行程 140mm、move_state=False)。
  MODE="smooth"  拐角压缩 + 组内背靠背 armMoveWorlds(interpolation_en
                 =True) + 组末可靠到位。原理: armMoveWorlds 本身就是"直线运动",
                 每段直线只需要【段末拐角】一个命令 —— 之前逐 8mm 密集点是 79 次
                 完整停稳, 这就是卡顿根源。拐角压缩后全程只有 6 个运动命令;
                 组内(两次夹爪动作之间)背靠背下发让 interpolation_en 在拐角平滑
                 衔接。运动中 10Hz 采样实际位姿, 结束报告每个中途拐角的最近
                 通过距离(回答"中途点有没有被跳过")。
  MODE="servo"   [留档存档, 非主线] armWorldsToServo 密集流式。原理复杂(关保护+
                 高频透传), 按 20260724 决策仅留档, 不作为主攻方向。

教训留档(20260723 真机): 夹爪必须用已验证接口 armOpenCom("COM1",115200,8,0,1) +
armWriteCom(COM1, bytes) —— 之前误写 armSendManifoldData/armSendData(未验证接口)
导致夹爪不动。铁律: 封装/复用已在 execute_trajectory.py 真机成功的接口, 不发明新的。

姿态策略(20260725 放宽): 不再强制工具竖直朝下(REQUIRE_TOOL_DOWN=False)。多次真机
限位失败的主因是为凑竖直姿态把腕关节逼到限位边缘; 用 VR 起始位姿的自然倾斜姿态
同样能抓, 关节余量大得多。全程锁定 anchor 当时的 UVW。

限位预检(PREFLIGHT=True): 正式执行前逐个拐角低速试探并读关节角, 报告每个拐角的
关节余量; 余量 <3° 立即中止并给出对策(换 anchor 姿态/调 j1,j4/挪近抓放点)。
先知道"哪个拐角、哪个关节会顶到", 而不是跑到一半撞限位。

安全: ENABLE_REAL_MOTION=False 只打印不下发。真跑人守急停, 低速。
运行(Linux 容器内): python3 execute_world_grasp.py <轨迹.json>
"""
import hashlib
import math
import sys
import time

import robot_lock
import sdk_session as ss

ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP   = "192.168.8.148"
ARM_PORT = 8080
ARM_ID   = 1
GLOBAL_SPEED = 3.0
ENABLE_REAL_MOTION = False
MODE = "point"                 # "point"【主线】全 armMoveWorlds 直线, 逐条等到位 |
                               # "hybrid"(转运走关节, 20260729 真机 3 次失败, 暂停) |
                               # "curve"(不可用) | "smooth" | "servo"(留档)
# 20260728 真机定性: armMoveWorldsCurve 返回码 0 但控制器【完全不执行】(两次实测
# 手臂原地未动/位置差=整段行程/move_state=False), 而同一次运行的 armMoveWorlds
# 直线命令全部到位(误差 0.6~2.0mm)。因此主线改为全直线, curve 仅留档勿用。

# 取 anchor 的方式: "current"=读当前姿态(先把臂摆成工具朝下、夹爪中心在抓取点);
#                   "reference"=先移动到下方参考 tool-down 关节姿态再读(空爪测试用)。
ANCHOR_FROM = "current"
REFERENCE_TOOL_DOWN_Q_SDK = [-82.7, 39.4, 127.8, -85.8, -72.4, -14.7, 45.7]

# 姿态要求(20260725 放宽): 不再强制工具竖直朝下。实测多次限位失败的主因就是
# 为凑竖直姿态把腕部关节逼到限位边缘 —— 而抓取并不要求严格竖直, 用 VR 起始
# 位姿的自然倾斜姿态同样能抓, 且关节余量大得多。
#   REQUIRE_TOOL_DOWN=True  -> 恢复旧行为(V 必须≈90°, 否则拒绝)
#   False(默认)             -> 任何姿态都接受, 全程锁定 anchor 当时的姿态
REQUIRE_TOOL_DOWN = False
ANCHOR_V_TOL_DEG = 15.0        # 仅当 REQUIRE_TOOL_DOWN=True 时生效

# 限位预检: 运行前先把每个拐角"试探"一遍(低速点到点 + 读关节角), 检查关节余量。
# 任一关节余量低于阈值就中止, 避免跑到一半撞限位。PREFLIGHT=False 可跳过。
PREFLIGHT = True
LIMIT_MARGIN_WARN_DEG = 8.0    # 关节距限位小于此值 -> 警告
LIMIT_MARGIN_ABORT_DEG = 3.0   # 关节距限位小于此值 -> 中止

# 夹爪(串口透传, 与 execute_trajectory 真机成功接口完全一致)
GRIPPER_ID = 2                 # 左夹爪
GRIPPER_COM = "COM1"
GRIPPER_SERIAL = (115200, 8, 0, 1)
GRIPPER_CLOSE = [500, 1000]    # [速度, 力度]
GRIPPER_OPEN = [500]           # [开口位置]

ARRIVE_TOL_MM = 4.0            # 拐角到位容差
ARRIVE_TIMEOUT_S = 60.0        # 组末到位超时(smooth 组可能含多段直线)
SAMPLE_S = 0.1                 # smooth 模式运动中采样周期(过点监控)

# 转角门限(20260725 真机日志实证): interpolation_en 的平滑衔接【会切拐角】——
# 实测 90° 拐角被切掉 64mm, 180° 掉头(上升后立刻下降)被整个吃掉(差 119.9mm,
# 等于手臂根本没上升)。切多少取决于"下一条命令到达时上一条走了多远", 是时序
# 相关的不确定行为 —— 这正是 5 次只成 1 次的原因: 抓起后若上升被吃掉, 手臂就
# 贴着桌面斜拖过去(拖拽物体/逼近限位)。
# 对策: 只有【平缓拐角】才背靠背混合; 转角大于门限的拐角必须单独下发并等到位,
# 保证几何确定可复现。我们这条轨迹的拐角都是 90~180°, 因此实际全部走确定路径,
# 平滑性由"拐角压缩后每段都是一条直线运动"保证(6 段 vs 原来 79 次停顿)。
BLEND_MAX_TURN_DEG = 60.0
SKIP_SATISFIED_LEAD = True     # 跳过"手臂已在该位置"的开头拐角(见下)
SKIP_TOL_MM = 8.0
SERVO_STEP_MM = 5.0            # [留档] servo 模式加密步长
SERVO_PERIOD_S = 0.02


def build_gripper_cmd(gid, command, values):
    data = []
    for v in values:
        v = max(0, min(0xFFFF, int(v)))
        data += [v & 0xFF, (v >> 8) & 0xFF]
    payload = [int(gid) & 0xFF, 1 + len(data), int(command) & 0xFF] + data
    checksum = sum(payload) & 0xFF
    return " ".join(f"{b:02x}" for b in [0xEB, 0x90] + payload + [checksum])


def open_gripper_com(sdk):
    code = sdk.armOpenCom(GRIPPER_COM, *GRIPPER_SERIAL)
    print("armOpenCom:", (code, GRIPPER_COM, *GRIPPER_SERIAL))
    ss.require_code_zero(code, "armOpenCom")


def gripper(sdk, action):
    hexcmd = (build_gripper_cmd(GRIPPER_ID, 0x11, GRIPPER_OPEN) if action == "open"
              else build_gripper_cmd(GRIPPER_ID, 0x10, GRIPPER_CLOSE))
    data = bytes.fromhex(hexcmd.replace(" ", ""))
    print(f"  [夹爪{GRIPPER_ID}] {action} hex={hexcmd}")
    if ENABLE_REAL_MOTION:
        code = sdk.armWriteCom(GRIPPER_COM, data)
        ss.require_code_zero(code, f"夹爪{GRIPPER_ID} {action} armWriteCom")
        time.sleep(5.0 if action == "close" else 2.5)


def load_traj(path):
    import json
    raw = open(path, "rb").read()
    print(f"轨迹 SHA-256: {hashlib.sha256(raw).hexdigest()}")
    d = json.loads(raw.decode("utf-8"))
    if "single_relative" not in d["meta"].get("task", ""):
        raise SystemExit("非锚点+相对偏移轨迹(task 应含 single_relative), 拒绝")
    return d["meta"], d["waypoints"]


def corners_of(wps):
    """拐角压缩: 每个直线段只保留段末点(armMoveWorlds 原生走直线, 中间密集点
    只会造成逐点停顿)。夹爪事件原样保留。"""
    out = []
    for i, w in enumerate(wps):
        if "gripper" in w:
            out.append(dict(w))
            continue
        nxt = wps[i + 1] if i + 1 < len(wps) else None
        if nxt is None or "gripper" in nxt or nxt["seg"] != w["seg"]:
            out.append(dict(w))            # 段末 = 拐角
    return out


def groups_of(corners):
    """按夹爪事件切组: [[拐角...], 夹爪, [拐角...], 夹爪, [拐角...]] 的交替列表。"""
    groups, cur = [], []
    for c in corners:
        if "gripper" in c:
            groups.append(cur)
            groups.append(c)
            cur = []
        else:
            cur.append(c)
    if cur:
        groups.append(cur)
    return groups


def joint_margins(q_deg):
    """每个关节距最近限位的余量(度), 用于限位预检与诊断。"""
    out = []
    for k, (v, (lo, hi)) in enumerate(zip(q_deg, ss.LIMITS_DEG[ARM_ID]), start=1):
        out.append(f"j{k}:{min(v - lo, hi - v):.0f}")
    return " ".join(out)


def worst_margin(q_deg):
    """返回 (最小余量, 关节号)。"""
    worst, idx = 1e9, 0
    for k, (v, (lo, hi)) in enumerate(zip(q_deg, ss.LIMITS_DEG[ARM_ID]), start=1):
        m = min(v - lo, hi - v)
        if m < worst:
            worst, idx = m, k
    return worst, idx


def preflight(sdk, anchor, corners):
    """限位预检: 逐个拐角低速试探(点到点到位), 读关节角检查余量。
    任一拐角余量 < LIMIT_MARGIN_ABORT_DEG 立即中止, 避免正式跑到一半撞限位。
    这是应对"多次限位失败"的主要手段 —— 先知道哪个拐角、哪个关节会顶到。"""
    print("\n[限位预检] 逐拐角试探(低速), 检查关节余量...")
    worst_all, worst_where = 1e9, ""
    captured = {}
    for c in corners:
        if "gripper" in c:
            continue
        pose = abs_pose(anchor, c["off_mm"])
        ss.check_soft_stop(sdk, f"预检 {c['seg']}")
        ss.move_worlds(sdk, ARM_ID, pose, interpolation_en=False)
        ss.wait_until_worlds(sdk, ARM_ID, pose, label=f"(预检 {c['seg']})")
        q = ss.read_joints(sdk, ARM_ID)
        # 关节角快照: MODE=hybrid 的 via="joints" 路点要靠这个走 armMoveJoints。
        # SDK 没有逆解接口, 世界位姿对应的关节角只能这样【在真机现场量出来】,
        # 所以 hybrid 下预检是必需步骤(见 main 的守卫)。
        captured[c["seg"]] = list(q)
        m, j = worst_margin(q)
        # 20260729 修: 预检和正式执行前用的是【两套不同标准】——
        #   worst_margin 拿【原始限位】算余量, <3° 才中止;
        #   ss.check_limits 拿【收 5° 后】的区间判, 超出即拒。
        # 实测 j4=-116.67 在预检里算 3.3° 余量"通过", 到正式执行前被 check_limits
        # 拒掉 —— 而那时物体已经抓在手上了。现在预检就按最终生效的那套标准判,
        # 让它在【夹爪闭合之前】失败。
        violations = ss.check_limits(ARM_ID, q)
        flag = ("✗ 中止" if violations or m < LIMIT_MARGIN_ABORT_DEG else
                "⚠ 偏紧" if m < LIMIT_MARGIN_WARN_DEG else "✓")
        print(f"  {c['seg']:16s} 最小余量 j{j}={m:5.1f}°  {flag}   [{joint_margins(q)}]")
        if m < worst_all:
            worst_all, worst_where = m, f"{c['seg']} j{j}"
        if violations:
            raise SystemExit(
                f"✗ 预检中止: {c['seg']} 的实测关节角未通过 check_limits(执行前必过"
                f"的那道门):\n    " + "\n    ".join(violations) +
                f"\n  这个点靠 armMoveWorlds 能到, 但解出来的位形贴着限位, 正式执行"
                f"会被拒。\n  对策: 换个 anchor 起始姿态(尤其调 j1/j4)让位形更舒展,"
                f" 或把抓放点挪近机器人。")
        if m < LIMIT_MARGIN_ABORT_DEG:
            raise SystemExit(
                f"✗ 预检中止: {c['seg']} 处 j{j} 距限位仅 {m:.1f}°。\n"
                f"  对策(按推荐顺序): ①换个 anchor 起始姿态(VR 摆到关节余量更大的\n"
                f"  位形, 尤其调 j1/j4); ②REQUIRE_TOOL_DOWN=False 用倾斜姿态抓取;\n"
                f"  ③把抓放点挪近机器人; ④缩小 off_mm 行程。")
    print(f"[限位预检] 通过 — 全程最小余量 {worst_all:.1f}° ({worst_where})")
    return worst_all, captured


def get_anchor(sdk):
    if ANCHOR_FROM == "reference" and ENABLE_REAL_MOTION:
        print("移动到参考 tool-down 关节姿态取 anchor(仅进场准备, armMoveJoints)...")
        bad = ss.check_limits(ARM_ID, REFERENCE_TOOL_DOWN_Q_SDK)
        if bad:
            raise SystemExit(f"参考姿态超限: {bad}")
        ss.move_joints_abs(sdk, ARM_ID, REFERENCE_TOOL_DOWN_Q_SDK)
        ss.wait_until_joints(sdk, ARM_ID, REFERENCE_TOOL_DOWN_Q_SDK)
    anchor = ss.read_worlds(sdk, ARM_ID)
    q = ss.read_joints(sdk, ARM_ID)
    print(f"anchor 世界位姿 = {[round(v, 2) for v in anchor]}")
    print(f"anchor 关节角   = {[round(v, 2) for v in q]}")
    print(f"anchor 关节余量 = {joint_margins(q)}")
    if REQUIRE_TOOL_DOWN and abs(anchor[4] - 90.0) > ANCHOR_V_TOL_DEG:
        raise SystemExit(
            f"✗ 拒绝执行: anchor 的 V={anchor[4]:.1f}° 不接近 90°(工具朝下), 且当前"
            f" REQUIRE_TOOL_DOWN=True。把臂摆成竖直朝下, 或改 REQUIRE_TOOL_DOWN=False"
            f"(允许倾斜姿态抓取, 关节余量更大)。")
    print(f"姿态: 全程锁定 anchor 当时的 UVW=[{anchor[3]:.1f},{anchor[4]:.1f},"
          f"{anchor[5]:.1f}]{'(要求竖直)' if REQUIRE_TOOL_DOWN else '(不要求竖直)'}")
    return anchor


def abs_pose(anchor, off_mm):
    return [anchor[0] + off_mm[0], anchor[1] + off_mm[1], anchor[2] + off_mm[2],
            anchor[3], anchor[4], anchor[5]]


def _dist_mm(a, b):
    return max(abs(x - y) for x, y in zip(a[:3], b[:3]))


def _turn_deg(p_prev, p_cur, p_next):
    """三点构成的转角(度): 0°=直行, 180°=原路掉头。"""
    a = [c - b for b, c in zip(p_prev[:3], p_cur[:3])]
    b = [c - d for d, c in zip(p_cur[:3], p_next[:3])]
    na = math.sqrt(sum(v * v for v in a))
    nb = math.sqrt(sum(v * v for v in b))
    if na < 1e-9 or nb < 1e-9:
        return 180.0                       # 零长度段按最保守处理
    cos = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / (na * nb)))
    return math.degrees(math.acos(cos))


def run_group_smooth(sdk, start_pose, targets):
    """一组拐角(无夹爪事件)。按转角决定衔接方式:
      转角 <= BLEND_MAX_TURN_DEG -> 与下一条背靠背(interpolation_en 平滑过弯);
      转角 >  BLEND_MAX_TURN_DEG -> 单独下发并等到位(否则会被平滑掉/切角)。
    每个拐角都报告转角与实际到位/通过距离。"""
    prev = start_pose
    i = 0
    while i < len(targets):
        seg, pose = targets[i]
        # 与下一个拐角的转角(末点没有下一个 -> 必须停)
        if i + 1 < len(targets):
            turn = _turn_deg(prev, pose, targets[i + 1][1])
        else:
            turn = 180.0
        blend = turn <= BLEND_MAX_TURN_DEG
        print(f"  {seg:16s} -> world {[round(v, 1) for v in pose]}  "
              f"转角 {turn:5.1f}° -> {'平滑过弯' if blend else '停稳(防切角)'}")
        if ENABLE_REAL_MOTION:
            ss.check_soft_stop(sdk, seg)
            ss.move_worlds(sdk, ARM_ID, pose, interpolation_en=blend)
            if blend:
                time.sleep(0.05)           # 背靠背, 让控制器在拐角混合
            else:
                actual = ss.wait_until_worlds(sdk, ARM_ID, pose,
                                              tol_mm=ARRIVE_TOL_MM,
                                              timeout_s=ARRIVE_TIMEOUT_S,
                                              label=f"(直线 {seg})")
                print(f"    到位: 误差 {_dist_mm(actual, pose):.1f}mm")
        prev = pose
        i += 1


def run_smooth(sdk, anchor, wps):
    corners = corners_of(wps)
    n_move = sum(1 for c in corners if "off_mm" in c)
    print(f"拐角压缩: {sum(1 for w in wps if 'off_mm' in w)} 密集点 -> {n_move} 拐角")
    cur = ss.read_worlds(sdk, ARM_ID) if ENABLE_REAL_MOTION else list(anchor)
    first_group = True
    for g in groups_of(corners):
        if isinstance(g, dict):            # 夹爪事件(组间, 上一组已停稳)
            gripper(sdk, g["gripper"])
            continue
        if not g:
            continue
        targets = [(c["seg"], abs_pose(anchor, c["off_mm"])) for c in g]
        # 开头去掉"手臂已经在那儿"的拐角: anchor 取在抓取点时, 首组是
        # 上升120→再下降120回原地的 180° 掉头, 平滑衔接会把它整个吃掉
        # (20260725 实测 START 差 119.9mm = 根本没上升)。既然终点就是当前
        # 位置, 这段本就是多余动作, 直接跳过更确定也更安全。
        if first_group and SKIP_SATISFIED_LEAD and len(targets) >= 2:
            while len(targets) >= 2 and _dist_mm(cur, targets[-1][1]) <= SKIP_TOL_MM:
                skipped = [s for s, _ in targets[:-1]]
                print(f"  跳过开头多余拐角 {skipped}: 该组终点"
                      f"({targets[-1][0]})就是当前位置(差 "
                      f"{_dist_mm(cur, targets[-1][1]):.1f}mm), 无需先离开再回来")
                targets = targets[-1:]
                break
        first_group = False
        if not targets:
            continue
        run_group_smooth(sdk, cur, targets)
        cur = ss.read_worlds(sdk, ARM_ID) if ENABLE_REAL_MOTION else targets[-1][1]


def run_curve(sdk, anchor, wps):
    """MODE=curve: 转运弧用【一条 armMoveWorldsCurve】走完(手册 2.7.28)。

    为什么这是正解(20260725 真机实证): 把弧拆成 12 个点背靠背下发时, 控制器
    会卡住/漏命令(ARC12 等不到位, 只能 Ctrl-C); 而 interpolation_en 的混合又
    是时序相关的切角。弧线运动一条命令就走完整段弧 —— 无时序依赖, 控制器原生
    平滑。ARC 点在这里只用来取【弧中点 mid】与【终点 end】。"""
    groups = groups_of(corners_of(wps))
    for g in groups:
        if isinstance(g, dict):
            gripper(sdk, g["gripper"])
            continue
        if not g:
            continue
        # 按【原顺序】走: 遇到连续的 ARC 段就合成一条弧线命令, 其余逐条直线。
        # (早先把直线和弧分开处理会打乱顺序 —— 弧会跑到 PLACE_DESCEND 之后)
        i = 0
        while i < len(g):
            c = g[i]
            if c["seg"].startswith("ARC"):
                j = i
                while j < len(g) and g[j]["seg"].startswith("ARC"):
                    j += 1
                arcs = g[i:j]
                mid = abs_pose(anchor, arcs[len(arcs) // 2]["off_mm"])
                end = abs_pose(anchor, arcs[-1]["off_mm"])
                print(f"  弧线 {arcs[0]['seg']}..{arcs[-1]['seg']} "
                      f"({len(arcs)} 个弧点 -> 1 条 armMoveWorldsCurve)")
                print(f"    中点 {[round(v, 1) for v in mid[:3]]}  "
                      f"终点 {[round(v, 1) for v in end[:3]]}")
                if ENABLE_REAL_MOTION:
                    ss.check_soft_stop(sdk, "curve")
                    ss.move_worlds_curve(sdk, ARM_ID, mid, end)
                    actual = ss.wait_until_worlds(
                        sdk, ARM_ID, end, tol_mm=ARRIVE_TOL_MM,
                        timeout_s=ARRIVE_TIMEOUT_S,
                        label="(弧线 armMoveWorldsCurve)")
                    print(f"    弧线到位: 误差 {_dist_mm(actual, end):.1f}mm")
                i = j
                continue
            pose = abs_pose(anchor, c["off_mm"])
            print(f"  {c['seg']:16s} -> world {[round(v,1) for v in pose]} (直线)")
            if ENABLE_REAL_MOTION:
                ss.check_soft_stop(sdk, c["seg"])
                ss.move_worlds(sdk, ARM_ID, pose, interpolation_en=False)
                actual = ss.wait_until_worlds(sdk, ARM_ID, pose,
                                              tol_mm=ARRIVE_TOL_MM,
                                              timeout_s=ARRIVE_TIMEOUT_S,
                                              label=f"(直线 {c['seg']})")
                print(f"    到位: 误差 {_dist_mm(actual, pose):.1f}mm")
            i += 1


def run_hybrid(sdk, anchor, wps, captured):
    """MODE=hybrid 【主线】: 分工执行。

      via="joints" 的路点 -> 一条 armMoveJoints 到【预检时现场量到的关节角】,
                            关节空间插值, 一次加减速, 大范围转运又快又连贯;
      其余路点         -> armMoveWorlds 直线 + 等到位, 几何确定(抓放要的就是这个)。

    20260729 真机现场结论: 转运段拆成 12 个 armMoveWorlds 逐条停稳, 又慢又顿;
    而 PICK/PLACE 的 ascend/descend 是单条长直线, 又快又丝滑。所以按运动性质分工,
    而不是全用一种指令。

    关节角为什么必须来自预检: SDK 没有逆解接口, 世界位姿 -> 关节角只能靠真机
    实测。预检本来就把每个拐角走了一遍并读了关节角, 直接复用, 不额外增加动作。"""
    for c in corners_of(wps):
        if "gripper" in c:
            gripper(sdk, c["gripper"])
            continue
        pose = abs_pose(anchor, c["off_mm"])
        if c.get("via") == "joints":
            q = captured.get(c["seg"])
            if q is None:
                raise SystemExit(
                    f"✗ {c['seg']} 标了 via=joints 但预检没量到关节角。"
                    f"hybrid 模式必须 PREFLIGHT=True。")
            bad = ss.check_limits(ARM_ID, q)
            if bad:
                raise SystemExit(f"✗ {c['seg']} 的预检关节角超限: {bad}")
            print(f"  {c['seg']:16s} -> joints {[round(v, 2) for v in q]}  "
                  f"[一条 armMoveJoints, 关节空间]")
            if ENABLE_REAL_MOTION:
                ss.check_soft_stop(sdk, c["seg"])
                ss.move_joints_abs(sdk, ARM_ID, q)
                actual = ss.wait_until_joints(sdk, ARM_ID, q)
                err = max(abs(a - b) for a, b in zip(actual, q))
                w = ss.read_worlds(sdk, ARM_ID)
                print(f"    到位: 关节误差 {err:.3f}°, 实际世界位置 "
                      f"{[round(v, 1) for v in w[:3]]} (目标 "
                      f"{[round(v, 1) for v in pose[:3]]})")
            continue
        print(f"  {c['seg']:16s} -> world {[round(v, 1) for v in pose]}")
        if ENABLE_REAL_MOTION:
            ss.check_soft_stop(sdk, c["seg"])
            ss.move_worlds(sdk, ARM_ID, pose, interpolation_en=False)
            actual = ss.wait_until_worlds(sdk, ARM_ID, pose,
                                          tol_mm=ARRIVE_TOL_MM,
                                          timeout_s=ARRIVE_TIMEOUT_S,
                                          label=f"(直线 {c['seg']})")
            print(f"    到位: 误差 {_dist_mm(actual, pose):.1f}mm")


def run_point(sdk, anchor, wps):
    for c in corners_of(wps):
        if "gripper" in c:
            gripper(sdk, c["gripper"])
            continue
        pose = abs_pose(anchor, c["off_mm"])
        print(f"  {c['seg']:16s} -> world {[round(v, 1) for v in pose]}")
        if ENABLE_REAL_MOTION:
            ss.check_soft_stop(sdk, c["seg"])
            ss.move_worlds(sdk, ARM_ID, pose, interpolation_en=False)
            actual = ss.wait_until_worlds(sdk, ARM_ID, pose,
                                          tol_mm=ARRIVE_TOL_MM,
                                          timeout_s=ARRIVE_TIMEOUT_S,
                                          label=f"(直线 {c['seg']})")
            print(f"    到位: 误差 {_dist_mm(actual, pose):.1f}mm")


def run_servo(sdk, anchor, wps):
    """[留档, 非主线] 密集流式送 armWorldsToServo。保留仅供将来参考。"""
    dense, last = [], None
    for w in wps:
        if "gripper" in w:
            dense.append(w)
            continue
        pose = abs_pose(anchor, w["off_mm"])
        if last is not None:
            span = max(abs(a - b) for a, b in zip(pose[:3], last[:3]))
            n = max(1, int(span / SERVO_STEP_MM))
            for k in range(1, n):
                dense.append({"pose": [a + (b - a) * k / n
                                       for a, b in zip(last, pose)]})
        dense.append({"pose": pose})
        last = pose
    print(f"servo 密集点: {sum(1 for d in dense if 'pose' in d)} 个")
    if not ENABLE_REAL_MOTION:
        return
    ss.set_protect_status(sdk, ARM_ID, False)
    try:
        t = time.time()
        for d in dense:
            if "gripper" in d:
                gripper(sdk, d["gripper"])
                t = time.time()
                continue
            ss.check_soft_stop(sdk, "servo")
            ss.worlds_to_servo(sdk, ARM_ID, d["pose"])
            t += SERVO_PERIOD_S
            time.sleep(max(0, t - time.time()))
    finally:
        ss.set_protect_status(sdk, ARM_ID, True)


def main(argv=None):
    global MODE
    argv = argv if argv is not None else sys.argv[1:]
    if not argv:
        print("用法: execute_world_grasp.py <轨迹.json> [--mode point|hybrid|...]")
        return 2
    # --mode 命令行覆盖: 全直线版和 hybrid 版共用这一个执行器, 只靠"轨迹 + 一个
    # 开关"区分, 不维护两份会各自漂移的副本。不给就用文件顶部的 MODE。
    if "--mode" in argv:
        i = argv.index("--mode")
        if i + 1 >= len(argv):
            print("--mode 后面要跟模式名")
            return 2
        MODE = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    meta, wps = load_traj(argv[0])
    n_via = sum(1 for w in wps if w.get("via") == "joints")
    print(f"轨迹: {argv[0]}  模式: {MODE}  夹爪力度: {GRIPPER_CLOSE[1]}")
    print(f"契约: {meta.get('exec_contract', '')}")
    # hybrid 的关节角只能靠预检现场量(SDK 无逆解), 所以这两个开关必须同时开。
    # 与其跑到转运段才发现没数据, 不如开跑前就拦下。
    if MODE == "hybrid":
        if n_via == 0:
            raise SystemExit("✗ MODE=hybrid 但轨迹里没有 via=\"joints\" 路点。"
                             "请用 TRANSFER_MODE=\"joint\" 生成的轨迹, 或改 MODE=\"point\"。")
        if ENABLE_REAL_MOTION and not PREFLIGHT:
            raise SystemExit("✗ MODE=hybrid 必须 PREFLIGHT=True: via=\"joints\" 路点的"
                             "关节角靠预检现场读取, 没有预检就没有关节角。")
        print(f"转运: {n_via} 个 via=joints 路点走 armMoveJoints(关节空间), "
              f"其余走 armMoveWorlds 直线")
    elif n_via and MODE != "hybrid":
        print(f"⚠ 轨迹含 {n_via} 个 via=\"joints\" 路点, 但 MODE={MODE} 会把它们"
              f"当普通直线点执行(几何仍正确, 只是转运不走关节空间)")
    # 机器人互斥锁: 同一台机械臂同时只允许一个进程操作。
    # 三条线共用这把锁 —— 只给透传加是没用的, 任何两个进程同时
    # 操作都会互相覆盖保护状态/全局速度/伺服指令。
    with robot_lock.acquire(ROBOT_IP, ARM_ID, "execute_world_grasp.py"):
      sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED, (ARM_ID,))
      open_gripper_com(sdk)
      # 取 anchor 前先张开夹爪: 否则若上次运行结束时夹爪是闭合的, 会闭着爪去
      # 定位 anchor 并一路走到抓取点(夹不住东西, 还可能撞到物体)。
      gripper(sdk, "open")
      protect_touched = False
      try:
          anchor = get_anchor(sdk)
          if not ENABLE_REAL_MOTION:
              print("\nDRY RUN: 下列目标世界位姿不下发。确认后改 ENABLE_REAL_MOTION=True。")
          else:
              input("回车开始(人守急停): ")
          captured = {}
          if PREFLIGHT and ENABLE_REAL_MOTION:
              _, captured = preflight(sdk, anchor, corners_of(wps))
              input("预检通过, 回车开始正式执行(人守急停): ")
          if MODE == "hybrid":
              run_hybrid(sdk, anchor, wps, captured)
          elif MODE == "servo":
              protect_touched = True
              run_servo(sdk, anchor, wps)
          elif MODE == "curve":
              run_curve(sdk, anchor, wps)
          elif MODE == "point":
              run_point(sdk, anchor, wps)
          else:
              run_smooth(sdk, anchor, wps)
          print("\n完成。")
      finally:
          if protect_touched and ENABLE_REAL_MOTION:
              try:
                  ss.set_protect_status(sdk, ARM_ID, True)
              except Exception as exc:      # noqa: BLE001
                  print("⚠ 恢复保护失败, 请手动确认:", exc)
          ss.close_session(sdk)
      return 0


if __name__ == "__main__":
    sys.exit(main())
