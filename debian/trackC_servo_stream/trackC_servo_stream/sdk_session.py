#!/usr/bin/env python3
"""真机采集脚本共用模块: SDK 会话 / 读数 / 运动 / 限位检查 / 样本保存。

只依赖 pypilot + 标准库。运行环境: Linux 操作电脑 Docker 容器内 Python 3.10。
初始化流程与已实测通过的 arm_sequential_joint_motion.py 完全一致。
"""
from __future__ import annotations

import datetime
import json
import os
import time

import pypilot

# ---------------------------------------------------------------- 限位表
# 来源: XF0112048 robot.urdf (SDK 关节序号 1..7 对应左臂 jid 5..11 / 右臂 jid 12..18)。
# 注意: 这是 URDF 限位, 真机 SDK 限位可能更紧 —— 首次逼近边界时人工盯紧。
# 左臂 j7 上限已改用真机实测值 76(20260708 全流程日志: 四次在 +76.0~76.6° 物理
# 停止, 其余关节误差为 0)。
# 右臂 j7 下限 -76 是把左臂那个实测值做矢状镜像得到的 ——【假设, 从未实测】。
# 宁可写上也不留 URDF 的 -90: 留 -90 等于右臂没有这道保护网, 而左臂当年正是
# 栽在这个关节上。保守方向: 真值若更宽只是白留余量, 更紧才会漏判。
# 上机前用单关节点动量出右臂 j7 真实停止位, 量到了就把 -76 换成实测值。
# 右臂其余关节的实测限位同样待测(现为 URDF 值)。
LIMITS_DEG = {
    1: [(-180.0, 60.0), (0.0, 150.0), (-140.0, 140.0), (-120.0, 0.0),
        (-170.0, 170.0), (-45.0, 45.0), (-90.0, 76.0)],       # 左臂
    2: [(-60.0, 180.0), (-150.0, 0.0), (-140.0, 140.0), (0.0, 120.0),
        (-170.0, 170.0), (-45.0, 45.0), (-76.0, 90.0)],       # 右臂
}
LIMIT_MARGIN_DEG = 5.0
ENABLE_STATUS_POLLS = 10       # 使能命令只发一次，随后最多只读检查 10 次
ENABLE_STATUS_INTERVAL_S = 0.5


def arm_name(arm_id):
    return {1: "左臂", 2: "右臂"}.get(arm_id, f"未知机械臂({arm_id})")


def require_code_zero(code, operation):
    if code != 0:
        raise RuntimeError(f"{operation}失败, 返回码: {code}")


def require_true_status(code, status, operation):
    if code != 0 or status is not True:
        raise RuntimeError(f"{operation}状态异常: code={code}, status={status}")


def wait_true_status(reader, operation, polls=ENABLE_STATUS_POLLS,
                     interval_s=ENABLE_STATUS_INTERVAL_S):
    """等待异步状态变为 True；只调用只读 getter，绝不重发控制命令。"""
    last = (None, None)
    for attempt in range(1, polls + 1):
        time.sleep(interval_s)
        last = reader()
        code, status = last
        print(f"{operation} status:", last, f"(检查 {attempt}/{polls})")
        if code == 0 and status is True:
            return last
    code, status = last
    raise RuntimeError(
        f"{operation}状态在 {polls * interval_s:.1f}s 内未就绪: "
        f"code={code}, status={status}")


def print_enable_diagnostics(sdk, arm_ids):
    """使能超时时补打一组只读交叉状态；任何一项异常都只记录，不放行运动。"""
    print("使能超时诊断(全部为只读查询，程序仍中止):")
    checks = [
        ("整体使能", sdk.armGetRobotEnableStatus),
        ("伺服 OP", sdk.armServoIsOpOrNot),
    ]
    checks.extend(
        (f"{arm_name(aid)}单臂使能",
         lambda aid=aid: sdk.armGetSingleRobotEnableStatus(aid))
        for aid in arm_ids
    )
    for label, reader in checks:
        try:
            print(f"  {label}:", reader())
        except Exception as exc:                 # noqa: BLE001 诊断不能盖掉主异常
            print(f"  {label}: 查询异常: {exc}")


def make_float_vector(values):
    vec = pypilot.FloatVector()
    for v in values:
        vec.append(float(v))
    return vec


def make_double_vector(values):
    """⚠ 厂商 API 自己不统一, 必须逐个接口对照手册:
         armPluseToServo / armWorldsToServo / armMoveWorlds -> FloatVector
         armTryWorlds                                       -> DoubleVector
    20260731 真机实证: 给 armTryWorlds 传 FloatVector 会直接 Boost.Python
    ArgumentError, 而手册 2.7.53 本来就写着 DoubleVector —— 是抄漏了。"""
    vec = pypilot.DoubleVector()
    for v in values:
        vec.append(float(v))
    return vec


# ---------------------------------------------------------------- 接口能力探测
# 手册版本(V260526.1)比容器里装的 SDK 轮子【新】: 20260731 实测有 4 个手册写了的
# 接口在真机上不存在或签名不同(armGetPosReFreshMS 不存在 / armUnsetLoad 不存在 /
# armGetAxisParameter 第三参要枚举 / armSetRobotLoad 是 (int,int))。
# 教训: 按手册写 = 按一份还没实现的规格写。新接口一律先探再用。
_CAPS = {}


def probe_caps(sdk, names):
    """探测这些接口在【当前这台机器】上存不存在, 返回 {名字: bool} 并缓存。
    只查属性是否存在, 不调用 —— 签名对不对要靠实际调用时的 try/except。"""
    for n in names:
        _CAPS[n] = hasattr(sdk, n)
    return dict(_CAPS)


def has_cap(name):
    """没探过就返回 None(未知), 探过返回 True/False。"""
    return _CAPS.get(name)


def print_caps(caps, title="SDK 接口能力"):
    ok = [n for n, v in caps.items() if v]
    no = [n for n, v in caps.items() if not v]
    print(f"\n[{title}] 可用 {len(ok)}/{len(caps)}")
    if no:
        print("  ✗ 本机没有: " + ", ".join(no))
        print("    (手册里有但轮子里没有 —— 相关功能自动跳过, 不再逐次告警)")


# ---------------------------------------------------------------- 会话
_SAVED = {}


def open_session(robot_ip, local_ip, arm_ip, arm_port, global_speed, arm_ids=(1,),
                 enable=True):
    """启动 SDK 并完成与真机验证过的完整初始化/使能/检查流程, 返回 sdk。

    enable=False = 【只读会话】: 只做 启动/软急停检查/armInitData/链路检查, 【不】
    清报警、不使能、不设全局速度、不查伺服。用途是"只想算点东西"的场景 ——
    典型就是 --precheck-only 只跑 armTryWorlds 逆解: 那是纯运动学计算, 不需要
    机械臂上电, 更不该顺手把它使能起来。"""
    sdk = pypilot.PilotSDK(robot_ip)
    print("=" * 60)
    print("启动 PilotSDK")
    print("=" * 60)
    if not sdk.start():
        raise RuntimeError("PilotSDK 启动失败")
    time.sleep(1)

    code, status = sdk.getSoftStopSwitch()
    print("soft_stop:", (code, status))
    if code != 0 or status != "CLOSE":
        raise RuntimeError("软急停状态异常, 中止")

    code = sdk.armInitData(local_ip, arm_ip, arm_port)
    print("armInitData:", code)
    require_code_zero(code, "机械臂初始化")

    code, linked = sdk.armGetLinkStatus()
    print("link:", (code, linked))
    require_true_status(code, linked, "机械臂链路")

    if not enable:
        # ⚠ 20260803 真机实测: 只读会话如果 link 一通就立刻返回, 数据管道还没建好 ——
        # armGetAxisParameter 读到全 0、armGetWorlds 返回 -1。原来的 enable 流程
        # 顺带提供了这段时间, 只读路径把它跳过了。这里显式等 + 校验就绪。
        print("只读会话: 不清报警/不使能/不设速度(--precheck-only 用)")
        for attempt in range(1, 6):
            time.sleep(0.6)
            try:
                code, _vec = sdk.armGetWorlds(arm_ids[0], True)
                if code == 0:
                    print(f"  数据就绪(第 {attempt} 次探测)")
                    return sdk
            except Exception:                      # noqa: BLE001
                pass
        raise RuntimeError(
            "只读会话数据未就绪: armGetWorlds 连续 5 次读不到。\n"
            "  常见原因: 机械臂未上电/伺服未开, 此时世界位姿与轴极限都读不出来。\n"
            "  --precheck-only 需要能读到 anchor 才有意义 —— 先在示教器上确认"
            "机械臂已上电, 再重试。")
    

    code = sdk.armClearAlarm()
    print("clear alarm:", code)
    require_code_zero(code, "清除报警")

    # 使能命令仍只发一次。旧版固定等 1 秒只查一次，真实日志出现过 code=0 但
    # status=False、紧接着重跑成功；而循环重发又会打断使能过程。因此这里只轮询
    # 只读 getter，最多等 5 秒，始终不重发使能命令、不放宽最终门槛。
    code = sdk.armRobotEnableOrNot(True)
    print("enable all:", code)
    require_code_zero(code, "整体使能")
    try:
        wait_true_status(sdk.armGetRobotEnableStatus, "enable")
        wait_true_status(sdk.armServoIsOpOrNot, "servo op")
        for aid in arm_ids:
            wait_true_status(
                lambda aid=aid: sdk.armGetSingleRobotEnableStatus(aid),
                f"{arm_name(aid)} single enable")
    except Exception:                            # noqa: BLE001 打诊断后原样抛出
        print_enable_diagnostics(sdk, arm_ids)
        raise

    # 全局速度是【控制器上的持久设置】, 不恢复会留给下一个脚本。开机存一下原值,
    # close_session 时还原(复查指出的真实缺口)。
    try:
        c0, spd0 = sdk.armGetGlobalSpeed()
        if c0 == 0:
            _SAVED["global_speed"] = float(spd0)
    except Exception:                          # noqa: BLE001 没这个接口就算了
        pass
    code = sdk.armSetGlobalSpeed(float(global_speed))
    print(f"set speed({global_speed}%):", code)
    require_code_zero(code, "设置全局速度")

    return sdk


def close_session(sdk):
    # 还原全局速度: 它是控制器上的持久设置, 我们改了就该改回去, 不能指望
    # "下一个脚本反正会再设一次"(复查指出)。读不到原值就跳过。
    spd = _SAVED.pop("global_speed", None)
    if spd is not None:
        try:
            code = sdk.armSetGlobalSpeed(float(spd))
            print(f"全局速度已还原为 {spd}%" if code == 0
                  else f"⚠ 还原全局速度返回码 {code}")
        except Exception as exc:  # noqa: BLE001
            print("⚠ 还原全局速度失败:", exc)
    try:
        sdk.stop()
    except Exception as exc:      # noqa: BLE001 — 收尾失败只提示不掩盖主流程异常
        print("sdk.stop() 异常(忽略):", exc)


def check_soft_stop(sdk, operation):
    code, status = sdk.getSoftStopSwitch()
    if code != 0 or status != "CLOSE":
        raise RuntimeError(f"{operation}: 软急停状态异常 ({code}, {status})")


def read_move_state(sdk, arm_id):
    """返回 True=运动中 / False=已停止；接口异常立即中止上层流程。
    真机 pypilot 实测(20260714/15)返回 (code, int 0/1) 而非 bool ——
    只接受 bool 会在第一个路点就抛"运动状态类型异常: 0"。"""
    code, moving = sdk.armGetRobotMoveState(arm_id)
    require_code_zero(code, f"读取{arm_name(arm_id)}运动状态")
    if isinstance(moving, bool):
        return moving
    if isinstance(moving, int) and moving in (0, 1):
        return moving == 1
    raise RuntimeError(f"{arm_name(arm_id)}运动状态类型异常: {moving!r}")


def clear_robot_route(sdk, arm_id, emergency_stop=True):
    """清除控制器残留路径；emergency_stop=True 时立即停止。"""
    code = sdk.armClearRobotRoute(arm_id, bool(emergency_stop))
    require_code_zero(code, f"清除{arm_name(arm_id)}路径")


# ---------------------------------------------------------------- 读数
def read_joints(sdk, arm_id, fb=True):
    """读 7 关节角(度)。

    手册 2.7.33: armGetJoints(robotId, fb)
      fb=True  -> 【反馈值】机械臂实时位置(默认, 我们一直用的)
      fb=False -> 【指令值】机械臂目标位置
    20260731 才注意到 fb=False 这一支 —— 两者相减就是【跟随误差】, 即"伺服有没有
    跟上我们喂的指令"。透传是开环流式, 这是唯一能直接看见跟随情况的手段, 而且不用
    自己记账, 控制器算好了。"""
    code, vec = sdk.armGetJoints(arm_id, bool(fb))
    require_code_zero(code, f"读取{arm_name(arm_id)}关节角(fb={fb})")
    joints = list(vec)
    if len(joints) != 7:
        raise RuntimeError(f"{arm_name(arm_id)}返回 {len(joints)} 个关节值, 预期 7 个")
    return joints


def read_follow_error(sdk, arm_id):
    """跟随误差 = 指令值 - 反馈值, 返回 (逐关节误差, 最大绝对值, 指令, 反馈)。
    透传时伺服总是滞后于指令; 这个值持续变大 = 跟不上了(降载/过热/步长过大)。"""
    cmd = read_joints(sdk, arm_id, fb=False)
    act = read_joints(sdk, arm_id, fb=True)
    err = [c - a for c, a in zip(cmd, act)]
    return err, max(abs(v) for v in err), cmd, act


def read_speed_feedback(sdk, arm_id):
    """逐关节速度反馈(手册 2.7.42 armGetSpeedFeedback)。
    用途: 区分"走得慢"和"停住了"—— armGetRobotMoveState 只有整臂一个 bool,
    看不出是哪个关节、还有没有在动。"""
    code, vec = sdk.armGetSpeedFeedback(arm_id)
    require_code_zero(code, f"读取{arm_name(arm_id)}速度反馈")
    return [float(v) for v in vec]


def read_axis_move_state(sdk, arm_id, axis_id):
    """单轴运动状态(手册 2.7.50, axis_id 从 0 开始)。"""
    code, moving = sdk.armGetRobotAxisMoveState(arm_id, int(axis_id))
    require_code_zero(code, f"读取{arm_name(arm_id)}轴{axis_id}运动状态")
    return bool(moving) if isinstance(moving, bool) else moving == 1


def read_robot_paused(sdk, arm_id):
    """暂停状态(手册 2.7.47)。语义比 armGetRobotMoveState 更强:
    True = 【已暂停且停稳】, False = 运动中【或减速中】。"""
    code, paused = sdk.armGetRobotPaused(arm_id)
    require_code_zero(code, f"读取{arm_name(arm_id)}暂停状态")
    return bool(paused)


def read_axis_limits(sdk, arm_id, n_axis=7):
    """从控制器读【真实】轴极限(手册 2.7.51 armGetAxisParameter,
    index 1=正极限 2=负极限, axis_id 从 0 开始), 返回 [(lo, hi)] × n。

    为什么重要: LIMITS_DEG 是照 URDF 抄的硬编码表, 与控制器真值不一定一致 ——
    真机 j7 实际是 +76° 而 URDF 写 90°, 当初只能靠试错发现。而 20260729 hybrid
    run1 那次"预检说通过、执行前又拒"的冲突, 根子也在于我们在拿一张不权威的表
    做判断。开机读一次, 整张表就是真的。"""
    pos_i, neg_i = _axis_param_index()
    out = []
    # 有效性由调用方用 axis_limits_look_valid() 判 —— 真机实测过"全 0"的读数
    # (数据未就绪时), 那种值一旦被当成真限位采纳, 会把任何轨迹全判超限。
    for a in range(n_axis):
        c1, hi = sdk.armGetAxisParameter(arm_id, a, pos_i)
        require_code_zero(c1, f"读{arm_name(arm_id)}轴{a}正极限")
        c2, lo = sdk.armGetAxisParameter(arm_id, a, neg_i)
        require_code_zero(c2, f"读{arm_name(arm_id)}轴{a}负极限")
        out.append((float(lo), float(hi)))
    return out


def _axis_param_index():
    """取 (正极限, 负极限) 的枚举值。

    20260803 probe 实测签名是 armGetAxisParameter(int, int, AxisParameterIndex) ——
    第 4 参是【枚举】不是 int, 传 1/2 会直接 Boost.Python ArgumentError。手册
    2.7.51 写的"index: int (1正 2负)"是简化说法。枚举类型 pypilot.AxisParameterIndex
    已导出, 按名字取; 名字对不上就按顺序取前两个成员并打印出来供人工核对。"""
    E = getattr(pypilot, "AxisParameterIndex", None)
    if E is None:
        raise RuntimeError("pypilot 没有 AxisParameterIndex 枚举, 无法读轴极限")
    names = [n for n in dir(E) if not n.startswith("_") and n != "names" and n != "values"]
    def pick(*keys):
        for k in keys:
            for n in names:
                if k.lower() in n.lower():
                    return getattr(E, n)
        return None
    pos = pick("positive", "pos", "max", "upper")
    neg = pick("negative", "neg", "min", "lower")
    if pos is None or neg is None:
        vals = getattr(E, "values", None)
        members = ([vals[k] for k in sorted(vals)] if isinstance(vals, dict)
                   else [getattr(E, n) for n in names])
        if len(members) < 2:
            raise RuntimeError(f"AxisParameterIndex 成员不足: {names}")
        pos, neg = members[0], members[1]
        print(f"  ⚠ AxisParameterIndex 成员名不含 positive/negative: {names}\n"
              f"    按顺序取前两个当 (正,负), 请人工核对读回的极限是否合理")
    return pos, neg


def axis_limits_look_valid(limits):
    """判断读回来的轴极限像不像真的。

    20260803 真机实测: 数据未就绪时 armGetAxisParameter 返回全 0 —— 若把
    [0,0] 当成真限位采纳, 任何关节角都会被判超限, 整条轨迹全废。
    判据: 每个轴必须 lo < hi, 且区间宽度至少 10°。"""
    bad = []
    for k, (lo, hi) in enumerate(limits, start=1):
        if not (hi - lo >= 10.0):
            bad.append(f"j{k}=[{lo:.1f},{hi:.1f}]")
    return (not bad), bad


def joint_out_limit(sdk, arm_id, axis_id, joint_deg):
    """问控制器"这个角度超限没"(手册 2.7.54), 返回 (超正极限, 超负极限)。
    权威判定 —— 不用我们拿本地表算余量。"""
    code, over_pos, over_neg = sdk.armJointOutLimit(arm_id, int(axis_id),
                                                    float(joint_deg))
    require_code_zero(code, f"{arm_name(arm_id)}轴{axis_id}限位检查")
    return bool(over_pos), bool(over_neg)


def try_worlds(sdk, arm_id, pose6):
    """世界位姿 -> 关节角【逆解】(手册 2.7.53 armTryWorlds)。

    ⚠ 20260731 才发现这个接口。此前 Track A hybrid 因为"SDK 没有逆解"这个错误
    前提, 绕道用"预检走一遍把关节角量下来"。有了它, 世界点可以直接离线换算成
    关节角做限位预检 —— 这是 worlds 透传能安全上机的前提。"""
    code, vec = sdk.armTryWorlds(arm_id, make_double_vector(pose6))
    if code != 0:
        return None                     # 不可达/奇异, 由调用方决定怎么处理
    joints = list(vec)
    return joints if len(joints) == 7 else None


def read_protect_status(sdk, arm_id):
    """回读保护状态(手册 2.7.22)。finally 里恢复保护后必须核一次 ——
    "调用返回 0"不等于"保护真的开了"。"""
    code, protect = sdk.armGetRobotProtectStatus(arm_id)
    require_code_zero(code, f"读取{arm_name(arm_id)}保护状态")
    return bool(protect)


def set_robot_load(sdk, arm_id, value_int):
    """告诉控制器末端负载(手册 2.7.63)。

    ⚠ 20260803 probe 实测本机签名是 armSetRobotLoad(int, int) -> tuple, 第 3 参是
    【整数】, 不是手册写的 LoadParameter。这个整数代表什么(kg? 克? 预设档位?)
    【未知】—— 传错可能让动力学补偿更离谱, 所以在探明之前不要调用它。
    本机还有手册没写的 armSetLoadParam / armGetLoadParam, 那两个更可能是
    LoadParameter 版本, 下次 probe 时一并查。
    armUnsetLoad 在本机【不存在】, 所以"卸载"目前也没有对应手段。"""
    code = sdk.armSetRobotLoad(arm_id, int(value_int))
    if isinstance(code, tuple):
        code = code[0]
    require_code_zero(code, f"{arm_name(arm_id)}设置负载 {value_int}")


# ⚠ armSetPosReFreshMS / armGetPosReFreshMS 在本机 SDK 轮子(1.0.0422.1)里【不存在】
# (20260803 probe 实测; 手册 2.7.65/66 有, 属于手册比轮子新)。相关封装已删除 ——
# 留着只会让调用方以为可用。含义: 关节位置反馈固定 40ms 刷新、世界 50ms, 改不了。
# 做跟随误差/停稳判据时必须按这个刷新率来, 采样比它更密只会读到重复值。


def read_worlds(sdk, arm_id):
    code, vec = sdk.armGetWorlds(arm_id, True)
    require_code_zero(code, f"读取{arm_name(arm_id)}世界坐标")
    worlds = list(vec)
    if len(worlds) != 6:
        raise RuntimeError(f"{arm_name(arm_id)}返回 {len(worlds)} 个世界坐标值, 预期 6 个")
    return worlds


# ---------------------------------------------------------------- 限位 / 运动
def check_limits(arm_id, q_deg, margin=LIMIT_MARGIN_DEG):
    """返回违反限位(含余量)的描述列表; 空列表 = 通过。"""
    bad = []
    for k, (q, (lo, hi)) in enumerate(zip(q_deg, LIMITS_DEG[arm_id]), start=1):
        if not (lo + margin <= q <= hi - margin):
            bad.append(f"j{k}={q:+.2f}deg 超出 [{lo + margin:+.1f}, {hi - margin:+.1f}] "
                       f"(URDF 限位 [{lo:+.1f}, {hi:+.1f}] 收 {margin}deg)")
    return bad


def move_joints_abs(sdk, arm_id, target_deg):
    """下发 7 关节绝对角度(度)。调用前必须已通过 check_limits。
    注意: armMoveJoints 是【点到点】—— 每点自带完整加减速并停稳, 官方无 blend
    参数(手册 2.7.30: armMoveJoints(id, joints, needCallback=False, delay_in_ms=0),
    第 2/3 参只是"是否回调 / 滞后毫秒", 与平滑无关)。要连续平滑请用伺服透传
    pulse_to_servo(见下)。"""
    code = sdk.armMoveJoints(arm_id, make_float_vector(target_deg), False, 0)
    require_code_zero(code, f"{arm_name(arm_id)} armMoveJoints")


# ---------------------------------------------------------------- 伺服透传(平滑连续)
# 手册依据: 2.7.31 armPluseToServo(角度直送伺服) / 2.7.32 armWorldsToServo(世界直送),
# "配合 setRobotProtectStatus 使用, 用于关节/末端摇操透传"。透传 = 高频流式把密集
# 路点直接送伺服, 由伺服在点间插值, 走出连续不停的运动(等同 UR servoJ)。
# 与 move_joints_abs 的区别: 后者点到点每点停; 透传全程不停。
def set_protect_status(sdk, arm_id, protect):
    """开/关保护状态(手册 2.7.21)。保护开时系统监测指令突变(如 100->10000)判异常。
    透传流式下发时必须【关闭】保护(protect=False), 否则正常密集指令会被误判触发保护。
    用完务必恢复 protect=True。"""
    code = sdk.armSetRobotProtectStatus(arm_id, bool(protect))
    require_code_zero(code, f"{arm_name(arm_id)}设置保护状态={protect}")


def pulse_to_servo(sdk, arm_id, target_deg):
    """把 7 关节角(度)直接送伺服(手册 2.7.31 armPluseToServo, 注意 SDK 拼写 'Pluse')。
    仅在保护已关闭(set_protect_status False)的透传模式下高频循环调用才有意义;
    单次调用等于给伺服一个新目标, 密集连续调用即平滑轨迹。调用前须过 check_limits。"""
    code = sdk.armPluseToServo(arm_id, make_float_vector(target_deg))
    require_code_zero(code, f"{arm_name(arm_id)} armPluseToServo")


def worlds_to_servo(sdk, arm_id, pose6):
    """世界位姿 [X,Y,Z,U,V,W] 直接送伺服(手册 2.7.32 armWorldsToServo)。
    伺服透传的世界坐标版, 高频流式密集世界点即平滑; 机器人内部解 IK。
    仅在 set_protect_status(False) 下循环调用有意义。"""
    code = sdk.armWorldsToServo(arm_id, make_float_vector(pose6))
    require_code_zero(code, f"{arm_name(arm_id)} armWorldsToServo")


def move_worlds(sdk, arm_id, pose6, interpolation_en=True):
    """世界位姿点到点(手册 2.7.27 armMoveWorlds)。interpolation_en=True 平滑衔接
    (需背靠背下发才生效); 单点可靠到位请配 wait_until_worlds。"""
    code = sdk.armMoveWorlds(arm_id, make_float_vector(pose6),
                            bool(interpolation_en), False, 0)
    require_code_zero(code, f"{arm_name(arm_id)} armMoveWorlds")


def move_worlds_curve(sdk, arm_id, mid6, end6):
    """世界坐标【弧线运动】(手册 2.7.28 armMoveWorldsCurve): 经 mid6 划弧到 end6。
    相比"发很多点让控制器混合", 弧线是【一条命令走完整段弧】—— 没有背靠背下发
    的时序不确定性(实测 12 条连发会卡住/被覆盖), 且是控制器原生的平滑运动。
    这是世界坐标做平滑转运的正解。"""
    code = sdk.armMoveWorldsCurve(arm_id, make_float_vector(mid6),
                                  make_float_vector(end6), False, 0)
    require_code_zero(code, f"{arm_name(arm_id)} armMoveWorldsCurve")


def wait_until_worlds(sdk, arm_id, target6, tol_mm=3.0, tol_deg=1.5,
                      timeout_s=25.0, poll_s=0.2, motion_timeout_s=4.0,
                      motion_eps_mm=1.0, label=""):
    """轮询直到【实际世界位姿到达目标】(位置<tol_mm 且姿态<tol_deg), 返回实际位姿。
    区别于旧 wait_stable(只看"两次读数无变化", 会在启动/瞬停处误判): 本函数比对
    与【目标】的差, 超时 raise, 不静默返回。

    滚动停滞检测(20260727 复查后升级): 不只看"最初几秒有没有动"—— 那样只能发现
    "完全没动", 抓不到"先动了 2mm 再卡住"。改为全程滚动判定: 只要【与目标的误差
    连续 motion_timeout_s 秒没有明显改善(>motion_eps_mm)】, 就读运动状态与报警,
    确认停滞后清除控制器路径并报错。误差同时看位置与姿态, 避免"只有姿态在转"
    被误判成未生效。
    真机症状对照: 弧线命令发出后手臂纹丝不动/走一半停住, 只能 Ctrl-C(Track A)。"""
    start = time.time()
    deadline = start + timeout_s

    def _err(w):
        dp = max(abs(a - b) for a, b in zip(w[:3], target6[:3]))
        da = max(abs(a - b) for a, b in zip(w[3:], target6[3:]))
        # 姿态误差折算成等效毫米一并计入, 免得纯转动被当成"没动"
        return dp, da, dp + da * 5.0

    w = read_worlds(sdk, arm_id)
    best = _err(w)[2]
    best_t = time.time()
    while True:
        w = read_worlds(sdk, arm_id)
        dp, da, score = _err(w)
        if dp <= tol_mm and da <= tol_deg:
            return w
        if score < best - motion_eps_mm:          # 有实质改善 -> 刷新
            best, best_t = score, time.time()
        elif time.time() - best_t > motion_timeout_s:
            try:
                busy = read_move_state(sdk, arm_id)
            except Exception:                      # noqa: BLE001
                busy = None
            alarms = None
            try:
                code, alarms = sdk.armIsAlarming()
            except Exception:                      # noqa: BLE001
                pass
            try:
                clear_robot_route(sdk, arm_id, emergency_stop=True)
            except Exception:                      # noqa: BLE001
                pass
            raise RuntimeError(
                f"运动停滞{(' ' + label) if label else ''}: 连续 "
                f"{motion_timeout_s:.0f}s 与目标的误差无改善"
                f"(位置差 {dp:.1f}mm, 姿态差 {da:.2f}°), "
                f"move_state={busy}, alarms={alarms}。已清除控制器路径。\n"
                f"  目标 {[round(v, 1) for v in target6[:3]]}, 当前 "
                f"{[round(v, 1) for v in w[:3]]}\n"
                f"  常见原因: 目标不可达/超限位; 或弧线三点定出的圆弧异常"
                f"(半径过小/扫角>180°)被控制器拒绝。")
        if time.time() > deadline:
            raise RuntimeError(f"{timeout_s}s 内未到位: 位置差 {dp:.1f}mm 姿态差 {da:.2f}°")
        time.sleep(poll_s)


def wait_until_joints(sdk, arm_id, target_deg, tol_deg=0.5, timeout_s=25.0,
                      poll_s=0.3, motion_timeout_s=4.0, motion_eps_deg=0.3,
                      label=""):
    """轮询直到 7 关节全部到位(最大误差 <= tol), 返回实际关节角。

    20260729 升级(hybrid 转运连续两次超时后补的诊断): 原版超时只报一个"最大误差
    18.533°", 分不清【还在慢慢走】和【卡住不动了】—— 这两种情况的处理方式完全
    相反(前者该加时间, 后者加多久都没用)。现在与 wait_until_worlds 对齐:
      * 全程滚动判定: 误差连续 motion_timeout_s 秒没有实质改善(>motion_eps_deg)
        才判定停滞, 并读运动状态/报警后报错;
      * 超时报错时给出【逐关节误差 + 走了多少 + 平均速率 + 还需多久】, 而不是
        只有一个总误差数字 —— 下次一眼就能看出是哪个关节、是慢还是停。
    真机症状对照: armMoveJoints 大行程(j4 摆 53°)在 25s 内没走完, 残差 18.5°。"""
    start = time.time()
    deadline = start + timeout_s
    q0 = read_joints(sdk, arm_id)
    span0 = max(abs(a - t) for a, t in zip(q0, target_deg))
    best, best_t = span0, time.time()
    tag = f"{label} " if label else ""
    while True:
        joints = read_joints(sdk, arm_id)
        err = max(abs(a - t) for a, t in zip(joints, target_deg))
        print(f"  等待到位: 最大误差 {err:.3f} deg", end="\r")
        if err <= tol_deg:
            print()
            return joints
        stalled = False
        if err < best - motion_eps_deg:
            best, best_t = err, time.time()
        elif time.time() - best_t > motion_timeout_s:
            stalled = True
        if stalled or time.time() > deadline:
            print()
            raise RuntimeError(_joint_wait_report(
                sdk, arm_id, joints, target_deg, span0, err,
                time.time() - start, stalled, tag, timeout_s, motion_timeout_s))
        time.sleep(poll_s)


def _joint_wait_report(sdk, arm_id, joints, target, span0, err, elapsed,
                       stalled, tag, timeout_s, motion_timeout_s):
    """组织一份能直接定位问题的失败报告: 是哪个关节、走了多少、慢还是停。"""
    per = [f"j{k}:{a - t:+.2f}" for k, (a, t) in
           enumerate(zip(joints, target), start=1)]
    moved = span0 - err
    rate = moved / elapsed if elapsed > 0 else 0.0
    try:
        busy = read_move_state(sdk, arm_id)
    except Exception:                          # noqa: BLE001
        busy = None
    alarms = None
    try:
        _, alarms = sdk.armIsAlarming()
    except Exception:                          # noqa: BLE001
        pass
    head = (f"运动停滞 {tag}: 连续 {motion_timeout_s:.0f}s 误差无改善"
            if stalled else f"{tag}{timeout_s}s 内未到位")
    eta = (f", 按此速率还需 {err / rate:.0f}s" if rate > 1e-6 else "")
    return (f"{head}, 最大误差 {err:.3f}°\n"
            f"  逐关节误差(实际-目标): {' '.join(per)}\n"
            f"  总行程 {span0:.1f}° -> 已走 {moved:.1f}° ({elapsed:.1f}s, "
            f"平均 {rate:.2f}°/s){eta}\n"
            f"  move_state={busy}, alarms={alarms}\n"
            f"  判读: move_state=True 且还在改善 = 走得慢, 该加 timeout_s;\n"
            f"        move_state=False 或误差不再改善 = 真停了, 加时间没用。")


# ---------------------------------------------------------------- 采样 / 保存
def snapshot(sdk, arm_id, name, description=""):
    """记录当前时刻: 关节角 + 世界坐标 XYZ/UVW。"""
    joints = read_joints(sdk, arm_id)
    worlds = read_worlds(sdk, arm_id)
    print(f"  [记录] {name}")
    print(f"    q_deg = {[round(v, 3) for v in joints]}")
    print(f"    xyz_mm = {[round(v, 3) for v in worlds[:3]]}  uvw_deg = {[round(v, 3) for v in worlds[3:]]}")
    return {
        "name": name,
        "description": description,
        "timestamp": datetime.datetime.now().isoformat(timespec="seconds"),
        "arm_id": arm_id,
        "q_sdk_deg": joints,
        "sdk_world_xyz_mm": worlds[:3],
        "sdk_world_uvw_deg": worlds[3:],
    }


def save_samples(out_dir, prefix, meta, samples):
    """写 时间戳版 + _latest 两份 JSON, 返回路径。"""
    os.makedirs(out_dir, exist_ok=True)
    payload = {"meta": meta, "samples": samples}
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    paths = [os.path.join(out_dir, f"{prefix}_{stamp}.json"),
             os.path.join(out_dir, f"{prefix}_latest.json")]
    for path in paths:
        with open(path, "w") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)
    print(f"\n已保存 {len(samples)} 个样本:")
    for path in paths:
        print(" ", path)
    return paths
