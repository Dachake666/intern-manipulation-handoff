#!/usr/bin/env python3
"""Track C: 伺服透传(armPluseToServo)执行关节抓取轨迹 —— 保底的平滑方案。

原理(手册 2.7.31 + 2.7.21): 关闭保护状态后, 把【密集插值的关节点】按固定周期
高频流给伺服, 由伺服在点间插值, 走出完全连续不停的运动(等同 UR servoJ)。
这是三条线里唯一"物理上不可能有停顿"的方案 —— 因为根本没有点到点指令。

已验证基础: test_servo_stream.py 真机跑通(121 密集点/20ms, 终点误差 0.012~0.116°,
用户评价"运行的还行, 偶尔卡顿(过热/使能)")。本脚本把它从"测试小段"升级为
"执行完整抓放轨迹"。

与 execute_trajectory.py 的分工:
  execute_trajectory.py  点到点/近目标交接, 已多次真机成功, 精度最高
  execute_servo_grasp.py 伺服透传, 全程无停顿, 但关保护期间失去突变保护

安全(关键):
  * 关保护期间【失去突变保护】-> 密集点步长必须小(STEP_DEG 默认 0.2°), 且
    下发前整条轨迹过限位检查;
  * 夹爪动作前后强制停流并静止(抓取精度靠停稳保证);
  * finally 无论如何恢复保护;
  * 环境变量 XIFENG_ALLOW_REAL_MOTION 未设为 1 时只打印计划；真跑人守急停。

用法(容器内): python3 execute_servo_grasp.py <关节轨迹.json>
"""
import hashlib
import json
import math
import argparse
import contextlib
import datetime as dt
import os
import shlex
import subprocess
import sys
import time
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_WORK = _HERE.parent.parent
sys.path.insert(0, str(_WORK))

import robot_lock
import sdk_session as ss
import servo_common as sc
import arm_profiles
import execution_authorization

ROBOT_IP = os.environ.get("XIFENG_ROBOT_IP")
LOCAL_IP = os.environ.get("XIFENG_LOCAL_IP")
ARM_IP   = os.environ.get("XIFENG_ARM_IP", ROBOT_IP)
ARM_PORT = int(os.environ.get("XIFENG_ARM_PORT", "8080"))
ARM_ID   = 1
GLOBAL_SPEED = 3.0
ENABLE_REAL_MOTION = os.environ.get("XIFENG_ALLOW_REAL_MOTION") == "1"
EXTERNAL_TASK_LOCK = False       # 仅 mission_runner 同进程持整机锁时设置

STEP_DEG = 0.2                 # 相邻透传点最大关节步长(越小越平滑越安全)
PERIOD_S = 0.02                # 透传下发周期(50Hz; 与手册默认 40ms 刷新相称)
SETTLE_TOL_DEG = 0.8           # 与目标的最大允许误差
SETTLE_DELTA_DEG = 0.05        # 判"不动了"的帧间变化上限
SETTLE_FRAMES = 3              # 需连续满足的帧数(单帧读数不能证明停稳)
SETTLE_TIMEOUT_S = 15.0

# ---- 20260731 优化(依据重读 SDK 手册后发现的接口) ----
# 跟随误差监控: armGetJoints(fb=False)=指令值, fb=True=反馈值, 相减即跟随误差。
# 透传是开环流式, 这是唯一能看见"伺服跟没跟上"的手段(手册 2.7.33)。
FOLLOW_SAMPLE_EVERY = 0        # 0 = 关闭(默认)。⚠ 20260731 教训: 开着它会往
                               # 透传热循环里塞阻塞 SDK 读, 实测造成 5Hz 周期性微顿,
                               # 观感明显不如未优化版。只在诊断时开。       # 每 N 帧采一次(20ms×10=200ms; 反馈本身 40ms 才刷新)
FOLLOW_WARN_DEG = 3.0          # 跟随误差超过此值告警
# 负载(手册 2.7.63/64): 抓到物体后告诉控制器, 否则动力学补偿按空爪算, 跟随误差变大。
PAYLOAD_KG = 0.2               # 方块+夹持的估计质量, 按实物改
PAYLOAD_COM_M = (0.0, 0.0, 0.0)
USE_PAYLOAD = False            # ⚠ 20260803 probe: 本机 armSetRobotLoad 签名是
                               # (int, int), 第 3 参那个整数代表什么【未知】;
                               # armUnsetLoad 本机不存在, 连"卸载"都没手段。
                               # 探明语义前不要打开 —— 传错会让动力学补偿更离谱。
# 限位: 开机从控制器读真实轴极限(手册 2.7.51 armGetAxisParameter)。
#   "report"  【默认】只读出来和本地表对比打印, 判定仍用本地表。
#   "enforce" 用控制器真值【替换】本地表并重新预检, 不通过就拒绝执行。
#   "off"     完全不读。
# 为什么默认只 report: axis_id 与我们 j1..j7 的对应关系、以及返回值的单位, 手册
# 没有明说, 属于未经真机核实的假设。而这条轨迹是真机三次成功过的 —— 万一读回来
# 的值把它判死, 更可能是我们读错了而不是轨迹错了。先看一轮真实数字, 核对无误
# 再改 "enforce"。
CONTROLLER_LIMITS_MODE = "report"      # report | enforce | off
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
# 会把任何轨迹全判超限。

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

GRIPPER_ID_BY_ARM = {1: 2, 2: 1}
GRIPPER_ID = GRIPPER_ID_BY_ARM[ARM_ID]
GRIPPER_COM = "COM1"
GRIPPER_SERIAL = (115200, 8, 0, 1)
GRIPPER_CLOSE = [500, 1000]
GRIPPER_OPEN = [500]


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


def load_traj(path):
    raw = Path(path).read_bytes()
    print(f"轨迹 SHA-256: {hashlib.sha256(raw).hexdigest()}")
    d = json.loads(raw.decode("utf-8"))
    if d.get("schema_version") == "trajectory.v2":
        contract = d.get("contract", {})
        meta = {"arm": contract.get("arm"), "arm_id": contract.get("arm_id"),
                "gripper_id": contract.get("gripper_id"), "j6_flipped": True,
                "trajectory_id": d.get("trajectory_id"), "schema_version": "trajectory.v2"}
        wps = d.get("waypoints", [])
    else:
        meta, wps = d.get("meta", {}), d.get("waypoints", [])
    if meta.get("j6_flipped") is not True:
        raise SystemExit("meta.j6_flipped 不为 True, 拒绝下发")
    if meta.get("arm_id") != ARM_ID:
        raise SystemExit(f"meta.arm_id={meta.get('arm_id')} 与 ARM_ID={ARM_ID} 不符")
    if meta.get("gripper_id", GRIPPER_ID) != GRIPPER_ID:
        raise SystemExit(f"轨迹 gripper_id={meta.get('gripper_id')} 与运行时 {GRIPPER_ID} 不符")
    return meta, wps


def densify(a, b, step_deg=None):
    """两个关节点之间线性加密到 step_deg, 返回不含 a 的点列。

    ⚠ step_deg 默认【不能】写成 =STEP_DEG: 默认参数在函数定义时就求值绑定了,
    之后 --step-deg 改了全局也不会生效 —— 实测过, 打印说 0.4° 但密集点还是按
    0.2° 算的 3232 个。必须在调用时才读全局。"""
    step_deg = STEP_DEG if step_deg is None else step_deg
    span = max(abs(x - y) for x, y in zip(a, b))
    n = max(1, int(math.ceil(span / step_deg)))
    return [[x + (y - x) * k / n for x, y in zip(a, b)] for k in range(1, n + 1)]


def wait_settled(sdk, target, tag):
    """等待实际关节【真正停稳】在 target 附近。

    20260727 加强(复查指出): 原来只看"单次读数误差 < 0.8°"就宣布停稳 ——
    实测第二轮开爪前读到 0.770°, 已经贴着门限, 而且单帧读数无法区分"停住了"
    和"正好路过". 现在要求同时满足三条:
      ① 控制器报【未在运动】(armGetRobotMoveState);
      ② 连续 SETTLE_FRAMES 帧的关节变化都 < SETTLE_DELTA_DEG(真的不动了);
      ③ 与目标的误差 <= SETTLE_TOL_DEG。
    任一不满足就继续等, 超时抛错。夹爪的抓取精度全靠这一步。"""
    deadline = time.time() + SETTLE_TIMEOUT_S
    prev, stable = None, 0
    while time.time() < deadline:
        q = ss.read_joints(sdk, ARM_ID)
        err = max(abs(a - b) for a, b in zip(q, target))
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
                print(f"    {tag} 停稳: 误差 {err:.3f}° "
                      f"(连续{SETTLE_FRAMES}帧变化<{SETTLE_DELTA_DEG}°, 控制器报已停)")
                return q
        else:
            stable = 0
        time.sleep(0.1)
    # 超时: 把"慢"和"停"分开报。速度反馈(手册 2.7.42)是唯一能直接回答这个的东西 ——
    # armGetRobotMoveState 只有整臂一个 bool, 看不出是哪个关节还在动。
    speeds = paused = None
    try:
        speeds = ss.read_speed_feedback(sdk, ARM_ID)
    except Exception:                          # noqa: BLE001
        pass
    try:
        paused = ss.read_robot_paused(sdk, ARM_ID)
    except Exception:                          # noqa: BLE001
        pass
    per = " ".join(f"j{k+1}:{a-b:+.2f}" for k, (a, b) in enumerate(zip(q, target)))
    spd = ("  逐关节速度: " + " ".join(f"j{k+1}:{v:g}" for k, v in enumerate(speeds))
           if speeds else "")
    verdict = ("判读: 速度全为 0 = 真停住了, 加时间没用" if speeds and
               not any(abs(v) > 1e-6 for v in speeds) else
               "判读: 还有关节在动 = 走得慢, 可放宽 SETTLE_TIMEOUT_S")
    raise RuntimeError(
        f"{tag} {SETTLE_TIMEOUT_S}s 未达停稳判据\n"
        f"  最大误差 {err:.3f}°, 帧间变化 {delta:.3f}°, "
        f"move_state={moving}, paused={paused}\n"
        f"  逐关节误差: {per}\n{spd}\n  {verdict}")


def _world_pose_from_meta(meta, key):
    """从新瓶子候选的 meta 取 SDK [xyzuvw]；无此字段则返回 None。"""
    item = meta.get(key)
    if not isinstance(item, dict):
        return None
    xyz = item.get("sdk_world_xyz_mm", item.get("xyz_mm"))
    uvw = item.get("sdk_world_uvw_deg", item.get("uvw_deg"))
    if not (isinstance(xyz, list) and isinstance(uvw, list)
            and len(xyz) == 3 and len(uvw) == 3):
        return None
    return [float(v) for v in xyz + uvw]


def _check_limits_table(q_deg, limits, margin=ss.LIMIT_MARGIN_DEG):
    """对显式的控制器限位表检查；不改写共享基线常量。"""
    bad = []
    for joint, (value, pair) in enumerate(zip(q_deg, limits), 1):
        lo, hi = map(float, pair)
        if not lo + margin <= float(value) <= hi - margin:
            bad.append({"joint": joint, "value_deg": float(value),
                        "required_deg": [lo + margin, hi - margin],
                        "controller_deg": [lo, hi]})
    return bad


def controller_precheck_only(args, meta, moves):
    """
    只读控制器预检：不清故障、不使能、不开夹爪、不下发运动。

    用来裁决本候选中“q2=-7.854° 是否真的在控制器限位内”，
    同时记录当前到首点距离和 SDK armTryWorlds 的分支结果。
    """
    sdk = None
    report = {
        "schema_version": "servo_controller_precheck.v1",
        "created": dt.datetime.now(dt.timezone.utc).isoformat(),
        "trajectory": str(Path(args.trajectory).resolve()),
        "trajectory_sha256": hashlib.sha256(
            Path(args.trajectory).read_bytes()).hexdigest(),
        "arm_id": ARM_ID,
        "motion_commands_sent": 0,
        "enable_requested": False,
        "gripper_commands_sent": 0,
    }
    lock_context = (contextlib.nullcontext() if EXTERNAL_TASK_LOCK else
                    robot_lock.acquire_robot(ROBOT_IP,
                                              "execute_servo_grasp.py --controller-precheck-only"))
    try:
        with lock_context:
            sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT,
                                  GLOBAL_SPEED, (ARM_ID,), enable=False)
            caps = ss.probe_caps(sdk, ("armGetAxisParameter", "armTryWorlds"))
            ss.print_caps(caps, title="只读预检 SDK 能力")
            if not caps.get("armGetAxisParameter"):
                raise RuntimeError("当前 SDK 没有 armGetAxisParameter，不能裁决真限位")
            limits = ss.read_axis_limits(sdk, ARM_ID)
            valid, invalid_axes = ss.axis_limits_look_valid(limits)
            if not valid:
                raise RuntimeError("控制器返回无效限位: " + ", ".join(invalid_axes))
            effective_limits = ss.intersect_axis_limits(
                limits, ss.LIMITS_DEG[ARM_ID])

            violations = []
            minimum_margin = float("inf")
            for index, waypoint in enumerate(moves, 1):
                q = waypoint["q_sdk_deg"]
                for value, (lo, hi) in zip(q, effective_limits):
                    minimum_margin = min(minimum_margin, float(value) - float(lo),
                                         float(hi) - float(value))
                for item in _check_limits_table(q, effective_limits):
                    item.update({"waypoint": index, "seg": waypoint.get("seg")})
                    violations.append(item)

            current_q = [float(v) for v in ss.read_joints(sdk, ARM_ID)]
            current_world = [float(v) for v in ss.read_worlds(sdk, ARM_ID)]
            first_gap = max(abs(a - b) for a, b in
                            zip(current_q, moves[0]["q_sdk_deg"]))
            ik = {}
            for key in ("pick_sdk_world", "place_sdk_world"):
                pose = _world_pose_from_meta(meta, key)
                if pose is None:
                    continue
                solution = ss.try_worlds(sdk, ARM_ID, pose) if caps.get("armTryWorlds") else None
                ik[key] = {"pose_xyzuvw": pose,
                           "solution_q_sdk_deg": ([float(v) for v in solution]
                                                   if solution is not None else None)}
                if key == "pick_sdk_world" and solution is not None:
                    reference = meta[key].get("q_sdk_deg")
                    if isinstance(reference, list) and len(reference) == 7:
                        ik[key]["max_delta_to_supplied_q_deg"] = max(
                            abs(a - b) for a, b in zip(solution, reference))

            blockers = []
            warnings = []
            if violations:
                blockers.append("CONTROLLER_LIMIT_VIOLATION")
            if first_gap > 20.0:
                warnings.append("CURRENT_TO_FIRST_POINT_GAP_GT_20_DEG_REQUIRES_ENTRY_CONFIRMATION")
            if any(item["solution_q_sdk_deg"] is None for item in ik.values()):
                blockers.append("SDK_IK_UNREACHABLE")
            report.update({
                "status": "PASS" if not blockers else "BLOCKED",
                "blockers": blockers,
                "warnings": warnings,
                "entry_motion_policy": (
                    "operator confirms clear point-to-point path; executor moves to first "
                    "point, verifies settled, then asks again before servo streaming"),
                "controller_limits_deg": [[float(lo), float(hi)] for lo, hi in limits],
                "effective_operational_limits_deg": [
                    [float(lo), float(hi)] for lo, hi in effective_limits],
                "minimum_raw_joint_margin_deg": minimum_margin,
                "limit_margin_required_deg": ss.LIMIT_MARGIN_DEG,
                "limit_violation_count": len(violations),
                "limit_violations": violations[:50],
                "current_q_sdk_deg": current_q,
                "current_world_xyzuvw": current_world,
                "first_point_q_sdk_deg": moves[0]["q_sdk_deg"],
                "current_to_first_point_max_gap_deg": first_gap,
                "sdk_ik": ik,
            })
    except BaseException as exc:
        report.update({"status": "ERROR", "blockers": ["PRECHECK_EXCEPTION"],
                       "error": f"{type(exc).__name__}: {exc}"})
    finally:
        if sdk is not None:
            ss.close_session(sdk)

    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print("\n[只读控制器预检报告]\n" + rendered)
    if args.controller_precheck_report:
        report_path = Path(args.controller_precheck_report).resolve()
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(rendered + "\n", encoding="utf-8")
        print("预检报告已写入:", report_path)
    return 0 if report.get("status") == "PASS" else 1


def main(argv=None):
    # ⚠ 20260804 复查指出: 真机日志是现场手改常量跑出来的(0.4°/15%), 而仓库默认
    # still 是 0.2°/3% —— 结果【无法严格复现】, 归档的配置也对不上。参数改走命令行。
    global STEP_DEG, PERIOD_S, GLOBAL_SPEED, ARM_ID, GRIPPER_ID
    global ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, ENABLE_REAL_MOTION
    global CONTROLLER_LIMITS_MODE
    ap = argparse.ArgumentParser(description="Track C 参数化双臂执行器")
    ap.add_argument("trajectory")
    ap.add_argument("--step-deg", type=float, default=STEP_DEG)
    ap.add_argument("--period-ms", type=float, default=PERIOD_S * 1000.0)
    ap.add_argument("--speed", type=float, default=GLOBAL_SPEED)
    ap.add_argument("--arm-id", type=int, choices=(1, 2))
    ap.add_argument("--gripper-id", type=int, choices=(1, 2))
    ap.add_argument("--robot-ip", default=ROBOT_IP); ap.add_argument("--local-ip", default=LOCAL_IP)
    ap.add_argument("--arm-ip", default=ARM_IP); ap.add_argument("--arm-port", type=int, default=ARM_PORT)
    ap.add_argument("--run", action="store_true", help="请求真机下发；仍需源码安全开关允许")
    ap.add_argument("--controller-precheck-only", action="store_true",
                    help="只读真机限位/当前姿态/SDK IK；不使能不运动")
    ap.add_argument("--controller-precheck-report",
                    help="只读预检 JSON 报告输出路径（必须配合 --controller-precheck-only）")
    ap.add_argument("--preflight-report"); ap.add_argument("--approval-token")
    ap.add_argument("--scene-recheck"); ap.add_argument("--robot-config-identity")
    ap.add_argument("--vision-check-command",
                    help="每次抬升/放置后调用的视觉验收命令；退出码非0立即停止")
    ap.add_argument("--scene-watchdog-command",
                    help="独立静态场景监控进程；执行中退出即停止机器人")
    ap.add_argument("--soft-stop-watchdog-command",
                    help="独立软急停监控进程；执行中退出即停止机器人")
    a = ap.parse_args(list(argv if argv is not None else sys.argv[1:]))
    STEP_DEG, PERIOD_S, GLOBAL_SPEED = a.step_deg, a.period_ms / 1000.0, a.speed
    peek = json.loads(Path(a.trajectory).read_text(encoding="utf-8"))
    peek_arm = (peek.get("contract", {}).get("arm_id") if
                peek.get("schema_version") == "trajectory.v2" else
                peek.get("meta", {}).get("arm_id"))
    ARM_ID = int(a.arm_id or peek_arm or 1)
    GRIPPER_ID = int(a.gripper_id or GRIPPER_ID_BY_ARM[ARM_ID])
    ROBOT_IP, LOCAL_IP = a.robot_ip, a.local_ip
    ARM_IP, ARM_PORT = (a.arm_ip or a.robot_ip), a.arm_port
    source_gate = bool(ENABLE_REAL_MOTION)
    ENABLE_REAL_MOTION = bool(a.run and source_gate)
    if a.run and a.controller_precheck_only:
        ap.error("--run 与 --controller-precheck-only 互斥")
    if a.controller_precheck_report and not a.controller_precheck_only:
        ap.error("--controller-precheck-report 必须配合 --controller-precheck-only")
    if a.run and not source_gate:
        print("✗ --run 已给出，但 XIFENG_ALLOW_REAL_MOTION!=1；拒绝下发")
        return 2
    meta, wps = load_traj(a.trajectory)
    if ENABLE_REAL_MOTION and meta.get("real_motion_authorized") is False:
        print("✗ 轨迹 meta.real_motion_authorized=false；该文件仍是候选，拒绝真机下发")
        return 2
    visual_run = meta.get("schema_version") == "trajectory.v2"
    if visual_run or bool(meta.get("enforce_live_controller_limits")):
        CONTROLLER_LIMITS_MODE = "enforce"
    authorization = None
    if ENABLE_REAL_MOTION or a.controller_precheck_only:
        missing_network = [name for name, value in
                           (("robot-ip", ROBOT_IP), ("local-ip", LOCAL_IP), ("arm-ip", ARM_IP))
                           if not value]
        if missing_network:
            ap.error("控制器会话缺运行时网络参数: " + ", ".join(missing_network))
    if ENABLE_REAL_MOTION:
        arm_profiles.require_hardware_ready(arm_profiles.profile_for_arm_id(ARM_ID))
        if visual_run:
            required = (a.preflight_report, a.approval_token, a.scene_recheck,
                        a.robot_config_identity)
            if not all(required):
                ap.error("trajectory.v2 真机执行必须提供 preflight/approval/scene-recheck/robot identity")
            if not a.vision_check_command:
                ap.error("trajectory.v2 真机执行必须提供 --vision-check-command")
            if not a.scene_watchdog_command:
                ap.error("trajectory.v2 真机执行必须提供 --scene-watchdog-command")
            if not a.soft_stop_watchdog_command:
                ap.error("trajectory.v2 真机执行必须提供 --soft-stop-watchdog-command")
            authorization = execution_authorization.verify_authorization(
                a.trajectory, a.preflight_report, a.approval_token,
                a.scene_recheck, a.robot_config_identity)
            print(f"视觉执行授权: operator={authorization['operator']} "
                  f"report={authorization['report_sha256']}")
    moves = [w for w in wps if "q_sdk_deg" in w]
    import hashlib as _h
    print(f"轨迹: {a.trajectory}  模式: 伺服透传  步长 {STEP_DEG}° / 周期 "
          f"{PERIOD_S*1000:.0f}ms  夹爪力度 {GRIPPER_CLOSE[1]}")
    print(f"配置: {ss.arm_name(ARM_ID)}(arm_id={ARM_ID}) / 夹爪{GRIPPER_ID}; "
          f"关节速度 {STEP_DEG/PERIOD_S:.0f}°/s  全局速度 {GLOBAL_SPEED}%  "
          f"脚本 SHA {_h.sha256(Path(__file__).read_bytes()).hexdigest()[:16]}")

    # 该路径故意放在本地 URDF 限位判定之前：新目标 q2=-7.854°
    # 正是要通过只读真机数据判定，不能先被旧表 [0,150]° 拦掉。
    if a.controller_precheck_only:
        return controller_precheck_only(a, meta, moves)

    # 全轨迹限位预检(关保护后失去突变保护, 这一步不能省)
    bad = []
    for i, w in enumerate(moves, 1):
        bad += [f"路点#{i}: {b}" for b in ss.check_limits(ARM_ID, w["q_sdk_deg"])]
    if bad:
        print("✗ 限位预检失败:\n  " + "\n  ".join(bad[:10]))
        return 1
    print(f"限位预检: {len(moves)} 个路点全部通过 ✓")

    # 展开成 [("move", 密集点列) | ("grip", 动作)] 的执行计划
    plan, prev = [], None
    for w in wps:
        if "gripper" in w:
            plan.append(("grip", w["gripper"], prev))
            continue
        if "vision_check" in w:
            plan.append(("vision", w["vision_check"], prev))
            continue
        q = w["q_sdk_deg"]
        if prev is not None:
            plan.append(("move", densify(prev, q), q))
        prev = q
    n_dense = sum(len(x[1]) for x in plan if x[0] == "move")
    print(f"透传密集点: {n_dense} 个, 预计时长 {n_dense*PERIOD_S:.1f}s")

    if not ENABLE_REAL_MOTION:
        print("\nDRY RUN: 未下发。真机需同时满足 XIFENG_ALLOW_REAL_MOTION=1 与 --run，"
              "并由运行时提供 IP。")
        return 0

    # ⚠ 20260731 复查指出的缺口: open_session / 开串口 / 初始张开 原来在
    # try/finally 【之外】—— 这三步里任何一步抛异常, 都会绕过收尾, 把全局速度、
    # 保护状态留在改过的样子。现在 sdk 先置 None, 初始化整体搬进 try, finally
    # 里按 sdk 是否已建立分别处理。
    # 外层再套机器人互斥锁: 同一台机械臂同时只允许一个进程操作(见 robot_lock)。
    sdk = None
    protect_off = False
    loaded = False
    run_success = False
    run_failure = None
    scene_watchdog = None
    soft_stop_watchdog = None
    pacer = sc.Pacer(PERIOD_S)
    follow = sc.FollowMonitor(ARM_ID, FOLLOW_SAMPLE_EVERY, FOLLOW_WARN_DEG)
    lock_context = (contextlib.nullcontext() if EXTERNAL_TASK_LOCK else
                    robot_lock.acquire_robot(ROBOT_IP, "execute_servo_grasp.py"))
    with lock_context:
      try:
          sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT, GLOBAL_SPEED,
                                (ARM_ID,))
          open_gripper_com(sdk)
          gripper(sdk, "open")   # 先张开: 轨迹第一个夹爪事件是 close, 起手必须是开的
          caps = ss.probe_caps(sdk, OPTIONAL_CAPS)
          ss.print_caps(caps)
          # ---- 用【控制器真值】替换本地硬编码限位表 ----
          # 本地 LIMITS_DEG 抄自 URDF, 与控制器不一定一致(真机 j7 实际 +76° 而 URDF
          # 写 90°, 当初只能靠试错发现)。手册 2.7.51 可以直接问控制器要。
          if CONTROLLER_LIMITS_MODE != "off" and caps.get("armGetAxisParameter"):
              try:
                  real = ss.read_axis_limits(sdk, ARM_ID)
                  ok, bad_ax = ss.axis_limits_look_valid(real)
                  if not ok:
                      print(f"⚠ 读回的轴极限不像真值({', '.join(bad_ax)}), "
                            f"多半是数据未就绪 —— 忽略, 沿用本地表")
                  else:
                      effective = ss.intersect_axis_limits(
                          real, ss.LIMITS_DEG[ARM_ID])
                      adopt = (CONTROLLER_LIMITS_MODE == "enforce")
                      sc.describe_axis_limits(real, ss.LIMITS_DEG[ARM_ID],
                                              adopted=adopt)
                      if adopt:
                          ss.LIMITS_DEG[ARM_ID] = effective
                          if effective != real:
                              print("采用控制器限位与已证实物理/运行包络的交集；"
                                    "不会用控制器回读放宽本地安全上限")
                          bad2 = []
                          for i, w in enumerate(moves, 1):
                              bad2 += [f"路点#{i}: {b}" for b in
                                       ss.check_limits(ARM_ID, w["q_sdk_deg"])]
                          if bad2:
                              raise SystemExit("✗ 按控制器真实限位复检失败:\n  "
                                               + "\n  ".join(bad2[:10]))
                          print("按控制器真实限位复检: 全部通过 ✓")
              except SystemExit:
                  raise
              except Exception as exc:          # noqa: BLE001
                  print(f"⚠ 读取控制器轴极限失败({exc}), 沿用本地表")

          # ---- 提高反馈刷新率, 让跟随误差采样有分辨率 ----
          # 位置刷新周期: armSet/GetPosReFreshMS 在本机轮子里【不存在】(20260803 probe
          # 实测)。反馈固定 关节 40ms / 世界 50ms, 改不了 —— 任何比这更密的采样都是
          # 读重复值。相关代码已删, 不再徒劳尝试。

          first = moves[0]["q_sdk_deg"]
          cur = ss.read_joints(sdk, ARM_ID)
          if visual_run:
              bound_report = json.loads(Path(a.preflight_report).read_text(encoding="utf-8"))
              expected_active = bound_report["robot_state"][
                  "left_q_sdk_deg" if ARM_ID == 1 else "right_q_sdk_deg"]
              inactive_id = 2 if ARM_ID == 1 else 1
              expected_inactive = bound_report["robot_state"][
                  "right_q_sdk_deg" if ARM_ID == 1 else "left_q_sdk_deg"]
              inactive_now = ss.read_joints(sdk, inactive_id)
              active_change = max(abs(x-y) for x, y in zip(cur, expected_active))
              inactive_change = max(abs(x-y) for x, y in zip(inactive_now, expected_inactive))
              if active_change > 0.5 or inactive_change > 0.5:
                  raise RuntimeError(
                      f"机器人姿态自 preflight 后变化: active={active_change:.3f}° "
                      f"inactive={inactive_change:.3f}°；批准失效，重新规划")
          gap = max(abs(a - b) for a, b in zip(cur, first))
          print(f"当前姿态离首路点 {gap:.1f}°")
          if gap > 20.0 and not visual_run:
              input("⚠ 差角较大, 确认路径无障碍后回车(先用点到点移到首路点): ")
          ss.move_joints_abs(sdk, ARM_ID, first)
          ss.wait_until_joints(sdk, ARM_ID, first, label="(移到首路点)")

          if not visual_run:
              input("回车开始【伺服透传】执行(人守急停): ")
          else:
              scene_watchdog = subprocess.Popen(shlex.split(a.scene_watchdog_command))
              soft_stop_watchdog = subprocess.Popen(
                  shlex.split(a.soft_stop_watchdog_command))
              print(f"场景看门狗 pid={scene_watchdog.pid}; "
                    f"软急停看门狗 pid={soft_stop_watchdog.pid}")
          print("关闭保护状态(透传前置条件)...")
          ss.set_protect_status(sdk, ARM_ID, False)
          protect_off = True

          pacer.reset()
          for item in plan:
              if item[0] == "vision":
                  if item[2] is not None:
                      wait_settled(sdk, item[2], "视觉验收前")
                  ss.set_protect_status(sdk, ARM_ID, True)
                  protect_off = False
                  check = item[1]
                  command = shlex.split(a.vision_check_command) + [
                      "--kind", check["kind"], "--object-id", check["object_id"],
                      "--max-error-m", str(check["max_error_m"])]
                  print("运行视觉验收钩子:", command)
                  completed = subprocess.run(command, check=False)
                  if completed.returncode != 0:
                      raise RuntimeError(
                          f"视觉验收失败 kind={check['kind']} object={check['object_id']} "
                          f"rc={completed.returncode}")
                  ss.set_protect_status(sdk, ARM_ID, False)
                  protect_off = True
                  pacer.reset()
                  continue
              if item[0] == "grip":
                  # 夹爪动作期间: 先停稳, 再【临时恢复保护】才动夹爪, 完事重新关。
                  # 透传关保护是为了让密集指令不被误判; 夹爪等待的几秒里机械臂本就
                  # 静止, 没必要继续处于无保护状态。
                  if item[2] is not None:
                      wait_settled(sdk, item[2], "夹爪前")
                  ss.set_protect_status(sdk, ARM_ID, True)
                  protect_off = False
                  gripper(sdk, item[1])
                  # 负载告知(手册 2.7.63/64): 抓起来之后控制器才知道末端多了重量,
                  # 动力学补偿才对得上; 放下之后要卸载。不做的话跟随误差会偏大。
                  # armSetRobotLoad 在本机签名是 (int,int) 而非 LoadParameter,
                  # 语义未探明 -> 探到才用, 否则整段跳过(不再每次刷警告)。
                  if USE_PAYLOAD and caps.get("armSetRobotLoad") \
                          and caps.get("armUnsetLoad"):     # 本机 armUnsetLoad 没有
                      try:
                          if item[1] == "close":
                              ss.set_robot_load(sdk, ARM_ID, PAYLOAD_KG, PAYLOAD_COM_M)
                              loaded = True
                              print(f"  已告知控制器负载 {PAYLOAD_KG}kg")
                          else:
                              ss.unset_load(sdk, ARM_ID)
                              loaded = False
                              print("  已卸载负载")
                      except Exception as exc:   # noqa: BLE001
                          print(f"  ⚠ 负载设置失败({exc}), 继续执行")
                  ss.set_protect_status(sdk, ARM_ID, False)
                  protect_off = True
                  pacer.reset()        # ⚠ 必须: 否则节拍器会把夹爪那几秒"追回来"
                  continue
              for q in item[1]:                 # item[2] 是段末目标, 停稳时才用
                  if scene_watchdog is not None and scene_watchdog.poll() is not None:
                      raise RuntimeError(
                          f"场景看门狗提前退出 rc={scene_watchdog.returncode}; 停止任务")
                  if soft_stop_watchdog is not None and soft_stop_watchdog.poll() is not None:
                      raise RuntimeError(
                          f"软急停看门狗提前退出 rc={soft_stop_watchdog.returncode}; 停止任务")
                  # 视觉线的软急停读取由独立进程完成，热循环不做任何阻塞式 SDK 读取。
                  # 冻结 legacy Track C 保持原行为，避免改变已验证下发基线。
                  if not visual_run:
                      ss.check_soft_stop(sdk, "servo")
                  ss.pulse_to_servo(sdk, ARM_ID, q)
                  # ⚠ 默认关闭时这一句【零 SDK 调用】直接返回。开启时也只读一次
                  # (拿刚发出去的 q 当指令值)。热循环里绝不能再有第二次阻塞读。
                  follow.maybe_sample(sdk, commanded=q, label="透传中")
                  pacer.tick()
          # 终点: 用与夹爪前同一套"停稳"判据确认真的静止, 而不是 sleep 一下就读数
          final_target = moves[-1]["q_sdk_deg"]
          final_actual = wait_settled(sdk, final_target, "终点")
          err = max(abs(a - b) for a, b in zip(final_actual, final_target))
          print(f"\n✅ 透传执行完成: 终点最大误差 {err:.3f}° (已确认停稳)")
          run_success = True
          pacer.print_report("pulse 透传")
          follow.print_report()
      except BaseException as exc:
        run_failure = f"{type(exc).__name__}: {exc}"
        if sdk is not None:
            try:
                ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
                print("异常收尾: 已清路线并请求立即停止；持物状态不自动松爪。")
            except Exception as stop_exc:      # noqa: BLE001
                print("⚠ 清路线/停止失败，请现场人工处理:", stop_exc)
        raise
      finally:
        if scene_watchdog is not None and scene_watchdog.poll() is None:
            scene_watchdog.terminate()
            try:
                scene_watchdog.wait(timeout=3)
            except subprocess.TimeoutExpired:
                scene_watchdog.kill()
        if soft_stop_watchdog is not None and soft_stop_watchdog.poll() is None:
            soft_stop_watchdog.terminate()
            try:
                soft_stop_watchdog.wait(timeout=3)
            except subprocess.TimeoutExpired:
                soft_stop_watchdog.kill()
        # ⚠ 这里【不能 return】—— 在 finally 里 return 会把正在传播的异常吞掉,
        # 初始化失败时你就看不到原因了。只做清理, 让异常照常抛出去。
        if sdk is None:
            print("会话未建立(初始化阶段就失败), 无需收尾")
        else:
          # armUnsetLoad 本机不存在(20260803 probe), 负载分支也因此永不会开;
          # 万一将来探明了再恢复这一段。
          if protect_off:
              print("恢复保护状态...")
              try:
                  ss.set_protect_status(sdk, ARM_ID, True)
              except Exception as exc:      # noqa: BLE001
                  print("⚠ 恢复保护失败, 请手动确认:", exc)
          # 回读确认 —— "调用返回 0" 不等于 "保护真的开了"(手册 2.7.22)
          try:
              st = ss.read_protect_status(sdk, ARM_ID)
              print(f"保护状态回读: {st}  "
                    f"{'✓' if st else '✗ 仍未开启, 请手动处理!'}")
          except Exception as exc:          # noqa: BLE001
              print("⚠ 保护状态回读失败, 请手动确认:", exc)
          ss.close_session(sdk)
        if visual_run:
          try:
              out_dir = Path(os.environ.get("XIFENG_AS_RUN_DIR", "as_run"))
              out_dir.mkdir(parents=True, exist_ok=True)
              stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
              out = out_dir / f"as_run_{stamp}.json"
              manifest = {
                  "schema_version": "as_run.v1",
                  "status": "PASS" if run_success and run_failure is None else "FAILED",
                  "created_at": dt.datetime.now(dt.timezone.utc).isoformat(),
                  "robot_config_identity": a.robot_config_identity,
                  "robot_ip": ROBOT_IP, "local_ip": LOCAL_IP,
                  "arm_ip": ARM_IP, "arm_port": ARM_PORT,
                  "arm_id": ARM_ID, "gripper_id": GRIPPER_ID,
                  "trajectory_sha256": hashlib.sha256(Path(a.trajectory).read_bytes()).hexdigest(),
                  "preflight_report_sha256": authorization.get("report_sha256") if authorization else None,
                  "approval_token_sha256": (hashlib.sha256(
                      authorization["approval_token"].encode()).hexdigest()
                      if authorization else None),
                  "operator": authorization.get("operator") if authorization else None,
                  "observation_sha256": authorization.get("current_observation_sha256") if authorization else None,
                  "pulse": {"step_deg": STEP_DEG, "period_ms": PERIOD_S * 1000,
                            "global_speed_percent": GLOBAL_SPEED},
                  "script_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                  "failure": run_failure,
                  "full_scrubbed_log_required": True,
              }
              with out.open("x", encoding="utf-8") as fh:
                  json.dump(manifest, fh, ensure_ascii=False, indent=2); fh.write("\n")
              print(f"as-run manifest: {out}")
          except Exception as archive_exc:    # noqa: BLE001
              print("⚠ as-run manifest 保存失败:", archive_exc)
    return 0


if __name__ == "__main__":
    sys.exit(main())
