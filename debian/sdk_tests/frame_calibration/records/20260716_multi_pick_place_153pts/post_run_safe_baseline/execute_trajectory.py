#!/usr/bin/env python3
"""真机轨迹执行器 —— 回放 pick_place_coord 导出并通过校验的关节轨迹。

流程:
  1. 读取轨迹 JSON, 内置复验(j6_flipped 标记/限位余量/步长/夹爪事件顺序), 不过即拒绝;
  2. 建立 SDK 会话(与已实测的初始化/使能流程一致), 低速;
  3. 夹爪张开 -> 从当前姿态缓慢移动到首路点(距离大会要求确认);
  4. 按段回放: 默认逐点停稳; --continuous 对已验证轨迹的段内中间点采用
     近目标交接, 段末/夹爪动作前/终点仍完整停稳;
  5. 每个新段落前可回车确认(CONFIRM_EACH_SEGMENT), q 随时中止。

诊断与恢复(20260708 真机测试后增强):
  * 真跑默认写 exec_YYYYmmdd_HHMMSS.log; dry-run 默认只输出终端,
    可用 --save-log 开启;
  * 到位等待带停滞检测: 误差 6s 无改善即提前诊断, 不再干等 30s;
  * 停滞/超时打印逐关节诊断表后只允许重试或中止; P2 真物执行禁止跳点;
  * 支持续跑: python3 execute_trajectory.py traj.json 37
    -> 跳过全局路点 #1..#36, 从 #37 开始(仅建议在 PICK 夹爪闭合之前的段落使用)。

用法(容器内):
    python3 execute_trajectory.py traj_latest.json [续跑起始路点号] [选项]
安全: ENABLE_REAL_MOTION=False 为干跑(打印全部计划, 不动机器人不动夹爪)。
     首次真跑保持 GLOBAL_SPEED<=5, 人守急停, 桌面/工作区清空。
"""
from __future__ import annotations

import argparse
import datetime
import hashlib
import json
import math
import sys
import time

import sdk_session as ss

# ---------------------------------------------------------------- 配置
ROBOT_IP = "192.168.8.148"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.148"
ARM_PORT = 8080

ARM_ID = 1                     # 1=左臂(轨迹 meta.arm_id 必须一致)
GLOBAL_SPEED = 5.0             # 全局速度百分比, 首次执行不要加大
ENABLE_REAL_MOTION = False
ENABLE_EXPERIMENTAL_CONTINUOUS = False  # 完成三点能力测试前不得改 True
CONFIRM_EACH_SEGMENT = True    # 每个段落前回车确认(q 中止)
FIRST_MOVE_WARN_DEG = 20.0     # 当前姿态离首路点超过此值时要求显式确认
WAIT_TOL_DEG = 0.8
WAIT_TIMEOUT_S = 30.0
STALL_S = 6.0                  # 误差持续无改善判定为停滞的时长
STALL_EPS_DEG = 0.05           # "有改善"的最小幅度
CONTINUOUS_MAX_TOL_DEG = 1.5   # 段内中间点允许的最大交接误差
CONTINUOUS_HANDOFF_RATIO = 0.30
CONTINUOUS_POLL_S = 0.05
CONTINUOUS_MAX_TURN_DEG = 20.0
SETTLE_STABLE_SAMPLES = 2
SETTLE_DELTA_DEG = 0.08
LIMIT_MARGIN_DEG = 5.0
MAX_STEP_DEG = 6.0
RECORD_WORLDS = False          # 默认不生成; 需要标定记录时加 --record-worlds
CONTINUOUS_SAFE_SEGMENTS = {
    "HOME->READY", "READY->PICK_HOVER", "PICK_HOVER->READY",
    "READY->PLACE_HOVER", "PLACE_HOVER->READY", "READY->HOME",
}

# 夹爪(串口透传, 命令与 SDK 使用指南 PDF p18 逐字一致)。
# 真机单发实测确认(20260709): 左夹爪 = 2 号, 右夹爪 = 1 号。
# (20260708 执行夹爪没动的原因就是当时只发了 1 号。)
GRIPPER_IDS = (2,)             # 左臂任务只发左夹爪
GRIPPER_COM = ("COM1", 115200, 8, 0, 1)
GRIPPER_CMDS = {
    (1, "close"): "eb 90 01 05 10 f4 01 64 00 6f",
    (2, "close"): "eb 90 02 05 10 f4 01 64 00 70",
    (1, "open"): "eb 90 01 03 11 f4 01 0a",
    (2, "open"): "eb 90 02 03 11 f4 01 0b",
}
GRIPPER_ACT_WAIT_S = {"close": 5.0, "open": 2.5}   # close 等待与 PDF 示例一致

_RUN_STAMP = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
_LOG_PATH = None


def configure_log(enabled):
    global _LOG_PATH
    _LOG_PATH = f"exec_{_RUN_STAMP}.log" if enabled else None


def log(msg=""):
    print(msg)
    if _LOG_PATH is not None:
        with open(_LOG_PATH, "a", encoding="utf-8") as f:
            f.write(msg + "\n")


def file_sha256(path):
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


# ---------------------------------------------------------------- 轨迹载入与复验
def load_and_check(path):
    with open(path) as f:
        data = json.load(f)
    meta, wps = data.get("meta", {}), data.get("waypoints", [])
    errors = []
    if meta.get("j6_flipped") is not True:
        errors.append("meta.j6_flipped 不为 True: 禁止下发(j6 符号未翻转?)")
    if meta.get("arm_id") != ARM_ID:
        errors.append(f"meta.arm_id={meta.get('arm_id')} 与配置 ARM_ID={ARM_ID} 不符")
    moves = [w for w in wps if "q_sdk_deg" in w]
    if not moves:
        errors.append("没有运动路点")
    for i, w in enumerate(wps):
        if "q_sdk_deg" in w:
            q = w["q_sdk_deg"]
            if len(q) != 7 or not all(math.isfinite(v) for v in q):
                errors.append(f"路点#{i} 数值非法")
                continue
            bad = ss.check_limits(ARM_ID, q, margin=LIMIT_MARGIN_DEG)
            errors.extend(f"路点#{i} {b}" for b in bad)
    for w1, w2 in zip(moves, moves[1:]):
        step = max(abs(a - b) for a, b in zip(w1["q_sdk_deg"], w2["q_sdk_deg"]))
        if step > MAX_STEP_DEG:
            errors.append(f"存在超步长路点 {step:.2f}° > {MAX_STEP_DEG}°")
    acts = [w["gripper"] for w in wps if "gripper" in w]
    # 多点任务为 close/open 交替 N 次; 单点任务即 N=1 的特例
    if acts and not (len(acts) % 2 == 0 and all(
            a == ("close" if k % 2 == 0 else "open")
            for k, a in enumerate(acts))):
        errors.append(f"夹爪事件序列 {acts}, 预期 close/open 交替偶数次")
    if errors:
        log("轨迹复验 FAIL:")
        for e in errors:
            log(f"  ✗ {e}")
        raise SystemExit(1)
    log(f"轨迹复验 PASS: {len(moves)} 路点, {len(acts)} 夹爪事件, "
        f"pick_sdk={meta.get('pick_sdk_mm')}, place_sdk={meta.get('place_sdk_mm')}")
    return meta, wps


def confirm(prompt):
    ans = input(f"{prompt} -> 回车继续 / q 中止: ").strip().lower()
    if ans == "q":
        raise KeyboardInterrupt("用户中止")


# ---------------------------------------------------------------- 到位等待 + 诊断
def _diag_table(target, actual):
    log("    逐关节诊断(deg):")
    errs = [a - t for a, t in zip(actual, target)]
    worst = max(range(7), key=lambda k: abs(errs[k]))
    for k in range(7):
        mark = "  <-- 最大误差, 疑似真实限位/阻挡" if k == worst else ""
        log(f"      j{k+1}: 目标 {target[k]:+8.3f}  实际 {actual[k]:+8.3f}  "
            f"误差 {errs[k]:+7.3f}{mark}")


def wait_with_diag(sdk, target, tag, tol_deg=WAIT_TOL_DEG, poll_s=0.3,
                   arrival_label="到位", allow_skip=True, handoff=False):
    """轮询到位。成功返回实际关节角; 用户选择跳过返回 None; 中止抛 KeyboardInterrupt。
    停滞(误差 STALL_S 秒无改善)或超时 -> 打印逐关节诊断 -> 交互选择。"""
    while True:
        deadline = time.time() + WAIT_TIMEOUT_S
        best_err, best_time = None, time.time()
        reason = "超时"
        previous = None
        stable_samples = 0
        while time.time() < deadline:
            ss.check_soft_stop(sdk, tag)
            actual = ss.read_joints(sdk, ARM_ID)
            moving = ss.read_move_state(sdk, ARM_ID)
            err = max(abs(a - t) for a, t in zip(actual, target))
            if err <= tol_deg:
                delta = (max(abs(a - b) for a, b in zip(actual, previous))
                         if previous is not None else float("inf"))
                if handoff and moving:
                    stable_samples = SETTLE_STABLE_SAMPLES
                elif not moving and delta <= SETTLE_DELTA_DEG:
                    stable_samples += 1
                else:
                    stable_samples = 0
                if stable_samples >= SETTLE_STABLE_SAMPLES:
                    errs = [abs(a - t) for a, t in zip(actual, target)]
                    worst = max(range(7), key=lambda k: errs[k])
                    state = "运动中" if moving else "已停稳"
                    log(f"    {tag} {arrival_label}: 最大误差 {err:.3f}° "
                        f"(阈值 {tol_deg:.3f}°, j{worst+1}, {state})")
                    return actual
            else:
                stable_samples = 0
            if best_err is None or err < best_err - STALL_EPS_DEG:
                best_err, best_time = err, time.time()
            elif time.time() - best_time > STALL_S:
                reason = f"停滞(误差 {STALL_S:.0f}s 无改善)"
                break
            previous = actual
            time.sleep(poll_s)
        actual = ss.read_joints(sdk, ARM_ID)
        err = max(abs(a - t) for a, t in zip(actual, target))
        log(f"  ⚠ {tag} 未到位({reason}): 剩余最大误差 {err:.3f}°")
        _diag_table(target, actual)
        choices = ("    [r]重试等待 / [s]跳过此路点(继续下一个) / [q]中止: "
                   if allow_skip else
                   "    [r]重试等待 / [q]中止(连续模式禁止跳点): ")
        ans = input(choices).strip().lower()
        log(f"    用户选择: {ans or 'r'}")
        if ans == "s" and allow_skip:
            return None
        if ans == "q":
            raise KeyboardInterrupt("用户中止")
        # 默认重试: 重新下发同一目标再等
        ss.move_joints_abs(sdk, ARM_ID, target)


def waypoint_turn_deg(waypoints, index):
    """返回当前运动路点的关节空间转角；0° 为近似直行。"""
    current = waypoints[index]
    if "q_sdk_deg" not in current:
        return 180.0
    previous = waypoints[index - 1] if index > 0 else None
    nxt = waypoints[index + 1] if index + 1 < len(waypoints) else None
    if (previous is None or nxt is None or
            "q_sdk_deg" not in previous or "q_sdk_deg" not in nxt or
            previous.get("seg") != current.get("seg") or
            nxt.get("seg") != current.get("seg")):
        return 0.0
    a = [q1 - q0 for q0, q1 in zip(previous["q_sdk_deg"],
                                    current["q_sdk_deg"])]
    b = [q2 - q1 for q1, q2 in zip(current["q_sdk_deg"],
                                    nxt["q_sdk_deg"])]
    na = math.sqrt(sum(v * v for v in a))
    nb = math.sqrt(sum(v * v for v in b))
    if na < 1e-9 or nb < 1e-9:
        return 0.0
    cosine = max(-1.0, min(1.0, sum(x * y for x, y in zip(a, b)) / (na * nb)))
    return math.degrees(math.acos(cosine))


def waypoint_requires_full_settle(waypoints, index):
    """近物段、急转角、段末、夹爪事件前和终点必须完整停稳。"""
    current = waypoints[index]
    if ("q_sdk_deg" not in current or
            current.get("seg") not in CONTINUOUS_SAFE_SEGMENTS or
            index + 1 >= len(waypoints)):
        return True
    nxt = waypoints[index + 1]
    return ("gripper" in nxt or "q_sdk_deg" not in nxt or
            nxt.get("seg") != current.get("seg") or
            waypoint_turn_deg(waypoints, index) > CONTINUOUS_MAX_TURN_DEG)


def continuous_handoff_tol(actual, target):
    """按本步实际距离自适应交接阈值, 避免小步路点被直接越过。"""
    step = max(abs(a - t) for a, t in zip(actual, target))
    return min(CONTINUOUS_MAX_TOL_DEG,
               max(WAIT_TOL_DEG, step * CONTINUOUS_HANDOFF_RATIO))


# ---------------------------------------------------------------- 夹爪
def gripper(sdk, action):
    for gid in GRIPPER_IDS:                # 与已验证的说明书示例一致: 逐个 ID 连发
        hexcmd = GRIPPER_CMDS[(gid, action)]
        log(f"  [夹爪{gid}] {action}  hex={hexcmd}")
        if not ENABLE_REAL_MOTION:
            log("  DRY RUN: 未发送")
            continue
        code = sdk.armWriteCom(GRIPPER_COM[0], bytes.fromhex(hexcmd))
        ss.require_code_zero(code, f"夹爪{gid} {action}")
    if ENABLE_REAL_MOTION:
        time.sleep(GRIPPER_ACT_WAIT_S[action])


# ---------------------------------------------------------------- worlds 记录
def snap_worlds(sdk, records, mi, seg, q_target, q_actual, enabled):
    """路点到位后读一次 armGetWorlds, 追加进 records。读失败只提示不打断轨迹。"""
    if not (ENABLE_REAL_MOTION and enabled):
        return
    try:
        w = ss.read_worlds(sdk, ARM_ID)
        records.append({"i": mi, "seg": seg,
                        "q_target_deg": list(q_target),
                        "q_sdk_deg": list(q_actual),
                        "sdk_world_xyz_mm": w[:3],
                        "sdk_world_uvw_deg": w[3:]})
    except Exception as exc:      # noqa: BLE001 — 记录是附加功能, 不影响执行
        log(f"    (worlds 读取失败, 本点未记录: {exc})")


def save_worlds(records, traj_path):
    if not records:
        return
    path = f"worlds_record_{_RUN_STAMP}.json"
    with open(path, "w", encoding="utf-8") as f:
        json.dump({"meta": {"source_traj": traj_path, "arm_id": ARM_ID,
                            "note": "每条含 到位实际关节角+armGetWorlds 读数; "
                                    "喂给 analysis/verify_worlds_record.py"},
                   "records": records}, f, indent=1, ensure_ascii=False)
    log(f"worlds 记录已保存: {path} ({len(records)} 条)")


# ---------------------------------------------------------------- 主流程
def build_arg_parser():
    ap = argparse.ArgumentParser(
        description="低速回放已校验的 pick-and-place 关节轨迹")
    ap.add_argument("trajectory_file", help="轨迹 JSON")
    ap.add_argument("start_from", nargs="?", type=int, default=1,
                    help="续跑起始运动路点号(默认 1)")
    ap.add_argument(
        "--continuous", action="store_true",
        help="实验性段内近目标交接; 仅用于已冻结并完成低速空爪验证的安全轨迹")
    ap.add_argument(
        "--record-worlds", action="store_true", default=RECORD_WORLDS,
        help="逐个完整到位路点保存 armGetWorlds JSON(默认关闭)")
    log_group = ap.add_mutually_exclusive_group()
    log_group.add_argument("--save-log", dest="save_log", action="store_true",
                           help="保存 exec_*.log(dry-run 默认不保存)")
    log_group.add_argument("--no-log", dest="save_log", action="store_false",
                           help="不保存 exec_*.log(真实运动不允许关闭)")
    ap.set_defaults(save_log=None)
    return ap


def main(argv=None):
    ap = build_arg_parser()
    args = ap.parse_args(argv)
    if args.start_from < 1:
        ap.error("续跑起始路点号必须 >= 1")
    if args.continuous and args.record_worlds:
        ap.error("--continuous 与 --record-worlds 不能同时使用; 标定采数必须逐点停稳")
    if (ENABLE_REAL_MOTION and args.continuous and
            not ENABLE_EXPERIMENTAL_CONTINUOUS):
        ap.error("真实连续实验尚未放行: 先完成开阔区三点能力测试，再显式设置 "
                 "ENABLE_EXPERIMENTAL_CONTINUOUS=True")
    if ENABLE_REAL_MOTION and args.save_log is False:
        ap.error("真实运动必须保存执行日志; --no-log 只允许 dry-run")

    save_log = ENABLE_REAL_MOTION if args.save_log is None else args.save_log
    configure_log(save_log)
    log(f"日志文件: {_LOG_PATH or '关闭(仅终端输出)'}")
    log(f"轨迹 SHA-256: {file_sha256(args.trajectory_file)}")
    meta, wps = load_and_check(args.trajectory_file)
    if args.continuous:
        declared = set(meta.get("continuous_candidate_segments", []))
        if meta.get("continuous_candidate") is not True:
            ap.error("轨迹未声明 continuous_candidate=True; 请用新版规划器重新生成")
        if declared != CONTINUOUS_SAFE_SEGMENTS:
            ap.error("轨迹 continuous_candidate_segments 与执行器白名单不一致")
    has_gripper = any("gripper" in w for w in wps)

    # 给每个运动路点编全局号 1..N(与日志/续跑参数对应)
    n_moves = sum(1 for w in wps if "q_sdk_deg" in w)
    if args.start_from > n_moves:
        log(f"起始路点 #{args.start_from} 超出轨迹运动路点总数 {n_moves}")
        return 2
    if args.start_from > 1:
        log(f"\n⚠ 续跑模式: 从全局路点 #{args.start_from}/{n_moves} 开始。"
            f"仅建议用于 PICK 闭合之前的段落; 被跳过的夹爪事件不会补发。")
    log(f"运动模式: {'段内近目标交接(实验)' if args.continuous else '逐点完整停稳'}")
    log(f"worlds JSON: {'开启' if args.record_worlds else '关闭'}")

    records = []
    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT,
                          GLOBAL_SPEED, (ARM_ID,))
    try:
        if has_gripper:
            code = sdk.armOpenCom(*GRIPPER_COM)
            log(f"armOpenCom: {code}")
            ss.require_code_zero(code, "打开夹爪串口")

        cur = ss.read_joints(sdk, ARM_ID)
        mi = 0
        first = first_seg = None
        for w in wps:
            if "q_sdk_deg" in w:
                mi += 1
                if mi >= args.start_from:
                    first = w["q_sdk_deg"]
                    first_seg = w["seg"]
                    break
        gap = max(abs(a - b) for a, b in zip(cur, first))
        log(f"\n当前关节: {[round(v, 2) for v in cur]}")
        log(f"起始路点(#{args.start_from}): {[round(v, 2) for v in first]}   "
            f"最大差 {gap:.1f}°")
        log(f"\nENABLE_REAL_MOTION = {ENABLE_REAL_MOTION}")

        if not ENABLE_REAL_MOTION:
            log("\nDRY RUN 计划(未发送任何运动):")
            seg, mi = None, 0
            seg_start = seg_n = 0
            lines = []
            for w in wps + [{"seg": "_END_"}]:
                if w["seg"] != seg:
                    if seg is not None and seg_n:
                        lines.append((seg, seg_start, mi, seg_n))
                    seg, seg_start, seg_n = w["seg"], mi + 1, 0
                if "q_sdk_deg" in w:
                    mi += 1
                    seg_n += 1
                if "gripper" in w:
                    lines.append((f"{w['seg']} 夹爪:{w['gripper']}", None, None, None))
            for name, a, b, n in lines:
                log(f"  {name:30s}" + (f" 路点 #{a}..#{b} ({n} 个)" if n else ""))
            if args.continuous:
                n_flow = move_no = 0
                for i, w in enumerate(wps):
                    if "q_sdk_deg" not in w:
                        continue
                    move_no += 1
                    if (move_no > args.start_from and
                            not waypoint_requires_full_settle(wps, i)):
                        n_flow += 1
                log(f"连续交接预览: {n_flow} 个段内中间点; 其余运动点完整停稳。")
            log("确认计划后改 ENABLE_REAL_MOTION=True 重跑。")
            return 0

        if gap > FIRST_MOVE_WARN_DEG:
            confirm(f"⚠ 当前姿态离起始路点 {gap:.1f}°(>{FIRST_MOVE_WARN_DEG}°), "
                    f"确认路径上无障碍再继续")
        if has_gripper and args.start_from == 1:
            confirm("先张开夹爪")
            gripper(sdk, "open")

        confirm(f"移动到起始路点 #{args.start_from}")
        ss.check_soft_stop(sdk, "起始路点")
        ss.move_joints_abs(sdk, ARM_ID, first)
        first_actual = wait_with_diag(
            sdk, first, f"起始路点#{args.start_from}", allow_skip=False)
        if first_actual is None:
            log("起始路点被跳过, 中止更安全。")
            return 1
        snap_worlds(sdk, records, args.start_from, first_seg, first,
                    first_actual, args.record_worlds)

        seg, mi, n_done, n_skip = first_seg, 0, 1, 0
        last_actual = first_actual
        for wi, w in enumerate(wps):
            if "q_sdk_deg" in w:
                mi += 1
                if mi <= args.start_from:
                    continue
            elif mi < args.start_from:
                log(f"  (续跑跳过夹爪事件: {w.get('gripper')})")
                continue
            if w["seg"] != seg:
                seg = w["seg"]
                if CONFIRM_EACH_SEGMENT:
                    confirm(f"\n段落 [{seg}]")
                else:
                    log(f"\n段落 [{seg}]")
            if "gripper" in w:
                gripper(sdk, w["gripper"])
                continue
            q = w["q_sdk_deg"]
            full_settle = (not args.continuous or
                           waypoint_requires_full_settle(wps, wi))
            tol = (WAIT_TOL_DEG if full_settle else
                   continuous_handoff_tol(last_actual, q))
            mode = "停稳" if full_settle else "连续交接"
            log(f"  [{mi}/{n_moves}] {seg} ({mode}) -> "
                f"{[round(v, 2) for v in q]}")
            ss.check_soft_stop(sdk, seg)
            ss.move_joints_abs(sdk, ARM_ID, q)
            actual = wait_with_diag(
                sdk, q, f"#{mi}", tol_deg=tol,
                poll_s=0.3 if full_settle else CONTINUOUS_POLL_S,
                arrival_label="到位" if full_settle else "交接",
                allow_skip=False,
                handoff=not full_settle)
            if actual is None:
                n_skip += 1
            else:
                n_done += 1
                last_actual = actual
                if full_settle:
                    snap_worlds(sdk, records, mi, seg, q, actual,
                                args.record_worlds)
        if n_skip:
            log(f"\n❌ 轨迹结束但存在 {n_skip} 个跳过路点; 本次判定失败。")
            return 1
        log(f"\n✅ 轨迹执行完成: {n_done}/{n_moves - args.start_from + 1} "
            "个路点已执行, 零跳点。")
        return 0
    except KeyboardInterrupt:
        log("\n已中止，正在请求控制器立即清除残留路径。")
        try:
            ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
            log("控制器残留路径已清除。")
        except Exception as exc:      # noqa: BLE001 — 仍需继续关闭 SDK 会话
            log(f"⚠ 清除控制器路径失败: {exc}; 保持急停监护。")
        log(f"续跑方式: python3 execute_trajectory.py "
            f"{args.trajectory_file} <路点号>")
        return 1
    except Exception:
        try:
            ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
            log("异常中止: 控制器残留路径已清除。")
        except Exception as exc:      # noqa: BLE001 — 不掩盖原异常
            log(f"⚠ 异常中止且清除路径失败: {exc}; 保持急停监护。")
        raise
    finally:
        save_worlds(records, args.trajectory_file)
        ss.close_session(sdk)
        if _LOG_PATH is not None:
            log(f"\n日志已保存: {_LOG_PATH}")
        else:
            log("\n日志文件未生成(本次仅终端输出)。")
        log("提醒: 请确认 ENABLE_REAL_MOTION 已恢复为 False。")


if __name__ == "__main__":
    sys.exit(main())
