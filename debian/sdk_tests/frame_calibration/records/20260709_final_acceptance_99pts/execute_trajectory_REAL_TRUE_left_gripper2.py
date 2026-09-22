#!/usr/bin/env python3
"""真机轨迹执行器 —— 回放 pick_place_coord 导出并通过校验的关节轨迹。

流程:
  1. 读取轨迹 JSON, 内置复验(j6_flipped 标记/限位余量/步长/夹爪事件顺序), 不过即拒绝;
  2. 建立 SDK 会话(与已实测的初始化/使能流程一致), 低速;
  3. 夹爪张开 -> 从当前姿态缓慢移动到首路点(距离大会要求确认);
  4. 按段回放: armMoveJoints 逐点下发 + 轮询到位; 夹爪事件发 armWriteCom;
  5. 每个新段落前可回车确认(CONFIRM_EACH_SEGMENT), q 随时中止。

诊断与恢复(20260708 真机测试后增强):
  * 全程逐点日志自动写入 exec_YYYYmmdd_HHMMSS.log(目标/实际/逐关节误差);
  * 到位等待带停滞检测: 误差 6s 无改善即提前诊断, 不再干等 30s;
  * 停滞/超时不再直接抛异常, 打印逐关节诊断表后交互选择: 重试/跳过/中止;
  * 支持续跑: python3 execute_trajectory.py traj.json 37
    -> 跳过全局路点 #1..#36, 从 #37 开始(仅建议在 PICK 夹爪闭合之前的段落使用)。

用法(容器内):
    python3 execute_trajectory.py traj_latest.json [续跑起始路点号]
安全: ENABLE_REAL_MOTION=False 为干跑(打印全部计划, 不动机器人不动夹爪)。
     首次真跑保持 GLOBAL_SPEED<=5, 人守急停, 桌面/工作区清空。
"""
from __future__ import annotations

import datetime
import json
import math
import sys
import time

import sdk_session as ss

# ---------------------------------------------------------------- 配置
ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = "192.168.8.147"
ARM_PORT = 8080

ARM_ID = 1                     # 1=左臂(轨迹 meta.arm_id 必须一致)
GLOBAL_SPEED = 5.0             # 全局速度百分比, 首次执行不要加大
ENABLE_REAL_MOTION = True     # False=干跑; True=真实运动+真实夹爪
CONFIRM_EACH_SEGMENT = True    # 每个段落前回车确认(q 中止)
FIRST_MOVE_WARN_DEG = 20.0     # 当前姿态离首路点超过此值时要求显式确认
WAIT_TOL_DEG = 0.8
WAIT_TIMEOUT_S = 30.0
STALL_S = 6.0                  # 误差持续无改善判定为停滞的时长
STALL_EPS_DEG = 0.05           # "有改善"的最小幅度
LIMIT_MARGIN_DEG = 5.0
MAX_STEP_DEG = 6.0

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

_LOG_PATH = f"exec_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.log"


def log(msg=""):
    print(msg)
    with open(_LOG_PATH, "a") as f:
        f.write(msg + "\n")


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
    if acts and acts != ["close", "open"]:
        errors.append(f"夹爪事件序列 {acts}, 预期 ['close', 'open']")
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


def wait_with_diag(sdk, target, tag):
    """轮询到位。成功返回实际关节角; 用户选择跳过返回 None; 中止抛 KeyboardInterrupt。
    停滞(误差 STALL_S 秒无改善)或超时 -> 打印逐关节诊断 -> 交互选择。"""
    while True:
        deadline = time.time() + WAIT_TIMEOUT_S
        best_err, best_time = None, time.time()
        reason = "超时"
        while time.time() < deadline:
            actual = ss.read_joints(sdk, ARM_ID)
            err = max(abs(a - t) for a, t in zip(actual, target))
            if err <= WAIT_TOL_DEG:
                errs = [abs(a - t) for a, t in zip(actual, target)]
                worst = max(range(7), key=lambda k: errs[k])
                log(f"    {tag} 到位: 最大误差 {err:.3f}° (j{worst+1})")
                return actual
            if best_err is None or err < best_err - STALL_EPS_DEG:
                best_err, best_time = err, time.time()
            elif time.time() - best_time > STALL_S:
                reason = f"停滞(误差 {STALL_S:.0f}s 无改善)"
                break
            time.sleep(0.3)
        actual = ss.read_joints(sdk, ARM_ID)
        err = max(abs(a - t) for a, t in zip(actual, target))
        log(f"  ⚠ {tag} 未到位({reason}): 剩余最大误差 {err:.3f}°")
        _diag_table(target, actual)
        ans = input("    [r]重试等待 / [s]跳过此路点(继续下一个) / "
                    "[q]中止: ").strip().lower()
        log(f"    用户选择: {ans or 'r'}")
        if ans == "s":
            return None
        if ans == "q":
            raise KeyboardInterrupt("用户中止")
        # 默认重试: 重新下发同一目标再等
        ss.move_joints_abs(sdk, ARM_ID, target)


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


# ---------------------------------------------------------------- 主流程
def main():
    if len(sys.argv) not in (2, 3):
        print("用法: python3 execute_trajectory.py <轨迹.json> [续跑起始路点号]")
        return 2
    start_from = int(sys.argv[2]) if len(sys.argv) == 3 else 1
    log(f"日志文件: {_LOG_PATH}")
    meta, wps = load_and_check(sys.argv[1])
    has_gripper = any("gripper" in w for w in wps)

    # 给每个运动路点编全局号 1..N(与日志/续跑参数对应)
    n_moves = sum(1 for w in wps if "q_sdk_deg" in w)
    if start_from > 1:
        log(f"\n⚠ 续跑模式: 从全局路点 #{start_from}/{n_moves} 开始。"
            f"仅建议用于 PICK 闭合之前的段落; 被跳过的夹爪事件不会补发。")

    sdk = ss.open_session(ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT,
                          GLOBAL_SPEED, (ARM_ID,))
    try:
        if has_gripper:
            code = sdk.armOpenCom(*GRIPPER_COM)
            log(f"armOpenCom: {code}")
            ss.require_code_zero(code, "打开夹爪串口")

        cur = ss.read_joints(sdk, ARM_ID)
        mi = 0
        first = None
        for w in wps:
            if "q_sdk_deg" in w:
                mi += 1
                if mi >= start_from:
                    first = w["q_sdk_deg"]
                    break
        gap = max(abs(a - b) for a, b in zip(cur, first))
        log(f"\n当前关节: {[round(v, 2) for v in cur]}")
        log(f"起始路点(#{start_from}): {[round(v, 2) for v in first]}   "
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
            log("确认计划后改 ENABLE_REAL_MOTION=True 重跑。")
            return 0

        if gap > FIRST_MOVE_WARN_DEG:
            confirm(f"⚠ 当前姿态离起始路点 {gap:.1f}°(>{FIRST_MOVE_WARN_DEG}°), "
                    f"确认路径上无障碍再继续")
        if has_gripper and start_from == 1:
            confirm("先张开夹爪")
            gripper(sdk, "open")

        confirm(f"移动到起始路点 #{start_from}")
        ss.check_soft_stop(sdk, "起始路点")
        ss.move_joints_abs(sdk, ARM_ID, first)
        if wait_with_diag(sdk, first, f"起始路点#{start_from}") is None:
            log("起始路点被跳过, 中止更安全。")
            return 1

        seg, mi, n_done, n_skip = None, 0, 0, 0
        for w in wps:
            if "q_sdk_deg" in w:
                mi += 1
                if mi < start_from:
                    continue
            elif mi < start_from:
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
            log(f"  [{mi}/{n_moves}] {seg} -> {[round(v, 2) for v in q]}")
            ss.check_soft_stop(sdk, seg)
            ss.move_joints_abs(sdk, ARM_ID, q)
            if wait_with_diag(sdk, q, f"#{mi}") is None:
                n_skip += 1
            else:
                n_done += 1
        log(f"\n✅ 轨迹执行完成: 到位 {n_done} 个, 跳过 {n_skip} 个路点。")
        return 0
    except KeyboardInterrupt:
        log("\n已中止。机器人停在当前位置, 请人工确认状态。")
        log(f"续跑方式: python3 execute_trajectory.py {sys.argv[1]} <路点号>")
        return 1
    finally:
        ss.close_session(sdk)
        log(f"\n日志已保存: {_LOG_PATH}")
        log("提醒: 请确认 ENABLE_REAL_MOTION 已恢复为 False。")


if __name__ == "__main__":
    sys.exit(main())
