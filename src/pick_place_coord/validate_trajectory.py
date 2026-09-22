#!/usr/bin/env python3
"""发真机前的离线轨迹校验门(不连机器人)。

============================ 恢复重建说明 ============================
2026-07-22 重建(原文件随工程被误删)。输出格式逐字对齐本会话多次见到的
真实打印; 关节软限位表 LIMITS_SDK_DEG 中【j7 max=76 为高置信实测】,
其余关节上下限为 URDF 名义值占位, 首次严格使用前请与 URDF/真机复核。
详见工作区根目录 RECOVERY_STATUS.md。

校验项:
  1. 相邻路点最大关节步长(过大=跳点风险);
  2. 每个关节在软限位内(特别是左 j7 <= 76°);
  3. 夹爪事件 close/open 交替、偶数次(空序列=能力测试轨迹, 仅告警);
  4. meta 必备字段(arm_id/j6_flipped)。
=====================================================================
"""
from __future__ import annotations

import json
import os
import sys

_WORK = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _WORK)
import arm_profiles

# [高置信 j7] 左臂 SDK 关节软限位(度)。j7 max 76 为实测; 其余为占位, 需复核。
LIMITS_SDK_DEG = {
    1: (-170, 170), 2: (-120, 120), 3: (-170, 170), 4: (-135, 135),
    5: (-170, 170), 6: (-120, 120), 7: (-90, 76),
}
MAX_STEP_DEG = 6.0        # 相邻路点单关节最大允许步长(密集轨迹)
# 少点位轨迹(gen_minimal_joint_grasp)例外: armMoveJoints 在相邻路点间自行插值,
# 点少=停顿少。放宽步长的前提是 meta 同时声明 minimal_waypoints 与
# path_verified_dense(段内插值已在仿真逐点复核无碰撞); 仍设绝对上限兜底,
# 防止损坏/丢点的文件蒙混过关。
MINIMAL_MAX_STEP_DEG = 90.0


def validate(path, allow_candidate=False):
    with open(path) as f:
        d = json.load(f)
    if d.get("schema_version") == "trajectory.v2":
        contract = d.get("contract", {})
        meta = {"arm": contract.get("arm"), "arm_id": contract.get("arm_id"),
                "gripper_id": contract.get("gripper_id"), "j6_flipped": True}
    else:
        meta = d.get("meta", {})
    wps = d["waypoints"]
    moves = [w for w in wps if "q_sdk_deg" in w]
    grips = [w for w in wps if "gripper" in w]

    print(f"文件: {os.path.relpath(path)}")
    errs = []

    # 1. 相邻最大步长(少点位轨迹需 meta 双重声明才放宽)
    minimal = (meta.get("minimal_waypoints") is True
               and meta.get("path_verified_dense") is True)
    step_limit = MINIMAL_MAX_STEP_DEG if minimal else MAX_STEP_DEG
    max_step = 0.0
    for a, b in zip(moves, moves[1:]):
        step = max(abs(x - y) for x, y in zip(a["q_sdk_deg"], b["q_sdk_deg"]))
        max_step = max(max_step, step)
        if step > step_limit + 1e-6:
            errs.append(f"相邻步长 {step:.2f}° > {step_limit}° "
                        f"(段 {a['seg']}->{b['seg']})")
    if minimal:
        print(f"提示: 少点位轨迹(meta 已声明段内插值经仿真复核), "
              f"步长上限放宽至 {MINIMAL_MAX_STEP_DEG}°")

    # 2. 关节软限位（左右臂来自同一 ArmProfile；右臂未测时只可 candidate）
    arm = meta.get("arm") or ("left" if meta.get("arm_id") == 1 else
                              "right" if meta.get("arm_id") == 2 else None)
    try:
        profile = arm_profiles.arm_profile(arm)
    except Exception as exc:
        profile = None
        errs.append(f"无法解析臂配置: {exc}")
    limits = None
    if profile:
        limits = profile.get("controller_limits_deg")
        if limits is None:
            limits = profile.get("candidate_nominal_urdf_limits_deg")
            msg = f"{arm} 控制器真实限位未确认，只能标为 CANDIDATE"
            if allow_candidate:
                print("提示: " + msg)
            else:
                errs.append(msg + "；加 --candidate 仅做离线候选校验")
    for w in moves:
        for k, v in enumerate(w["q_sdk_deg"], 1):
            lo, hi = limits[k - 1] if limits else LIMITS_SDK_DEG[k]
            if not (lo + 5.0 <= v <= hi - 5.0):
                errs.append(f"j{k}={v:.2f}° 不满足 5° 余量 [{lo},{hi}] (段 {w['seg']})")
                break

    # 3. 夹爪交替
    seq = [w["gripper"] for w in grips]
    if seq:
        if len(seq) % 2 != 0:
            errs.append(f"夹爪事件 {seq} 应为 close/open 交替偶数次")
        for k, g in enumerate(seq):
            if g != ("close" if k % 2 == 0 else "open"):
                errs.append(f"夹爪事件序列非 close/open 交替: {seq}")
                break

    # 4. meta 必备
    if meta.get("arm_id") is None:
        errs.append("meta 缺 arm_id")
    if meta.get("j6_flipped") is not True:
        errs.append("meta.j6_flipped 非 True(q_sdk_deg 必须已含腕滚翻转)")
    if profile:
        if meta.get("arm_id") != profile["arm_id"]:
            errs.append(f"arm={arm} 与 arm_id={meta.get('arm_id')} 不匹配")
        if meta.get("gripper_id", profile["gripper_id"]) != profile["gripper_id"]:
            errs.append(f"{arm} 必须使用 gripper_id={profile['gripper_id']}")

    print(f"路点 {len(moves)} 个, 夹爪事件 {len(grips)} 个, "
          f"相邻最大步长 {max_step:.2f}°")
    print(f"meta: arm_id={meta.get('arm_id')} "
          f"j6_flipped={meta.get('j6_flipped')} "
          f"created={meta.get('created')}")
    if meta.get("pick_sdk_mm") or meta.get("place_sdk_mm"):
        print(f"pick_sdk={meta.get('pick_sdk_mm')}  "
              f"place_sdk={meta.get('place_sdk_mm')}")
    if not seq:
        print("提示: 无夹爪事件(能力测试轨迹, 允许)。")

    if errs:
        print("校验 FAIL ✗:")
        for e in errs:
            print(f"  ✗ {e}")
        return False
    if allow_candidate or (profile and "CANDIDATE" in profile.get("status", "")):
        print("校验 PASS ✓ 仅表示离线 CANDIDATE；不得据此执行或标记 REAL_VERIFIED")
    else:
        print("校验 PASS ✓ 可进入后续无运动预检；不等同于执行批准")
    return True


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    allow_candidate = "--candidate" in argv
    argv = [x for x in argv if x != "--candidate"]
    if not argv:
        print("用法: validate_trajectory.py <轨迹.json> [--candidate]")
        return 2
    return 0 if validate(argv[0], allow_candidate=allow_candidate) else 1


if __name__ == "__main__":
    sys.exit(main())
