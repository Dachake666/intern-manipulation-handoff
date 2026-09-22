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
# 停止, 其余关节误差为 0); 右臂 j7 与其他关节的实测限位待测。
LIMITS_DEG = {
    1: [(-180.0, 60.0), (0.0, 150.0), (-140.0, 140.0), (-120.0, 0.0),
        (-170.0, 170.0), (-45.0, 45.0), (-90.0, 76.0)],       # 左臂
    2: [(-60.0, 180.0), (-150.0, 0.0), (-140.0, 140.0), (0.0, 120.0),
        (-170.0, 170.0), (-45.0, 45.0), (-90.0, 90.0)],       # 右臂
}
LIMIT_MARGIN_DEG = 5.0


def arm_name(arm_id):
    return {1: "左臂", 2: "右臂"}.get(arm_id, f"未知机械臂({arm_id})")


def require_code_zero(code, operation):
    if code != 0:
        raise RuntimeError(f"{operation}失败, 返回码: {code}")


def require_true_status(code, status, operation):
    if code != 0 or status is not True:
        raise RuntimeError(f"{operation}状态异常: code={code}, status={status}")


def make_float_vector(values):
    vec = pypilot.FloatVector()
    for v in values:
        vec.append(float(v))
    return vec


# ---------------------------------------------------------------- 会话
def open_session(robot_ip, local_ip, arm_ip, arm_port, global_speed, arm_ids=(1,)):
    """启动 SDK 并完成与真机验证过的完整初始化/使能/检查流程, 返回 sdk。"""
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

    code = sdk.armClearAlarm()
    print("clear alarm:", code)
    require_code_zero(code, "清除报警")

    code = sdk.armRobotEnableOrNot(True)
    print("enable all:", code)
    require_code_zero(code, "整体使能")
    time.sleep(1)

    code, enabled = sdk.armGetRobotEnableStatus()
    print("enable status:", (code, enabled))
    require_true_status(code, enabled, "整体使能")

    code, servo = sdk.armServoIsOpOrNot()
    print("servo op:", (code, servo))
    require_true_status(code, servo, "伺服")

    code = sdk.armSetGlobalSpeed(float(global_speed))
    print(f"set speed({global_speed}%):", code)
    require_code_zero(code, "设置全局速度")

    for aid in arm_ids:
        code, single = sdk.armGetSingleRobotEnableStatus(aid)
        print(f"{arm_name(aid)} single enable:", (code, single))
        require_true_status(code, single, f"{arm_name(aid)}使能")
    return sdk


def close_session(sdk):
    try:
        sdk.stop()
    except Exception as exc:      # noqa: BLE001 — 收尾失败只提示不掩盖主流程异常
        print("sdk.stop() 异常(忽略):", exc)


def check_soft_stop(sdk, operation):
    code, status = sdk.getSoftStopSwitch()
    if code != 0 or status != "CLOSE":
        raise RuntimeError(f"{operation}: 软急停状态异常 ({code}, {status})")


# ---------------------------------------------------------------- 读数
def read_joints(sdk, arm_id):
    code, vec = sdk.armGetJoints(arm_id, True)
    require_code_zero(code, f"读取{arm_name(arm_id)}关节角")
    joints = list(vec)
    if len(joints) != 7:
        raise RuntimeError(f"{arm_name(arm_id)}返回 {len(joints)} 个关节值, 预期 7 个")
    return joints


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
    """下发 7 关节绝对角度(度)。调用前必须已通过 check_limits。"""
    code = sdk.armMoveJoints(arm_id, make_float_vector(target_deg), False, 0)
    require_code_zero(code, f"{arm_name(arm_id)} armMoveJoints")


def wait_until_joints(sdk, arm_id, target_deg, tol_deg=0.5, timeout_s=25.0, poll_s=0.3):
    """轮询直到 7 关节全部到位(最大误差 <= tol), 返回实际关节角。"""
    deadline = time.time() + timeout_s
    while True:
        joints = read_joints(sdk, arm_id)
        err = max(abs(a - t) for a, t in zip(joints, target_deg))
        print(f"  等待到位: 最大误差 {err:.3f} deg", end="\r")
        if err <= tol_deg:
            print()
            return joints
        if time.time() > deadline:
            print()
            raise RuntimeError(f"{timeout_s}s 内未到位, 最大误差 {err:.3f} deg")
        time.sleep(poll_s)


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
