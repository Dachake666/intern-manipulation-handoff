#!/usr/bin/env python3
"""单目标世界坐标运动模板。

填写 get_target_world() 返回的 [X,Y,Z(mm),U,V,W(deg)]，先保持
ENABLE_REAL_MOTION=False 做只读预检；确认后改 True，人工回车才执行。

Debian 容器内：python3 move_worlds_once.py
同目录需要：sdk_session.py、robot_lock.py
"""
from __future__ import annotations

import math


# ---------------------------------------------------------------- 配置
ROBOT_IP = "192.168.8.147"       # 机器人变化时只改这里
LOCAL_IP = "192.168.8.185"
ARM_IP = ROBOT_IP
ARM_PORT = 8080

ARM_ID = 1                       # 1=左臂
GLOBAL_SPEED = 1.0               # 全局速度百分比
ENABLE_REAL_MOTION = False       # 默认只读预检

SAMPLE_MM = 2.0                  # 世界直线密集预检间距
SAMPLE_DEG = 1.0
LIMIT_MARGIN_DEG = 5.0
MAX_JOINT_STEP_DEG = 4.0


def get_target_world():
    """接口接入点：以后只需把六个 None 换成目标 XYZUVW。"""
    return [
        None,  # X (mm)
        None,  # Y (mm)
        None,  # Z (mm)
        None,  # U (deg)
        None,  # V (deg)
        None,  # W (deg)
    ]


def _pose6(values):
    if len(values) != 6 or any(v is None for v in values):
        raise SystemExit("请先在 get_target_world() 中填写完整的 X,Y,Z,U,V,W")
    pose = [float(v) for v in values]
    if not all(math.isfinite(v) for v in pose):
        raise SystemExit("目标 XYZUVW 必须是有限数值")
    return pose


def _dense_line(start, target):
    distance = math.sqrt(sum((b - a) ** 2 for a, b in zip(start[:3], target[:3])))
    angle = max(abs(b - a) for a, b in zip(start[3:], target[3:]))
    count = max(1, math.ceil(max(distance / SAMPLE_MM, angle / SAMPLE_DEG)))
    return [[a + (b - a) * i / count for a, b in zip(start, target)]
            for i in range(1, count + 1)]


def _precheck(ss, sdk, start_world, start_joints, target):
    limits = ss.read_axis_limits(sdk, ARM_ID)
    valid, details = ss.axis_limits_look_valid(limits)
    if not valid:
        raise RuntimeError(f"控制器轴限位读数无效: {details}")

    previous = list(start_joints)
    dense = _dense_line(start_world, target)
    for index, pose in enumerate(dense, 1):
        joints = ss.try_worlds(sdk, ARM_ID, pose)
        if joints is None:
            raise RuntimeError(f"密集点 {index}/{len(dense)} 不可达或奇异")
        bad = ss.check_limits(ARM_ID, joints, margin=LIMIT_MARGIN_DEG)
        for axis, (q, (lo, hi)) in enumerate(zip(joints, limits), 1):
            if not lo + LIMIT_MARGIN_DEG <= q <= hi - LIMIT_MARGIN_DEG:
                bad.append(f"j{axis}={q:+.2f}° 接近控制器限位 [{lo:+.1f},{hi:+.1f}]°")
        if bad:
            raise RuntimeError(f"密集点 {index}/{len(dense)} 限位失败:\n  " + "\n  ".join(bad))
        jump = max(abs(a - b) for a, b in zip(joints, previous))
        if jump > MAX_JOINT_STEP_DEG:
            raise RuntimeError(
                f"密集点 {index}/{len(dense)} IK 跳变 {jump:.2f}° > "
                f"{MAX_JOINT_STEP_DEG:.2f}°")
        previous = list(joints)
    return len(dense), previous


def run(ss, target, input_fn=input):
    sdk = None
    try:
        sdk = ss.open_session(
            ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT,
            GLOBAL_SPEED, (ARM_ID,), enable=ENABLE_REAL_MOTION)
        current_world = ss.read_worlds(sdk, ARM_ID)
        current_joints = ss.read_joints(sdk, ARM_ID)
        count, target_joints = _precheck(
            ss, sdk, current_world, current_joints, target)

        print("当前 XYZUVW:", [round(v, 3) for v in current_world])
        print("目标 XYZUVW:", [round(v, 3) for v in target])
        print(f"预检通过: {count} 个密集 IK 点")
        print("目标关节角:", [round(v, 3) for v in target_joints])
        print("注意: 程序没有现场障碍物模型，执行前必须人工确认路径已清空。")

        if not ENABLE_REAL_MOTION:
            print("DRY RUN: 未清报警、未使能、未设置速度、未发送运动。")
            return 0
        if input_fn("回车执行 / q 中止: ").strip().lower() == "q":
            return 2

        ss.check_soft_stop(sdk, "世界坐标运动前")
        ss.move_worlds(sdk, ARM_ID, target, interpolation_en=False)
        actual = ss.wait_until_worlds(
            sdk, ARM_ID, target, tol_mm=2.0, tol_deg=1.0,
            timeout_s=25.0, label="单目标")
        print("实际到位 XYZUVW:", [round(v, 3) for v in actual])
        return 0
    except BaseException:
        if sdk is not None and ENABLE_REAL_MOTION:
            try:
                ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
            except Exception:
                pass
        raise
    finally:
        if sdk is not None:
            ss.close_session(sdk)


def main():
    target = _pose6(get_target_world())

    import robot_lock
    import sdk_session as ss

    with robot_lock.acquire(ROBOT_IP, ARM_ID, "move_worlds_once.py"):
        return run(ss, target)


if __name__ == "__main__":
    raise SystemExit(main())
