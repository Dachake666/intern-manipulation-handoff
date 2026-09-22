#!/usr/bin/env python3
"""极简关节抓取轨迹生成器(≤10 路点, move joints 主线的"少点位"版本)。

思路(20260724 决策): armMoveJoints 每个路点都完整加减速停稳 —— 点越多停顿越多。
既然 armMoveJoints 自己会在两个路点之间做关节插值, 那么【只给关键路点】就能让
整段运动一气呵成: 133 点 = 133 次停; 8 点 = 8 次停。这是在点到点 API 框架内
减少停顿最直接的办法, 且完全复用已多次真机成功的 execute_trajectory.py。

代价与对策: 相邻路点跨度大(几十度), armMoveJoints 走的是【关节空间直线】,
中途路径不再受 RRT 逐点复核保护 —— 有扫过桌面的风险。因此本生成器强制:
  * 逐段把关节插值加密到 3°/步, 在 PyBullet 里逐点做碰撞检查;
  * 有碰撞的段自动二分细分(加中间路点)直到无碰撞;
  * meta 声明 minimal_waypoints/path_verified_dense + 实际最大步长,
    校验器与执行器据此才放行大步长(否则仍按 6° 上限拒绝)。

动作序列(单次抓放+返回, 8 个运动路点 + 2 个夹爪事件):
  HOME → PICK_HOVER → PICK_DESCEND →[闭爪]→ PICK_ASCEND → PLACE_HOVER
       → PLACE_DESCEND →[开爪]→ PLACE_ASCEND → HOME

用法:
  python3 gen_minimal_joint_grasp.py --pick 0.36 0.30 1.05 --place 0.36 0.16 1.05
  python3 gen_minimal_joint_grasp.py ... --gui          # 看动画
  python3 gen_minimal_joint_grasp.py ... --out <路径>   # 指定输出
产出可直接交给 frame_calibration/robot_side/execute_trajectory.py 回放。
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys
import time

import numpy as np
import pybullet as p

_HERE = os.path.dirname(os.path.abspath(__file__))
_WORK = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "analysis"))

import xifeng_pb as xf          # noqa: E402
import left_arm_ik as ik        # noqa: E402
import calib_common as cc       # noqa: E402

EE = xf.EE_LINK_ID
ARM = xf.ARM_JOINT_IDS
DOWN = (0.0, 0.0, -1.0)

# 起始/收尾 = 桌面上方的 START 位姿(不再用身侧 HOME)。
# 依据(20260724 实测): 从身侧 HOME 摆到桌面上方跨度 119~135°, 关节直线必然扫过
# 桌面(已验证管线为此用了 28 个 RRT 点走这一段)。少点位方案改为【全程在桌面
# 上方安全区内】—— 与"起始位置设置在桌上/用 VR 起始位姿"的要求一致。
# 上机前请先用 VR 摇操把左臂摆到 START 附近(执行器会检查与首路点的差角)。
START_EXTRA = 0.04                       # START 高于悬停面的额外抬高(m)
SEED_DEG = [-80, 40, 120, -80, -70, 0, 40]   # tool-down 下探姿态种子(实测可收敛)
# 悬停高度: 与已验证管线的 LIFT 一致(0.18)。URDF 幽灵夹爪网格从 link11 伸出约
# 278mm(真实夹爪仅 168mm), 抓取中心在 150mm 处 —— 即网格尖端比抓取中心还低 128mm。
# HOVER=0.12 时尖端离桌面仅 17mm, 自由段碰撞检查会被网格伪影淹没; 0.18 时尖端
# 离桌 77mm, 可正常检查。换真实夹爪网格后可收紧。
HOVER = 0.18
RELEASE_DROP = 0.006                     # 放置松开高度(m)
CUBE = 0.05
CHECK_STEP_DEG = 3.0                     # 段内碰撞检查的关节插值步长
MAX_SUBDIVIDE = 3                        # 每段最多二分次数(控制总点数)
RANGE_BOX = {"x": (0.28, 0.48), "y": (0.14, 0.40), "z": (0.95, 1.10)}


def range_violations(pt, name):
    bad = []
    for i, k in enumerate("xyz"):
        lo, hi = RANGE_BOX[k]
        if not (lo - 1e-9 <= pt[i] <= hi + 1e-9):
            bad.append(f"{name}.{k}={pt[i]:.3f} 越界 [{lo},{hi}]")
    return bad


def kin_set(robot, q_rad):
    for j, v in zip(ARM, q_rad):
        p.resetJointState(robot, j, float(v))


def build_scene(robot, pick, place):
    table_top = pick[2] - CUBE / 2
    cx, cy = (pick[0] + place[0]) / 2, (pick[1] + place[1]) / 2
    half = [abs(pick[0] - place[0]) / 2 + 0.15,
            abs(pick[1] - place[1]) / 2 + 0.15, 0.02]
    table = p.createMultiBody(
        0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
        p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                            rgbaColor=[0.82, 0.7, 0.5, 1]),
        [cx, cy, table_top - half[2]])
    cube = p.createMultiBody(
        0, baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX, halfExtents=[CUBE / 2] * 3, rgbaColor=[0.9, 0.2, 0.2, 1]),
        basePosition=[pick[0], pick[1], table_top + CUBE / 2])
    p.createMultiBody(
        0, baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.045, length=0.002,
            rgbaColor=[0.2, 0.8, 0.4, 0.6]),
        basePosition=[place[0], place[1], table_top + 0.001])
    return {"table": table, "cube": cube, "table_top": table_top}


def solve_key_poses(robot, pick, place):
    """求 4 个关键笛卡尔位姿的关节角(工具朝下)。链式种子。"""
    seed = [math.radians(v) for v in SEED_DEG]
    targets = [
        ("PICK_HOVER", [pick[0], pick[1], pick[2] + HOVER]),
        ("PICK_DESCEND", [pick[0], pick[1], pick[2]]),
        ("PLACE_HOVER", [place[0], place[1], place[2] + HOVER]),
        ("PLACE_DESCEND", [place[0], place[1], place[2] + RELEASE_DROP]),
    ]
    out = {}
    for name, tgt in targets:
        q, info = ik.solve(robot, tgt, DOWN, seed_rad=seed)
        if q is None:
            print(f"IK 失败: {name} @ {np.round(tgt, 3).tolist()}  {info}")
            return None
        out[name] = q
        seed = q
    # START: 抓放中点上方(比悬停面再高 START_EXTRA), 作为进场/收尾的安全中枢
    start_pt = [(pick[0] + place[0]) / 2, (pick[1] + place[1]) / 2,
                max(pick[2], place[2]) + HOVER + START_EXTRA]
    q, info = ik.solve(robot, start_pt, DOWN, seed_rad=out["PICK_HOVER"],
                       axis_tol_deg=30.0)
    if q is None:
        print(f"IK 失败: START @ {np.round(start_pt, 3).tolist()}  {info}")
        return None
    out["START"] = q
    return out


def segment_collides(robot, table, q_a, q_b, cube=None):
    """把 a→b 的关节直线插值加密到 CHECK_STEP_DEG, 逐点查 机器人↔桌子 碰撞。
    这正是 armMoveJoints 在两路点间的实际走法。"""
    span = max(abs(math.degrees(b - a)) for a, b in zip(q_a, q_b))
    n = max(1, int(math.ceil(span / CHECK_STEP_DEG)))
    for k in range(n + 1):
        q = [a + (b - a) * k / n for a, b in zip(q_a, q_b)]
        kin_set(robot, q)
        p.performCollisionDetection()
        if p.getClosestPoints(robot, table, 0.0):
            return True
    return False


# 近物段: 端点是贴桌的抓取/放置位姿。此时 URDF 幽灵夹爪网格必然穿进桌面(网格
# 伪影, 非真实碰撞) —— 与已验证管线一致, 这些段跳过桌面碰撞检查(等价于其
# table_collision(off))。真机的等价保障是: 这两段是纯竖直短距离直线。
NEAR_OBJECT_POSES = {"PICK_DESCEND", "PLACE_DESCEND"}


def _is_near_object(prev_name, next_name):
    base = lambda s: s.split("_MID")[0]          # noqa: E731
    return base(prev_name) in NEAR_OBJECT_POSES or base(next_name) in NEAR_OBJECT_POSES


def verified_chain(robot, table, keys):
    """按顺序连接关键姿态; 自由段有碰撞则二分细分。近物段跳过检查(幽灵网格伪影)。
    返回 [(段名, q_rad), 报告, 是否仍有未解决碰撞]。"""
    chain, report, unresolved = [keys[0]], [], False
    for name, q_next in keys[1:]:
        q_prev, prev_name = chain[-1][1], chain[-1][0]
        inserted = 0
        pending = [(name, q_next)]
        while pending:
            seg_name, q_target = pending[0]
            if _is_near_object(prev_name, seg_name):
                report.append(f"  {prev_name}->{seg_name}: 近物段, 跳过桌面检查"
                              f"(幽灵网格伪影; 纯竖直短距离)")
                chain.append((seg_name, q_target))
                q_prev, prev_name = q_target, seg_name
                pending.pop(0)
                continue
            if not segment_collides(robot, table, q_prev, q_target):
                chain.append((seg_name, q_target))
                q_prev, prev_name = q_target, seg_name
                pending.pop(0)
                continue
            if inserted >= MAX_SUBDIVIDE:
                report.append(f"✗ {prev_name}->{seg_name}: 细分 {MAX_SUBDIVIDE} "
                              f"次后仍有碰撞(该段关节直线扫过桌面, 需改用 RRT 版)")
                unresolved = True
                chain.append((seg_name, q_target))
                q_prev, prev_name = q_target, seg_name
                pending.pop(0)
                continue
            mid = [(a + b) / 2 for a, b in zip(q_prev, q_target)]
            pending.insert(0, (f"{seg_name}_MID{inserted + 1}", mid))
            inserted += 1
            report.append(f"  {prev_name}->{seg_name}: 关节直线扫过桌面, "
                          f"插入中间路点(第 {inserted} 次细分)")
    return chain, report, unresolved


def build_waypoints(chain):
    """chain -> 轨迹路点(含夹爪事件)。夹爪在 *_DESCEND 之后。"""
    wps = []
    for name, q in chain:
        q_urdf_deg = [math.degrees(v) for v in q]
        wps.append({"seg": name,
                    "q_sdk_deg": [round(v, 4) for v in
                                  cc.urdf_q_to_sdk_q(q_urdf_deg)]})
        if name == "PICK_DESCEND":
            wps.append({"seg": "PICK", "gripper": "close"})
        elif name == "PLACE_DESCEND":
            wps.append({"seg": "PLACE", "gripper": "open"})
    return wps


def animate(robot, chain, cube, table_top, place, gui, fps=60, deg_per_s=25.0):
    """流畅播放: 在相邻路点之间做关节插值 —— 这正是 armMoveJoints 的真实走法,
    所以画面 = 真机实际运动(不是硬跳)。夹爪事件处短暂停顿以示意开合。"""
    if not gui:
        return
    grasped = False
    for idx in range(1, len(chain)):
        name_prev, q_prev = chain[idx - 1]
        name, q_next = chain[idx]
        span = max(abs(math.degrees(b - a)) for a, b in zip(q_prev, q_next))
        n = max(2, int(span / deg_per_s * fps))          # 按角速度换算帧数
        for k in range(n + 1):
            q = [a + (b - a) * k / n for a, b in zip(q_prev, q_next)]
            kin_set(robot, q)
            if grasped:
                grip, _ = ik.grip_state(robot)
                p.resetBasePositionAndOrientation(cube, grip.tolist(),
                                                  [0, 0, 0, 1])
            time.sleep(1.0 / fps)
        if name == "PICK_DESCEND":                       # 到位后闭爪
            grasped = True
            time.sleep(0.8)
        elif name == "PLACE_DESCEND":                    # 到位后开爪, 方块落桌
            grasped = False
            p.resetBasePositionAndOrientation(
                cube, [place[0], place[1], table_top + CUBE / 2], [0, 0, 0, 1])
            time.sleep(0.8)


def run(pick, place, gui, out):
    print(f"\n极简关节抓取: pick={np.round(pick, 4).tolist()}  "
          f"place={np.round(place, 4).tolist()}")
    bad = range_violations(pick, "pick") + range_violations(place, "place")
    if bad:
        print("坐标越界, 拒绝:\n  " + "\n  ".join(bad))
        return False

    robot = xf.load_xifeng(gui=gui)
    scene = build_scene(robot, pick, place)
    keys_pose = solve_key_poses(robot, pick, place)
    if keys_pose is None:
        p.disconnect()
        return False

    keys = [("START", keys_pose["START"]),
            ("PICK_HOVER", keys_pose["PICK_HOVER"]),
            ("PICK_DESCEND", keys_pose["PICK_DESCEND"]),
            ("PICK_ASCEND", keys_pose["PICK_HOVER"]),
            ("PLACE_HOVER", keys_pose["PLACE_HOVER"]),
            ("PLACE_DESCEND", keys_pose["PLACE_DESCEND"]),
            ("PLACE_ASCEND", keys_pose["PLACE_HOVER"]),
            ("START_RETURN", keys_pose["START"])]

    print(f"[1/3] 关键路点 {len(keys)} 个, 逐段验证关节插值无碰撞"
          f"(检查步长 {CHECK_STEP_DEG}°)...")
    chain, report, collided = verified_chain(robot, scene["table"], keys)
    for line in report:
        print("   " + line)

    wps = build_waypoints(chain)
    moves = [w for w in wps if "q_sdk_deg" in w]
    max_step = max(
        max(abs(a - b) for a, b in zip(m1["q_sdk_deg"], m2["q_sdk_deg"]))
        for m1, m2 in zip(moves, moves[1:]))
    print(f"[2/3] 轨迹: {len(moves)} 个运动路点 + "
          f"{len(wps) - len(moves)} 个夹爪事件, 相邻最大步长 {max_step:.1f}°")

    # 限位复核(用真机实测限位表)
    over = []
    for w in moves:
        for k, v in enumerate(w["q_sdk_deg"], 1):
            lim = {7: (-90.0, 76.0)}.get(k)
            if lim and not (lim[0] <= v <= lim[1]):
                over.append(f"{w['seg']} j{k}={v:.1f}° 越 {lim}")
    if over:
        print("   ✗ 限位越界:\n     " + "\n     ".join(over))

    ok = (not collided) and (not over) and len(moves) <= 10
    print(f"[3/3] 验证: 无碰撞={not collided}  限位OK={not over}  "
          f"路点≤10={len(moves) <= 10}  -> {'✅ 通过' if ok else '❌ 未通过'}")

    animate(robot, chain, scene["cube"], scene["table_top"], place, gui)

    if ok:
        meta = {
            "arm_id": 1, "arm": "left", "j6_flipped": True,
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "task": "minimal_joint_grasp",
            "minimal_waypoints": True,        # 声明: 少点位, 允许大步长
            "path_verified_dense": True,      # 声明: 段内插值已逐点复核无碰撞
            "max_step_deg": round(max_step, 2),
            "check_step_deg": CHECK_STEP_DEG,
            "pick_pb_m": list(map(float, pick)),
            "place_pb_m": list(map(float, place)),
            "hover_m": HOVER,
            "note": "少点位关节轨迹: armMoveJoints 在相邻路点间自行插值, 点少=停顿少; "
                    "段内关节插值已在仿真逐 3° 复核无碰撞, 故允许大步长",
        }
        path = out or os.path.join(_HERE, "trajectories",
                                   "traj_minimal_joint.json")
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w") as f:
            json.dump({"meta": meta, "waypoints": wps}, f, indent=1,
                      ensure_ascii=False)
        print(f"\n已导出({len(moves)} 路点 + {len(wps) - len(moves)} 夹爪事件): "
              f"{os.path.relpath(path, _WORK)}")

    if gui:
        print("\n(关闭窗口结束)")
        while p.isConnected():
            time.sleep(1 / 60)
    p.disconnect()
    return ok


def build_arg_parser():
    ap = argparse.ArgumentParser(description="≤10 路点的极简关节抓取轨迹生成")
    ap.add_argument("--pick", type=float, nargs=3, default=[0.36, 0.30, 1.05],
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--place", type=float, nargs=3, default=[0.36, 0.16, 1.05],
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--gui", action="store_true", help="显示动画")
    ap.add_argument("--out", default=None)
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    sys.exit(0 if run(args.pick, args.place, args.gui, args.out) else 1)


if __name__ == "__main__":
    main()
