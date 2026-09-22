#!/usr/bin/env python3
"""三条主线的 Mac 端仿真演示 —— 上真机之前先在这里看清楚。

  Track A  世界坐标直线抓取   armMoveWorlds, 6 条直线拐角(弧线实测不执行, 已弃用)
  Track B  8 路点关节抓取     armMoveJoints 逐点停稳(真机三次成功)
  Track C  伺服透传抓取       armPluseToServo 密集流式, 全程无停顿(保底)

三条线用【同一组抓放坐标】, 输出:
  * 终端: 每条线的命令数/停顿数/末端轨迹长度/最大转角 对比表
  * preview/TRACK_*.png: 侧视剖面图(能直接看出轨迹形状)
  * 回答"我发过去的起点和目标点是否达到": 逐条打印起点/抓取点/放置点的
    仿真落点误差

用法:
  python3 demo_three_tracks.py                      # 全部三条
  python3 demo_three_tracks.py --track A            # 只看一条
  python3 demo_three_tracks.py --gui --track B      # 实时动画
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import pybullet as p

_HERE = os.path.dirname(os.path.abspath(__file__))
_WORK = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "analysis"))
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "robot_side"))

import xifeng_pb as xf          # noqa: E402
import left_arm_ik as ik        # noqa: E402
import calib_common as cc       # noqa: E402
import pick_place_coord as ppc  # noqa: E402

OUT_DIR = os.path.join(_HERE, "preview")
W, H = 780, 560


def _stub_pypilot():
    import types
    if "pypilot" not in sys.modules:
        m = types.ModuleType("pypilot")
        m.FloatVector = list
        sys.modules["pypilot"] = m


def shot(name, target):
    from PIL import Image
    view = p.computeViewMatrixFromYawPitchRoll(target, 0.95, 90, -8, 0, 2)
    proj = p.computeProjectionMatrixFOV(55, W / H, 0.1, 4)
    img = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER)[2]
    os.makedirs(OUT_DIR, exist_ok=True)
    Image.fromarray(np.reshape(img, (H, W, 4))[:, :, :3].astype(np.uint8)).save(
        os.path.join(OUT_DIR, f"{name}.png"))


def draw_path(verts, rgba, every=1, radius=0.006):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgba)
    for v in verts[::every]:
        p.createMultiBody(0, baseVisualShapeIndex=vis, basePosition=v)


def turns_of(verts, every=1):
    """采样转角。注意: 抓取/放置点的"下探后立刻上升"是 180° 折返, 但那里本来
    就必须停(要夹要放), 不代表轨迹不连贯 —— 看平均值比看最大值有意义。"""
    v = verts[::every]
    out = []
    for a, b, c in zip(v, v[1:], v[2:]):
        u, w = np.array(b) - np.array(a), np.array(c) - np.array(b)
        if np.linalg.norm(u) < 1e-9 or np.linalg.norm(w) < 1e-9:
            continue
        out.append(math.degrees(math.acos(np.clip(
            u.dot(w) / (np.linalg.norm(u) * np.linalg.norm(w)), -1, 1))))
    return out


def ee_path_from_joints(robot, qs, step_deg=2.0):
    """关节序列 -> 末端路径(按 armMoveJoints 的关节插值展开)。"""
    verts = []
    for a, b in zip(qs, qs[1:]):
        span = max(abs(math.degrees(y - x)) for x, y in zip(a, b))
        n = max(2, int(span / step_deg))
        for k in range(n + 1):
            q = [x + (y - x) * k / n for x, y in zip(a, b)]
            for j, v in zip(ik.ARM, q):
                p.resetJointState(robot, j, float(v))
            g, _ = ik.grip_state(robot)
            verts.append(g.tolist())
    return verts


def report(name, cmds, stops, verts, targets, extra=""):
    length = sum(np.linalg.norm(np.array(b) - np.array(a))
                 for a, b in zip(verts, verts[1:])) * 1000
    t = turns_of(verts, every=6)
    print(f"\n【{name}】{extra}")
    print(f"  真机命令数: {cmds}    停顿次数: {stops}")
    print(f"  末端轨迹长度: {length:.0f} mm   采样转角 最大 "
          f"{max(t) if t else 0:.1f}° / 平均 {sum(t)/len(t) if t else 0:.1f}°")
    for label, want, got in targets:
        err = np.linalg.norm(np.array(want) - np.array(got)) * 1000
        print(f"  {label}: 目标 {np.round(want,3).tolist()} 实到 "
              f"{np.round(got,3).tolist()}  误差 {err:.1f}mm "
              f"{'✓' if err < 5 else '⚠'}")
    return {"name": name, "cmds": cmds, "stops": stops, "len": length,
            "max_turn": max(t) if t else 0}


def demo_a(pick, place, gui):
    import world_grasp as wg
    robot = xf.load_xifeng(gui=gui)
    scene = wg.build_scene(robot, pick, place)
    steps = wg.build_world_recipe(pick, place)
    seq, bad = wg.solve_all(robot, wg.densify_cartesian(steps))
    seq = wg.expand_joint_transfer(robot, seq)     # 转运段按关节插值还原真机形状
    qs = [s["q"] for s in seq if s.get("q") is not None]
    verts = []
    for q in qs:
        for j, v in zip(ik.ARM, q):
            p.resetJointState(robot, j, float(v))
        g, _ = ik.grip_state(robot)
        verts.append(g.tolist())
    # 真机命令数 = 拐角数(每段直线/关节段各压缩成 1 条命令)
    corners = [s for s in steps if "pos" in s]
    n_arc = sum(1 for s in steps if s["seg"].startswith("ARC"))
    cmds = (len(corners) - n_arc) + (1 if n_arc else 0)
    n_joint = sum(1 for s in steps if s.get("via") == "joints")
    probe = wg.joint_transfer_probe(seq, scene["table_top"])
    draw_path(verts, [0.1, 0.9, 0.2, 1])
    shot("TRACK_A_line",[(pick[0]+place[0])/2, (pick[1]+place[1])/2, pick[2]+0.05])
    # 落点核对: 取【夹爪事件发生时】的末端位置(闭爪=抓取点, 开爪=放置点)。
    # 必须遍历 seq(展开关节转运之后的那份)—— verts 就是从它算出来的; 早先遍历
    # 重新生成的 dense 会与 verts 错位(关节段点数不同), 把放置点误报成差 60mm。
    gpick = gplace = None
    vi = 0
    for s in seq:
        if "gripper" in s:
            snap = verts[vi - 1] if vi > 0 else verts[0]
            if s["gripper"] == "close":
                gpick = snap
            else:
                gplace = snap
        elif s.get("q") is not None:
            vi += 1
    extra = f"— IK 不可达 {len(bad)}"
    if probe:
        extra += (f", 关节转运段净空 {probe['clear_mm']:.0f}mm / "
                  f"鼓起 {probe['bow_mm']:.0f}mm")
    r = report(f"Track A · {cmds - n_joint} 条 worlds 直线 + {n_joint} 条 joints 转运",
               cmds, cmds, verts,
               [("抓取点", pick, gpick or verts[0]),
                ("放置点", [place[0], place[1], place[2] + wg.RELEASE_DROP],
                 gplace or verts[-1])],
               extra=extra)
    p.disconnect()
    return r


def demo_b(pick, place, gui, traj="traj_5pt.json"):
    _stub_pypilot()
    import execute_trajectory as ex
    import json
    path = os.path.join(_HERE, "trajectories", traj)
    if not os.path.exists(path):
        print(f"缺少 {traj}, 先跑 gen_minimal_joint_grasp.py")
        return None
    d = json.load(open(path, encoding="utf-8"))
    wps = d["waypoints"]
    lim = d["meta"].get("handoff_turn_limit_deg", 20.0)
    old = ex.CONTINUOUS_MAX_TURN_DEG
    ex.CONTINUOUS_MAX_TURN_DEG = lim
    stops = sum(1 for i, w in enumerate(wps) if "q_sdk_deg" in w
                and ex.waypoint_requires_full_settle(wps, i))
    ex.CONTINUOUS_MAX_TURN_DEG = old
    qs = [[math.radians(v) for v in cc.sdk_q_to_urdf_q(w["q_sdk_deg"])]
          for w in wps if "q_sdk_deg" in w]
    robot = xf.load_xifeng(gui=gui)
    ppc.build_scene(robot, pick, place)
    verts = ee_path_from_joints(robot, qs)
    draw_path(verts, [0.2, 0.45, 0.95, 1], every=3)
    shot("TRACK_B_joint", [(pick[0]+place[0])/2, (pick[1]+place[1])/2,
                              pick[2]+0.05])
    # 抓取点/放置点落点(夹爪事件前一个运动路点)
    mi, gpick, gplace = 0, None, None
    for w in wps:
        if "q_sdk_deg" in w:
            mi += 1
        elif w.get("gripper") == "close":
            q = qs[mi - 1]
            for j, v in zip(ik.ARM, q):
                p.resetJointState(robot, j, float(v))
            gpick = ik.grip_state(robot)[0].tolist()
        elif w.get("gripper") == "open":
            q = qs[mi - 1]
            for j, v in zip(ik.ARM, q):
                p.resetJointState(robot, j, float(v))
            gplace = ik.grip_state(robot)[0].tolist()
    r = report(f"Track B · 关节路点(armMoveJoints) {traj}",
               len(qs), stops, verts,
               [("抓取点", pick, gpick),
                ("放置点", [place[0], place[1], place[2] + ppc.RELEASE_DROP],
                 gplace)])
    p.disconnect()
    return r


def demo_c(pick, place, gui):
    _stub_pypilot()
    import execute_servo_grasp as sv
    import json
    path = os.path.join(_HERE, "trajectories", "traj_minimal_joint.json")
    if not os.path.exists(path):
        print("缺少 traj_minimal_joint.json")
        return None
    d = json.load(open(path, encoding="utf-8"))
    wps = d["waypoints"]
    moves = [w["q_sdk_deg"] for w in wps if "q_sdk_deg" in w]
    dense = [moves[0]]
    for a, b in zip(moves, moves[1:]):
        dense += sv.densify(a, b)
    qs = [[math.radians(v) for v in cc.sdk_q_to_urdf_q(q)] for q in dense]
    robot = xf.load_xifeng(gui=gui)
    ppc.build_scene(robot, pick, place)
    verts = []
    for q in qs:
        for j, v in zip(ik.ARM, q):
            p.resetJointState(robot, j, float(v))
        verts.append(ik.grip_state(robot)[0].tolist())
    draw_path(verts, [0.95, 0.55, 0.1, 1], every=6)
    shot("TRACK_C_servo", [(pick[0]+place[0])/2, (pick[1]+place[1])/2,
                           pick[2]+0.05])
    n_grip = sum(1 for w in wps if "gripper" in w)
    r = report("Track C · 伺服透传(armPluseToServo)", len(dense), n_grip,
               verts, [], extra=f"— 步长 {sv.STEP_DEG}° / 周期 "
               f"{sv.PERIOD_S*1000:.0f}ms, 预计 {len(dense)*sv.PERIOD_S:.0f}s")
    p.disconnect()
    return r


def main():
    ap = argparse.ArgumentParser(description="三条主线仿真演示")
    ap.add_argument("--pick", type=float, nargs=3, default=[0.36, 0.30, 1.05])
    ap.add_argument("--place", type=float, nargs=3, default=[0.36, 0.16, 1.05])
    ap.add_argument("--track", choices=["A", "B", "C", "all"], default="all")
    ap.add_argument("--gui", action="store_true")
    ap.add_argument("--traj-b", default="traj_5pt.json",
                    help="Track B 用哪份关节轨迹(trajectories/ 下的文件名)")
    a = ap.parse_args()
    rows = []
    if a.track in ("A", "all"):
        rows.append(demo_a(a.pick, a.place, a.gui))
    if a.track in ("B", "all"):
        rows.append(demo_b(a.pick, a.place, a.gui, a.traj_b))
    if a.track in ("C", "all"):
        rows.append(demo_c(a.pick, a.place, a.gui))
    rows = [r for r in rows if r]
    if len(rows) > 1:
        print("\n" + "=" * 68)
        print(f"{'线路':34s} {'命令数':>6s} {'停顿':>5s} {'长度mm':>8s} {'最大转角':>8s}")
        for r in rows:
            print(f"{r['name']:34s} {r['cmds']:>6d} {r['stops']:>5d} "
                  f"{r['len']:>8.0f} {r['max_turn']:>7.1f}°")
    print(f"\n图片: {os.path.relpath(OUT_DIR, _WORK)}/TRACK_*.png")


if __name__ == "__main__":
    main()
