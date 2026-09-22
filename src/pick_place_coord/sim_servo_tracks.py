#!/usr/bin/env python3
"""Mac 端仿真: 两条伺服透传线的对照演示 —— 上真机之前先在这里看清楚。

  pulse   关节透传 armPluseToServo  —— 喂 7 个关节角, 路径由我们自己插
  worlds  末端透传 armWorldsToServo —— 喂 [X,Y,Z,U,V,W], 逆解由控制器做

两条线跑【同一组抓放坐标】, 输出:
  * 终端: 透传点数 / 时长 / 步长 / 末端轨迹长度 / 逐段几何 / 关节跳变
  * GUI(--gui): PyBullet 实时动画, 按真实节拍播放, 能直接看出连不连贯
  * preview/SERVO_*.png: 侧视剖面图

worlds 线独有的复核 —— 【逆解分支连续性】:
  关节透传的路径是我们自己插的, 相邻点天然连续; 末端透传由控制器每帧独立逆解,
  相邻帧解到不同分支时关节角会瞬间跳几十度, 而那时保护是关的。真机侧用
  armTryWorlds(手册 2.7.53)逐点预检, Mac 侧这里用我们自己的 IK 做同样的事,
  提前把"这条路径会不会跳"看出来。

用法:
  python3 sim_servo_tracks.py                  # 无界面, 出数据和图
  python3 sim_servo_tracks.py --gui            # 实时动画(两条线依次播放)
  python3 sim_servo_tracks.py --gui --track worlds
  python3 sim_servo_tracks.py --speed 4        # 动画放快 4 倍
"""
from __future__ import annotations

import argparse
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
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "robot_side"))

import xifeng_pb as xf          # noqa: E402
import left_arm_ik as ik        # noqa: E402
import calib_common as cc       # noqa: E402

OUT_DIR = os.path.join(_HERE, "preview")
W, H = 820, 580
CUBE = 0.05


def _stub_pypilot():
    """真机模块 import pypilot; Mac 上没有, 塞个壳让参数能被读到。"""
    import types
    if "pypilot" not in sys.modules:
        m = types.ModuleType("pypilot")
        m.FloatVector = list
        m.DoubleVector = list

        class LP:
            quality = 0.0
            center_mass_x = center_mass_y = center_mass_z = 0.0
        m.LoadParameter = LP
        sys.modules["pypilot"] = m


def shot(name, target):
    from PIL import Image
    view = p.computeViewMatrixFromYawPitchRoll(target, 0.95, 90, -8, 0, 2)
    proj = p.computeProjectionMatrixFOV(55, W / H, 0.1, 4)
    img = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER)[2]
    os.makedirs(OUT_DIR, exist_ok=True)
    Image.fromarray(np.reshape(img, (H, W, 4))[:, :, :3].astype(np.uint8)).save(
        os.path.join(OUT_DIR, f"{name}.png"))


def draw_path(verts, rgba, every=4, radius=0.005):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgba)
    for v in verts[::every]:
        p.createMultiBody(0, baseVisualShapeIndex=vis, basePosition=v)


# ---------------------------------------------------------------- pulse 线
def build_pulse(traj_path):
    """关节透传: 读关节轨迹 -> 按 STEP_DEG 加密 -> 返回密集关节点流 + 夹爪事件位置。"""
    _stub_pypilot()
    import json
    import execute_servo_grasp as sv
    d = json.load(open(traj_path, encoding="utf-8"))
    wps = d["waypoints"]
    stream, grips, prev = [], [], None
    for w in wps:
        if "gripper" in w:
            grips.append((len(stream), w["gripper"]))
            continue
        q = w["q_sdk_deg"]
        if prev is None:
            stream.append(q)
        else:
            stream += sv.densify(prev, q)
        prev = q
    return {"name": "pulse · 关节透传 armPluseToServo",
            "stream_sdk": stream, "grips": grips,
            "step": f"{sv.STEP_DEG}°", "period": sv.PERIOD_S,
            "speed": f"{sv.STEP_DEG/sv.PERIOD_S:.0f} °/s"}


# ---------------------------------------------------------------- worlds 线
def build_worlds(traj_path, robot, pick):
    """末端透传: 读世界轨迹 -> 按 STEP_MM 加密 -> 【逐点 IK】-> 密集关节点流。

    这里的 IK 就是真机侧 armTryWorlds 的 Mac 对应物: 我们要看的不是"末端能不能
    到"(那当然能, 是我们画的直线), 而是"逐点逆解出来的关节角连不连续"。"""
    _stub_pypilot()
    import json
    import execute_worlds_servo_grasp as wsv
    d = json.load(open(traj_path, encoding="utf-8"))
    wps = d["waypoints"]
    # anchor 用仿真里的抓取点(与真机"把夹爪放到抓取点读 anchor"等价)
    anchor_mm = list(np.array(pick) * 1000.0) + [0.0, 0.0, 0.0]
    corners = wsv.corners_of(wps)
    poses, grips, prev = [], [], None
    for c in corners:
        if "gripper" in c:
            grips.append((len(poses), c["gripper"]))
            continue
        pose = [anchor_mm[i] + c["off_mm"][i] for i in range(3)] + [0, 0, 0]
        if prev is None:
            poses.append(pose)
        else:
            poses += wsv.densify_pose(prev, pose, wsv.STEP_MM, wsv.STEP_ORI_DEG)
        prev = pose
    # 逐点 IK(链式种子, 与真机控制器逐帧逆解对应)
    import world_grasp as wg
    seed = [math.radians(v) for v in wg.SEED_DEG]
    stream, bad, jumps = [], [], []
    prev_q = None
    for i, pose in enumerate(poses):
        pos_m = [v / 1000.0 for v in pose[:3]]
        q, info = ik.solve(robot, pos_m, wg.DOWN, seed_rad=seed)
        if q is None:
            bad.append((i, pos_m, info))
            continue
        seed = q
        if prev_q is not None:
            jumps.append(max(abs(math.degrees(a - b)) for a, b in zip(q, prev_q)))
        prev_q = q
        stream.append(q)
    return {"name": "worlds · 末端透传 armWorldsToServo",
            "stream_rad": stream, "grips": grips, "bad": bad, "jumps": jumps,
            "step": f"{wsv.STEP_MM}mm", "period": wsv.PERIOD_S,
            "speed": f"{wsv.STEP_MM/wsv.PERIOD_S:.0f} mm/s",
            "abort_deg": wsv.IK_JUMP_ABORT_DEG}


# ---------------------------------------------------------------- 播放/统计
def to_rad_stream(info):
    if "stream_rad" in info:
        return info["stream_rad"]
    return [[math.radians(v) for v in cc.sdk_q_to_urdf_q(q)]
            for q in info["stream_sdk"]]


def play(robot, qs, grips, cube, table_top, gui, period, speed):
    """按真实节拍播放, 顺便采末端轨迹。GUI 下能直接看出连不连贯。"""
    verts, grasped, gi = [], False, 0
    dt = period / max(0.01, speed)
    for i, q in enumerate(qs):
        while gi < len(grips) and grips[gi][0] <= i:
            grasped = grips[gi][1] == "close"
            if not grasped:
                g, _ = ik.grip_state(robot)
                p.resetBasePositionAndOrientation(
                    cube, [g[0], g[1], table_top + CUBE / 2], [0, 0, 0, 1])
            gi += 1
        for j, v in zip(ik.ARM, q):
            p.resetJointState(robot, j, float(v))
        g, _ = ik.grip_state(robot)
        verts.append(g.tolist())
        if grasped:
            p.resetBasePositionAndOrientation(cube, g.tolist(), [0, 0, 0, 1])
        if gui:
            time.sleep(dt)
    return verts


def report(info, qs, verts):
    n = len(qs)
    length = sum(np.linalg.norm(np.array(b) - np.array(a))
                 for a, b in zip(verts, verts[1:])) * 1000
    dj = [max(abs(math.degrees(a - b)) for a, b in zip(x, y))
          for x, y in zip(qs, qs[1:])] or [0.0]
    print(f"\n【{info['name']}】")
    print(f"  透传点 {n} 个 × {info['period']*1000:.0f}ms = {n*info['period']:.1f}s"
          f"   步长 {info['step']}  ->  {info['speed']}")
    print(f"  末端轨迹长度 {length:.0f}mm   相邻关节跳变 最大 {max(dj):.3f}° / "
          f"平均 {sum(dj)/len(dj):.3f}°")
    if "jumps" in info:
        j = info["jumps"] or [0.0]
        lim = info["abort_deg"]
        ok = max(j) <= lim
        print(f"  [逆解连续性] 逐点 IK 相邻跳变 最大 {max(j):.3f}° "
              f"(真机门限 {lim}°)  {'✓ 无分支跳变' if ok else '✗ 会被真机拒绝'}")
        if info["bad"]:
            print(f"  ✗ {len(info['bad'])} 个点 IK 不可达, 真机 armTryWorlds 会拒")
    return {"name": info["name"], "n": n, "sec": n * info["period"],
            "len": length, "max_dj": max(dj)}


def run_track(kind, traj, pick, place, gui, speed):
    robot = xf.load_xifeng(gui=gui)
    import world_grasp as wg
    scene = wg.build_scene(robot, pick, place)
    if kind == "pulse":
        info = build_pulse(traj["pulse"])
    else:
        info = build_worlds(traj["worlds"], robot, pick)
    qs = to_rad_stream(info)
    if not qs:
        print(f"✗ {kind}: 没有可播放的点")
        p.disconnect()
        return None
    verts = play(robot, qs, info["grips"], scene["cube"], scene["table_top"],
                 gui, info["period"], speed)
    draw_path(verts, [0.95, 0.55, 0.1, 1] if kind == "pulse"
              else [0.2, 0.55, 0.95, 1])
    shot(f"SERVO_{kind}", [(pick[0] + place[0]) / 2, (pick[1] + place[1]) / 2,
                           pick[2] + 0.05])
    r = report(info, qs, verts)
    cpos = np.array(p.getBasePositionAndOrientation(scene["cube"])[0])
    want = np.array([place[0], place[1], scene["table_top"] + CUBE / 2])
    err = np.linalg.norm(cpos - want) * 1000
    print(f"  方块落点偏差 {err:.1f}mm  {'✓' if err < 30 else '✗'}")
    r["place_err"] = err
    if gui:
        print("  (关闭窗口继续)")
        while p.isConnected():
            time.sleep(1 / 60)
    p.disconnect()
    return r


def main():
    ap = argparse.ArgumentParser(description="两条伺服透传线的 Mac 仿真对照")
    ap.add_argument("--pick", type=float, nargs=3, default=[0.36, 0.30, 1.05])
    ap.add_argument("--place", type=float, nargs=3, default=[0.36, 0.16, 1.05])
    ap.add_argument("--track", choices=["pulse", "worlds", "all"], default="all")
    ap.add_argument("--gui", action="store_true", help="实时动画")
    ap.add_argument("--speed", type=float, default=1.0, help="动画倍速")
    a = ap.parse_args()
    traj = {"pulse": os.path.join(_HERE, "trajectories", "traj_minimal_joint.json"),
            "worlds": os.path.join(_HERE, "trajectories", "world_grasp_latest.json")}
    for k, v in traj.items():
        if not os.path.exists(v):
            print(f"缺少 {k} 轨迹: {v}")
            return
    rows = []
    for k in (["pulse", "worlds"] if a.track == "all" else [a.track]):
        rows.append(run_track(k, traj, a.pick, a.place, a.gui, a.speed))
    rows = [r for r in rows if r]
    if len(rows) > 1:
        print("\n" + "=" * 72)
        print(f"{'线路':42s} {'点数':>5s} {'时长s':>6s} {'长度mm':>7s} {'跳变°':>7s}")
        for r in rows:
            print(f"{r['name']:42s} {r['n']:>5d} {r['sec']:>6.1f} "
                  f"{r['len']:>7.0f} {r['max_dj']:>7.3f}")
    print(f"\n图片: {os.path.relpath(OUT_DIR, _WORK)}/SERVO_*.png")


if __name__ == "__main__":
    main()
