#!/usr/bin/env python3
"""两条主线的可视化预览: 离屏渲染关键帧 + 轨迹几何/平滑性报告。

用途: 上真机之前, 在 Mac 上把两条线【看清楚】。
  Track A (world_grasp)          世界坐标弧形抓取 —— 平滑连贯为目标
  Track B (gen_minimal_joint)    ≤10 路点关节抓取 —— 复用已验证执行器

产出:
  preview/A_*.png / B_*.png   关键帧图(可直接看)
  终端报告: 每个拐角的转角、是否落入混合门限(=能否真正连续过弯)、末端轨迹长度

用法:
  python3 preview_tracks.py            # 两条线都渲染
  python3 preview_tracks.py --track A  # 只看某一条
GUI 实时动画请直接跑 world_grasp.py / gen_minimal_joint_grasp.py --gui
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

import xifeng_pb as xf          # noqa: E402
import left_arm_ik as ik        # noqa: E402

OUT_DIR = os.path.join(_HERE, "preview")
W, H = 760, 560
BLEND_MAX_TURN_DEG = 60.0       # 与执行器 execute_world_grasp 保持一致


def _cam(target):
    return (p.computeViewMatrixFromYawPitchRoll(target, 1.45, 55, -20, 0, 2),
            p.computeProjectionMatrixFOV(48, W / H, 0.1, 4))


def draw_path(verts, rgba, every=3, radius=0.005):
    """把末端轨迹画成一串小球(真实可见物体)。
    注意: p.addUserDebugLine 在 ER_TINY_RENDERER 离屏渲染里【不显示】,
    所以用球体, 才能在导出的 PNG 里看到轨迹形状。"""
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=rgba)
    for v in verts[::every]:
        p.createMultiBody(0, baseVisualShapeIndex=vis, basePosition=v)


def shot(view, proj, name):
    from PIL import Image
    img = p.getCameraImage(W, H, view, proj, renderer=p.ER_TINY_RENDERER)[2]
    arr = np.reshape(img, (H, W, 4))[:, :, :3].astype(np.uint8)
    os.makedirs(OUT_DIR, exist_ok=True)
    Image.fromarray(arr).save(os.path.join(OUT_DIR, f"{name}.png"))
    return name


def turn_report(groups, label):
    """按【连续运动组】统计转角。夹爪开合处本来就必须停稳(要夹要放), 那里的
    180° 折返不算问题, 所以分组统计才反映真实的连贯性。
    groups: [[点...], [点...]] 每组是一段不含夹爪事件的连续运动。"""
    all_turns, worst = [], 0.0
    for gi, pts in enumerate(groups, start=1):
        turns = []
        for a, b, c in zip(pts, pts[1:], pts[2:]):
            u = np.array(b[:3]) - np.array(a[:3])
            v = np.array(c[:3]) - np.array(b[:3])
            if np.linalg.norm(u) < 1e-9 or np.linalg.norm(v) < 1e-9:
                continue
            cos = np.clip(u.dot(v) / (np.linalg.norm(u) * np.linalg.norm(v)), -1, 1)
            turns.append(math.degrees(math.acos(cos)))
        if not turns:
            print(f"  {label} 组{gi}: 单段直线, 无中间拐角(天然平滑)")
            continue
        ok = sum(1 for t in turns if t <= BLEND_MAX_TURN_DEG)
        worst = max(worst, max(turns))
        all_turns += turns
        print(f"  {label} 组{gi}: {len(turns)} 拐角, 最大 {max(turns):5.1f}° / "
              f"平均 {sum(turns)/len(turns):5.1f}° -> 可混合 {ok}/{len(turns)} "
              f"{'✅ 全程连续过弯' if ok == len(turns) else '⚠ 含尖角需停稳'}")
    if all_turns:
        ok = sum(1 for t in all_turns if t <= BLEND_MAX_TURN_DEG)
        print(f"  {label} 合计: 组内拐角 {ok}/{len(all_turns)} 可混合, "
              f"最尖 {worst:.1f}°")


def preview_a(pick, place):
    import world_grasp as wg
    print("\n=== Track A: 世界坐标弧形抓取(move worlds) ===")
    steps = wg.build_world_recipe(pick, place)
    corners = [s["pos"] for s in steps if "pos" in s]
    print(f"  角点(=真机每条 armMoveWorlds 命令): {len(corners)} 个")
    groups, cur = [], []                 # 按夹爪事件切成连续运动组
    for s in steps:
        if "gripper" in s:
            groups.append(cur)
            cur = []
        else:
            cur.append(s["pos"])
    groups.append(cur)
    turn_report([g for g in groups if g], "轨迹")

    dense = wg.densify_cartesian(steps)
    robot = xf.load_xifeng(gui=False)
    scene = wg.build_scene(robot, pick, place)
    seq, bad = wg.solve_all(robot, dense)
    print(f"  IK: {sum(1 for s in seq if s.get('q') is not None)} 点可达, "
          f"不可达 {len(bad)}")
    verts = []
    for s in seq:
        if s.get("q") is None:
            continue
        for j, v in zip(ik.ARM, s["q"]):
            p.resetJointState(robot, j, float(v))
        g, _ = ik.grip_state(robot)
        verts.append(g.tolist())
    length = sum(np.linalg.norm(np.array(b) - np.array(a))
                 for a, b in zip(verts, verts[1:]))
    print(f"  末端实际轨迹长度: {length*1000:.0f} mm")

    view, proj = _cam([(pick[0]+place[0])/2, (pick[1]+place[1])/2, pick[2]-0.03])
    draw_path(verts, [0.1, 0.85, 0.25, 1.0])      # 绿色小球串 = 末端弧形轨迹
    grasped = False
    names = []
    marks = {"START": "A1_hover", "PICK_DESCEND": "A2_at_object"}
    arc_shots = {f"ARC{k:02d}": f"A{i+3}_arc{k:02d}"
                 for i, k in enumerate((3, 6, 9, 12))}
    marks.update(arc_shots)
    marks["PLACE_ASCEND"] = "A7_done"
    last_of = {}
    for s in seq:
        if s.get("q") is not None:
            last_of[s["seg"]] = s["q"]
    for seg, fname in marks.items():
        if seg not in last_of:
            continue
        for j, v in zip(ik.ARM, last_of[seg]):
            p.resetJointState(robot, j, float(v))
        if seg == "PICK_DESCEND":
            grasped = True
        if seg == "PLACE_ASCEND":
            grasped = False
            p.resetBasePositionAndOrientation(
                scene["cube"], [place[0], place[1],
                                scene["table_top"] + wg.CUBE / 2], [0, 0, 0, 1])
        if grasped:
            g, _ = ik.grip_state(robot)
            p.resetBasePositionAndOrientation(scene["cube"], g.tolist(),
                                              [0, 0, 0, 1])
        names.append(shot(view, proj, fname))
    p.disconnect()
    print(f"  已渲染: {', '.join(names)}")


def preview_b(pick, place):
    import gen_minimal_joint_grasp as gj
    print("\n=== Track B: ≤10 路点关节抓取(move joints) ===")
    robot = xf.load_xifeng(gui=False)
    scene = gj.build_scene(robot, pick, place)
    keys_pose = gj.solve_key_poses(robot, pick, place)
    if keys_pose is None:
        p.disconnect()
        return
    keys = [("START", keys_pose["START"]),
            ("PICK_HOVER", keys_pose["PICK_HOVER"]),
            ("PICK_DESCEND", keys_pose["PICK_DESCEND"]),
            ("PICK_ASCEND", keys_pose["PICK_HOVER"]),
            ("PLACE_HOVER", keys_pose["PLACE_HOVER"]),
            ("PLACE_DESCEND", keys_pose["PLACE_DESCEND"]),
            ("PLACE_ASCEND", keys_pose["PLACE_HOVER"]),
            ("START_RETURN", keys_pose["START"])]
    chain, report, collided = gj.verified_chain(robot, scene["table"], keys)
    print(f"  路点: {len(chain)} 个 (armMoveJoints 逐点停稳 -> {len(chain)} 次停顿)")
    print(f"  段内插值碰撞复核: {'有未解决碰撞 ✗' if collided else '全部无碰撞 ✓'}")

    # 末端轨迹(关节插值下的实际末端路径), 按夹爪事件(抓/放)切成连续运动组
    verts, groups, cur = [], [], []
    for (na, qa), (nb, qb) in zip(chain, chain[1:]):
        span = max(abs(math.degrees(y - x)) for x, y in zip(qa, qb))
        n = max(2, int(span / 2))
        for k in range(n + 1):
            q = [x + (y - x) * k / n for x, y in zip(qa, qb)]
            gj.kin_set(robot, q)
            g, _ = ik.grip_state(robot)
            verts.append(g.tolist())
            cur.append(g.tolist())
        if nb in ("PICK_DESCEND", "PLACE_DESCEND"):      # 夹爪动作处必停
            groups.append(cur)
            cur = []
    groups.append(cur)
    turn_report([g[::6] for g in groups if len(g) > 12], "末端路径")
    length = sum(np.linalg.norm(np.array(b) - np.array(a))
                 for a, b in zip(verts, verts[1:]))
    print(f"  末端实际轨迹长度: {length*1000:.0f} mm")

    view, proj = _cam([(pick[0]+place[0])/2, (pick[1]+place[1])/2, pick[2]-0.03])
    draw_path(verts, [0.2, 0.45, 0.95, 1.0], every=8)   # 蓝色小球串 = 末端轨迹
    grasped = False
    names = []
    for i, (name, q) in enumerate(chain, start=1):
        gj.kin_set(robot, q)
        if name == "PICK_DESCEND":
            grasped = True
        if name == "PLACE_DESCEND":
            grasped = False
            p.resetBasePositionAndOrientation(
                scene["cube"], [place[0], place[1],
                                scene["table_top"] + gj.CUBE / 2], [0, 0, 0, 1])
        if grasped:
            g, _ = ik.grip_state(robot)
            p.resetBasePositionAndOrientation(scene["cube"], g.tolist(),
                                              [0, 0, 0, 1])
        names.append(shot(view, proj, f"B{i}_{name}"))
    p.disconnect()
    print(f"  已渲染: {', '.join(names)}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick", type=float, nargs=3, default=[0.36, 0.30, 1.05])
    ap.add_argument("--place", type=float, nargs=3, default=[0.36, 0.16, 1.05])
    ap.add_argument("--track", choices=["A", "B", "both"], default="both")
    a = ap.parse_args()
    if a.track in ("A", "both"):
        preview_a(a.pick, a.place)
    if a.track in ("B", "both"):
        preview_b(a.pick, a.place)
    print(f"\n图片目录: {os.path.relpath(OUT_DIR, _WORK)}")


if __name__ == "__main__":
    main()
