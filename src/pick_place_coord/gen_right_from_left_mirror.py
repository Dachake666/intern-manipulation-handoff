#!/usr/bin/env python3
"""右臂轨迹生成器 —— 从【已验收的左臂轨迹】镜像重定向, 而不是重新规划。

为什么不用 pick_place_coord.py --arm right 重新 RRT:
  目标是"和左臂那条 multi 轨迹基本一致", 那么最忠实的做法就是让右臂逐点
  复现左臂夹爪走过的【空间轨迹】, 而不是让 RRT 另找一条形状不同的路。
  顺带绕开了一个已知缺陷: 重建版 left_arm_ik 的 DLS 在下探链上会把 j1 顶到
  软限位后卡死(见 RECOVERY_STATUS / 本文件末尾说明), 从零规划目前跑不通。

做法(三步, 每步都可单独验):
  1. 逐点 FK 左臂已验收轨迹 -> 夹爪中心世界位置 + 末端整个旋转矩阵
  2. 关于机器人矢状面(y=0)镜像: 位置 y 取反, 姿态取 M·R·P(P 见下)
  3. 以【左臂关节角取反】为种子做 6D 全位姿逆解, 补掉两臂的结构不对称

关于"关节角取反"(20260806 实测澄清, 此前一度判断反了):
  q_右 = -q_左 【就是】两臂的镜像构型 —— 腕部原点差 0.65mm、姿态差 0.944°,
  且与关节角无关(纯粹是 URDF 左右臂本身不严格对称)。所以取反是很好的起点。
  真正的坑不在关节角, 而在【夹爪向量】: 右腕坐标系相对左腕的镜像多一次 X/Y
  互换, 夹爪在 link18 里是 [150,0,0] 而不是 [0,150,0]。用错会算出 200mm 级的
  假偏差, 把本来对的轨迹误判成废品。详见 cc.GRIP_R_LINK_MM。
  本脚本在取反基础上再做一次全位姿精修, 把那 0.944° 也补掉(实测差异 ≤0.95°,
  几乎全在 j6 腕滚)。

用法:
    python3 gen_right_from_left_mirror.py                      # 只算+校验, 不写文件
    python3 gen_right_from_left_mirror.py --out <路径>          # 写出轨迹
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import sys

import numpy as np
import pybullet as p

_HERE = os.path.dirname(os.path.abspath(__file__))
_WORK = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "analysis"))

import calib_common as cc
import left_arm_ik as ik
import xifeng_pb as xf

# 源: 冻结的真机验收轨迹(Track C pulse 双抓放, 20260804 五轮全过)
SRC = os.path.join(_WORK, "pick_place_coord", "trajectories", "verified",
                   "traj_multi_2grasp_20260804_REALVERIFIED.json")
TASK_RIGHT = os.path.join(_HERE, "tasks", "task_2pairs_table72_right.json")
MIRROR = np.array([1.0, -1.0, 1.0])          # 关于 y=0 的矢状面镜像

CUBE = 0.05
POS_TOL_MM = 2.0                             # 单点位置残差上限
AXIS_TOL_DEG = 6.0
MARGIN_WARN_DEG = 5.0                        # 关节余量低于此值告警


def fk_tool(robot, arm, ee, grip_m, q_sdk_deg):
    """SDK 关节角 -> (夹爪中心世界坐标, 末端连杆旋转矩阵)。"""
    q = cc.sdk_q_to_urdf_q(q_sdk_deg)
    for j, v in zip(arm, q):
        p.resetJointState(robot, j, math.radians(v))
    st = p.getLinkState(robot, ee, computeForwardKinematics=True)
    pos = np.array(st[4])
    R = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
    return pos + R @ grip_m, R


# ---------------------------------------------------------------- 整姿态逆解
# 只约束"位置 + 工具轴"会留下绕工具轴的 1 个自由度, 右臂逆解会挑出与左臂不同的
# 臂型 —— 实测那样解出来的轨迹在 PLACE1/PLACE2 会蹭到隔墙, 而左臂原轨迹不会。
# 姿态其实完全可定: 右腕的目标旋转 = M · R_左 · P
#   M = diag(1,-1,1)  矢状面镜像
#   P = 交换 X/Y 两列  (link18 的 +X 落在 mirror(link11 +Y) 上, 见 cc.GRIP_R_LINK_MM)
# 故这里自带一个 6D 全位姿 DLS, 不改共享的 left_arm_ik(那条路左臂在用)。
MIRROR_M = np.diag([1.0, -1.0, 1.0])
SWAP_P = np.array([[0.0, 1.0, 0.0],
                   [1.0, 0.0, 0.0],
                   [0.0, 0.0, 1.0]])
_LAMBDA = 0.05
_STEP_CLAMP = math.radians(10.0)


def _rot_err(R_cur, R_tgt):
    """把 R_tgt·R_curᵀ 取对数映射成轴角矢量(世界系), 小角度下即角速度方向。"""
    E = R_tgt @ R_cur.T
    ang = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0)))
    if ang < 1e-9:
        return np.zeros(3), 0.0
    axis = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    axis /= (2.0 * math.sin(ang))
    return axis * ang, math.degrees(ang)


# 姿态收敛阈值 1.2°: 理想镜像姿态在结构上就够不到 —— URDF 左右臂并非严格镜像,
# 同一构型下两腕姿态恒差 0.944°(与关节角无关, 见 20260806 实测)。阈值必须高于
# 这个结构下限, 否则会把"已经解到最好"误判成失败。位置没有这个问题(差 0.65mm)。
STRUCT_ASYM_DEG = 0.944


def solve_pose(robot, arm, ee, grip_m, lo, hi, pos_tgt, R_tgt, seed,
               pos_tol_m=1e-3, ori_tol_deg=1.2, iters=300):
    """6D 全位姿 DLS。返回 (q_rad 或 None, info)。"""
    def state(q):
        for j, v in zip(arm, q):
            p.resetJointState(robot, j, float(v))
        st = p.getLinkState(robot, ee, computeForwardKinematics=True)
        R = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
        return np.array(st[4]) + R @ grip_m, R

    def err(q):
        pos, R = state(q)
        e_r, ang = _rot_err(R, R_tgt)
        return np.concatenate([pos_tgt - pos, e_r]), \
            float(np.linalg.norm(pos_tgt - pos)), ang

    q = np.array(seed if seed is not None else (lo + hi) / 2.0, float)
    for _ in range(iters):
        e, pe, ae = err(q)
        if pe < pos_tol_m and ae < ori_tol_deg:
            return q.tolist(), {"pos_err_mm": pe * 1000, "ori_err_deg": ae}
        J = np.zeros((6, len(q)))
        for k in range(len(q)):
            qk = q.copy()
            qk[k] += 1e-5
            J[:, k] = (err(qk)[0] - e) / 1e-5
        # err() 返回的是误差 e = 目标 - 当前, 所以 de/dq = -d(fk)/dq。
        # DLS 要的是 d(fk)/dq, 少这个负号会朝背离目标的方向迭代(实测发散到
        # 1300mm)。left_arm_ik._jacobian 末尾的 `return -J` 是同一处修正。
        J = -J
        dq = J.T @ np.linalg.solve(J @ J.T + (_LAMBDA ** 2) * np.eye(6), e)
        q = np.clip(q + np.clip(dq, -_STEP_CLAMP, _STEP_CLAMP), lo, hi)
    e, pe, ae = err(q)
    return None, {"pos_err_mm": pe * 1000, "ori_err_deg": ae}


def build_mirror_scene(pairs, obstacles):
    """按右臂任务坐标搭出与左臂对称的场景(桌面 + 障碍), 用于碰撞复核。

    只建静态几何: 方块与放置标记不参与——转运时方块是被夹住的 attachment,
    逐点静态检查里把它当障碍会误报。
    """
    pts = [pr[k] for pr in pairs for k in ("pick", "place")]
    table_top = pairs[0]["pick"][2] - CUBE / 2
    xs, ys = [q[0] for q in pts], [q[1] for q in pts]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half = [(max(xs) - min(xs)) / 2 + 0.15,
            (max(ys) - min(ys)) / 2 + 0.15, 0.025]
    table = p.createMultiBody(
        0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
        baseVisualShapeIndex=-1,
        basePosition=[cx, cy, table_top - half[2]])
    obs = []
    for ob in obstacles:
        obs.append(p.createMultiBody(
            0, p.createCollisionShape(p.GEOM_BOX, halfExtents=ob["half"]),
            baseVisualShapeIndex=-1, basePosition=list(ob["center"])))
    return table, obs


def contact_profile(robot, arm, wps, pairs, obstacles):
    """逐点统计接触, 返回 {(段名, 类型): 点数}。

    近桌段(DESCEND/ASCEND/SETTLE)的桌面接触单列为 "桌面(近桌段)": URDF 的
    "幽灵手"网格伸出约 168mm, 在抓取深度必然蹭桌面, 左臂已验收轨迹同样如此,
    原生成器在这些段是临时关掉机器人↔桌子碰撞的(20260723: 不关会被接触脉冲
    顶偏 9mm)。换真实夹爪 mesh 后可去掉这条豁免。
    """
    home_rad = [math.radians(v) for v in cc.sdk_q_to_urdf_q(
        next(w["q_sdk_deg"] for w in wps if "q_sdk_deg" in w))]
    white = self_collision_whitelist(robot, arm, home_rad)
    table, obs = build_mirror_scene(pairs, obstacles)
    ee_near = ("DESCEND", "ASCEND", "SETTLE")
    prof = {}

    def bump(seg, kind):
        prof[(seg, kind)] = prof.get((seg, kind), 0) + 1

    for w in wps:
        if "q_sdk_deg" not in w:
            continue
        seg = w["seg"]
        for j, v in zip(arm, cc.sdk_q_to_urdf_q(w["q_sdk_deg"])):
            p.resetJointState(robot, j, math.radians(v))
        p.performCollisionDetection()
        for c in p.getContactPoints(robot, robot):
            if tuple(sorted((c[3], c[4]))) not in white and abs(c[3] - c[4]) > 1:
                bump(seg, "自碰撞")
        if p.getContactPoints(robot, table):
            bump(seg, "桌面(近桌段)" if any(k in seg for k in ee_near)
                 else "桌面(非近桌段)")
        for b in obs:
            if p.getContactPoints(robot, b):
                bump(seg, "障碍墙")
    for b in [table] + obs:
        p.removeBody(b)
    return prof


def self_collision_whitelist(robot, arm, home_rad, margin=0.012):
    """HOME 姿态下本就贴近的连杆对, 判碰时忽略(与生成器同一判据)。"""
    for j, v in zip(arm, home_rad):
        p.resetJointState(robot, j, v)
    for _ in range(30):
        p.stepSimulation()
    white = set()
    n = p.getNumJoints(robot)
    for a in range(-1, n):
        for b in range(a + 1, n):
            if p.getClosestPoints(robot, robot, margin, a, b):
                white.add((a, b))
    return white


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--src", default=SRC, help="源左臂轨迹(已验收)")
    ap.add_argument("--task", default=TASK_RIGHT, help="右臂任务文件(供场景/元信息)")
    ap.add_argument("--out", default=None, help="输出路径; 不给则只算不写")
    ap.add_argument("--pos-tol-mm", type=float, default=POS_TOL_MM)
    a = ap.parse_args(argv)

    src = json.load(open(a.src))
    src_meta, src_wps = src["meta"], src["waypoints"]
    if src_meta.get("arm") != "left":
        raise SystemExit(f"源轨迹不是左臂: arm={src_meta.get('arm')!r}")
    task = json.load(open(a.task))
    pairs = [{"pick": list(map(float, pr["pick"])),
              "place": list(map(float, pr["place"]))} for pr in task["pairs"]]
    obstacles = [{"center": list(map(float, ob["center"])),
                  "half": list(map(float, ob["half"]))}
                 for ob in task.get("obstacles", [])]

    print("=" * 70)
    print("右臂轨迹 = 已验收左臂轨迹的矢状面镜像重定向")
    print("=" * 70)
    print(f"源      : {os.path.relpath(a.src, _WORK)}")
    print(f"          {len(src_wps)} 条记录, 战绩见 verified/README")
    print(f"右臂任务: {os.path.relpath(a.task, _WORK)}")

    robot = xf.load_xifeng(gui=False)
    L_ARM, L_EE = cc.ARM_JOINT_IDS["left"], cc.EE_LINK_ID["left"]
    GL = cc.GRIP_R_LINK_MM["left"] / 1000.0

    # ---- 1. 左臂逐点 FK ----
    targets = []
    for i, w in enumerate(src_wps):
        if "q_sdk_deg" not in w:
            targets.append((i, w["seg"], None, None, None))
            continue
        pos, R = fk_tool(robot, L_ARM, L_EE, GL, w["q_sdk_deg"])
        # 种子 = 左臂关节角逐个取反。实测(20260806)这就是两臂的镜像构型:
        # 腕部原点差 0.65mm、姿态差 0.944°, 且与姿态无关(= URDF 左右臂固有的
        # 那点不对称)。用它做种子, 逆解就停在与左臂同一个臂型分支上, 精修只
        # 补掉那 0.9° 的不对称, 不会跳到别的解。
        seed_neg = [math.radians(-v) for v in cc.sdk_q_to_urdf_q(w["q_sdk_deg"])]
        targets.append((i, w["seg"], pos * MIRROR, MIRROR_M @ R @ SWAP_P,
                        seed_neg))
    n_pose = sum(1 for t in targets if t[2] is not None)
    print(f"\n[1/4] 左臂 FK + 镜像: {n_pose} 个位姿(位置 + 整姿态)")

    # ---- 2. 右臂逆解(热启动) ----
    ik.set_arm("right")
    GR = cc.GRIP_R_LINK_MM["right"] / 1000.0
    lo, hi = ik._limits(robot)
    print(f"[2/4] 右臂 6D 全位姿逆解: 关节{ik.ARM} 末端 link{ik.EE} "
          f"夹爪向量{[float(v) for v in cc.GRIP_R_LINK_MM['right']]}mm")
    out_wps = []
    worst_p, worst_o, fails = (0.0, None), (0.0, None), []
    for i, seg, pos, R_tgt, seed_neg in targets:
        if pos is None:
            out_wps.append({"seg": seg, "gripper": src_wps[i]["gripper"]})
            continue
        q, info = solve_pose(robot, ik.ARM, ik.EE, GR, lo, hi, pos, R_tgt,
                             np.clip(seed_neg, lo, hi))
        if q is None:
            fails.append((i, seg, info["pos_err_mm"], info["ori_err_deg"]))
            continue
        if info["pos_err_mm"] > worst_p[0]:
            worst_p = (info["pos_err_mm"], seg)
        if info["ori_err_deg"] > worst_o[0]:
            worst_o = (info["ori_err_deg"], seg)
        q_urdf_deg = [math.degrees(v) for v in q]
        out_wps.append({"seg": seg,
                        "q_sdk_deg": [round(v, 4) for v in
                                      cc.urdf_q_to_sdk_q(q_urdf_deg)]})
    print(f"      解出 {n_pose - len(fails)}/{n_pose}, 最大位置残差 "
          f"{worst_p[0]:.2f}mm @ {worst_p[1]}, 最大姿态残差 "
          f"{worst_o[0]:.3f}° @ {worst_o[1]}")
    if fails:
        for i, seg, e, o in fails[:10]:
            print(f"      ✗ 点{i} {seg} 位置 {e:.1f}mm 姿态 {o:.2f}°")
        raise SystemExit("有点解不出, 中止(不写文件)。")
    if worst_p[0] > a.pos_tol_mm:
        raise SystemExit(f"最大残差 {worst_p[0]:.2f}mm 超过阈值 "
                         f"{a.pos_tol_mm}mm, 中止。")
    worst = worst_p

    # ---- 3. 限位复核 ----
    print("[3/4] 限位复核(URDF 右臂硬限位)")
    lims = [p.getJointInfo(robot, j)[8:10] for j in ik.ARM]
    Q = np.array([w["q_sdk_deg"] for w in out_wps if "q_sdk_deg" in w])
    Q_urdf = np.array([cc.sdk_q_to_urdf_q(q) for q in Q])
    bad = False
    for k, (lo, hi) in enumerate(lims):
        lo_d, hi_d = math.degrees(lo), math.degrees(hi)
        a_, b_ = Q_urdf[:, k].min(), Q_urdf[:, k].max()
        m = min(a_ - lo_d, hi_d - b_)
        flag = ""
        if m < 0:
            flag, bad = "  ✗ 越限", True
        elif m < MARGIN_WARN_DEG:
            flag = "  ⚠ 余量偏小"
        print(f"      j{k+1}: 用到[{a_:8.2f},{b_:8.2f}]  "
              f"限位[{lo_d:8.2f},{hi_d:8.2f}]  余量 {m:6.2f}°{flag}")
    if bad:
        raise SystemExit("存在越限点, 中止。")

    # ---- 4. 碰撞复核: 与左臂源轨迹【对照】, 判据是"不比左臂差" ----
    # 绝对零接触不是合适的判据 —— 左臂那条真机跑通 5 轮的轨迹, 在同一套检查下
    # 本身就有接触(幽灵手网格蹭桌面/蹭隔墙)。所以这里逐(段,类型)比对: 右臂只要
    # 不出现左臂没有的接触、也不比左臂更多, 就算通过。
    print("[4/4] 碰撞复核(与左臂源轨迹逐段对照)")
    mirror_back = [{"pick": [pr["pick"][0], -pr["pick"][1], pr["pick"][2]],
                    "place": [pr["place"][0], -pr["place"][1], pr["place"][2]]}
                   for pr in pairs]
    obs_back = [{"center": [ob["center"][0], -ob["center"][1], ob["center"][2]],
                 "half": ob["half"]} for ob in obstacles]
    base = contact_profile(robot, L_ARM, src_wps, mirror_back, obs_back)
    mine = contact_profile(robot, ik.ARM, out_wps, pairs, obstacles)
    keys = sorted(set(base) | set(mine))
    worse = []
    for k in keys:
        b, m = base.get(k, 0), mine.get(k, 0)
        mark = ""
        if m > b:
            mark, _ = "  ✗ 比左臂多", worse.append((k, b, m))
        print(f"      {k[0]:30s} {k[1]:14s} 左 {b:3d} 点 / 右 {m:3d} 点{mark}")
    if worse:
        raise SystemExit("右臂出现左臂没有的接触, 中止。")
    print("      右臂接触剖面与左臂一致或更好。")

    steps = [max(abs(x - y) for x, y in zip(Q[i], Q[i + 1]))
             for i in range(len(Q) - 1)]
    print(f"\n相邻路点最大关节跨度 {max(steps):.2f}°  "
          f"中位 {sorted(steps)[len(steps)//2]:.2f}°  总行程 {sum(steps):.1f}°")

    if not a.out:
        print("\n未指定 --out, 只算不写。")
        p.disconnect()
        return 0

    meta = {
        "arm_id": 2, "arm": "right", "j6_flipped": True,
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "task": "multi_right_mirror",
        "derived_from": {
            "src": os.path.relpath(a.src, _WORK),
            "src_created": src_meta.get("created"),
            "method": "逐点 FK 左臂已验收轨迹 -> 夹爪位姿关于 y=0 镜像 -> "
                      "右臂 DLS 逆解(热启动)。关节角未做任何取反。",
            "max_pos_residual_mm": round(worst[0], 3),
            "axis_tol_deg": AXIS_TOL_DEG,
        },
        "pairs_pb_m": pairs,
        "obstacles_pb_m": obstacles,
        "t_session_used": src_meta.get("t_session_used"),
        "grip_r_link_mm": [float(v) for v in cc.GRIP_R_LINK_MM["right"]],
        "ee_link_id": ik.EE,
        "export_step_deg": src_meta.get("export_step_deg"),
        "continuous_candidate": False,
        "note": "右臂 pulse servo 用。q_sdk_deg 已含 j6 翻转。"
                "⚠ 上机前必须先实测右臂 j7 物理停止位: 左臂实测 +76.5°, "
                "右臂对应值从未验证, 本轨迹按 URDF 限位生成。"
                "⚠ meta.arm_id=2(右臂)为厂商约定推断, 亦未真机验证。",
    }
    payload = {"meta": meta, "waypoints": out_wps}
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(payload, f, indent=1, ensure_ascii=False)
    n_mv = sum(1 for w in out_wps if "q_sdk_deg" in w)
    print(f"\n已写出({n_mv} 路点 + {len(out_wps) - n_mv} 夹爪事件): "
          f"{os.path.relpath(a.out, _WORK)}")
    p.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
