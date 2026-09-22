#!/usr/bin/env python3
"""坐标指定的 pick-and-place 规划管线(IK -> RRT -> 验证 -> 导出关节轨迹)。

============================ 恢复重建说明 ============================
本文件于 2026-07-22 从聊天记录 + workspace/sdk_tests 幸存轨迹重建
(原 Mac 工程连同 git 仓库被误删, 废纸篓/全盘均无副本)。

置信度:
  * 高(逐字来自聊天): Traj、build_scene_multi、pick_and_place_multi、
    ready_target、scan、build_arg_parser、main、全部常量数值。
  * 中(按已知行为重建): 顶部 import、坐标换算、load_task_file、plan_path/
    plan_verified/densify、Monitor/play/kin_seq、solve_task_poses、
    单点 build_scene/pick_and_place。逻辑与原版一致, 但细节可能有微小出入。
依赖模块 xf(xifeng 加载器)、ik(left_arm_ik)、cc(calib_common) 同为重建件,
标定常量务必在首次真机使用前逐项复核。详见工作区根目录 RECOVERY_STATUS.md。
=====================================================================

用法:
  # 单组抓放(PB 世界系, 米)
  python3 pick_place_coord.py --pick 0.36 0.30 1.00 --place 0.36 0.16 1.00
  # 多组抓放 + 障碍(视觉模块未来输出同一 JSON 格式)
  python3 pick_place_coord.py --task tasks/task_2pairs_table72.json --seed 7 --export-json
  # 可达范围扫描
  python3 pick_place_coord.py --scan
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import os
import random
import sys
import time

import numpy as np
import pybullet as p

# 工作区内的模块
_HERE = os.path.dirname(os.path.abspath(__file__))
_WORK = os.path.dirname(_HERE)
sys.path.insert(0, _HERE)
sys.path.insert(0, _WORK)
sys.path.insert(0, os.path.join(_WORK, "frame_calibration", "analysis"))

import calib_common as cc          # 标定常量 + SDK<->URDF 关节换算
import left_arm_ik as ik           # DLS 数值 IK
import xifeng_pb as xf             # PyBullet 加载/可视化/抓取约束封装
import pybullet_planning as pp     # RRT-Connect(birrt) + attachment
import arm_profiles as profiles

# ---------------------------------------------------------------- 常量
ARM_NAME = "left"
PROFILE = profiles.arm_profile(ARM_NAME)
EE = xf.EE_LINK_ID                       # 末端(link11)连杆号
ARM = xf.ARM_JOINT_IDS                    # 当前臂 7 个关节 id
HOME_DEG = PROFILE["home_deg"]
HOME = [math.radians(v) for v in HOME_DEG]
IK_SEED = [math.radians(v) for v in PROFILE["ik_seed_deg"]]

LIFT = 0.18                              # 抓取点上方悬停高度(m), 保留已验收净空
READY_EXTRA = 0.04                       # READY 相对悬停面的额外抬高(m), P2: 原 0.08
DESCEND_STEP = 0.015                     # 竖直下探的笛卡尔步长(m)
RELEASE_DROP = 0.008                     # 放置时松开高度(m, 方块自由落下这段)
CUBE = 0.05
RESOLUTION = math.radians(3.0)           # RRT 路径致密化步长(rad)
SMOOTH_ITERS = 150                       # RRT 后处理捷径平滑次数
# 真实夹爪指尖比 URDF 抓取中心多伸出的量; 用于算抓取点相对桌面的安全余量
REAL_TIP_BEYOND_GRIP_MM = 18.0
TABLE_SAFETY_MM = 5.0                    # 指尖距桌面最小允许余量
EXPORT_STEP_DEG = 5.0                    # 导出轨迹的关节步长上限(deg)

RUN_SEED = None                          # --seed 固定随机种子(可复现轨迹), 由 main 写入
PLAN_METRICS = {"legs": 0, "time_s": 0.0, "len_rad": 0.0}   # 每次任务重置累计
CONTINUOUS_CANDIDATE_SEGMENTS = [
    "HOME->READY", "READY->PICK_HOVER", "PICK_HOVER->READY",
    "READY->PLACE_HOVER", "PLACE_HOVER->READY", "READY->HOME",
]
T_SESSION_KEY = "20260707"
T_MM = cc.CONFIRMED_T_SESSIONS_MM[T_SESSION_KEY]

# 可达范围盒(PB 世界系, 夹爪中心坐标)。由 --scan 实测校准。
RANGE_BOX = {k: tuple(v) for k, v in PROFILE["workspace_m"].items()}

DISABLED = []                            # 自碰撞白名单(相邻连杆对), main 里计算


def set_arm(arm):
    """把规划链整体切到指定臂；右臂输出始终是 CANDIDATE。"""
    global ARM_NAME, PROFILE, ARM, EE, HOME_DEG, HOME, IK_SEED, RANGE_BOX
    PROFILE = profiles.arm_profile(arm)
    xf.set_arm(arm)
    ik.set_arm(arm)
    ARM_NAME = arm
    ARM = xf.ARM_JOINT_IDS
    EE = xf.EE_LINK_ID
    HOME_DEG = list(PROFILE["home_deg"])
    HOME = [math.radians(v) for v in HOME_DEG]
    IK_SEED = [math.radians(v) for v in PROFILE["ik_seed_deg"]]
    RANGE_BOX = {k: tuple(v) for k, v in PROFILE["workspace_m"].items()}
    return ARM_NAME


# ---------------------------------------------------------------- 坐标换算
def sdk_to_pb_m(xyz_mm):
    """SDK 世界系(mm) -> PB 世界系(m)。p_pb = (p_sdk - T)/1000, 轴向同向。"""
    return (np.asarray(xyz_mm, float) - np.asarray(T_MM, float)) / 1000.0


def pb_m_to_sdk_mm(xyz_m):
    """PB 世界系(m) -> SDK 世界系(mm)。"""
    return np.asarray(xyz_m, float) * 1000.0 + np.asarray(T_MM, float)


def range_violations(pt, name):
    bad = []
    for i, k in enumerate("xyz"):
        lo, hi = RANGE_BOX[k]
        if not (lo - 1e-9 <= pt[i] <= hi + 1e-9):
            bad.append(f"{name}.{k}={pt[i]:.3f} 越界 [{lo}, {hi}]")
    return bad


def ready_target(pick, place):
    """READY 位姿目标点。只降低 READY_EXTRA, 不改变抓取/放置悬停净空。"""
    return [(pick[0] + place[0]) / 2, (pick[1] + place[1]) / 2,
            max(pick[2], place[2]) + LIFT + READY_EXTRA]


def load_task_file(path):
    """读取多点任务 JSON: {pairs:[{pick,place}], obstacles:[{center,half}]}。"""
    with open(path) as f:
        d = json.load(f)
    pairs = [{"pick": list(map(float, pr["pick"])),
              "place": list(map(float, pr["place"]))} for pr in d["pairs"]]
    obstacles = [{"center": list(map(float, ob["center"])),
                  "half": list(map(float, ob["half"]))}
                 for ob in d.get("obstacles", [])]
    return pairs, obstacles


# ---------------------------------------------------------------- 轨迹容器
class Traj:
    """收集关节路点与夹爪事件; 导出时按 EXPORT_STEP_DEG 抽稀并换算到 SDK 系。"""

    def __init__(self):
        self.items = []                  # {"seg","q_rad"} 或 {"seg","gripper"}

    def record(self, seg, q_rad):
        self.items.append({"seg": seg, "q_rad": [float(v) for v in q_rad]})

    def gripper(self, seg, action):
        self.items.append({"seg": seg, "gripper": action})

    def _downsample(self):
        out, last_q = [], None
        for i, it in enumerate(self.items):
            if "gripper" in it:
                out.append(it)
                last_q = None            # 事件后强制保留下一个路点
                continue
            nxt = self.items[i + 1] if i + 1 < len(self.items) else None
            seg_end = nxt is None or "gripper" in nxt or nxt["seg"] != it["seg"]
            if last_q is None or seg_end or max(
                    abs(a - b) for a, b in zip(it["q_rad"], last_q)
            ) >= math.radians(EXPORT_STEP_DEG):
                out.append(it)
                last_q = it["q_rad"]
        return out

    def export(self, path, meta):
        waypoints = []
        for it in self._downsample():
            if "gripper" in it:
                waypoints.append({"seg": it["seg"], "gripper": it["gripper"]})
            else:
                q_deg = [math.degrees(v) for v in it["q_rad"]]
                waypoints.append({"seg": it["seg"],
                                  "q_sdk_deg": [round(v, 4) for v in
                                                cc.urdf_q_to_sdk_q(q_deg)]})
        payload = {"meta": meta, "waypoints": waypoints}
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        # P2 起只写 traj_latest(不再每次留时间戳副本); 真机 as-run 版本由归档保存
        with open(path, "w") as f:
            json.dump(payload, f, indent=1, ensure_ascii=False)
        n_moves = sum(1 for w in waypoints if "q_sdk_deg" in w)
        print(f"\n轨迹已导出({n_moves} 个路点 + "
              f"{len(waypoints) - n_moves} 个夹爪事件): "
              f"{os.path.relpath(path, _WORK)}")
        return [path]


# ---------------------------------------------------------------- 规划机械
def compute_disabled_pairs(robot, margin=0.012):
    """初始 HOME 姿态下本就接触/极近的连杆对, 加入自碰撞白名单避免误报。"""
    _kin_set_rad(robot, HOME)
    for _ in range(30):
        p.stepSimulation()
    disabled = set()
    n = p.getNumJoints(robot)
    for a in range(-1, n):
        for b in range(a + 1, n):
            pts = p.getClosestPoints(robot, robot, margin, a, b)
            if pts:
                disabled.add((a, b))
    return disabled


def _custom_limits(robot):
    lims = {}
    for j in ARM:
        info = p.getJointInfo(robot, j)
        lims[j] = (info[8], info[9])
    return lims


def plan_path(robot, end_rad, obstacles, attachments=()):
    """RRT-Connect(birrt) 关节空间规划; 返回路点列表或 None。"""
    start = [p.getJointState(robot, j)[0] for j in ARM]
    _kin_set(robot, [math.degrees(v) for v in start])
    path = pp.plan_joint_motion(
        robot, ARM, end_rad,
        obstacles=obstacles, attachments=attachments,
        self_collisions=True, disabled_collisions=DISABLED,
        custom_limits=_custom_limits(robot),
        resolutions=[RESOLUTION] * len(ARM),
        smooth=SMOOTH_ITERS)
    return path


def path_len_rad(path):
    """关节空间路径总长(rad), 用于规划指标。"""
    return sum(math.sqrt(sum((b - a) ** 2 for a, b in zip(p0, p1)))
               for p0, p1 in zip(path, path[1:])) if path and len(path) > 1 else 0.0


def densify(path, max_step=RESOLUTION):
    """把相邻路点间距大于 max_step 的段插值细分。"""
    if not path:
        return path
    out = [path[0]]
    for a, b in zip(path, path[1:]):
        d = max(abs(y - x) for x, y in zip(a, b))
        n = max(1, int(math.ceil(d / max_step)))
        for k in range(1, n + 1):
            out.append([x + (y - x) * k / n for x, y in zip(a, b)])
    return out


def verify_path_free(robot, path, obstacles, attachments=()):
    """逐点复核路径无碰撞(RRT 内部已查, 这里是导出前的独立门)。"""
    for q in path:
        _kin_set(robot, [math.degrees(v) for v in q])
        for att in attachments:
            att.assign()
        p.performCollisionDetection()
        for ob in obstacles:
            if p.getClosestPoints(robot, ob, 0.0):
                return False
    return True


def plan_verified(robot, end_rad, obstacles, attachments=()):
    """规划 + 致密化 + 独立复核; 累计规划指标。"""
    def restore():
        pass
    t0 = time.time()
    path = plan_path(robot, end_rad, obstacles, attachments)
    if path is None:
        return None
    path = densify(path)
    if not verify_path_free(robot, path, obstacles, attachments):
        return None
    PLAN_METRICS["legs"] += 1
    PLAN_METRICS["time_s"] += time.time() - t0
    PLAN_METRICS["len_rad"] += path_len_rad(path)
    return path


def _kin_set(robot, q_arm_deg):
    """把左臂关节摆到给定 URDF 角度(度), 纯运动学重置。"""
    active = set(ARM)
    for jid in range(p.getNumJoints(robot)):
        if jid not in active and p.getJointInfo(robot, jid)[2] != p.JOINT_FIXED:
            p.resetJointState(robot, jid, 0.0)
    for j, q in zip(ARM, q_arm_deg):
        p.resetJointState(robot, j, math.radians(q))


def _kin_set_rad(robot, q_arm_rad):
    """把当前臂摆到 URDF 弧度；避免把弧度误当度再转换一次。"""
    active = set(ARM)
    for jid in range(p.getNumJoints(robot)):
        if jid not in active and p.getJointInfo(robot, jid)[2] != p.JOINT_FIXED:
            p.resetJointState(robot, jid, 0.0)
    for j, q in zip(ARM, q_arm_rad):
        p.resetJointState(robot, j, float(q))


def table_collision(robot, table, enable):
    """开/关 机器人<->桌子 的碰撞检测(下探到桌面时需暂时关闭)。"""
    n = p.getNumJoints(robot)
    for link in range(-1, n):
        p.setCollisionFilterPair(robot, table, link, -1, 1 if enable else 0)


class Monitor:
    """自由段运动时统计 机器人/搬运物 与 桌子/障碍 的接触点峰值。"""

    def __init__(self, robot, bodies):
        self.robot = robot
        self.bodies = bodies
        self.on = False
        self.max_hits = 0

    def sample(self):
        if not self.on:
            return
        hits = 0
        for b in self.bodies:
            hits += len(p.getClosestPoints(self.robot, b, 0.0))
        self.max_hits = max(self.max_hits, hits)


def play(robot, name, path, traj, mon, gui, attachments=()):
    """执行一段 RRT 路径: 逐点摆位、记录、监测。"""
    print(f"   [执行] {name}: {len(path)} 路点(RRT, 已逐点复核无碰撞)")
    for q in path:
        _kin_set(robot, [math.degrees(v) for v in q])
        for att in attachments:
            att.assign()
        traj.record(name, q)
        mon.sample()
        if gui:
            p.stepSimulation()


def kin_seq(robot, name, qs, traj, mon, gui, settle=0, attachments=()):
    """执行一串预解算好的关节角(近物下探/上升链), 无 RRT。"""
    print(f"   [执行] {name}: {len(qs)} 路点(近物直线链)")
    for q in qs:
        _kin_set(robot, [math.degrees(v) for v in q])
        for att in attachments:
            att.assign()
        traj.record(name, q)
        mon.sample()
        if gui:
            p.stepSimulation()
    # resetJointState 后若只 step，URDF 默认关节电机会把姿态拉回零位；settle 期间
    # 必须持续保持末点，否则抓取前会出现数十毫米假漂移。
    if qs:
        for _ in range(settle):
            _kin_set_rad(robot, qs[-1])
            p.stepSimulation()
            for att in attachments:
                att.assign()
        _kin_set_rad(robot, qs[-1])
        for att in attachments:
            att.assign()


# ---------------------------------------------------------------- 单组抓放
def solve_task_poses(robot, pick, place):
    """求解单组抓放的全部关键姿态: 悬停 + 下探链(工具轴朝下)。"""
    down = (0, 0, -1.0)
    poses = {}

    def req(name, target, seed, axis_tol=6.0):
        # READY 只是转运过渡点, 工具朝向无需精确向下 —— 放宽轴约束可让腕关节
        # 有更多回旋余地, 提高可解率。
        q, info = ik.solve(robot, target, down, seed_rad=seed,
                           axis_tol_deg=axis_tol)
        poses[name] = q
        return q

    poses["ready"] = req("ready", ready_target(pick, place), IK_SEED,
                         axis_tol=30.0)
    if poses["ready"] is None:
        return None
    if req("pick_hover", [pick[0], pick[1], pick[2] + LIFT],
           poses["ready"]) is None:
        return None
    if req("place_hover", [place[0], place[1], place[2] + LIFT],
           poses["ready"]) is None:
        return None

    def chain(name, x, y, z_hi, z_lo, seed):
        qs, q_prev = [], seed
        n_step = max(1, int(round((z_hi - z_lo) / DESCEND_STEP)))
        for k in range(0, n_step + 1):
            z = z_hi - (z_hi - z_lo) * k / n_step
            q, _ = ik.solve(robot, [x, y, z], down, seed_rad=q_prev)
            if q is None:
                return None
            qs.append(q)
            q_prev = q
        return qs

    down_pick = chain("pick", pick[0], pick[1], pick[2] + LIFT, pick[2],
                      poses["pick_hover"])
    down_place = chain("place", place[0], place[1], place[2] + LIFT,
                       place[2] + RELEASE_DROP, poses["place_hover"])
    if down_pick is None or down_place is None:
        return None
    poses["down_pick"] = down_pick
    poses["down_place"] = down_place
    return poses


def build_scene(robot, pick, place):
    """单组抓放场景: 桌子 + 一个方块 + 放置标记。"""
    table_top = pick[2] - CUBE / 2
    cx, cy = (pick[0] + place[0]) / 2, (pick[1] + place[1]) / 2
    half = [abs(pick[0] - place[0]) / 2 + 0.15,
            abs(pick[1] - place[1]) / 2 + 0.15, 0.025]
    table = p.createMultiBody(
        0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
        p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                            rgbaColor=[0.82, 0.7, 0.5, 1]),
        [cx, cy, table_top - half[2]])
    cube = p.createMultiBody(
        0.0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE / 2] * 3),
        p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE / 2] * 3,
                            rgbaColor=[0.9, 0.2, 0.2, 1]),
        [pick[0], pick[1], table_top + CUBE / 2])
    for _ in range(60):
        _kin_set_rad(robot, HOME)
        p.stepSimulation()
    return {"table": table, "cube": cube, "table_top": table_top}


def pick_and_place(robot, pick, place, gui, do_export=True, out=None):
    """单组抓放主流程。"""
    print(f"\n单组抓放: pick={np.round(pick, 4).tolist()}  "
          f"place={np.round(place, 4).tolist()}")
    bad = range_violations(pick, "pick") + range_violations(place, "place")
    if bad:
        print("坐标越界, 拒绝执行:\n  " + "\n  ".join(bad))
        return False

    tip_clear_mm = CUBE / 2 * 1000 - REAL_TIP_BEYOND_GRIP_MM
    grasp_off_m = max(0.0, (TABLE_SAFETY_MM - tip_clear_mm) / 1000.0)
    pick_g = [pick[0], pick[1], pick[2] + grasp_off_m]
    place_g = [place[0], place[1], place[2] + grasp_off_m]

    poses = solve_task_poses(robot, pick_g, place_g)
    if poses is None:
        print("IK 失败, 换坐标或调姿态。")
        return False

    scene = build_scene(robot, pick_g, place_g)
    obstacles = [scene["table"]]
    mon = Monitor(robot, obstacles)
    traj = Traj()
    xf.draw_frame(robot, EE, length=0.1, width=2)
    xf.set_robot_object_collision(robot, scene["cube"], False)
    _kin_set_rad(robot, HOME)
    traj.record("HOME", HOME)

    PLAN_METRICS.update(legs=0, time_s=0.0, len_rad=0.0)

    def leg(name, end, attachments=()):
        path = plan_verified(robot, end, obstacles, attachments)
        if path is None:
            print(f"规划失败: {name}")
            return False
        mon.on = True
        play(robot, name, path, traj, mon, gui, attachments)
        mon.on = False
        return True

    if not leg("HOME->READY", poses["ready"]):
        return False
    if not leg("READY->PICK_HOVER", poses["pick_hover"]):
        return False
    table_collision(robot, scene["table"], False)
    kin_seq(robot, "PICK_DESCEND", poses["down_pick"], traj, mon, gui,
            settle=30)
    cid = xf.attach_grasp(robot, scene["cube"], ARM)
    p.changeConstraint(cid, maxForce=1e5)
    p.changeDynamics(scene["cube"], -1, mass=0.02)
    attachment = pp.create_attachment(robot, EE, scene["cube"])
    traj.gripper("PICK", "close")
    kin_seq(robot, "PICK_ASCEND", list(reversed(poses["down_pick"])),
            traj, mon, gui, attachments=(attachment,))
    table_collision(robot, scene["table"], True)
    if not leg("PICK_HOVER->READY", poses["ready"], (attachment,)):
        return False
    if not leg("READY->PLACE_HOVER", poses["place_hover"], (attachment,)):
        return False
    table_collision(robot, scene["table"], False)
    kin_seq(robot, "PLACE_DESCEND", poses["down_place"], traj, mon, gui,
            settle=80, attachments=(attachment,))
    traj.gripper("PLACE", "open")
    p.changeDynamics(scene["cube"], -1, mass=0.2)
    p.removeConstraint(cid)
    kin_seq(robot, "PLACE_ASCEND", list(reversed(poses["down_place"])),
            traj, mon, gui)
    table_collision(robot, scene["table"], True)
    leg("PLACE_HOVER->READY", poses["ready"])
    leg("READY->HOME", HOME)

    ok = mon.max_hits == 0
    print(f"\n自由段接触点峰值: {mon.max_hits} (期望 0)")
    print(f"规划指标: {PLAN_METRICS['legs']} 段 RRT, 耗时 "
          f"{PLAN_METRICS['time_s']:.1f}s, 路径总长 "
          f"{PLAN_METRICS['len_rad']:.2f} rad (seed={RUN_SEED})")
    print("✅ 单组抓放达成。" if ok else "❌ 未达成。")

    if ok and do_export:
        pick_hover = ready_target(pick_g, place_g)
        meta = {
            "arm_id": PROFILE["arm_id"], "arm": ARM_NAME, "j6_flipped": True,
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "task": "single",
            "pick_sdk_mm": [round(float(v), 2) for v in pb_m_to_sdk_mm(pick_g)],
            "place_sdk_mm": [round(float(v), 2)
                             for v in pb_m_to_sdk_mm(place_g)],
            "t_session_used": T_SESSION_KEY,
            "grip_r_link_mm": PROFILE["grip_center_link_mm"],
            "ee_link_id": PROFILE["ee_link_id"],
            "qualification": PROFILE["status"],
            "lift_m": LIFT, "ready_extra_m": READY_EXTRA,
            "ready_sdk_mm": [round(float(v), 2)
                             for v in pb_m_to_sdk_mm(pick_hover)],
            "export_step_deg": EXPORT_STEP_DEG,
            # 仅表示这些自由空间段可进入真机连续能力验证，不等于已通过真机融合验收。
            "continuous_candidate": True,
            "continuous_candidate_segments": CONTINUOUS_CANDIDATE_SEGMENTS,
            "note": "q_sdk_deg 已含 j6 符号翻转, 可直接逐点 armMoveJoints; "
                    "腰部若动过, SDK 坐标含义随 t 漂移, 关节轨迹本身不受影响",
        }
        traj.export(out or os.path.join(_HERE, "trajectories",
                                        "traj_latest.json"), meta)
    elif ok:
        print("\n轨迹 JSON: 未生成(需要时加 --export-json; --out 也会开启导出)。")
    return ok


# ---------------------------------------------------------------- 多点任务(P3 复杂化)
CUBE_COLORS = [(0.9, 0.2, 0.2, 1), (0.2, 0.4, 0.9, 1), (0.9, 0.7, 0.1, 1),
               (0.6, 0.2, 0.8, 1), (0.2, 0.8, 0.5, 1)]


def build_scene_multi(robot, pairs, obstacles_spec):
    """多抓放对场景: 一张覆盖全部点位的桌子 + 每对一个方块/标记 + 静态障碍物。"""
    pts = [pr[k] for pr in pairs for k in ("pick", "place")]
    table_top = pairs[0]["pick"][2] - CUBE / 2
    xs, ys = [q[0] for q in pts], [q[1] for q in pts]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
    half = [(max(xs) - min(xs)) / 2 + 0.15, (max(ys) - min(ys)) / 2 + 0.15, 0.025]
    table = p.createMultiBody(
        0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
        p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                            rgbaColor=[0.82, 0.7, 0.5, 1]),
        [cx, cy, table_top - half[2]])
    cubes, markers = [], []
    for i, pr in enumerate(pairs):
        color = CUBE_COLORS[i % len(CUBE_COLORS)]
        # 抓取前保持静态，避免重建 URDF 的幽灵手网格在场景 settle 时把目标推走。
        # 真正夹住后改为 0.02kg，释放前再恢复 0.2kg。
        cubes.append(p.createMultiBody(
            0.0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[CUBE / 2] * 3),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[CUBE / 2] * 3,
                                rgbaColor=color),
            [pr["pick"][0], pr["pick"][1], table_top + CUBE / 2]))
        markers.append(p.createMultiBody(
            0, baseVisualShapeIndex=p.createVisualShape(
                p.GEOM_CYLINDER, radius=0.045, length=0.002,
                rgbaColor=[color[0], color[1], color[2], 0.6]),
            basePosition=[pr["place"][0], pr["place"][1], table_top + 0.001]))
    obstacle_bodies = []
    for ob in obstacles_spec:
        obstacle_bodies.append(p.createMultiBody(
            0, p.createCollisionShape(p.GEOM_BOX, halfExtents=ob["half"]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=ob["half"],
                                rgbaColor=[0.45, 0.45, 0.45, 0.9]),
            list(ob["center"])))
    for _ in range(60):
        _kin_set_rad(robot, HOME)
        p.stepSimulation()
    return {"table": table, "cubes": cubes, "markers": markers,
            "obstacle_bodies": obstacle_bodies, "table_top": table_top}


def pick_and_place_multi(robot, pairs, obstacles_spec, gui,
                         do_export=False, out=None):
    """多点连续抓放 + 静态障碍物绕行。搬运段把方块作为 attachment 一起避障。"""
    n = len(pairs)
    print(f"\n多点任务: {n} 组抓放, {len(obstacles_spec)} 个障碍物")
    bad = []
    for i, pr in enumerate(pairs, 1):
        print(f"  对{i}: pick(PB m)={np.round(pr['pick'], 4).tolist()}"
              f"  place={np.round(pr['place'], 4).tolist()}")
        bad += range_violations(pr["pick"], f"pick{i}")
        bad += range_violations(pr["place"], f"place{i}")
    if bad:
        print("坐标超出可达范围盒, 拒绝执行:\n  " + "\n  ".join(bad))
        return False

    tip_clear_mm = CUBE / 2 * 1000 - REAL_TIP_BEYOND_GRIP_MM
    grasp_off_m = max(0.0, (TABLE_SAFETY_MM - tip_clear_mm) / 1000.0)
    print(f"真实夹爪指尖距桌面余量: {tip_clear_mm + grasp_off_m * 1000:.1f} mm")
    picks_g = [[pr["pick"][0], pr["pick"][1], pr["pick"][2] + grasp_off_m]
               for pr in pairs]
    places_g = [[pr["place"][0], pr["place"][1], pr["place"][2] + grasp_off_m]
                for pr in pairs]

    print(f"\n[1/4] IK 求解全部关键姿态({2 * n + 1} 个 + {2 * n} 条下探链)...")
    down = (0, 0, -1.0)
    # READY 取【拾取区】质心而非全点位质心: 障碍常作为隔断摆在取/放区之间,
    # 全点位质心会正好悬在隔断上方, URDF 幽灵手网格(伸出278mm)会与墙顶误判碰撞。
    ready = [float(np.mean([q[0] for q in picks_g])),
             float(np.mean([q[1] for q in picks_g])),
             max(q[2] for q in picks_g + places_g) + LIFT + READY_EXTRA]
    q_ready, info = ik.solve(robot, ready, down, seed_rad=IK_SEED,
                             axis_tol_deg=30.0)
    if q_ready is None:
        print(f"IK 失败: READY @ {np.round(ready, 3).tolist()}")
        return False

    def hover_and_chain(tag, pt, z_lo_extra):
        hover = [pt[0], pt[1], pt[2] + LIFT]
        q_h, _ = ik.solve(robot, hover, down, seed_rad=q_ready)
        if q_h is None:
            print(f"IK 失败: {tag} 悬停 @ {np.round(hover, 3).tolist()}")
            return None, None
        qs, q_prev = [q_h], q_h
        n_step = max(1, int(round((LIFT - z_lo_extra) / DESCEND_STEP)))
        for k in range(1, n_step + 1):
            z = pt[2] + LIFT - (LIFT - z_lo_extra) * k / n_step
            q, _ = ik.solve(robot, [pt[0], pt[1], z], down, seed_rad=q_prev)
            if q is None:
                print(f"IK 失败: {tag} 下探 z={z:.3f}")
                return None, None
            qs.append(q)
            q_prev = q
        return q_h, qs

    plans = []
    for i, (pg, lg) in enumerate(zip(picks_g, places_g), 1):
        qp_h, down_pick = hover_and_chain(f"pick{i}", pg, 0.0)
        ql_h, down_place = hover_and_chain(f"place{i}", lg, RELEASE_DROP)
        if qp_h is None or ql_h is None:
            return False
        plans.append({"pick_hover": qp_h, "down_pick": down_pick,
                      "place_hover": ql_h, "down_place": down_place})

    print("[2/4] 搭建多点场景...")
    scene = build_scene_multi(robot, pairs, obstacles_spec)
    obstacles = [scene["table"]] + scene["obstacle_bodies"]
    mon = Monitor(robot, obstacles)
    traj = Traj()
    xf.draw_frame(robot, EE, length=0.1, width=2)
    for cube in scene["cubes"]:
        xf.set_robot_object_collision(robot, cube, False)
    _kin_set_rad(robot, HOME)
    traj.record("HOME", HOME)

    print("[3/4] RRT 规划 + 执行...")
    PLAN_METRICS.update(legs=0, time_s=0.0, len_rad=0.0)

    def leg(name, end, attachments=()):
        path = plan_verified(robot, end, obstacles, attachments)
        if path is None:
            print(f"规划失败: {name}")
            return False
        mon.on = True
        play(robot, name, path, traj, mon, gui, attachments)
        mon.on = False
        return True

    if not leg("HOME->READY", q_ready):
        return False
    # 转运段直接 HOVER->HOVER 规划, 不再折返固定 READY 中枢: 经停 READY 会
    # 产生肉眼可见的"回头"(实测 j4 先收 14~17 度再伸回)。READY 只保留进场/收尾。
    if not leg("READY->PICK1_HOVER", plans[0]["pick_hover"]):
        return False
    for i, (pl, pr) in enumerate(zip(plans, pairs), 1):
        cube = scene["cubes"][i - 1]
        table_collision(robot, scene["table"], False)
        kin_seq(robot, f"PICK{i}_DESCEND", pl["down_pick"], traj, mon, gui,
                settle=30)
        grip, _ = ik.grip_state(robot)
        cpos = np.array(p.getBasePositionAndOrientation(cube)[0])
        print(f"   [抓取{i}] 夹爪-方块距离 "
              f"{np.linalg.norm(grip - cpos) * 1000:.1f} mm -> 闭合")
        cid = xf.attach_grasp(robot, cube, ARM)
        p.changeConstraint(cid, maxForce=1e5)
        p.changeDynamics(cube, -1, mass=0.02)
        attachment = pp.create_attachment(robot, EE, cube)
        traj.gripper(f"PICK{i}", "close")
        kin_seq(robot, f"PICK{i}_ASCEND", list(reversed(pl["down_pick"])),
                traj, mon, gui, attachments=(attachment,))
        table_collision(robot, scene["table"], True)

        if not leg(f"PICK{i}_HOVER->PLACE{i}_HOVER", pl["place_hover"],
                   (attachment,)):
            return False

        table_collision(robot, scene["table"], False)
        kin_seq(robot, f"PLACE{i}_DESCEND", pl["down_place"], traj, mon, gui,
                settle=80, attachments=(attachment,))
        traj.gripper(f"PLACE{i}", "open")
        p.changeDynamics(cube, -1, mass=0.2)
        p.removeConstraint(cid)
        p.resetBaseVelocity(cube, [0, 0, 0], [0, 0, 0])
        kin_seq(robot, f"PLACE{i}_SETTLE", [pl["down_place"][-1]], traj, mon,
                gui, settle=120)
        kin_seq(robot, f"PLACE{i}_ASCEND", list(reversed(pl["down_place"])),
                traj, mon, gui)
        table_collision(robot, scene["table"], True)
        # 注意: 不恢复 机器人↔方块 碰撞 —— 幽灵手网格(278mm)会在后续邻近点位
        # 下探时把已放好的方块物理扫走(实测 131mm)。真机的等价约束是任务设计
        # 准则: 相邻点位间距 >=12cm(网格更换为真实夹爪后可收紧)。
        if i < n:
            if not leg(f"PLACE{i}_HOVER->PICK{i + 1}_HOVER",
                       plans[i]["pick_hover"]):
                return False
        elif not leg(f"PLACE{i}_HOVER->READY", q_ready):
            return False
    if not leg("READY->HOME", HOME):
        return False

    print("[4/4] 验证...")
    for _ in range(150):
        _kin_set_rad(robot, HOME)
        p.stepSimulation()
    all_ok = mon.max_hits == 0
    print("\n================= 验证 =================")
    expect_z = scene["table_top"] + CUBE / 2
    for i, (pr, cube) in enumerate(zip(pairs, scene["cubes"]), 1):
        cpos = np.array(p.getBasePositionAndOrientation(cube)[0])
        exy = np.linalg.norm(cpos[:2] - np.array(pr["place"][:2])) * 1000
        ez = abs(cpos[2] - expect_z) * 1000
        good = exy < 20.0 and ez < 10.0
        all_ok &= good
        print(f"方块{i}: 水平偏差 {exy:.1f} mm (阈值 20)  高度偏差 {ez:.1f} mm "
              f"(阈值 10)  {'✓' if good else '✗'}")
    print(f"自由段 机器人/搬运物 vs 桌子/障碍 接触点峰值: {mon.max_hits} (期望 0)")
    print(f"规划指标: {PLAN_METRICS['legs']} 段 RRT, 总耗时 "
          f"{PLAN_METRICS['time_s']:.1f}s, 关节路径总长 "
          f"{PLAN_METRICS['len_rad']:.2f} rad (seed={RUN_SEED}, "
          f"smooth={SMOOTH_ITERS})")
    print("✅ 多点任务达成。" if all_ok else "❌ 未达成, 检查上方数值。")

    if all_ok and do_export:
        meta = {
            "arm_id": PROFILE["arm_id"], "arm": ARM_NAME, "j6_flipped": True,
            "created": datetime.datetime.now().isoformat(timespec="seconds"),
            "task": "multi",
            "pairs_pb_m": [{"pick": list(map(float, pr["pick"])),
                            "place": list(map(float, pr["place"]))}
                           for pr in pairs],
            "pairs_sdk_mm": [{"pick": [round(float(v), 2)
                                       for v in pb_m_to_sdk_mm(pr["pick"])],
                              "place": [round(float(v), 2)
                                        for v in pb_m_to_sdk_mm(pr["place"])]}
                             for pr in pairs],
            "obstacles_pb_m": obstacles_spec,
            "pick_sdk_mm": [round(float(v), 2)
                            for v in pb_m_to_sdk_mm(pairs[0]["pick"])],
            "place_sdk_mm": [round(float(v), 2)
                             for v in pb_m_to_sdk_mm(pairs[-1]["place"])],
            "t_session_used": T_SESSION_KEY,
            "grip_r_link_mm": PROFILE["grip_center_link_mm"],
            "ee_link_id": PROFILE["ee_link_id"],
            "qualification": PROFILE["status"],
            "planner": {"algo": "RRT-Connect(birrt)+shortcut",
                        "smooth_iters": SMOOTH_ITERS, "seed": RUN_SEED,
                        "plan_time_s": round(PLAN_METRICS["time_s"], 2),
                        "path_len_rad": round(PLAN_METRICS["len_rad"], 3)},
            "lift_m": LIFT, "ready_extra_m": READY_EXTRA,
            "export_step_deg": EXPORT_STEP_DEG,
            # 可连续交接的段 = RRT 自由转运段(段名含 "->"); 近物下探/上升/落定
            # 与夹爪事件段一律逐点停稳。执行器按同一判据复核后才允许交接。
            "continuous_candidate": True,
            "continuous_candidate_segments": sorted(
                {it["seg"] for it in traj.items if "->" in it["seg"]}),
            "note": "多点任务: q_sdk_deg 已含 j6 翻转, 逐点 armMoveJoints; "
                    "夹爪事件为 close/open 交替共 %d 次" % (2 * n),
        }
        traj.export(out or os.path.join(_HERE, "trajectories",
                                        "traj_multi_latest.json"), meta)
    elif all_ok:
        print("\n轨迹 JSON: 未生成(需要时加 --export-json 或 --out)。")
    return all_ok


# ---------------------------------------------------------------- 可达扫描
def scan(robot, step=0.04):
    xs = np.arange(RANGE_BOX["x"][0], RANGE_BOX["x"][1] + 1e-9, step)
    ys = np.arange(RANGE_BOX["y"][0], RANGE_BOX["y"][1] + 1e-9, step)
    zs = np.arange(RANGE_BOX["z"][0], RANGE_BOX["z"][1] + 1e-9, step)
    total = ok = 0
    print(f"扫描 {len(xs)}x{len(ys)}x{len(zs)} 网格"
          f"(判据: 抓取点与其上方 {LIFT}m 悬停点都可解):")
    for z in zs:
        print(f"\nz = {z:.2f} m   (行=x {xs[0]:.2f}->{xs[-1]:.2f}, "
              f"列=y {ys[0]:.2f}->{ys[-1]:.2f})")
        for x in xs:
            row = ""
            for y in ys:
                total += 1
                q1, _ = ik.solve(robot, [x, y, z], iters=150)
                q2, _ = (ik.solve(robot, [x, y, z + LIFT], seed_rad=q1, iters=150)
                         if q1 else (None, None))
                good = q1 is not None and q2 is not None
                ok += good
                row += " ■" if good else " ·"
            print(f"  x={x:.2f} {row}")
    print(f"\n可达率: {ok}/{total} = {ok / total * 100:.0f}%")


# ---------------------------------------------------------------- 入口
def build_arg_parser():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick", type=float, nargs=3, default=[0.36, 0.30, 1.00],
                    metavar=("X", "Y", "Z"), help="抓取坐标(PB 世界系, m)")
    ap.add_argument("--place", type=float, nargs=3, default=[0.36, 0.16, 1.00],
                    metavar=("X", "Y", "Z"), help="放置坐标(PB 世界系, m)")
    ap.add_argument("--pick-sdk", type=float, nargs=3, default=None,
                    help="抓取坐标(SDK 世界系, mm), 优先于 --pick")
    ap.add_argument("--place-sdk", type=float, nargs=3, default=None)
    ap.add_argument("--task", default=None, metavar="TASK_JSON",
                    help="多点任务文件(pairs+obstacles, PB 米); 给出后忽略 "
                         "--pick/--place")
    ap.add_argument("--direct", action="store_true", help="无界面自测")
    ap.add_argument("--scan", action="store_true", help="可达范围扫描后退出")
    export_group = ap.add_mutually_exclusive_group()
    export_group.add_argument("--export-json", action="store_true",
                              help="仿真验证通过后导出轨迹 JSON(默认不导出)")
    export_group.add_argument("--no-export", action="store_true",
                              help="兼容旧命令; 当前默认已不导出")
    ap.add_argument("--out", default=None,
                    help="轨迹输出路径; 指定本项也会开启 JSON 导出")
    ap.add_argument("--seed", type=int, default=None,
                    help="固定随机种子: 同一任务生成完全相同的轨迹(可复现)")
    ap.add_argument("--arm", choices=("left", "right"), default="left",
                    help="规划臂；右臂只能生成 CANDIDATE，真机门默认阻断")
    return ap


def main(argv=None):
    ap = build_arg_parser()
    args = ap.parse_args(argv)
    if args.no_export and args.out:
        ap.error("--no-export 不能与 --out 同时使用")

    set_arm(args.arm)

    global RUN_SEED
    if args.seed is not None:
        RUN_SEED = args.seed
        random.seed(args.seed)          # pybullet_planning 的 RRT 用 random 模块
        np.random.seed(args.seed)

    gui = not args.direct and not args.scan
    robot = xf.load_xifeng(gui=gui)
    global DISABLED
    DISABLED = compute_disabled_pairs(robot)

    if args.scan:
        scan(robot)
        p.disconnect()
        return

    do_export = (args.export_json or args.out is not None) and not args.no_export
    if args.task:
        pairs, obstacles_spec = load_task_file(args.task)
        ok = pick_and_place_multi(robot, pairs, obstacles_spec, gui,
                                  do_export=do_export, out=args.out)
    else:
        pick = (sdk_to_pb_m(args.pick_sdk).tolist() if args.pick_sdk
                else args.pick)
        place = (sdk_to_pb_m(args.place_sdk).tolist() if args.place_sdk
                 else args.place)
        ok = pick_and_place(robot, pick, place, gui, do_export=do_export,
                            out=args.out)
    if gui:
        print("\n(关闭窗口结束)")
        while p.isConnected():
            _kin_set_rad(robot, HOME)
            p.stepSimulation()
            time.sleep(1 / 240)
    p.disconnect()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
