#!/usr/bin/env python3
"""世界坐标驱动的抓取(move worlds 主线)。

与 pick_place_coord.py(关节空间 RRT + armMoveJoints)并列的新管线:
给 pick/place 世界坐标 → 生成一串【工具朝下】的世界位姿(悬停/下探/抓/抬/放)
→ 位置直线加密 → 逐点 IK 求关节角(仅用于仿真可视化+可达验证)→ PyBullet 动画
→ 导出 SDK 世界坐标轨迹 [X,Y,Z,U,V,W], 真机用 armWorldsToServo(世界伺服流式,
平滑) 或 armMoveWorlds 回放。真机侧自己从世界位姿解 IK。

用法:
  python3 world_grasp.py --pick 0.36 0.30 1.05 --place 0.36 0.16 1.05        # GUI
  python3 world_grasp.py --pick ... --place ... --direct                     # 无界面自检
  python3 world_grasp.py --pick ... --place ... --export-json                # 导出世界轨迹
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
DOWN = (0.0, 0.0, -1.0)                  # 工具轴期望朝向(世界 -Z)

HOVER = 0.12                             # 抓取点上方悬停高度(m); "桌面局部"取小
CUBE = 0.05
STEP_MM = 8.0                            # 世界直线加密步长(mm, 仅用于仿真动画/IK 复核)
ARC_POINTS = 12                          # 转运弧的角点数(转角≈180/n, 12->15°, 可混合)
# 浅弧的弧顶抬高(m)。armMoveWorldsCurve 的圆弧由【起点/中点/终点三点】唯一确定:
# 弧顶抬得越高, 定出的圆越小、扫过的角度越大。实测抬 120mm -> 半径 81mm / 扫角
# 241°(等于绕小圆走一大圈, 真机就卡在这里); 抬 25mm -> 扫角约 90°, 平缓可靠。
# 抬升/下降改由前后的竖直直线段单独负责, 弧只做水平转运。
ARC_APEX = 0.025
# 转运段怎么走。20260729 真机现场结论确立的分工原则:
#   "大范围转运用关节空间, 尽可能少的点; 抓放这种贴着直线的短程用世界坐标直线"
#
#   "line"  【默认/主线, 20260729 回退至此】转运走【逐轴】armMoveWorlds 直线
#           (悬停面上, X 一条 Y 一条; 只有一个方向有位移时就一条)。与已三次真机
#           成功的 ARC 版是同一类命令, 只是把 12 小步并成 1~2 大步。几何完全可
#           预测, 每条命令只有一个坐标在变。
#   "joint" [暂停使用, 未验证] 转运一条 armMoveJoints。原理上更快更连贯, 但
#           20260729 真机连续 3 次失败: 1 次限位标准冲突, 2 次 armMoveJoints
#           25s 未到位(残差 18.5°/23.7°)。失败都发生在【夹爪已闭合、物体在手】
#           之后。要继续研究先用空爪诊断脚本, 不要带物重复跑。
#   "arc"   [留档勿用] 12 个 ARC 点。真机 20260729 三次成功但【又慢又顿】——
#           12 个点各是一条独立 armMoveWorlds + 等停稳, 就是卡顿的来源。
TRANSFER_MODE = "line"
RELEASE_DROP = 0.006                     # 放置松开高度(m)
# IK 首点种子: "下探姿态"(URDF 度)。工具朝下(Y轴向下)相对直臂是大幅重定向,
# 从直臂种子 DLS 会卡在局部解(实测停在 197mm); 用这个下探形状做种子可稳定收敛
# (实测 0.15mm/0.0° 竖直)。后续点链式沿用上一解, 都在同一basin内。
SEED_DEG = [-80, 40, 120, -80, -70, 0, 40]
T_MM = cc.CONFIRMED_T_SESSIONS_MM["20260707"]

RANGE_BOX = {"x": (0.28, 0.48), "y": (0.14, 0.40), "z": (0.95, 1.10)}

# 真机实测 tool-down 世界姿态(20260723 capture_tool_down_pose): V=90=工具竖直朝下,
# W=0, U=-20.08(万向锁下 U 编码绕竖直的偏航)。导出仅作记录, 真机执行以现场读到的
# anchor.UVW 为准(工具朝下时近似恒定)。约定已用两组真机样本核实: R_sdk=R11·Rz(90),
# 标准 RPY; VR初始样本解算与真机吻合 0.1°, tool-down 样本 U-W 不变量吻合(-20.1)。
TOOL_DOWN_UVW = [-20.08, 90.0, 0.0]


# ---------------------------------------------------------------- 坐标/姿态换算
def pb_to_sdk_mm(xyz_m):
    return (np.asarray(xyz_m, float) * 1000.0 + np.asarray(T_MM, float)).tolist()


def _rotz(deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def matrix_to_uvw(R):
    """R = Rz(W)·Ry(V)·Rx(U) 标准 RPY 分解 -> (U,V,W) 度。"""
    V = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    W = math.atan2(R[1, 0], R[0, 0])
    U = math.atan2(R[2, 1], R[2, 2])
    return [math.degrees(U), math.degrees(V), math.degrees(W)]


def sdk_world_pose(robot, q_rad, pos_pb_m):
    """由 IK 关节角的 FK 求末端姿态 -> SDK 世界位姿 [X,Y,Z(mm), U,V,W(deg)]。
    R_sdk = R_link11·Rz(90°)(标定结论); 位置用 T 换算。
    注意: UVW 依赖重建标定, 真机首测须确认工具确实朝下, 必要时用起点 armGetWorlds
    的 UVW 覆盖(工具朝下时 UVW 近似恒定)。"""
    for j, v in zip(ARM, q_rad):
        p.resetJointState(robot, j, float(v))
    st = p.getLinkState(robot, EE, computeForwardKinematics=True)
    R11 = np.array(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
    R_sdk = R11 @ _rotz(90.0)
    return pb_to_sdk_mm(pos_pb_m) + matrix_to_uvw(R_sdk)


# ---------------------------------------------------------------- 世界位姿配方
def range_violations(pt, name):
    bad = []
    for i, k in enumerate("xyz"):
        lo, hi = RANGE_BOX[k]
        if not (lo - 1e-9 <= pt[i] <= hi + 1e-9):
            bad.append(f"{name}.{k}={pt[i]:.3f} 越界 [{lo},{hi}]")
    return bad


def arc_between(a, b, apex_h, n):
    """从 a 到 b 的半椭圆弧(竖直平面内), 返回 n 个点(不含 a, 含 b)。

    为什么用弧(20260725 真机实证的结论): armMoveWorlds 的 interpolation_en
    平滑衔接【会切拐角】—— 实测 90° 拐角被切 64mm, 180° 掉头被整个吞掉。
    直角形("垂直升→水平移→垂直降")的拐角太尖, 只能逐个停稳, 无法真正连贯。
    改成半椭圆弧后每个拐角只有 ~180/n 度(n=12 时 15°), 落在混合门限内 ->
    控制器可以真正连续过弯, 且切角量极小(浅拐角切一点点不影响几何)。

    形状: 水平位移用 s(t)=(1-cos(πt))/2, 高度叠加 apex·sin(πt)。
    两端导数: t→0/1 时 ds/dt=0 而 dz/dt 最大 => 【垂直抬离、垂直落下】,
    抓取/放置的净空与直角形完全一致, 只有中间变成圆滑过渡。"""
    pts = []
    for k in range(1, n + 1):
        t = k / n
        s = (1 - math.cos(math.pi * t)) / 2
        pts.append([a[0] + (b[0] - a[0]) * s,
                    a[1] + (b[1] - a[1]) * s,
                    a[2] + (b[2] - a[2]) * s + apex_h * math.sin(math.pi * t)])
    return pts


def build_world_recipe(pick, place, arc_n=ARC_POINTS, arc_apex=ARC_APEX):
    """工具朝下的世界位姿配方(角点)。move=位置点; gripper=在当前位置的夹爪事件。

    ⚠ 20260727 结构性修改(真机卡死复盘):
    旧版让【一条弧从抓取点直接划到放置点】, 弧顶抬高 = HOVER(120mm)。但
    armMoveWorldsCurve 的圆弧由【起点/中点/终点三点唯一确定】—— 实测这三点
    定出的圆半径仅 81mm 却要扫过 241°, 即控制器被要求绕着小圆走一大圈。
    这极可能就是"跑到一半卡住不动"的原因, 也让关节(尤其 j2)被逼到限位。

    ⚠ 20260728 再次修改(弧线定性为不可用):
    浅弧仍然跑不动 —— 实测 armMoveWorldsCurve 返回码 0, 但手臂【一动没动】
    (位置差 = 整段行程 140mm, move_state=False), 而同一次运行里 3 条
    armMoveWorlds 直线全部到位(0.6~2.0mm)。故转运改为【单条水平直线】:
      抬升/下降  -> 竖直直线段
      横移       -> 悬停面上的一条水平直线(物体已离桌面 HOVER=120mm)
    全程 6 条 armMoveWorlds, 没有任何弧线命令。弧线代码留档但默认关闭。

    ⚠ 20260729 三次真机成功后的现场结论(本版):
    ARC 12 点版跑通了(误差 0.8~3.9mm), 但【转运又慢又顿】—— 12 个点各是一条
    独立 armMoveWorlds + 等停稳; 而 PICK/PLACE 的 ascend/descend 是单条长直线,
    "又快又丝滑"。由此确立分工原则:
      大范围转运  -> 关节空间(armMoveJoints), 路点越少越好
      抓放短程    -> 世界坐标直线(armMoveWorlds), 几何确定
    ⚠ 20260729 晚(hybrid 三连败后回退, 本版):
    "转运走 armMoveJoints"真机连续三次没成: 一次限位标准冲突, 两次 25s 未到位
    (残差 23.7°/18.5°), 且都发生在物体已抓在手上之后。关节空间那条路暂时封存。
    现在转运回到 armMoveWorlds 直线, 但按【单轴】拆分 —— 每条命令只有一个坐标
    在变, 几何完全可预测, 与三次成功的 ARC 版是同一类命令, 只是把 12 小步并成
    1~2 大步。"""
    hp = [pick[0], pick[1], pick[2] + HOVER]
    dp = [pick[0], pick[1], pick[2]]
    hq = [place[0], place[1], place[2] + HOVER]
    dq = [place[0], place[1], place[2] + RELEASE_DROP]
    steps = [
        {"seg": "START", "pos": hp},
        {"seg": "PICK_DESCEND", "pos": dp},
        {"seg": "PICK", "gripper": "close"},
        {"seg": "PICK_ASCEND", "pos": hp},          # 竖直抬升(直线)
    ]
    arc = arc_between(hp, hq, arc_apex, arc_n)       # 弧上 n 个点(不含起点 hp)
    if TRANSFER_MODE == "arc":                       # 留档: 12 点逐条直线, 慢且顿
        for i, q in enumerate(arc, start=1):
            steps.append({"seg": f"ARC{i:02d}", "pos": q})
    elif TRANSFER_MODE == "line":
        # 悬停面 -> 悬停面, 【逐轴】直线。物体已离桌面 HOVER, 横移安全。
        # 为什么拆成单轴(20260729 决定): 单轴位移的几何完全可预测 —— 每条命令
        # 只有一个坐标在变, 出问题时一眼看得出是哪个方向没走完; 而且和已经三次
        # 真机成功的 ARC 版走的是同一类命令(armMoveWorlds 直线), 只是把 12 小步
        # 并成 1~2 大步。X/Y 只有一个方向有位移时自然退化成一条命令。
        cur = list(hp)
        for axis, name in ((0, "TRANSFER_X"), (1, "TRANSFER_Y")):
            if abs(hq[axis] - cur[axis]) > 1e-6:
                cur[axis] = hq[axis]
                steps.append({"seg": name, "pos": list(cur)})
        if abs(hq[2] - cur[2]) > 1e-6:               # 抓放面不等高时才需要
            steps.append({"seg": "TRANSFER_Z", "pos": list(hq)})
    elif TRANSFER_MODE == "joint":
        # 转运只留【一个】路点: 弧末(原 ARC12)= 放置点上方的悬停点。中间 11 个
        # 点全删 —— 真机从 PICK_ASCEND 的关节位形一条 armMoveJoints 直接插过去。
        #
        # 20260729 二次精简: 上一版还留了个 TRANSFER_START(原 ARC01), 目的是比
        # 悬停面再抬 6.5mm 换点净空。但仿真复核显示关节插值这一段是【向上鼓 4mm】
        # 而不是塌下去, 那 6.5mm 纯属多余, 却要多花一条命令和一次完整加减速。
        # 删掉后转运彻底变成"一条命令"。
        #
        # ⚠ 关节插值的笛卡尔轨迹【不是直线】, 会鼓也会塌。塌下去就会拖着物体
        # 蹭桌面。所以生成侧每次都必须仿真复核这一段的最低点(joint_transfer_probe),
        # 执行侧必须靠预检读到的真实关节角来走, 不能拍脑袋。
        steps.append({"seg": "TRANSFER_END", "pos": arc[-1], "via": "joints"})
    else:
        raise ValueError(f"TRANSFER_MODE 只能是 joint/line/arc, 得到 {TRANSFER_MODE!r}")
    steps += [
        {"seg": "PLACE_DESCEND", "pos": dq},        # 竖直下降(直线)
        {"seg": "PLACE", "gripper": "open"},
        {"seg": "PLACE_ASCEND", "pos": hq},
    ]
    return steps


def densify_cartesian(steps, step_mm=STEP_MM):
    """角点之间按 step_mm 直线插值(位置); 夹爪事件原样保留在当前位置。

    via="joints" 的路点【不做笛卡尔加密】—— 真机那一段走 armMoveJoints, 末端
    的实际轨迹由关节插值决定, 不是直线。硬按直线加密只会画出一条骗人的路径。
    它的真实形状由 expand_joint_transfer() 在 IK 之后展开。"""
    dense, last = [], None
    for s in steps:
        if "gripper" in s:
            dense.append(dict(s))
            continue
        pos = s["pos"]
        if s.get("via") == "joints":
            dense.append({"seg": s["seg"], "pos": list(pos), "via": "joints"})
        elif last is None:
            dense.append({"seg": s["seg"], "pos": list(pos)})
        else:
            span_mm = np.linalg.norm(np.array(pos) - np.array(last)) * 1000.0
            n = max(1, int(math.ceil(span_mm / step_mm)))
            for k in range(1, n + 1):
                q = [a + (b - a) * k / n for a, b in zip(last, pos)]
                dense.append({"seg": s["seg"], "pos": q})
        last = pos
    return dense


def expand_joint_transfer(robot, seq, step_deg=2.0):
    """把 via="joints" 的路点展开成【关节空间插值】的中间态, 仅用于仿真显示与
    净空复核。

    真机在这一段发的是一条 armMoveJoints: 控制器在关节空间线性插值, 末端的
    笛卡尔轨迹既不是直线也不一定单调 —— 可能鼓起来, 也可能塌下去。塌下去就会
    拖着物体蹭桌面(Track B 的 4 点版就是这么翻车的)。这里用同样的插值方式 +
    正解, 把真机会走的形状原样还原出来。"""
    out, prev_q = [], None
    for s in seq:
        if s.get("via") == "joints" and prev_q is not None and s.get("q") is not None:
            a, b = prev_q, s["q"]
            span = max(abs(math.degrees(y - x)) for x, y in zip(a, b))
            n = max(2, int(math.ceil(span / step_deg)))
            for k in range(1, n + 1):
                q = [x + (y - x) * k / n for x, y in zip(a, b)]
                for j, v in zip(ARM, q):
                    p.resetJointState(robot, j, float(v))
                out.append({"seg": s["seg"], "pos": ik.grip_state(robot)[0].tolist(),
                            "q": q, "joint_interp": k < n,
                            **({"via": "joints"} if k == n else {})})
            prev_q = b
            continue
        out.append(s)
        if s.get("q") is not None:
            prev_q = s["q"]
    return out


def joint_transfer_probe(seq, table_top, cube=CUBE):
    """复核关节空间转运段: 最低点净空 + 相对直线的最大偏离。

    抓着物体时, 物体底面 = 夹爪中心 - cube/2。只要这个值还在桌面之上就安全。"""
    pts = [s["pos"] for s in seq if s.get("seg", "").startswith("TRANSFER_END")]
    if len(pts) < 2:
        return None
    a, b = np.array(pts[0]), np.array(pts[-1])
    lo = min(q[2] for q in pts)
    seg = b - a
    devs = [float(np.linalg.norm(np.cross(seg, np.array(q) - a)) / np.linalg.norm(seg))
            for q in pts] if np.linalg.norm(seg) > 1e-9 else [0.0]
    return {"n": len(pts), "min_z": lo,
            "clear_mm": (lo - cube / 2 - table_top) * 1000.0,
            "bow_mm": max(devs) * 1000.0,
            "peak_mm": (max(q[2] for q in pts) - min(a[2], b[2])) * 1000.0}


# ---------------------------------------------------------------- 场景/动画
def build_scene(robot, pick, place):
    table_top = pick[2] - CUBE / 2
    cx, cy = (pick[0] + place[0]) / 2, (pick[1] + place[1]) / 2
    half = [abs(pick[0] - place[0]) / 2 + 0.15,
            abs(pick[1] - place[1]) / 2 + 0.15, 0.02]
    p.createMultiBody(
        0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
        p.createVisualShape(p.GEOM_BOX, halfExtents=half,
                            rgbaColor=[0.82, 0.7, 0.5, 1]),
        [cx, cy, table_top - half[2]])
    cube = p.createMultiBody(
        0, baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_BOX, halfExtents=[CUBE / 2] * 3, rgbaColor=[0.9, 0.2, 0.2, 1]),
        basePosition=[pick[0], pick[1], table_top + CUBE / 2])
    marker = p.createMultiBody(
        0, baseVisualShapeIndex=p.createVisualShape(
            p.GEOM_CYLINDER, radius=0.045, length=0.002,
            rgbaColor=[0.2, 0.8, 0.4, 0.6]),
        basePosition=[place[0], place[1], table_top + 0.001])
    return {"cube": cube, "marker": marker, "table_top": table_top}


def solve_all(robot, dense):
    """逐点 IK, 链式种子。返回 (带 q 的序列, 不可达列表)。"""
    seed = [math.radians(v) for v in SEED_DEG]
    out, bad = [], []
    for i, d in enumerate(dense):
        if "gripper" in d:
            out.append(dict(d))
            continue
        q, info = ik.solve(robot, d["pos"], DOWN, seed_rad=seed)
        via = {"via": d["via"]} if "via" in d else {}       # via 标记必须透传下去
        if q is None:
            bad.append((i, d["seg"], d["pos"], info))
            out.append({"seg": d["seg"], "pos": d["pos"], "q": None, **via})
        else:
            seed = q
            out.append({"seg": d["seg"], "pos": d["pos"], "q": q, **via})
    return out, bad


def animate(robot, seq, cube, table_top, gui):
    """纯运动学播放(不跑动力学, 躯干等非臂关节保持零位不漂移)。
    抓取中: 方块吸附到 grip 中心; 松开: 方块落到桌面正确高度。"""
    grasped = False
    for step in seq:
        if step.get("gripper") == "close":
            grasped = True
            continue
        if step.get("gripper") == "open":
            grasped = False
            grip, _ = ik.grip_state(robot)       # 松开: 方块落到当前 x,y 的桌面
            p.resetBasePositionAndOrientation(
                cube, [grip[0], grip[1], table_top + CUBE / 2], [0, 0, 0, 1])
            continue
        if step["q"] is None:
            continue
        for j, v in zip(ARM, step["q"]):
            p.resetJointState(robot, j, float(v))
        if grasped:
            grip, _ = ik.grip_state(robot)
            p.resetBasePositionAndOrientation(cube, grip.tolist(), [0, 0, 0, 1])
        if gui:
            time.sleep(1 / 120)                  # 仅节拍, GUI 自动渲染, 不 stepSimulation


# ---------------------------------------------------------------- 导出
def export_world_traj(robot, seq, pick, place, path):
    """锚点+相对偏移导出(免标定): 真机把臂摆成"工具朝下悬停在物体正上方"读一次
    armGetWorlds 得 anchor; 每点世界坐标 = anchor.xyz + off_mm, 姿态 = anchor.uvw
    (工具朝下固定)。PB 与 SDK 世界系坐标轴对齐(仅差平移 T), 故相对偏移免 T 标定;
    姿态用真机实测 tool-down UVW, 绕开重建标定不确定。"""
    anchor_pb = np.array(pick, float)                # anchor = 抓取点(工具朝下放到物体处)
    waypoints = []
    for step in seq:
        if step.get("joint_interp"):
            continue          # 关节插值中间态只用于仿真, 真机由控制器自己插
        if step.get("gripper"):
            waypoints.append({"seg": step["seg"], "gripper": step["gripper"]})
        elif step["q"] is not None:
            off = (np.array(step["pos"]) - anchor_pb) * 1000.0   # 相对抓取点的偏移
            w = {"seg": step["seg"], "off_mm": [round(float(v), 2) for v in off]}
            if step.get("via") == "joints":
                w["via"] = "joints"
            waypoints.append(w)
    meta = {
        "task": "world_grasp_single_relative",
        "created": datetime.datetime.now().isoformat(timespec="seconds"),
        "arm_id": 1, "arm": "left", "tool": "down",
        "pick_pb_m": list(map(float, pick)), "place_pb_m": list(map(float, place)),
        "anchor_desc": "anchor = 工具朝下放在【物体抓取点】处; 真机把张开的夹爪竖直"
                       "放到物体抓取位, 读 armGetWorlds 得 anchor[X,Y,Z,U,V,W]。"
                       "off_mm 相对此点: START 在其上方 %.0fmm" % (HOVER * 1000),
        "anchor_uvw_captured": TOOL_DOWN_UVW,
        "exec_contract": "每点 world = [anchor.X+off.x, anchor.Y+off.y, anchor.Z+off.z, "
                         "anchor.U, anchor.V, anchor.W]; off_mm 免标定(坐标轴对齐), "
                         "姿态全程用 anchor 的 tool-down UVW(固定夹爪朝向)",
        "step_mm": STEP_MM, "hover_m": HOVER,
        "off_format": "off_mm = 相对 START 的世界系偏移 [dx,dy,dz](mm)",
        "transfer": TRANSFER_MODE,
        "via_contract": "路点带 via=\"joints\" 时: 不下发 armMoveWorlds, 改用一条 "
                        "armMoveJoints 到【限位预检时在该拐角读到的真实关节角】。"
                        "SDK 无逆解接口, 关节角只能这样现场取, 因此 hybrid 模式下"
                        "预检是必需步骤, 不可关闭。",
        "exec_hint": "真机 MODE=hybrid: 大范围转运走关节空间(1 条 armMoveJoints), "
                     "抓放的 ascend/descend 走世界坐标直线(armMoveWorlds)。"
                     "全程 5 条直线 + 1 条关节 + 2 次夹爪。"
                     "armMoveWorldsCurve 实测不执行, 勿用。",
    }
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        json.dump({"meta": meta, "waypoints": waypoints}, f, indent=1,
                  ensure_ascii=False)
    n = sum(1 for w in waypoints if "off_mm" in w)
    print(f"\n世界轨迹已导出({n} 个相对偏移点 + "
          f"{len(waypoints) - n} 个夹爪事件): {os.path.relpath(path, _WORK)}")
    print(f"  真机契约: 摆'工具朝下悬停物体上方'读 anchor, 每点 = anchor + off_mm, "
          f"姿态用 anchor 的 tool-down UVW")


# ---------------------------------------------------------------- 主流程
def run(pick, place, gui, do_export, out):
    print(f"\n世界坐标抓取: pick={np.round(pick, 4).tolist()}  "
          f"place={np.round(place, 4).tolist()}")
    bad = range_violations(pick, "pick") + range_violations(place, "place")
    if bad:
        print("坐标越界, 拒绝执行:\n  " + "\n  ".join(bad))
        return False

    steps = build_world_recipe(pick, place)
    dense = densify_cartesian(steps)
    n_move = sum(1 for d in dense if "pos" in d)
    print(f"世界位姿: {len(steps)} 角点 -> {n_move} 个加密点(步长 {STEP_MM}mm), "
          f"{sum(1 for d in dense if 'gripper' in d)} 个夹爪事件")

    robot = xf.load_xifeng(gui=gui)
    scene = build_scene(robot, pick, place)

    seq, unreachable = solve_all(robot, dense)
    seq = expand_joint_transfer(robot, seq)
    probe = joint_transfer_probe(seq, scene["table_top"])
    if probe:
        flag = "✓" if probe["clear_mm"] > 20.0 else "✗ 太低, 会拖桌面"
        print(f"关节空间转运段: {probe['n']} 个插值态  "
              f"物体底面最低净空 {probe['clear_mm']:.0f}mm {flag}\n"
              f"  相对直线最大鼓起 {probe['bow_mm']:.0f}mm, "
              f"最高点高出两端 {probe['peak_mm']:.0f}mm")
        if probe["clear_mm"] <= 20.0:
            unreachable = list(unreachable) + [
                (-1, "TRANSFER_END", [0, 0, probe["min_z"]],
                 f"关节插值最低点净空仅 {probe['clear_mm']:.0f}mm")]
    if unreachable:
        print(f"\n❌ {len(unreachable)} 个世界点 IK 不可达:")
        for i, seg, pos, info in unreachable[:8]:
            print(f"  #{i} {seg} @ {np.round(pos, 3).tolist()}  {info}")
        print("(调整 pick/place 高度或范围; 工具朝下在桌面高度须可达)")
        if not gui:
            p.disconnect()
            return False

    xf.draw_frame(robot, EE, length=0.1, width=2)
    animate(robot, seq, scene["cube"], scene["table_top"], gui)

    cpos = np.array(p.getBasePositionAndOrientation(scene["cube"])[0])
    expect = np.array([place[0], place[1], scene["table_top"] + CUBE / 2])
    err_mm = np.linalg.norm(cpos - expect) * 1000
    ok = not unreachable and err_mm < 30.0
    print(f"\n方块落点偏差: {err_mm:.1f} mm (阈值 30)  {'✓' if ok else '✗'}")
    print("✅ 世界坐标抓取仿真达成。" if ok else "❌ 未达成, 见上方。")

    if ok and do_export:
        export_world_traj(robot, seq, pick, place,
                          out or os.path.join(_HERE, "trajectories",
                                              "world_grasp_latest.json"))

    if gui:
        print("\n(关闭窗口结束)")
        while p.isConnected():
            time.sleep(1 / 60)          # 纯保活, 不跑动力学
    p.disconnect()
    return ok


def build_arg_parser():
    ap = argparse.ArgumentParser(description="世界坐标驱动的抓取仿真+导出")
    ap.add_argument("--pick", type=float, nargs=3, default=[0.36, 0.30, 1.05],
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--place", type=float, nargs=3, default=[0.36, 0.16, 1.05],
                    metavar=("X", "Y", "Z"))
    ap.add_argument("--direct", action="store_true", help="无界面自检")
    ap.add_argument("--export-json", action="store_true", help="导出世界轨迹 JSON")
    ap.add_argument("--out", default=None)
    return ap


def main(argv=None):
    args = build_arg_parser().parse_args(argv)
    gui = not args.direct
    ok = run(args.pick, args.place, gui,
             do_export=args.export_json or args.out is not None, out=args.out)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
