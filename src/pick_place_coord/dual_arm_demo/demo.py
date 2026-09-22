#!/usr/bin/env python3
"""双臂分阶段抓放的运动学 GUI Demo；没有 SDK、联网或真机执行入口。

沿用 A 的抓取前导，重新规划左右分区放回桌面的路线，右臂解镜像末端位姿 IK。
两臂同时运动，不以串行错峰代替互相避碰。同步关节插值是显示假设，
不代表厂商 MoveJ / MoveWorlds 的实际插值或两控制器同步能力。
"""
from __future__ import annotations

import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np
import pybullet as p
import pybullet_data

HERE = Path(__file__).resolve().parent
WORK = HERE.parents[1]
sys.path[:0] = [str(WORK), str(HERE.parent), str(WORK / "frame_calibration/analysis")]
import arm_profiles as ap
import calib_common as cc
from gen_right_from_left_mirror import MIRROR_M, SWAP_P, solve_pose

SOURCE = WORK / "pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json"
PLAN = HERE / "dual_arm_simulation.json"
REPORT = HERE / "validation_report.json"
ARMS = ("left", "right")
COLORS = {"left": [0.12, 0.58, 0.94, 1], "right": [1.0, 0.48, 0.12, 1]}
PROFILE_PATH = WORK / "arm_profiles.v1.json"
# 独立演示绑定仓库配置，避免环境变量悄悄换臂/改变源身份。
PROFILES = {arm: ap.arm_profile(arm, PROFILE_PATH) for arm in ARMS}
SHARED = ap.load_profiles(PROFILE_PATH)["shared"]
DT = SHARED["pulse_period_ms"] / 1000


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def save_json(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def load_robot(gui=False):
    cid = p.connect(p.GUI if gui else p.DIRECT)
    if cid < 0:
        raise RuntimeError("PyBullet 连接失败")
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    robot = p.loadURDF(cc.DEFAULT_URDF, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)
    if gui:
        p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)
        p.resetDebugVisualizerCamera(2.05, 90, -28, [0.24, 0, 0.96])
    return robot


def set_q(robot, arm, q):
    profile = PROFILES[arm]
    for joint, angle in zip(profile["joint_ids"], ap.sdk_to_urdf_deg(profile, q)):
        p.resetJointState(robot, joint, math.radians(angle))


def tool(robot, arm, q=None):
    profile = PROFILES[arm]
    if q is not None:
        set_q(robot, arm, q)
    state = p.getLinkState(robot, profile["ee_link_id"], computeForwardKinematics=True)
    rotation = np.array(p.getMatrixFromQuaternion(state[5])).reshape(3, 3)
    origin = np.array(state[4])
    grip = origin + rotation @ (np.array(profile["grip_center_link_mm"]) / 1000)
    return grip, rotation, origin, state[5]


def pose_ik(robot, arm, pos, rotation, seed_sdk):
    pr = PROFILES[arm]
    limits = np.array(pr.get("controller_limits_deg") or pr["candidate_nominal_urdf_limits_deg"], float)
    # 所有新解都保留共享余量；右臂依然只是标称范围，不能冒充实测硬限位。
    limits[:, 0] += SHARED["minimum_joint_margin_deg"]
    limits[:, 1] -= SHARED["minimum_joint_margin_deg"]
    mapped = limits * np.array(pr["sdk_from_urdf_sign"])[:, None]
    lo, hi = np.deg2rad(mapped.min(axis=1)), np.deg2rad(mapped.max(axis=1))
    seed = np.deg2rad(ap.sdk_to_urdf_deg(pr, seed_sdk))
    q, residual = solve_pose(robot, pr["joint_ids"], pr["ee_link_id"],
                             np.array(pr["grip_center_link_mm"]) / 1000,
                             lo, hi, pos, rotation,
                             np.clip(seed, lo, hi), pos_tol_m=0.0002,
                             ori_tol_deg=0.2, iters=500)
    if q is None:
        raise RuntimeError(f"{arm} IK 失败，不输出轨迹: {residual}")
    sdk_q = ap.urdf_to_sdk_deg(pr, np.rad2deg(q).tolist())
    return sdk_q, residual


def right_ik(robot, left_q, seed_sdk=None):
    pos, rotation, _, _ = tool(robot, "left", left_q)
    seed = (-np.array(left_q)).tolist() if seed_sdk is None else seed_sdk
    return pose_ik(robot, "right", MIRROR_M @ pos, MIRROR_M @ rotation @ SWAP_P, seed)


def build_reference(robot, source):
    """仅复用抓取前导；拿起后保持抓取姿态，在自己的半边桌面放下。"""
    rows = []
    for index, row in enumerate(source["offline_replay"]["frames"]):
        rows.append({**row, "source_index": index, "derivation": "PRESERVED_A_PICK_PREFIX"})
        if row.get("action") == "close":
            break
    q = next(r["q_sdk_deg"] for r in reversed(rows) if "q_sdk_deg" in r)
    pick, rotation, _, _ = tool(robot, "left", q)
    position = pick.copy()
    # [SIM ONLY] 新任务参数，不是复制的安全常量；不使用历史料框作为目标。
    place = pick + np.array([0.100, -0.040, 0.012])
    hover_z = pick[2] + 0.150
    targets = [("PICK_ASCEND", [pick[0], pick[1], hover_z]),
               ("PLACE_HOVER", [place[0], place[1], hover_z]),
               ("PLACE_DESCEND", place),
               ("PLACE_ASCEND", [place[0], place[1], pick[2] + 0.200]),
               ("RETREAT_CLEAR_OBJECT", [place[0] - 0.150, place[1], pick[2] + 0.200])]
    residuals = []
    spatial_step = min(0.005, source["offline_replay"]["worlds_sample_mm"] / 1000)
    for name, target in targets:
        target = np.asarray(target)
        n = max(1, math.ceil(float(np.linalg.norm(target - position)) / spatial_step))
        for t in np.linspace(0, 1, n + 1)[1:]:
            q, err = pose_ik(robot, "left", position + t * (target - position), rotation, q)
            residuals.append(err)
            rows.append({"stage": name, "q_sdk_deg": q, "source_index": None,
                         "derivation": "NEW_PARTITIONED_CARTESIAN_IK"})
        position = target
        if name == "PLACE_DESCEND":
            rows.append({"stage": "OPEN_AT_PLACE", "action": "open", "source_index": None,
                         "derivation": "NEW_TABLETOP_RELEASE"})
    # 先升高、退离已放物，再到 Home 上方；原直接 MoveJ 回程会扫到已放瓶的手部网格。
    home = source["home_joints_sdk_deg"]
    home_pos, home_rotation, _, _ = tool(robot, "left", home)
    home_hover = home_pos + [0, 0, .200]
    q, err = pose_ik(robot, "left", home_hover, home_rotation, home)
    residuals.append(err)
    rows.append({"stage": "RETURN_HOME_HOVER", "q_sdk_deg": q, "source_index": None,
                 "derivation": "HOME_POSE_IK_ABOVE_PLACED_OBJECT"})
    n = max(1, math.ceil(.200 / spatial_step))
    for t in np.linspace(0, 1, n + 1)[1:]:
        q, err = pose_ik(robot, "left", home_hover + t * (home_pos - home_hover), home_rotation, q)
        residuals.append(err)
        rows.append({"stage": "RETURN_HOME_DESCEND", "q_sdk_deg": q, "source_index": None,
                     "derivation": "HOME_CLEAR_VERTICAL_RETURN"})
    # 末尾仍对齐 A 的固定关节 Home，不把等位姿的冗余解当同一关节构型。
    rows.append({"stage": "RETURN_SAFE", "q_sdk_deg": source["home_joints_sdk_deg"],
                 "source_index": None, "derivation": "A_TASK_HOME_AFTER_CLEAR_RETREAT"})
    return rows, residuals


def densify_pair(previous, target, step):
    """同一时刻的 14 轴一起密化；一臂到位时保持，阶段末才进入下一阶段。"""
    before = np.array([previous[a] for a in ARMS], float)
    after = np.array([target[a] for a in ARMS], float)
    count = max(1, math.ceil(float(np.max(np.abs(after - before))) / step))
    return [{a: row[i].tolist() for i, a in enumerate(ARMS)}
            for row in (before + (after - before) * k / count for k in range(1, count + 1))]


def generate():
    source = json.loads(SOURCE.read_text())
    robot = load_robot()
    frames, residuals = [], []
    step = min(SHARED["pulse_step_deg"], source["offline_replay"]["joint_step_deg"])
    previous, carrying = None, False
    try:
        rows, left_residuals = build_reference(robot, source)
        for index, row in enumerate(rows):
            stage = row["stage"]
            origin = {"source_index": row["source_index"], "derivation": row["derivation"]}
            if "action" in row:
                carrying = row["action"] == "close"
                # 每个事件先两臂到位，再同时开合；停留仅是 GUI 节拍，不是夹爪时延标定。
                for hold in range(round(0.6 / DT)):
                    frames.append({"stage": stage, "q_sdk_deg": previous,
                                   "carrying": carrying, **origin,
                                   **({"event": row["action"], "event_arms": list(ARMS)} if hold == 0 else {})})
                continue
            right, residual = right_ik(robot, row["q_sdk_deg"], previous["right"] if previous else None)
            residuals.append(residual)
            target = {"left": row["q_sdk_deg"], "right": right}
            dense = [target] if previous is None else densify_pair(previous, target, step)
            for q in dense:
                frames.append({"stage": stage, "q_sdk_deg": q, "carrying": carrying,
                               **origin})
            previous = target
            if index % 200 == 0:
                print(f"IK: {index}/{len(rows)}", flush=True)
        endpoints = {}
        for frame in frames:
            endpoints[frame["stage"]] = frame["q_sdk_deg"]
        pick, place = {}, {}
        for arm in ARMS:
            pick[arm] = tool(robot, arm, endpoints["PICK_DESCEND"][arm])[0].tolist()
            place[arm] = tool(robot, arm, endpoints["PLACE_DESCEND"][arm])[0].tolist()
        plan = {
            "schema_version": "dual_arm_simulation.v1", "real_motion_authorized": False,
            "debian_execution_allowed": False, "coordinate_frame": "URDF_WORLD_SIM_ONLY",
            "units": {"joint": "degree_sdk_sign", "position": "meter", "time": "second"},
            "meta": {a: {k: PROFILES[a][k] for k in ("arm_id", "gripper_id", "joint_ids", "ee_link_id")} for a in ARMS},
            "provenance": {"source": str(SOURCE.relative_to(WORK)), "source_sha256": sha(SOURCE),
                           "generator_sha256": sha(__file__),
                           "profiles_sha256": sha(PROFILE_PATH),
                           "profile_module_sha256": sha(ap.__file__),
                           "calibration_module_sha256": sha(cc.__file__),
                           "urdf_sha256": sha(cc.DEFAULT_URDF),
                           "mirror_solver_sha256": sha(HERE.parent / "gen_right_from_left_mirror.py")},
            "playback": {"period_s": DT, "joint_step_deg": step,
                         "interpolation": "SIMULTANEOUS_JOINT_LINEAR_VISUALIZATION_ONLY",
                         "controller_timing_validated": False, "stage_end_barrier": "BOTH_ARMS_AT_TARGET"},
            "start": "A_CAPTURED_START_LEFT_PLUS_RIGHT_IK_SIM_INITIALIZATION_NOT_LIVE_HOME_RECOVERY",
            "scene": {"status": "SYNTHETIC_NOT_REGISTERED_TO_HARDWARE",
                      "layout": "SEPARATE_TABLETOP_TARGETS_NO_BIN",
                      "physical_requirement": "REMOVE_BIN_FROM_THE_VALIDATED_PATH_BEFORE_ANY_FUTURE_HARDWARE_TEST",
                      "pick_grip_m": pick, "place_grip_m": place,
                      "bottle_radius_m": 0.035, "bottle_height_m": 0.2,
                      "table_top_m": min(v[2] for v in pick.values()) - 0.1,
                      "table_size_m": [1.2, 0.8, 0.05], "table_center_xy_m": [0.7, 0],
                      "placement_support_offset_m": 0.0,
                      "release_model": "RIGID_CARRY_THEN_VERTICAL_SETTLING_VISUAL_ONLY_NO_CONTACT_DYNAMICS"},
            "left_replan_ik": {"max_position_error_mm": max(v["pos_err_mm"] for v in left_residuals),
                               "max_orientation_error_deg": max(v["ori_err_deg"] for v in left_residuals)},
            "right_ik": {"position_tolerance_mm": 0.2, "orientation_tolerance_deg": 0.2,
                         "max_position_error_mm": max(v["pos_err_mm"] for v in residuals),
                         "max_orientation_error_deg": max(v["ori_err_deg"] for v in residuals),
                         "limits_basis": "RIGHT_NOMINAL_ONLY_NOT_PHYSICAL_CONTROLLER_LIMITS"},
            "gui_review": {"status": "PENDING_USER_REVIEW", "automatic_pass_prohibited": True},
            "frames": frames,
        }
        validate_contract(plan)
        save_json(PLAN, plan)
        return plan
    finally:
        p.disconnect()


def validate_contract(plan):
    if plan.get("schema_version") != "dual_arm_simulation.v1":
        raise ValueError("不是双臂仿真专用格式")
    if plan.get("real_motion_authorized") is not False or plan.get("debian_execution_allowed") is not False:
        raise ValueError("Demo 必须禁止真机")
    if plan.get("coordinate_frame") != "URDF_WORLD_SIM_ONLY":
        raise ValueError("不能把演示坐标冒充 SDK 坐标")
    if plan["playback"]["period_s"] != DT:
        raise ValueError("回放周期与共享配置不一致")
    step = plan["playback"]["joint_step_deg"]
    if not 0 < step <= SHARED["pulse_step_deg"]:
        raise ValueError("密集步长超出共享上限")
    arrays = {}
    for arm in ARMS:
        for key in ("arm_id", "gripper_id", "joint_ids", "ee_link_id"):
            if plan["meta"][arm][key] != PROFILES[arm][key]:
                raise ValueError(f"{arm} {key} 映射错误")
        q = np.array([f["q_sdk_deg"][arm] for f in plan["frames"]], float)
        if q.ndim != 2 or q.shape[1] != 7 or len(q) < 2 or not np.isfinite(q).all():
            raise ValueError("需要有限的 Nx7 关节序列")
        if np.max(np.abs(np.diff(q, axis=0))) > step + 1e-8:
            raise ValueError(f"{arm} 相邻点跳变")
        lim = np.array(PROFILES[arm].get("controller_limits_deg") or
                       PROFILES[arm]["candidate_nominal_urdf_limits_deg"])
        if np.any(q < lim[:, 0] - 1e-8) or np.any(q > lim[:, 1] + 1e-8):
            raise ValueError(f"{arm} 超过配置限位（右臂仅标称）")
        arrays[arm] = q
    events = [(f["stage"], f["event"]) for f in plan["frames"] if "event" in f]
    if events != [("OPEN_BEFORE_PICK", "open"), ("CLOSE_AT_PICK", "close"), ("OPEN_AT_PLACE", "open")]:
        raise ValueError("夹爪事件丢失或乱序")
    carrying = False
    for i, frame in enumerate(plan["frames"]):
        if "event" in frame:
            if frame.get("event_arms") != list(ARMS):
                raise ValueError("事件必须同时包含两臂")
            carrying = frame["event"] == "close"
            if i and any(not np.allclose(arrays[a][i], arrays[a][i - 1], atol=1e-10, rtol=0) for a in ARMS):
                raise ValueError("夹爪事件前机械臂未停止")
        if frame["carrying"] != carrying:
            raise ValueError("持物状态不连续")
    return arrays


def box(center, size, color):
    half = np.asarray(size) / 2
    return p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=half),
                             p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=color),
                             basePosition=center)


class Scene:
    """姿态重置式运动学显示，物体绑定整个 EE 姿态，而非只跟随 XYZ。"""
    def __init__(self, robot, plan):
        self.robot, self.plan = robot, plan
        s = plan["scene"]
        top = s["table_top_m"]
        self.obstacles = {"table": box([*s["table_center_xy_m"], top - 0.025], s["table_size_m"], [.58, .61, .65, 1])}
        # 无筐场景是本轮明确的任务变化；若现场仍有筐，不能沿用此检查结论。
        for arm in ARMS:
            center = list(s["place_grip_m"][arm])
            center[2] = top + 0.001
            visual = p.createVisualShape(p.GEOM_CYLINDER, radius=.055, length=.001,
                                         rgbaColor=COLORS[arm][:3] + [.55])
            p.createMultiBody(0, -1, visual, basePosition=center)
        self.objects, self.proxies = {}, {}
        for arm in ARMS:
            self.objects[arm] = p.createMultiBody(0,
                p.createCollisionShape(p.GEOM_CYLINDER, radius=s["bottle_radius_m"], height=s["bottle_height_m"]),
                p.createVisualShape(p.GEOM_CYLINDER, radius=s["bottle_radius_m"], length=s["bottle_height_m"], rgbaColor=COLORS[arm]),
                basePosition=s["pick_grip_m"][arm])
            color = COLORS[arm][:3] + [.23]
            self.proxies[arm] = box([0, 0, 3], PROFILES[arm]["gripper_collision_box_m"], color)
        self.reset()

    def reset(self):
        self.attachments = {}
        self.released = set()
        self.settle_steps = 0
        for arm in ARMS:
            p.resetBasePositionAndOrientation(self.objects[arm], self.plan["scene"]["pick_grip_m"][arm], [0, 0, 0, 1])

    def update(self, frame):
        for arm in ARMS:
            set_q(self.robot, arm, frame["q_sdk_deg"][arm])
            grip, rotation, origin, quat = tool(self.robot, arm)
            center = origin + rotation @ (np.asarray(PROFILES[arm]["gripper_collision_center_link_mm"]) / 1000)
            p.resetBasePositionAndOrientation(self.proxies[arm], center, quat)
            if frame.get("event") == "close":
                pos, orn = p.getBasePositionAndOrientation(self.objects[arm])
                inv = p.invertTransform(origin, quat)
                self.attachments[arm] = p.multiplyTransforms(*inv, pos, orn)
            if arm in self.attachments:
                p.resetBasePositionAndOrientation(self.objects[arm], *p.multiplyTransforms(origin, quat, *self.attachments[arm]))
            if frame.get("event") == "open" and arm in self.attachments:
                del self.attachments[arm]
                self.released.add(arm)
            if arm in self.released:
                # [演示假设] 释放后沿Z落下；保留释放姿态，按包围盒底部落到桌面。
                pos, orn = p.getBasePositionAndOrientation(self.objects[arm])
                aabb = p.getAABB(self.objects[arm])
                support = self.plan["scene"]["table_top_m"] + self.plan["scene"]["placement_support_offset_m"]
                drop = min(0.003, max(0.0, aabb[0][2] - support))
                p.resetBasePositionAndOrientation(self.objects[arm], [pos[0], pos[1], pos[2] - drop], orn)


def validate_scene(plan):
    arrays = validate_contract(plan)
    robot = load_robot()
    try:
        scene = Scene(robot, plan)
        parents = {i: p.getJointInfo(robot, i)[16] for i in range(p.getNumJoints(robot))}
        left_ids, right_ids = (set(PROFILES[a]["joint_ids"]) for a in ARMS)
        collisions = {}
        frame_categories = defaultdict(set)
        stage_categories = defaultdict(lambda: defaultdict(set))
        cross_clearance = {"query_radius_m": .5, "minimum_distance_m": .5,
                           "bounded_below_only": True, "pair": None, "frame": None}
        table_clearance = {"query_radius_m": .2, "minimum_distance_m": .2,
                           "bounded_below_only": True, "pair": None, "frame": None}

        def closest(metric, distance, label, index):
            if distance < metric["minimum_distance_m"]:
                metric.update(minimum_distance_m=distance, bounded_below_only=False,
                              pair=label, frame=index)

        def record(category, label, depth, index, stage):
            if depth >= -1e-5:  # 10um 数值容差，不是安全间隙。
                return
            key = category + ":" + label
            if key not in collisions:
                collisions[key] = {"category": category, "pair": label, "worst_penetration_mm": 0,
                                   "first_frame": index, "first_stage": stage, "last_frame": index}
            r = collisions[key]
            r["worst_penetration_mm"] = max(r["worst_penetration_mm"], -1000 * depth)
            r["last_frame"] = index
            frame_categories[category].add(index)
            stage_categories[stage][category].add(index)

        for index, frame in enumerate(plan["frames"]):
            scene.update(frame)
            p.performCollisionDetection()
            for cp in p.getClosestPoints(robot, robot, cross_clearance["query_radius_m"]):
                if cp[3] in left_ids and cp[4] in right_ids:
                    closest(cross_clearance, cp[8], f"left_link{cp[3]}/right_link{cp[4]}", index)
            for contact in p.getContactPoints(robot, robot):
                a, b = sorted(contact[3:5])
                if a == b or parents.get(a) == b or parents.get(b) == a:
                    continue
                category = "cross_arm_mesh" if ((a in left_ids and b in right_ids) or (b in left_ids and a in right_ids)) else "nonadjacent_self_mesh"
                record(category, f"link{a}/link{b}", contact[8], index, frame["stage"])
            moving = {"robot": robot, **{a + "_gripper_proxy": scene.proxies[a] for a in ARMS},
                      **{a + "_object": scene.objects[a] for a in ARMS}}
            for name, body in moving.items():
                for obs_name, obs in scene.obstacles.items():
                    distance = table_clearance["query_radius_m"] if name == "robot" or "proxy" in name else 0
                    for cp in p.getClosestPoints(body, obs, distance):
                        if distance:
                            closest(table_clearance, cp[8], name + f"/link{cp[3]}/" + obs_name, index)
                        record("environment", name + "/" + obs_name + f"/link{cp[3]}", cp[8], index, frame["stage"])
            # 跨臂夹爪/物体包络与机器人；自身末端的夹持接触仅分类记录，不冒充无碰撞。
            for arm in ARMS:
                own_hand = set(PROFILES[arm]["joint_ids"][-3:])
                other = right_ids if arm == "left" else left_ids
                for label, body in [("gripper_proxy", scene.proxies[arm]), ("object", scene.objects[arm])]:
                    for cp in p.getClosestPoints(body, robot, cross_clearance["query_radius_m"]):
                        link = cp[4]
                        if link in other:
                            closest(cross_clearance, cp[8], f"{arm}_{label}/other_link{link}", index)
                        is_attachment = label == "gripper_proxy" or frame["carrying"]
                        category = ("attached_hand_contact" if link in own_hand and is_attachment else
                                    "cross_arm_envelope" if link in other else
                                    "own_object_robot" if label == "object" and link in own_hand else "envelope_robot")
                        record(category, f"{arm}_{label}/link{link}", cp[8], index, frame["stage"])
                for cp in p.getClosestPoints(scene.objects[arm], scene.proxies[arm], 0):
                    category = "holding_object_gripper" if frame["carrying"] else "unheld_object_gripper"
                    record(category, f"{arm}_object/{arm}_gripper", cp[8], index, frame["stage"])
            for ln, lb in [("gripper", scene.proxies["left"]), ("object", scene.objects["left"])]:
                for rn, rb in [("gripper", scene.proxies["right"]), ("object", scene.objects["right"])]:
                    for cp in p.getClosestPoints(lb, rb, cross_clearance["query_radius_m"]):
                        closest(cross_clearance, cp[8], f"left_{ln}/right_{rn}", index)
                        record("cross_arm_envelope", ln + "/" + rn, cp[8], index, frame["stage"])
            if index % 400 == 0:
                print(f"dense collision: {index}/{len(plan['frames'])}", flush=True)
        metrics = {}
        for arm, q in arrays.items():
            limits = np.array(PROFILES[arm].get("controller_limits_deg") or PROFILES[arm]["candidate_nominal_urdf_limits_deg"])
            metrics[arm] = {"joint_range_deg": np.stack([q.min(axis=0), q.max(axis=0)], axis=1).tolist(),
                            "minimum_margin_per_joint_deg": np.minimum(q.min(axis=0) - limits[:, 0], limits[:, 1] - q.max(axis=0)).tolist(),
                            "max_step_deg": float(np.max(np.abs(np.diff(q, axis=0)))),
                            "start_to_final_delta_deg": (q[-1] - q[0]).tolist(),
                            "limits_basis": "CONFIGURED_LEFT_LIMITS" if arm == "left" else "NOMINAL_RIGHT_URDF_ONLY"}
        both = np.logical_and(np.max(np.abs(np.diff(arrays["left"], axis=0)), axis=1) > 1e-8,
                              np.max(np.abs(np.diff(arrays["right"], axis=0)), axis=1) > 1e-8)
        report = {"schema_version": "dual_arm_simulation_validation.v1", "plan_sha256": sha(PLAN),
                  "status": "SIMULATION_DEMO_ONLY_HARDWARE_BLOCKED", "contract": "PASS",
                  "dense_frames_checked": len(plan["frames"]),
                  "display_duration_s": len(plan["frames"]) * DT,
                  "simultaneously_moving_intervals": int(np.count_nonzero(both)),
                  "joint_metrics": metrics, "right_ik": plan["right_ik"],
                  "collision_frames_by_category": {k: len(v) for k, v in frame_categories.items()},
                  "collision_frames_by_stage": {stage: {cat: len(frames) for cat, frames in cats.items()}
                                                for stage, cats in stage_categories.items()},
                  "cross_arm_clearance": cross_clearance,
                  "robot_gripper_table_clearance": table_clearance,
                  "task_checks": {"cross_arm": "FAIL" if frame_categories["cross_arm_mesh"] or frame_categories["cross_arm_envelope"] else "PASS_SAMPLED_MODEL",
                                  "environment": "FAIL" if frame_categories["environment"] else "PASS_SAMPLED_MODEL",
                                  "complete_collision_qualification": "REJECT_UNRESOLVED_TOOL_SELF_AND_OBJECT_CONTACTS"},
                  "collision_pairs": sorted(collisions.values(), key=lambda x: -x["worst_penetration_mm"]),
                  "collision_scope": "All displayed dense poses; both full arms, nonadjacent meshes, gripper boxes, carried and placed objects, table. Bin explicitly absent per new demo task. NOT a mathematical continuous swept-volume certificate.",
                  "exclusions": ["Direct parent-child robot links excluded; other wrist overlaps NOT excluded.",
                                 "Own last-three-link attachment/holding intersections classified separately, not declared collision-free.",
                                 "Base and fixed support mesh still reported; scene and gripper dimensions are assumptions."],
                  "hardware_blockers": ["Right controller limits and measured gripper/TCP unresolved.",
                                        "Synthetic scene has no current hardware registration.",
                                        "This scene requires the real bin and other unmodelled obstacles to be cleared from the path.",
                                        "URDF mesh intersections require resolution; see collision_pairs.",
                                        "SDK concurrent dispatch/timing unverified; GUI interpolation is not vendor motion.",
                                        "No hardware executor exists for this demo schema."],
                  "gui_review": "PENDING_USER_REVIEW_NO_AUTOMATIC_PASS"}
        save_json(REPORT, report)
        print(json.dumps({k: report[k] for k in ("status", "dense_frames_checked", "display_duration_s", "simultaneously_moving_intervals", "collision_frames_by_category")}, ensure_ascii=False, indent=2))
        return report
    finally:
        p.disconnect()


def load_validated():
    plan = json.loads(PLAN.read_text())
    report = json.loads(REPORT.read_text())
    validate_contract(plan)
    if report["plan_sha256"] != sha(PLAN):
        raise ValueError("轨迹改过，先重新 --build 校验")
    provenance = plan["provenance"]
    for path, key in [(SOURCE, "source_sha256"), (Path(__file__), "generator_sha256"),
                      (WORK / "arm_profiles.v1.json", "profiles_sha256"),
                      (Path(ap.__file__), "profile_module_sha256"),
                      (Path(cc.__file__), "calibration_module_sha256"),
                      (Path(cc.DEFAULT_URDF), "urdf_sha256"),
                      (HERE.parent / "gen_right_from_left_mirror.py", "mirror_solver_sha256")]:
        if sha(path) != provenance[key]:
            raise ValueError(f"依赖已变化，先重新 --build: {path.name}")
    return plan, report


def replay(plan, speed=1.0, loop=False, exit_after_replay=False):
    if not math.isfinite(speed) or speed <= 0:
        raise ValueError("speed 必须为正数")
    robot = load_robot(gui=True)
    try:
        scene = Scene(robot, plan)
        # 少量固定说明，不把所有路点名称叠在模型上。
        p.addUserDebugText("DUAL ARM | BLUE=LEFT ORANGE=RIGHT | SIM ONLY", [0.05, -.62, 1.65], [0.1, .1, .1], 1.25)
        p.addUserDebugText("SEPARATE TABLETOP TARGETS | tool model unresolved | SIM ONLY", [0.05, -.62, 1.57], [.85, .05, .05], .95)
        # 拾取与放置路径颜色区分；GUI 显示全程，未跳帧。
        for arm in ARMS:
            previous = None
            for f in plan["frames"][::8]:
                pos = tool(robot, arm, f["q_sdk_deg"][arm])[0]
                if previous is not None:
                    p.addUserDebugLine(previous, pos, COLORS[arm][:3], 1)
                previous = pos
        text_id = -1
        while p.isConnected():
            scene.reset()
            scene.update(plan["frames"][0])
            print("3秒后播放：双臂同时接近 → 抓取 → 抬起 → 转运 → 放置 → 返回。", flush=True)
            for _ in range(30):
                if not p.isConnected():
                    return
                time.sleep(.1)
            for index, frame in enumerate(plan["frames"]):
                if not p.isConnected():
                    return
                began = time.monotonic()
                scene.update(frame)
                if index == 0 or frame["stage"] != plan["frames"][index - 1]["stage"]:
                    text_id = p.addUserDebugText(frame["stage"] + "  | both-arm barrier", [0.05, -.62, 1.49], [.1, .1, .1], 1.15, replaceItemUniqueId=text_id)
                    print(frame["stage"], flush=True)
                # 不补发赶帧：GUI慢时放慢，所有密集姿态均显示。
                time.sleep(max(0, DT / speed - (time.monotonic() - began)))
            print("全程动态回放结束。仅完成显示，不自动登记人工GUI通过。", flush=True)
            if exit_after_replay:
                break
            if not loop:
                print("窗口保持在终点；关闭窗口或终端 Ctrl-C 退出。", flush=True)
                while p.isConnected():
                    time.sleep(.1)
                break
    except KeyboardInterrupt:
        print("Demo 已停止；未连接真机。")
    finally:
        if p.isConnected():
            p.disconnect()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--build", action="store_true", help="重建镜像 IK + 密集碰撞报告")
    parser.add_argument("--gui", action="store_true", help="打开 PyBullet 全程动画")
    parser.add_argument("--loop", action="store_true", help="反复播放；循环开始重置只是仿真，不是返回轨迹")
    parser.add_argument("--speed", type=float, default=1, help="仅显示倍速")
    parser.add_argument("--exit-after-replay", action="store_true", help="完整播放一遍后关闭，用于GUI自检")
    args = parser.parse_args(argv)
    if args.build:
        validate_scene(generate())
    if args.gui:
        plan, report = load_validated()
        print("碰撞诊断：", report["collision_frames_by_category"], flush=True)
        replay(plan, args.speed, args.loop, args.exit_after_replay)
    if not args.build and not args.gui:
        parser.print_help()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
