#!/usr/bin/env python3
"""由示教 TCP+关节生成左臂抓放候选。几何只用 FK，不用会话 T。"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

import numpy as np
import pybullet as p

HERE = Path(__file__).resolve().parent
WORK = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(WORK))
sys.path.insert(0, str(WORK / "frame_calibration" / "analysis"))

import arm_profiles
import gen_bottle_servo_candidate as gen
from robot_mission.preflight import _set_robot, densify

ARM = "left"
SCHEMA = "taught_tcp_pick_place.v1"


def _tcp_from_grip(grip, R_link, tcp_link_mm) -> np.ndarray:
    grip_off = np.asarray(arm_profiles.arm_profile(ARM)["grip_center_link_mm"], float) / 1000.0
    tcp_off = np.asarray(tcp_link_mm, float) / 1000.0
    return np.asarray(grip, float) - R_link @ grip_off + R_link @ tcp_off

def _fk_tcp(robot, q, tcp_link_mm):
    grip, R_link = gen.fk_grip(robot, q)
    return grip, _tcp_from_grip(grip, R_link, tcp_link_mm), R_link


def _lerp_joints(q0, q1, max_step_deg: float):
    q0 = [float(v) for v in q0]
    q1 = [float(v) for v in q1]
    span = max(abs(a - b) for a, b in zip(q0, q1))
    count = max(1, int(math.ceil(span / max_step_deg)))
    return [[a + (b - a) * (i / count) for a, b in zip(q0, q1)]
            for i in range(count + 1)]


def _check_uvw(robot, q, uvw_deg, limit_deg=1.0):
    _grip, R_link = gen.fk_grip(robot, q)
    uvw_fk = gen._matrix_to_uvw(R_link @ gen._rotz(90.0))
    delta = max(abs(gen._normalize_angle_deg(a - b))
                for a, b in zip(uvw_fk, uvw_deg))
    if delta > limit_deg:
        raise ValueError(f"示教关节与 UVW 差 {delta:.3f}deg > {limit_deg}")
    return uvw_fk, delta


def _table_spec(robot, task: dict) -> dict:
    table = task["table"]
    tcp_mm = task["tcp_link_mm"]
    _grip, tcp, _R = _fk_tcp(robot, table["corner"]["q_sdk_deg"], tcp_mm)
    reference = table.get("corner_reference")
    if reference == "EE_LINK_ORIGIN":
        corner = np.asarray(p.getLinkState(
            robot, arm_profiles.arm_profile(ARM)["ee_link_id"],
            computeForwardKinematics=True)[4], float)
    elif reference == "CONFIGURED_TCP":
        corner = tcp
    else:
        raise ValueError("必须明确 table.corner_reference，不能猜桌角是 TCP 或夹爪")
    dx = float(table["extent_from_corner_pb"]["x"])
    dy = float(table["extent_from_corner_pb"]["y"])
    thick = float(table["thickness_m"])
    top = float(corner[2])
    cx = float(corner[0]) + dx / 2.0
    cy = float(corner[1]) + dy / 2.0
    return {
        "source_obstacle_id": "taught_table_top",
        "center": [cx, cy, top - thick / 2.0],
        "half": [abs(dx) / 2.0, abs(dy) / 2.0, thick / 2.0],
        "top_pb_m": top,
        "corner_tcp_pb_m": tcp.tolist(),
        "corner_reference": reference,
        "corner_reference_pb_m": corner.tolist(),
        "corner_reference_evidence": table.get("corner_reference_evidence"),
        "old_tcp_table_height_error_mm": float((tcp[2] - corner[2]) * 1000),
        "support_contact_tolerance_m": float(
            task["scene"].get("support_contact_tolerance_m", 0.0)),
        "minimum_robot_clearance_m": float(
            task["scene"].get("minimum_robot_table_clearance_m", 0.0)),
        "minimum_gripper_clearance_m": float(
            task["scene"].get("minimum_gripper_table_clearance_m", 0.0)),
        "physical_top_from_floor_m": float(table["top_from_floor_m"]),
        "geometry_status": table.get("geometry_status", "UNCONFIRMED"),
        "comment": "桌角按明确的 endpoint 参考点做 FK，不给桌面叠加夹爪/TCP 偏移；XY 方向沿用任务定义。",
    }


def _equal_height_targets(fks, policy):
    """以用户取坐标的 EE 原点等高；抓放段锁定抓取朝向，防止瓶底倾斜。"""
    if (policy.get("frame"), policy.get("reference_point"),
            policy.get("pick_place_z_reference"), policy.get("hover_z_reference")) != (
            "PYBULLET_PHYSICAL_FK", "EE_LINK_ORIGIN", "pick", "max_taught_hover"):
        raise ValueError("不支持的等高坐标契约，禁止把 SDK Z 与物理 FK Z 混用")
    if policy.get("orientation_reference") != "pick":
        raise ValueError("抓放等高轨迹必须明确锁定 pick 朝向")
    low = float(fks["pick"]["ee"][2])
    high = max(float(fks[name]["ee"][2]) for name in ("pick_hover", "place_hover"))
    if high <= low:
        raise ValueError("hover 必须高于抓放点")
    targets = {}
    rotation = fks["pick"]["R_link"]
    for name, row in fks.items():
        ee = row["ee"].copy()
        ee[2] = high if "hover" in name else low
        targets[name] = {"ee": ee, "R_link": rotation,
                         "tcp": ee + rotation @ (row["R_link"].T @ (row["tcp"] - row["ee"])),
                         "grip": ee + rotation @ (row["R_link"].T @ (row["grip"] - row["ee"]))}
    return targets


def _repair_entry(robot, prefix, hover_q, table_spec, bottle_center, task, bounds):
    """保留 Track C 入口的无碰撞前段，仅绕开新桌/新瓶造成的入口冲突。

    本检查只供空爪入口规划，瓶子绝不享受抓取接触豁免；最终资格门仍须检查全程。
    """
    profile = arm_profiles.arm_profile(ARM)
    inactive = arm_profiles.arm_profile("right")
    scene = task["scene"]
    table = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=table_spec["half"]),
                              basePosition=table_spec["center"])
    bottle = p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[
        scene["bottle_radius_m"], scene["bottle_radius_m"], scene["bottle_height_m"] / 2]),
        basePosition=bottle_center.tolist())
    proxy = p.createMultiBody(0, p.createCollisionShape(
        p.GEOM_BOX, halfExtents=(np.asarray(profile["gripper_collision_box_m"]) / 2).tolist()))
    step = float(profile["shared"]["pulse_step_deg"])
    object_clearance = float(scene["minimum_unintended_object_clearance_m"])
    disabled = gen.parent_planner.compute_disabled_pairs(robot)
    self_collision = gen.pp.get_collision_fn(
        robot, profile["joint_ids"], obstacles=[], self_collisions=True,
        disabled_collisions=disabled, custom_limits=dict(zip(profile["joint_ids"], bounds)))
    checks = 0

    def collision(q_rad, **_kwargs):
        nonlocal checks
        checks += 1
        q_sdk = gen._urdf_rad_to_sdk_deg(q_rad)
        _set_robot(p, robot, profile, q_sdk, inactive, inactive["home_deg"])
        if self_collision(q_rad):
            return True
        st = p.getLinkState(robot, profile["ee_link_id"], computeForwardKinematics=True)
        R = np.asarray(p.getMatrixFromQuaternion(st[5])).reshape(3, 3)
        pc = np.asarray(st[4]) + R @ (np.asarray(profile["gripper_collision_center_link_mm"]) / 1000)
        p.resetBasePositionAndOrientation(proxy, pc, st[5])
        ghost_links = {profile["ee_link_id"], inactive["ee_link_id"]}
        # 沿用实测夹爪代理替代末端幽灵网格，其余机械臂连杆仍查桌面和瓶体。
        for obstacle, robot_margin, proxy_margin in (
                (table, table_spec["minimum_robot_clearance_m"], table_spec["minimum_gripper_clearance_m"]),
                (bottle, object_clearance, object_clearance)):
            if any(row[8] < robot_margin for row in p.getClosestPoints(robot, obstacle, robot_margin)
                   if row[3] not in ghost_links):
                return True
            if p.getClosestPoints(proxy, obstacle, proxy_margin):
                return True
        return False

    def free(q_sdk):
        return not collision(gen._sdk_deg_to_urdf_rad(q_sdk))

    try:
        if not free(prefix[0]["q_sdk_deg"]):
            raise RuntimeError("新场景中 HOME 不安全，不能从首点开始")
        kept = [prefix[0]]
        for row in prefix[1:]:
            if not all(free(q) for q in densify(kept[-1]["q_sdk_deg"], row["q_sdk_deg"], step)):
                break
            kept.append(row)
        start = kept[-1]["q_sdk_deg"]
        if not free(hover_q):
            raise RuntimeError("新场景中 pick_hover 不安全")
        a = gen._sdk_deg_to_urdf_rad(start).tolist()
        b = gen._sdk_deg_to_urdf_rad(hover_q).tolist()
        random.seed(int(task["motion"]["entry_rrt_seed"]))
        np.random.seed(int(task["motion"]["entry_rrt_seed"]))
        path = gen.pp.birrt(
            a, b, gen.pp.get_distance_fn(robot, profile["joint_ids"]),
            gen.pp.get_sample_fn(robot, profile["joint_ids"],
                                 custom_limits=dict(zip(profile["joint_ids"], bounds))),
            gen.pp.get_extend_fn(robot, profile["joint_ids"], resolutions=[math.radians(step)] * 7),
            collision, max_time=float(task["motion"]["entry_rrt_max_time_s"]),
            max_iterations=500, restarts=2, smooth=50)
        if path is None:
            raise RuntimeError("新桌/瓶子入口绕行求解失败；未修改原候选")
        anchors = [gen._urdf_rad_to_sdk_deg(q) for q in path]
        rows = [anchors[0]]
        for q0, q1 in zip(anchors, anchors[1:]):
            if not all(free(q) for q in densify(q0, q1, step)):
                raise RuntimeError("入口在 Servo 实际步长下复检失败")
            rows.extend(densify(q0, q1, float(task["motion"]["export_step_deg"])))
        return kept, rows, {
            "status": "ENTRY_ONLY_DENSE_PASS", "dense_step_deg": step,
            "verified_parent_prefix_points_retained": len(kept),
            "verified_parent_prefix_points_available": len(prefix),
            "entry_motion_points": len(rows), "collision_queries": checks,
            "unintended_object_clearance_m": object_clearance,
            "self_collision_disabled_link_pairs": sorted([list(pair) for pair in disabled]),
            "note": "只验证入口，不等于抓取/携带/放置/回撤资格通过。"}
    finally:
        for body in (proxy, bottle, table):
            p.removeBody(body)


def _continuous_equal_height_path(robot, targets, taught, bounds, motion):
    """先求最受限的放置悬停，再向两侧连续求解；回程复用同一路径。

    禁止末尾 snap 回旧示教关节。相同末端位姿允许不同冗余关节解。
    """
    pos_tol = float(motion["cartesian_position_tolerance_mm"])
    rot_tol = float(motion["cartesian_rotation_tolerance_deg"])
    anchor = targets["place_hover"]
    q, anchor_report = gen.solve_full_pose(
        robot, anchor["grip"], anchor["R_link"], taught["place_hover"]["q"],
        bounds, iterations=800, position_tolerance_mm=pos_tol,
        rotation_tolerance_deg=rot_tol)
    if q is None:
        raise RuntimeError(f"等高 place_hover IK 失败: {anchor_report}")

    def chain(start, end, seed):
        a, b = targets[start], targets[end]
        seed_urdf = gen._sdk_deg_to_urdf_rad(seed)
        if any(not lo - 1e-10 <= value <= hi + 1e-10
               for value, (lo, hi) in zip(seed_urdf, bounds)):
            raise RuntimeError(f"{start} 初始种子越过 IK 余量盒")
        rows, report = gen.solve_cartesian_chain(
            robot, a["grip"], b["grip"], a["R_link"], seed, bounds,
            float(motion["cartesian_sample_mm"]),
            float(motion["maximum_anchor_joint_step_deg"]), pos_tol, rot_tol,
            R_end=b["R_link"])
        # 共享 chain 的首种子不重新求解，因此连首帧残差一起严格复核。
        if (report["maximum_position_error_mm"] > pos_tol or
                report["maximum_rotation_error_deg"] > rot_tol):
            raise RuntimeError(f"{start}->{end} 全链残差超限: {report}")
        return rows, report

    backwards, transfer_report = chain("place_hover", "pick_hover", q)
    descend, descend_report = chain("pick_hover", "pick", backwards[-1])
    place_down, place_report = chain("place_hover", "place", q)
    return {
        "descend": descend, "lift": list(reversed(descend)),
        "transfer": list(reversed(backwards)), "place_down": place_down,
        "place_up": list(reversed(place_down)),
        "q": {"pick": descend[-1], "pick_hover": descend[0],
              "place_hover": q, "place": place_down[-1]},
        "reports": {"place_hover_anchor": anchor_report,
                    "pick_descend_and_reverse": descend_report,
                    "transfer_solved_reverse": transfer_report,
                    "place_descend_and_reverse": place_report},
    }


def build(task_path: Path, out_path: Path, qualification_task_out: Path):
    task = json.loads(task_path.read_text(encoding="utf-8"))
    if task.get("schema_version") != SCHEMA:
        raise ValueError(f"expected {SCHEMA}")
    profile = arm_profiles.arm_profile(ARM)
    if (task["arm"], task["arm_id"], task["gripper_id"]) != (
            ARM, profile["arm_id"], profile["gripper_id"]):
        raise ValueError("arm/gripper contract mismatch")
    parent_path = WORK / task["verified_parent"]["path"]
    parent_hash = gen.sha256_file(parent_path)
    if parent_hash != task["verified_parent"]["sha256"]:
        raise ValueError("verified parent SHA-256 mismatch")
    parent = json.loads(parent_path.read_text(encoding="utf-8"))
    prefix, return_home = gen._parent_parts(parent)
    ready = prefix[-1]["q_sdk_deg"]

    limit_evidence = arm_profiles.controller_limit_profile(
        task["controller_limit_profile"])
    planning_limits = limit_evidence["planning_limits_deg"]
    extra_margin = float(task["motion"].get("ik_extra_margin_deg", 0.0))
    if extra_margin < 0.0 or not math.isfinite(extra_margin):
        raise ValueError("ik_extra_margin_deg 必须为非负有限值")
    solve_margin = float(profile["shared"]["minimum_joint_margin_deg"]) + extra_margin
    bounds = gen._solver_bounds_from_limits(planning_limits, solve_margin)

    gen.parent_planner.set_arm(ARM)
    robot = gen.xf.load_xifeng(gui=False)
    try:
        poses = {name: task[name] for name in
                 ("pick", "pick_hover", "place_hover", "place")}
        uvw_deltas = {}
        fks = {}
        for name, spec in poses.items():
            uvw_fk, delta = _check_uvw(robot, spec["q_sdk_deg"],
                                       spec["sdk_world_uvw_deg"])
            uvw_deltas[name] = {"uvw_fk_deg": uvw_fk, "delta_deg": delta}
            grip, tcp, R_link = _fk_tcp(robot, spec["q_sdk_deg"], task["tcp_link_mm"])
            ee = np.asarray(p.getLinkState(robot, profile["ee_link_id"],
                                           computeForwardKinematics=True)[4], float)
            fks[name] = {"ee": ee, "grip": grip, "tcp": tcp, "R_link": R_link,
                         "q": [float(v) for v in spec["q_sdk_deg"]],
                         "tilt_from_down_deg": gen._axis_angle_deg(
                             gen._tool_axis_from_link_rotation(R_link), [0, 0, -1])}
        _check_uvw(robot, task["table"]["corner"]["q_sdk_deg"],
                   task["table"]["corner"]["sdk_world_uvw_deg"])
        table = _table_spec(robot, task)
        motion = task["motion"]
        export_step = float(motion.get("export_step_deg",
                                       profile["shared"]["export_step_deg"]))
        targets = _equal_height_targets(fks, task["height_policy"])
        path = _continuous_equal_height_path(robot, targets, fks, bounds, motion)
        descend, lift, transfer, place_down, place_up = (
            path[name] for name in ("descend", "lift", "transfer", "place_down", "place_up"))
        actual = {}
        for name, q in path["q"].items():
            grip, tcp, R = _fk_tcp(robot, q, task["tcp_link_mm"])
            ee = np.asarray(p.getLinkState(robot, profile["ee_link_id"],
                                           computeForwardKinematics=True)[4], float)
            actual[name] = {"ee": ee, "grip": grip, "tcp": tcp, "R_link": R, "q": q}

        pick_center = np.array([targets["pick"]["grip"][0], targets["pick"]["grip"][1],
                                table["top_pb_m"] + float(task["scene"]["bottle_height_m"]) / 2.0])
        prefix, entry, entry_report = _repair_entry(
            robot, prefix, path["q"]["pick_hover"], table, pick_center, task, bounds)
        exit_rows = _lerp_joints(path["q"]["place_hover"], ready, export_step)

        waypoints = [{"seg": w["seg"],
                      "q_sdk_deg": [float(v) for v in w["q_sdk_deg"]]}
                     for w in prefix]
        gen._append_motion(waypoints, "ENTRY->PICK_HOVER", entry)
        gen._append_motion(waypoints, "PICK_DESCEND", descend)
        waypoints.append({"seg": "PICK", "gripper": "close"})
        gen._append_motion(waypoints, "PICK_ASCEND", lift)
        gen._append_motion(waypoints, "TRANSFER", transfer)
        gen._append_motion(waypoints, "PLACE_DESCEND", place_down)
        waypoints.append({"seg": "PLACE", "gripper": "open"})
        gen._append_motion(waypoints, "PLACE_ASCEND", place_up)
        gen._append_motion(waypoints, "PLACE_HOVER->READY", exit_rows)
        # 当前末点与父 Track C 的 READY 完全相同；直接复用其真机已验收
        # READY->HOME，不重新插值一条未经验证的关节直线。
        gen._append_motion(waypoints, "READY->HOME",
                           [row["q_sdk_deg"] for row in return_home])

        limit_report = gen._limit_report(profile, waypoints, planning_limits)
        if limit_report["status"] != "PASS":
            raise RuntimeError(f"导出前整条轨迹限位门失败: {limit_report}")
        motion_rows = [w for w in waypoints if "q_sdk_deg" in w]
        max_adjacent = max(
            max(abs(a - b) for a, b in zip(x["q_sdk_deg"], y["q_sdk_deg"]))
            for x, y in zip(motion_rows, motion_rows[1:]))
        pick_center = np.array([targets["pick"]["grip"][0], targets["pick"]["grip"][1],
                                table["top_pb_m"] + float(task["scene"]["bottle_height_m"]) / 2.0])
        place_center = np.array([targets["place"]["grip"][0], targets["place"]["grip"][1],
                                 table["top_pb_m"] + float(task["scene"]["bottle_height_m"]) / 2.0])
        meta = {
            "arm_id": profile["arm_id"], "arm": ARM,
            "gripper_id": profile["gripper_id"], "j6_flipped": True,
            "task": task["task_name"],
            "source_task": str(task_path.resolve().relative_to(WORK)),
            "source_task_sha256": gen.sha256_file(task_path),
            "generator": str(Path(__file__).resolve().relative_to(WORK)),
            "generator_sha256": gen.sha256_file(Path(__file__).resolve()),
            "qualification": "CANDIDATE_GUI_REVIEW_REQUIRED",
            "requires_live_controller_limits": True,
            "requires_live_controller_recheck": True,
            "real_motion_authorized": False,
            "verified_parent_sha256": parent_hash,
            "verified_parent_reused_segments": ["HOME", "HOME->READY", "READY->HOME"],
            "verified_parent_reuse_scope": (
                "COLLISION_CHECKED_PREFIX_OF_HOME_TO_READY_PLUS_COMPLETE_REALVERIFIED_READY_TO_HOME"),
            "entry_repair_report": entry_report,
            "tcp_link_mm": task["tcp_link_mm"],
            "coordinate_contract": task["height_policy"],
            "ik_required_margin_deg": solve_margin,
            "scene_geometry_status": table["geometry_status"],
            "adjusted_targets_pb_m": {
                name: {"ee": row["ee"].tolist(), "tcp": row["tcp"].tolist(), "grip": row["grip"].tolist(),
                       "R_link": row["R_link"].tolist(),
                       "orientation_delta_from_taught_deg": gen.cc.rotation_angle_deg(row["R_link"] @ fks[name]["R_link"].T),
                       "ee_delta_from_taught_mm": ((row["ee"] - fks[name]["ee"]) * 1000).tolist(),
                       "tcp_delta_from_taught_mm": ((row["tcp"] - fks[name]["tcp"]) * 1000).tolist()}
                for name, row in targets.items()},
            "solved_poses_pb_m": {
                name: {"ee": row["ee"].tolist(), "tcp": row["tcp"].tolist(), "grip": row["grip"].tolist(),
                       "R_link": row["R_link"].tolist(), "q_sdk_deg": row["q"]}
                for name, row in actual.items()},
            "taught_poses_pb_m": {name: {"ee": row["ee"].tolist(), "grip": row["grip"].tolist(),
                                         "tcp": row["tcp"].tolist(),
                                         "tilt_from_down_deg": row["tilt_from_down_deg"],
                                         "q_sdk_deg": row["q"]}
                                  for name, row in fks.items()},
            "uvw_joint_consistency": uvw_deltas,
            "table": table,
            "scene": task["scene"],
            "motion": motion,
            "cartesian_chain_reports": path["reports"],
            "controller_limit_profile": task["controller_limit_profile"],
            "controller_limit_evidence": {k: limit_evidence[k] for k in
                ("profile_id", "status", "source_log_sha256", "reported_limits_deg",
                 "planning_limits_deg", "planning_note")},
            "local_limit_report": limit_report,
            "gui_review": {"required": True, "status": "PENDING_HUMAN_REVIEW",
                           "dense_step_deg": task["gui_review"]["dense_step_deg"],
                           "demo_script": "pick_place_coord/demo_bottle_trajectory.py"},
            "pulse": {"step_deg": profile["shared"]["pulse_step_deg"],
                      "period_ms": profile["shared"]["pulse_period_ms"]},
            "maximum_exported_adjacent_step_deg": max_adjacent,
            "kinematic_note": (
                "保留示教 EE 原点 XY，抓放等高、双 hover 等高；任务段锁定 pick 朝向，避免携带瓶子倾斜；用有余量的连续冗余解，"
                "不强制回旧示教关节。入口保留 Track C 安全前段再经 RRT 绕开瓶子，出口至 READY 为关节插值；"
                "升降回程复用同一条路径；任务末端到达父轨迹 READY 后，完整复用其真机已验收 READY→HOME。"),
            "note": "CANDIDATE only; GUI review is mandatory before staged testing.",
        }
        payload = {"meta": meta, "waypoints": waypoints}
        out_path.parent.mkdir(parents=True, exist_ok=True)
        out_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                            encoding="utf-8")
        qtask = {
            "comment": "桌面使用用户确认的 end effector 原点；瓶子 XY 由示教抓取中心推定，完整几何验收仍须通过。",
            "scene_geometry_status": table["geometry_status"],
            "self_collision_disabled_link_pairs": entry_report["self_collision_disabled_link_pairs"],
            "pairs": [{
                "pick": targets["pick"]["grip"].tolist(),
                "place": targets["place"]["grip"].tolist(),
                "actual_pick_grip": actual["pick"]["grip"].tolist(),
                "actual_place_grip": actual["place"]["grip"].tolist(),
                "object_pick_center": pick_center.tolist(),
                "object_place_center": place_center.tolist(),
                "dimensions_m": [task["scene"]["bottle_radius_m"] * 2,
                                 task["scene"]["bottle_radius_m"] * 2,
                                 task["scene"]["bottle_height_m"]],
                "carried_center_offset_grip_m": (
                    pick_center - actual["pick"]["grip"]).tolist(),
                "grasp_capture": {
                    "axis_range_m": profile["grasp_capture_axis_range_m"],
                    "lateral_tolerance_m": profile["grasp_capture_lateral_tolerance_m"],
                    "reference": "USER_CONFIRMED_TAUGHT_GRIP_POINT_ON_BOTTLE",
                    "object_grasp_point_world_m": fks["pick"]["grip"].tolist(),
                    "object_grasp_point_source": "原示教抓取关节 FK 加实测夹爪偏移；必须另查在瓶体内，不是瓶体质心"},
                "object_attachment_mode": "RIGID_FROM_CLOSE",
            }],
            "obstacles": [table],
        }
        qualification_task_out.parent.mkdir(parents=True, exist_ok=True)
        qualification_task_out.write_text(
            json.dumps(qtask, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return {"trajectory": str(out_path),
                "trajectory_sha256": gen.sha256_file(out_path),
                "qualification_task": str(qualification_task_out),
                "motion_points": len(motion_rows),
                "limit_status": limit_report["status"],
                "minimum_raw_margin_deg": limit_report["minimum_raw_margin_deg"]}
    finally:
        if p.isConnected():
            p.disconnect()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--task", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--qualification-task-out", type=Path, required=True)
    args = ap.parse_args(argv)
    print(json.dumps(build(args.task.resolve(), args.out.resolve(),
                           args.qualification_task_out.resolve()),
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
