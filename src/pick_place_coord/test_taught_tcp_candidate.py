#!/usr/bin/env python3
from __future__ import annotations

import json
import copy
import hashlib
import sys
import unittest
from unittest import mock
from pathlib import Path

import numpy as np
import pybullet as p


HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_taught_tcp_candidate as taught
import gen_bottle_servo_candidate as gen
import arm_profiles
import authorize_servo_candidate as authorizer
from robot_mission.preflight import expanded_frames, grasp_capture_geometry, collision_check, grasp_event_qs
import demo_bottle_trajectory as demo

TASK = HERE / "tasks" / "task_bottle_taught_tcp_20260831.json"
TRAJ = HERE / "trajectories" / "candidates" / "traj_bottle_taught_tcp_20260831_CANDIDATE.json"
QTASK = TRAJ.with_name("task_bottle_taught_tcp_20260831_qualification.json")


class TaughtTcpCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.task = json.loads(TASK.read_text(encoding="utf-8"))
        cls.traj = json.loads(TRAJ.read_text(encoding="utf-8"))
        cls.qtask = json.loads(QTASK.read_text(encoding="utf-8"))
        cls.meta = cls.traj["meta"]
        cls.profile = arm_profiles.arm_profile("left")

    def test_original_measurements_are_preserved_separately(self):
        for name in ("pick", "pick_hover", "place_hover", "place"):
            self.assertEqual(self.meta["taught_poses_pb_m"][name]["q_sdk_deg"],
                             self.task[name]["q_sdk_deg"])
        qs = [np.asarray(row["q_sdk_deg"]) for row in self.traj["waypoints"]
              if "q_sdk_deg" in row]
        self.assertFalse(any(np.max(np.abs(q - self.task["place_hover"]["q_sdk_deg"]))
                             < .001 for q in qs), "不得 snap 回旧超余量关节")

    def test_target_ee_xy_is_preserved_z_equalized_and_orientation_locked(self):
        targets = self.meta["adjusted_targets_pb_m"]
        for name, target in targets.items():
            np.testing.assert_allclose(target["ee"][:2],
                                       self.meta["taught_poses_pb_m"][name]["ee"][:2], atol=1e-12)
            np.testing.assert_allclose(target["R_link"], targets["pick"]["R_link"], atol=1e-12)
        self.assertEqual(targets["pick"]["ee"][2], targets["place"]["ee"][2])
        self.assertEqual(targets["pick_hover"]["ee"][2], targets["place_hover"]["ee"][2])
        self.assertEqual(targets["pick"]["ee"][2], self.meta["taught_poses_pb_m"]["pick"]["ee"][2])

    def test_roundtrip_file_hashes_match_current_sources(self):
        for path, key in ((TASK, "source_task_sha256"),
                          (Path(taught.__file__), "generator_sha256")):
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(), self.meta[key])

    def test_dense_limits_and_branch_continuity(self):
        rows = [w for w in self.traj["waypoints"] if "q_sdk_deg" in w]
        step = self.profile["shared"]["pulse_step_deg"]
        first, frames = expanded_frames(self.traj["waypoints"], rows[0]["q_sdk_deg"], step)
        limits = arm_profiles.controller_limit_profile(self.task["controller_limit_profile"])["planning_limits_deg"]
        margin = self.profile["shared"]["minimum_joint_margin_deg"]
        qs = np.asarray(first + [f["q_sdk_deg"] for f in frames])
        self.assertTrue(np.isfinite(qs).all())
        self.assertGreaterEqual(float(np.min(qs - np.asarray(limits)[:, 0])), margin - 1e-8)
        self.assertGreaterEqual(float(np.min(np.asarray(limits)[:, 1] - qs)), margin - 1e-8)
        self.assertLessEqual(float(np.max(np.abs(np.diff(qs, axis=0)))), step + 1e-8)
        task_rows = [r["q_sdk_deg"] for r in rows if r["seg"] not in ("HOME", "HOME->READY")]
        self.assertLessEqual(max(q[5] for q in task_rows), limits[5][1] - self.meta["ik_required_margin_deg"] + 1e-8)

    def test_exported_pose_and_dense_tcp_corridors(self):
        gen.parent_planner.set_arm("left")
        robot = gen.xf.load_xifeng(gui=False)
        try:
            mappings = {"ENTRY->PICK_HOVER": "pick_hover", "PICK_DESCEND": "pick",
                        "TRANSFER": "place_hover", "PLACE_DESCEND": "place"}
            targets = self.meta["adjusted_targets_pb_m"]
            for seg, name in mappings.items():
                row = [r for r in self.traj["waypoints"] if r.get("seg") == seg][-1]
                _grip, tcp, R = taught._fk_tcp(robot, row["q_sdk_deg"], self.task["tcp_link_mm"])
                self.assertLess(float(np.linalg.norm(tcp - targets[name]["tcp"])) * 1000, .2, name)
                angle = gen.cc.rotation_angle_deg(R @ np.asarray(targets[name]["R_link"]).T)
                self.assertLess(angle, self.task["motion"]["cartesian_rotation_tolerance_deg"] + .001, name)
            first_q = next(w["q_sdk_deg"] for w in self.traj["waypoints"] if "q_sdk_deg" in w)
            _, frames = expanded_frames(self.traj["waypoints"], first_q,
                                       self.profile["shared"]["pulse_step_deg"])
            corridors = {"PICK_DESCEND": ("pick_hover", "pick"),
                         "PICK_ASCEND": ("pick", "pick_hover"),
                         "TRANSFER": ("pick_hover", "place_hover"),
                         "PLACE_DESCEND": ("place_hover", "place"),
                         "PLACE_ASCEND": ("place", "place_hover")}
            for frame in frames:
                if frame["seg"] not in corridors:
                    continue
                a_name, b_name = corridors[frame["seg"]]
                a, b = (np.asarray(targets[n]["tcp"]) for n in (a_name, b_name))
                _grip, tcp, _R = taught._fk_tcp(robot, frame["q_sdk_deg"], self.task["tcp_link_mm"])
                t = np.clip(np.dot(tcp-a, b-a) / np.dot(b-a, b-a), 0., 1.)
                self.assertLess(float(np.linalg.norm(tcp - (a+t*(b-a)))) * 1000, .25, frame["seg"])
        finally:
            p.disconnect()

    def test_corrected_scene_and_capture_contract_are_explicit(self):
        self.assertEqual(self.meta["scene_geometry_status"], self.task["table"]["geometry_status"])
        self.assertEqual(self.qtask["scene_geometry_status"], self.meta["scene_geometry_status"])
        capture = self.qtask["pairs"][0]["grasp_capture"]
        self.assertEqual(capture["axis_range_m"], self.profile["grasp_capture_axis_range_m"])
        self.assertEqual(capture["lateral_tolerance_m"], self.profile["grasp_capture_lateral_tolerance_m"])
        self.assertEqual(self.meta["table"]["corner_reference"], "EE_LINK_ORIGIN")
        self.assertAlmostEqual(self.meta["table"]["top_pb_m"],
                               self.meta["table"]["corner_reference_pb_m"][2], places=12)
        self.assertGreater(abs(self.meta["table"]["old_tcp_table_height_error_mm"]), 140)
        self.assertEqual(self.qtask["pairs"][0]["object_attachment_mode"], "RIGID_FROM_CLOSE")

    def test_unknown_scene_cannot_be_authorized_even_with_pass_records(self):
        fake_hash = "a" * 64
        report = {"trajectory_sha256": fake_hash, "verdict": "PASS", "task_sha256": fake_hash,
                  "gates": dict.fromkeys(("arm_gripper_contract", "limits", "collision",
                                           "pybullet_gui_review"), "PASS")}
        gui = {"result": "PASS", "trajectory_sha256": fake_hash,
               "qualification_task_sha256": fake_hash}
        unconfirmed = copy.deepcopy(self.traj)
        unconfirmed["meta"]["scene_geometry_status"] = "UNCONFIRMED"
        with mock.patch.object(authorizer, "_load", side_effect=[unconfirmed, report, gui]), \
                mock.patch.object(authorizer, "sha256_file", return_value=fake_hash), \
                mock.patch.object(Path, "write_text") as write:
            with self.assertRaisesRegex(ValueError, "场景几何尚未确认"):
                authorizer.authorize(TRAJ, TRAJ, TRAJ, TRAJ.with_name("unused_RUN.json"))
            write.assert_not_called()

    def test_gripper_cycle_and_not_authorized(self):
        events = [row["gripper"] for row in self.traj["waypoints"] if "gripper" in row]
        self.assertEqual(events, ["close", "open"])
        self.assertFalse(self.traj["meta"]["real_motion_authorized"])
        self.assertIn("READY->HOME",
                      {row["seg"] for row in self.traj["waypoints"]})

    def test_return_home_is_exact_verified_parent_tail(self):
        parent_path = HERE.parent / self.task["verified_parent"]["path"]
        parent = json.loads(parent_path.read_text(encoding="utf-8"))
        expected = [row["q_sdk_deg"] for row in parent["waypoints"]
                    if row.get("seg") == "READY->HOME"]
        actual = [row["q_sdk_deg"] for row in self.traj["waypoints"]
                  if row.get("seg") == "READY->HOME"]
        self.assertEqual(actual, expected)
        motion = [row["q_sdk_deg"] for row in self.traj["waypoints"]
                  if "q_sdk_deg" in row]
        np.testing.assert_allclose(motion[-1], motion[0], atol=0.0)

    def test_matched_new_scene_review_does_not_require_rewriting_candidate(self):
        fake_hash = "a" * 64
        report = {"trajectory_sha256": fake_hash, "verdict": "PASS", "task_sha256": fake_hash,
                  "gates": dict.fromkeys(("arm_gripper_contract", "limits", "collision",
                                           "pybullet_gui_review"), "PASS")}
        gui = {"result": "PASS", "trajectory_sha256": fake_hash,
               "qualification_task_sha256": fake_hash}
        with mock.patch.object(authorizer, "_load", side_effect=[self.traj, report, gui]), \
                mock.patch.object(authorizer, "sha256_file", return_value=fake_hash), \
                mock.patch.object(Path, "write_text") as write:
            authorizer.authorize(TRAJ, TRAJ, TRAJ, TRAJ.with_name("unused_RUN.json"))
            exported = json.loads(write.call_args.args[0])
            self.assertEqual(exported["waypoints"], self.traj["waypoints"])
            self.assertEqual(exported["meta"]["scene_geometry_status"], "CONFIRMED")
        # 同一路点但旧桌面任务的 GUI 记录也必须拒绝。
        gui["qualification_task_sha256"] = "b" * 64
        with mock.patch.object(authorizer, "_load", side_effect=[self.traj, report, gui]), \
                mock.patch.object(authorizer, "sha256_file", return_value=fake_hash), \
                mock.patch.object(Path, "write_text") as write:
            with self.assertRaisesRegex(ValueError, "任务 SHA 不一致"):
                authorizer.authorize(TRAJ, TRAJ, TRAJ, TRAJ.with_name("unused_RUN.json"))
            write.assert_not_called()

    def test_uvw_matches_joints(self):
        for name, row in self.traj["meta"]["uvw_joint_consistency"].items():
            self.assertLess(row["delta_deg"], 0.01, name)

    def test_capture_uses_taught_grasp_point_not_bottle_centroid(self):
        pair = self.qtask["pairs"][0]
        actual = self.meta["solved_poses_pb_m"]["pick"]
        result = grasp_capture_geometry(self.profile, actual["grip"], actual["R_link"], pair)
        self.assertEqual(result["status"], "PASS")
        self.assertTrue(result["grasp_point_inside_object"])
        self.assertGreater(np.linalg.norm(np.array(result["grasp_point_world_m"]) -
                                          result["object_center_world_m"]), .03)
        self.assertEqual(result, demo._in_grasp_capture(
            self.profile, actual["grip"], actual["R_link"], pair["object_pick_center"], pair))
        invalid = copy.deepcopy(pair)
        invalid["grasp_capture"]["object_grasp_point_world_m"][2] += 1
        self.assertEqual(grasp_capture_geometry(self.profile, actual["grip"],
                                               actual["R_link"], invalid)["status"], "BLOCKED")
        for dims in ([.07, .07, -1], [.07, .07, float("nan")]):
            with self.assertRaises(ValueError):
                grasp_capture_geometry(self.profile, actual["grip"], actual["R_link"],
                                       dict(pair, dimensions_m=dims))

    def test_demo_timeline_matches_executor_dense_frames(self):
        rows = self.traj["waypoints"]
        q0 = next(row["q_sdk_deg"] for row in rows if "q_sdk_deg" in row)
        _, dense = expanded_frames(rows, q0, .4)
        timeline = [row["q_sdk_deg"] for row in demo._timeline(rows, .4) if row["kind"] == "move"]
        np.testing.assert_array_equal(timeline[1:], [row["q_sdk_deg"] for row in dense])

    def test_rigid_bottle_dense_path_includes_self_robot_and_release_checks(self):
        rows = self.traj["waypoints"]
        q0 = next(row["q_sdk_deg"] for row in rows if "q_sdk_deg" in row)
        first, dense = expanded_frames(rows, q0, .4)
        inactive = arm_profiles.arm_profile("right")
        result = collision_check(self.profile, inactive, inactive["home_deg"], first,
                                 dense, self.qtask,
                                 [pair["dimensions_m"] for pair in self.qtask["pairs"]],
                                 grasp_event_qs(rows))
        self.assertEqual(result["status"], "PASS", result["details"]["violations"])
        d = result["details"]
        self.assertTrue(d["completed_dense_check"])
        self.assertEqual(d["checked_trajectory_frames"], len(dense))
        self.assertEqual(d["self_collision_check_mode"], "EXPLICIT_LINK_CLOSEST_POINTS")
        self.assertGreater(d["minimum_clearances_by_kind_m"]["robot_object"], 0)
        center, orn = d["actual_release_poses"][0]
        np.testing.assert_allclose(center, self.qtask["pairs"][0]["object_place_center"], atol=.00005)
        self.assertLess(np.linalg.norm(orn[:3]), .0001)


if __name__ == "__main__":
    unittest.main(verbosity=2)
