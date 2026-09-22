#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import math
import unittest
from pathlib import Path

import numpy as np

import arm_profiles
from robot_mission.obstacle_scene_support import (
    adapt_tabletop_plan_to_pb_task,
    adapted_obstacle_task_gate,
    build_open_container_obstacles,
    build_static_obstacle,
    clearance_series_gate,
    executor_sequence_gate,
    placed_object_from_tabletop_plan,
    placement_fit_gate,
    quaternion_matrix_xyzw,
    released_object_clearance_gate,
    tabletop_plan_closure_gate,
    upright_axis_aligned_orientation_gate,
)
from robot_mission.preflight import (collision_check, expanded_frames,
                                     grasp_event_qs)
from robot_mission.tabletop_plan import build_tabletop_plan
from robot_mission.test_tabletop_plan import request as tabletop_request


WORK = Path(__file__).resolve().parent.parent
TRAJECTORY = (WORK / "pick_place_coord" / "trajectories" / "candidates" /
              "traj_bottle_taught_tcp_20260831_CANDIDATE.json")
QUALIFICATION_TASK = TRAJECTORY.with_name(
    "task_bottle_taught_tcp_20260831_qualification.json")


class ObstacleSceneSupportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.profile = arm_profiles.arm_profile("left")
        cls.inactive = arm_profiles.arm_profile("right")
        cls.trajectory = json.loads(TRAJECTORY.read_text(encoding="utf-8"))
        cls.task = json.loads(QUALIFICATION_TASK.read_text(encoding="utf-8"))
        cls.rows = cls.trajectory["waypoints"]
        cls.first_q = next(row["q_sdk_deg"] for row in cls.rows
                           if "q_sdk_deg" in row)
        cls.pair = cls.task["pairs"][0]
        cls.table_top = cls.task["obstacles"][0]["top_pb_m"]
        cls.plan = build_tabletop_plan(tabletop_request())
        cls.frame_gate = {
            "schema_version": "frame_gate_report.v1",
            "model": "A_TRANSLATION_ONLY",
            "transform": (
                "p_sdk_mm = p_pb_ee_link_origin_mm + t_session_mm"),
            "arm": "left",
            "reference": {
                "path": "synthetic-reference.json", "sha256": "0" * 64,
                "sample_count": 99, "t_session_mm": [0.0, 0.0, 0.0],
                "residual_rms_mm": 1.0, "residual_max_mm": 2.0,
                "residual_mean_abs_axis_mm": [1.0, 1.0, 1.0],
            },
            "candidate": {
                "path": "synthetic-candidate.json", "sha256": "1" * 64,
                "captured_at": "2026-09-02T00:00:00",
                "sample_count": 99, "t_session_mm": [0.0, 0.0, 0.0],
                "residual_rms_mm": 1.0, "residual_max_mm": 2.0,
                "residual_mean_abs_axis_mm": [1.0, 1.0, 1.0],
            },
            "drift_xyz_mm": [0.0, 0.0, 0.0],
            "drift_euclidean_mm": 0.0, "age_days": 0.0,
            "thresholds": {"axis_abs_lt_mm": 5.0,
                           "euclidean_lt_mm": 8.0,
                           "age_lte_days": 7.0},
            "checks": {"axis_drift": True, "euclidean_drift": True,
                       "freshness": True},
            "verdict": "PASS",
        }
        cls.pulse = {
            "step_deg": cls.profile["shared"]["pulse_step_deg"],
            "period_ms": cls.profile["shared"]["pulse_period_ms"],
        }
        cls.trajectory_v2 = {
            "schema_version": "trajectory.v2",
            "trajectory_id": "obstacle-scaffold-track-c",
            "contract": {
                "arm": "left", "arm_id": 1, "gripper_id": 2,
                "ee_link_id": cls.profile["ee_link_id"],
                "tcp_id": "left-sdk-worlds-endpoint",
                "joint_angle_unit": "degree",
                "sdk_from_urdf_sign": cls.profile["sdk_from_urdf_sign"],
                "executor": "track_c_armPluseToServo",
                "qualification_status": "CANDIDATE",
            },
            "provenance": {
                "observation_sha256": "0" * 64,
                "task_sha256": "1" * 64,
                "adapted_task_sha256": "2" * 64,
                "planner_id": "TEST_SCAFFOLD_ONLY",
                "calibration_id": "synthetic-frame-gate",
                "dependencies_sha256": "3" * 64,
                "derived_trajectory_sha256": "4" * 64,
            },
            "pulse": {**cls.pulse, "dense_frame_count": 1},
            "safety": {
                "minimum_joint_margin_deg": 5,
                "collision_report_sha256": "5" * 64,
                "first_point_checked": True,
                "carried_object_checked": True,
            },
            "waypoints": copy.deepcopy(cls.rows),
        }

    def _container(self, inner=(.20, .20), yaw_deg=0.0,
                   inflation=.003):
        angle = math.radians(yaw_deg) / 2.0
        quaternion = [0.0, 0.0, math.sin(angle), math.cos(angle)]
        return build_open_container_obstacles(
            self.profile, container_id="target-bin",
            floor_center_world_m=[self.pair["object_place_center"][0],
                                  self.pair["object_place_center"][1],
                                  self.table_top],
            quaternion_xyzw=quaternion,
            inner_size_xy_m=inner, wall_thickness_xy_m=(.02, .02),
            wall_height_m=.12, floor_thickness_m=.02,
            inflation_m=inflation,
            support_contact_tolerance_m=5e-5)

    def _plan_container(self, plan=None, inner=(.20, .20), yaw_deg=0.0,
                        inflation=.005, frame_gate=None):
        plan = plan or self.plan
        placed = plan["resolved"]["place_object_after_release"]
        translation = np.asarray(
            (frame_gate or self.frame_gate)["candidate"]["t_session_mm"],
            float)
        center = (np.asarray(placed["position_mm"], float) -
                  translation) / 1000.0
        dimensions = np.asarray(plan["input"]["dimensions_mm"], float) / 1000.0
        floor = center.copy(); floor[2] -= dimensions[2] / 2.0
        angle = math.radians(yaw_deg) / 2.0
        return build_open_container_obstacles(
            self.profile, container_id="target-bin",
            floor_center_world_m=floor,
            quaternion_xyzw=[0, 0, math.sin(angle), math.cos(angle)],
            inner_size_xy_m=inner, wall_thickness_xy_m=(.02, .02),
            wall_height_m=.12, floor_thickness_m=.02,
            inflation_m=inflation, support_contact_tolerance_m=5e-5)

    def _static_obstacles(self):
        specs = [
            ("table-slab", "table_slab", [.6, .1, .27], [1.2, .8, .10]),
            ("table-leg-1", "table_leg", [.05, -.25, .1], [.08, .08, .2]),
            ("table-leg-2", "table_leg", [1.15, -.25, .1], [.08, .08, .2]),
            ("table-leg-3", "table_leg", [.05, .45, .1], [.08, .08, .2]),
            ("table-leg-4", "table_leg", [1.15, .45, .1], [.08, .08, .2]),
            ("under-cabinet-1", "under_table_obstacle",
             [-1.0, -1.0, .2], [.2, .3, .4]),
        ]
        return [build_static_obstacle(
            self.profile, obstacle_id=obstacle_id, scene_role=role,
            origin_world_m=origin, quaternion_xyzw=[0, 0, 0, 1],
            dimensions_m=dimensions, inflation_m=.02)
            for obstacle_id, role, origin, dimensions in specs]

    def _adapted_task(self, container=None, plan=None, frame_gate=None):
        return adapt_tabletop_plan_to_pb_task(
            plan or self.plan, frame_gate or self.frame_gate, self.task,
            static_obstacles=self._static_obstacles(),
            container=container or self._plan_container(
                plan, frame_gate=frame_gate))

    def _adapted_gate(self, task, container, plan=None, frame_gate=None,
                      safety_template=None):
        return adapted_obstacle_task_gate(
            task, container_id="target-bin", tabletop_plan=plan or self.plan,
            frame_gate_report=frame_gate or self.frame_gate,
            container=container, profile=self.profile,
            inactive_profile=self.inactive,
            safety_template=safety_template or self.task)

    def test_shared_tabletop_plan_is_the_only_pose_contract(self):
        plan = self.plan
        placed = placed_object_from_tabletop_plan(plan)
        self.assertEqual(placed["plan_id"], plan["plan_id"])
        self.assertEqual(placed["source_frame"], "sdk_world")
        self.assertEqual(placed["dimensions_mm"], plan["input"]["dimensions_mm"])
        self.assertAlmostEqual(np.linalg.norm(placed["quaternion_xyzw"]), 1.0)
        expected = plan["resolved"]["place_object_after_release"]
        np.testing.assert_allclose(placed["position_sdk_world_mm"],
                                   expected["position_mm"])
        np.testing.assert_allclose(
            quaternion_matrix_xyzw(placed["quaternion_xyzw"]),
            quaternion_matrix_xyzw(expected["quaternion_xyzw"]))
        self.assertEqual(tabletop_plan_closure_gate(plan)["status"], "PASS")
        self.assertIn("PB<->SDK frame gate", placed["note"])

        inconsistent = copy.deepcopy(plan)
        inconsistent["resolved"]["place_object_after_release"][
            "position_mm"][0] += 1.0
        self.assertEqual(tabletop_plan_closure_gate(
            inconsistent)["status"], "BLOCKED")
        with self.assertRaises(ValueError):
            placed_object_from_tabletop_plan(inconsistent)
        wrong_event = copy.deepcopy(plan)
        next(stage for stage in wrong_event["stages"]
             if stage["name"] == "CLOSE_AT_PICK")["action"] = "open"
        self.assertEqual(tabletop_plan_closure_gate(
            wrong_event)["status"], "BLOCKED")

    def test_open_container_is_floor_plus_four_rims_and_fit_is_6d(self):
        model = self._container(inflation=.003)
        roles = [row["scene_role"] for row in model["obstacles"]]
        self.assertEqual(roles.count("target_container_floor"), 1)
        self.assertEqual(roles.count("target_container_rim"), 4)
        self.assertTrue(all(row["geometry_approximation"] ==
                            "CONSERVATIVE_AABB_FROM_OBB"
                            for row in model["obstacles"]))
        floor = next(row for row in model["obstacles"]
                     if row["scene_role"] == "target_container_floor")
        rim = next(row for row in model["obstacles"]
                   if row["scene_role"] == "target_container_rim")
        self.assertEqual(floor["inflation_m"][2], 0.0)
        self.assertEqual(rim["inflation_m"][2], .003)
        R = quaternion_matrix_xyzw(model["quaternion_xyzw"])
        center = (np.asarray(model["floor_center_world_m"]) +
                  R @ np.array([0.0, 0.0, .10]))
        fit = placement_fit_gate(
            model, object_center_world_m=center,
            object_dimensions_m=(.07, .07, .20),
            object_quaternion_xyzw=model["quaternion_xyzw"],
            minimum_wall_clearance_m=.01)
        self.assertEqual(fit["status"], "PASS", fit)
        shifted = center + R @ np.array([.08, 0.0, 0.0])
        self.assertEqual(placement_fit_gate(
            model, object_center_world_m=shifted,
            object_dimensions_m=(.07, .07, .20),
            object_quaternion_xyzw=model["quaternion_xyzw"],
            minimum_wall_clearance_m=.01)["status"], "BLOCKED")

    def test_rotated_open_container_is_blocked_until_obb_preflight_exists(self):
        model = self._plan_container(yaw_deg=30.0)
        self.assertEqual(model["orientation_qualification"]["status"],
                         "BLOCKED")
        task = self._adapted_task(model)
        result = self._adapted_gate(task, model)
        self.assertEqual(result["status"], "BLOCKED")
        self.assertEqual(result["scaffold_status"], "BLOCKED")
        self.assertFalse(result["checks"][
            "target_container_orientation_supported"])
        yaw90 = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        self.assertEqual(upright_axis_aligned_orientation_gate(
            yaw90)["status"], "PASS")
        self.assertEqual(upright_axis_aligned_orientation_gate(
            [1, 0, 0, 0])["status"], "BLOCKED")
        with self.assertRaises(ValueError):
            self._plan_container(inflation=(.003, .004, .003))

    def test_table_edge_and_under_table_boxes_use_same_profile_clearance(self):
        obstacle = build_static_obstacle(
            self.profile, obstacle_id="under-cabinet-1",
            scene_role="under_table_obstacle",
            origin_world_m=[.4, .1, .7], quaternion_xyzw=[0, 0, 0, 1],
            dimensions_m=[.2, .3, .4], inflation_m=.02)
        self.assertEqual(obstacle["minimum_robot_clearance_m"],
                         self.profile["minimum_table_clearance_m"])
        self.assertEqual(obstacle["minimum_gripper_clearance_m"],
                         self.profile["minimum_table_clearance_m"])
        self.assertEqual(obstacle["inflation_m"], [.02, .02, .02])

    def test_adapted_task_requires_rigid_object_self_pairs_and_open_bin(self):
        container = self._plan_container()
        task = self._adapted_task(container)
        self.assertNotIn("pairs", task)
        self.assertFalse(task["real_motion_authorized"])
        result = self._adapted_gate(task, container)
        self.assertEqual(result["status"], "BLOCKED", result)
        self.assertEqual(result["scaffold_status"], "PASS", result)
        self.assertTrue(result["authorization_blockers"])
        task["adapted_task"].pop("self_collision_disabled_link_pairs")
        self.assertEqual(self._adapted_gate(
            task, container)["scaffold_status"], "BLOCKED")

        task = self._adapted_task(container)
        task["adapted_task"]["pairs"][0]["object_place_center"][0] += .01
        result = self._adapted_gate(task, container)
        self.assertEqual(result["scaffold_status"], "BLOCKED")
        self.assertFalse(result["checks"][
            "adapted_pair_rederived_exactly_from_plan_and_frame_gate"])

        task = self._adapted_task(container)
        target_piece = next(item for item in task["adapted_task"]["obstacles"]
                            if str(item["source_obstacle_id"]).startswith(
                                "target-bin:"))
        target_piece["center"][0] += .01
        result = self._adapted_gate(task, container)
        self.assertFalse(result["checks"][
            "target_container_obstacles_match_bound_model_exactly"])

        mismatched_frame = copy.deepcopy(self.frame_gate)
        mismatched_frame["candidate"]["t_session_mm"][0] = 10.0
        result = self._adapted_gate(
            self._adapted_task(container), container,
            frame_gate=mismatched_frame)
        self.assertEqual(result["scaffold_status"], "BLOCKED")
        self.assertFalse(result["checks"][
            "plan_frame_pair_obstacle_binding_hashes_match"])

        translated_frame = copy.deepcopy(self.frame_gate)
        offset_mm = np.array([100.0, -50.0, 20.0])
        translated_frame["reference"]["t_session_mm"] = offset_mm.tolist()
        translated_frame["candidate"]["t_session_mm"] = offset_mm.tolist()
        translated_container = self._plan_container(
            frame_gate=translated_frame)
        translated = self._adapted_task(
            translated_container, frame_gate=translated_frame)
        original_center = np.asarray(self._adapted_task(
            container)["adapted_task"]["pairs"][0]["object_place_center"])
        translated_center = np.asarray(
            translated["adapted_task"]["pairs"][0]["object_place_center"])
        np.testing.assert_allclose(
            translated_center, original_center - offset_mm / 1000.0)
        self.assertEqual(self._adapted_gate(
            translated, translated_container, frame_gate=translated_frame)[
                "scaffold_status"], "PASS")

        changed_template = copy.deepcopy(self.task)
        changed_template["pairs"][0]["grasp_capture"][
            "lateral_tolerance_m"] += .001
        self.assertEqual(self._adapted_gate(
            self._adapted_task(container), container,
            safety_template=changed_template)["scaffold_status"], "BLOCKED")

    def test_executor_gate_eliminates_unknown_movej_to_first_point(self):
        common = dict(
            bound_inactive_q_sdk_deg=self.inactive["home_deg"],
            live_inactive_q_sdk_deg=self.inactive["home_deg"],
            trajectory_document=self.trajectory_v2,
            runtime_step_deg=self.pulse["step_deg"],
            runtime_period_ms=self.pulse["period_ms"],
            exact_start_tolerance_deg=0.0)
        result = executor_sequence_gate(
            self.rows, self.first_q, self.profile,
            live_current_q_sdk_deg=self.first_q, **common)
        self.assertEqual(result["status"], "BLOCKED", result)
        self.assertEqual(result["scaffold_status"], "PASS", result)
        changed = list(self.first_q); changed[0] += .01
        blocked = executor_sequence_gate(
            self.rows, self.first_q, self.profile,
            live_current_q_sdk_deg=changed, **common)
        self.assertEqual(blocked["status"], "BLOCKED")
        self.assertEqual(blocked["scaffold_status"], "BLOCKED")
        self.assertFalse(blocked["checks"][
            "live_current_is_exact_first_point_no_movej_sweep"])

        inactive_changed = list(self.inactive["home_deg"])
        inactive_changed[0] += .01
        inactive_common = dict(common)
        inactive_common["live_inactive_q_sdk_deg"] = inactive_changed
        blocked = executor_sequence_gate(
            self.rows, self.first_q, self.profile,
            live_current_q_sdk_deg=self.first_q, **inactive_common)
        self.assertFalse(blocked["checks"][
            "live_inactive_equals_bound_snapshot"])

        wrong_runtime = dict(common)
        wrong_runtime["runtime_step_deg"] = self.pulse["step_deg"] / 2.0
        blocked = executor_sequence_gate(
            self.rows, self.first_q, self.profile,
            live_current_q_sdk_deg=self.first_q, **wrong_runtime)
        self.assertFalse(blocked["checks"][
            "runtime_pulse_matches_bound_trajectory"])
        open_index = next(i for i, row in enumerate(self.rows)
                          if row.get("gripper") == "open")
        truncated = self.rows[:open_index + 1]
        self.assertEqual(executor_sequence_gate(
            truncated, self.first_q, self.profile,
            live_current_q_sdk_deg=self.first_q,
            **common)["scaffold_status"], "BLOCKED")

    def test_small_opening_can_fit_bottle_but_dense_gripper_rim_check_blocks(self):
        container = self._container(inner=(.10, .10))
        object_center = self.pair["object_place_center"]
        fit = placement_fit_gate(
            container, object_center_world_m=object_center,
            object_dimensions_m=self.pair["dimensions_m"],
            minimum_wall_clearance_m=0.0)
        self.assertEqual(fit["status"], "PASS", fit)

        task = copy.deepcopy(self.task)
        task["obstacles"].extend(container["obstacles"])
        _first, dense = expanded_frames(self.rows, self.first_q,
                                        self.profile["shared"]["pulse_step_deg"])
        place_frames = [row for row in dense if row["seg"] in
                        ("PLACE_DESCEND", "PLACE_ASCEND")]
        collision = collision_check(
            self.profile, self.inactive, self.inactive["home_deg"], [],
            place_frames, task, [self.pair["dimensions_m"]],
            grasp_event_qs(self.rows))
        self.assertEqual(collision["status"], "BLOCKED")
        rim_hits = [row for row in collision["details"]["violations"]
                    if str(row.get("obstacle", "")).startswith("target-bin:")]
        self.assertTrue(rim_hits, collision["details"]["violations"])

    def test_released_bottle_must_exit_then_never_reenter_on_return(self):
        first_path, dense = expanded_frames(
            self.rows, self.first_q, self.pulse["step_deg"])
        collision = collision_check(
            self.profile, self.inactive, self.inactive["home_deg"],
            first_path, dense, self.task, [self.pair["dimensions_m"]],
            grasp_event_qs(self.rows))
        self.assertEqual(collision["status"], "PASS", collision)
        result = released_object_clearance_gate(
            self.profile, self.inactive, self.inactive["home_deg"], self.rows,
            collision, 0, trajectory_document=self.trajectory_v2,
            minimum_clearance_m=.001,
            exit_progress_tolerance_m=.0001,
            maximum_exit_frames=40)
        self.assertEqual(result["status"], "BLOCKED", result)
        self.assertEqual(result["actual_release_pose"],
                         collision["details"]["actual_release_poses"][0])
        self.assertTrue(result["checks"]["release_event_frame_included"])
        self.assertFalse(result["checks"][
            "inactive_gripper_proxy_has_measured_verified_evidence"])
        self.assertIsNotNone(result["first_clear_frame"])
        self.assertEqual(result["checked_frames"],
                         result["planned_post_release_frames"])

    def test_clearance_state_machine_rejects_deeper_exit_and_zero_contact(self):
        deeper = clearance_series_gate(
            [-.01, -.02, .002], [.01, .01, .01],
            minimum_clearance_m=.001, exit_progress_tolerance_m=0.0,
            maximum_exit_frames=3)
        self.assertEqual(deeper["status"], "BLOCKED")
        self.assertTrue(deeper["exit_regressions"])
        touching = clearance_series_gate(
            [0.0, 0.0, 0.0], [.01, .01, .01],
            minimum_clearance_m=.001, exit_progress_tolerance_m=0.0,
            maximum_exit_frames=3)
        self.assertEqual(touching["status"], "BLOCKED")
        safe = clearance_series_gate(
            [-.01, -.005, .002, .01], [.01, .01, .01, .01],
            minimum_clearance_m=.001, exit_progress_tolerance_m=0.0,
            maximum_exit_frames=3)
        self.assertEqual(safe["status"], "PASS")
        cumulative = clearance_series_gate(
            [-.0100, -.01009, -.01018, -.01027, .002],
            [.01, .01, .01, .01, .01],
            minimum_clearance_m=.001, exit_progress_tolerance_m=.0001,
            maximum_exit_frames=5)
        self.assertEqual(cumulative["status"], "BLOCKED")
        self.assertTrue(cumulative["exit_regressions"])
        reentry = clearance_series_gate(
            [-.01, .002, 0.0], [.01, .01, .01],
            minimum_clearance_m=.001, exit_progress_tolerance_m=0.0,
            maximum_exit_frames=3)
        self.assertEqual(reentry["status"], "BLOCKED")
        self.assertTrue(reentry["reentries"])
        inactive_hit = clearance_series_gate(
            [-.01, .002, .01], [.01, 0.0, .01],
            minimum_clearance_m=.001, exit_progress_tolerance_m=0.0,
            maximum_exit_frames=3)
        self.assertEqual(inactive_hit["status"], "BLOCKED")
        self.assertTrue(inactive_hit["inactive_violations"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
