"""混合候选回归：仅离线计算，不连接 SDK，不写候选或发布包。"""
from __future__ import annotations

import copy
import json
from pathlib import Path
import sys
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_tabletop_hybrid_candidate as g
import execute_tabletop_pick_place_worlds as worlds_executor


def small_report():
    """只覆盖解析契约的小输入；不含控制器、网络或真实运动数据。"""
    pose = {"position_mm": [700.0, 250.0, 550.0],
            "sdk_world_uvw_deg": [-72.779, 13.241, -6.634],
            "reference_point": "EE_POSE"}
    joints = [0.0] * 7
    return {
        "schema_version": "tabletop_readonly_ik_capture.v1",
        "status": "WORLDS_IK_PRECHECK_PASS_NOT_MOTION_AUTHORIZED",
        "real_motion_authorized": False,
        "session_stopped": True,
        "plan_content": {"stages": [
            {"name": "PICK_ASCEND", "kind": "MOVE_WORLDS", "pose": pose},
        ]},
        "precheck_summary": {
            "segments": [{"stage": "PICK_ASCEND", "dense_points": 1}],
            "dense_points": 1,
        },
        "dense_ik_samples": [{"sample": 1, "q_sdk_deg": joints[:],
                              "worlds_xyzuvw": pose["position_mm"] + pose["sdk_world_uvw_deg"]}],
        "endpoint_ik": {"PICK_ASCEND": joints[:]},
    }


class HybridPureContractTest(unittest.TestCase):
    def test_complete_report_is_unpacked_without_mutation(self):
        report = small_report()
        before = copy.deepcopy(report)
        groups = g.unpack_report(report)
        self.assertEqual(list(groups), ["PICK_ASCEND"])
        self.assertEqual(groups["PICK_ASCEND"], report["dense_ik_samples"])
        self.assertEqual(report, before)

    def test_failed_or_unclosed_report_is_rejected(self):
        for field, value in (
            ("schema_version", "unknown.v1"),
            ("status", "PRECHECK_FAILED"),
            ("session_stopped", False),
            ("real_motion_authorized", True),
        ):
            with self.subTest(field=field):
                report = small_report()
                report[field] = value
                with self.assertRaises(ValueError):
                    g.unpack_report(report)

    def test_incomplete_and_reordered_reports_are_rejected(self):
        mutations = (
            lambda r: r["dense_ik_samples"].clear(),
            lambda r: r["precheck_summary"]["segments"].clear(),
            lambda r: r["precheck_summary"].update(dense_points=2),
            lambda r: r["dense_ik_samples"][0].update(sample=2),
            lambda r: r["precheck_summary"]["segments"][0].update(stage="PLACE_HOVER"),
            lambda r: r["endpoint_ik"].clear(),
        )
        for number, mutate in enumerate(mutations):
            with self.subTest(case=number):
                report = small_report()
                mutate(report)
                with self.assertRaises((ValueError, KeyError)):
                    g.unpack_report(report)

    def test_invalid_dense_counts_are_rejected(self):
        for value in (0, -1, 1.0, True, "1"):
            with self.subTest(value=value):
                report = small_report()
                report["precheck_summary"]["segments"][0]["dense_points"] = value
                with self.assertRaises(ValueError):
                    g.unpack_report(report)

    def test_nonfinite_samples_and_endpoints_are_rejected(self):
        for field in ("q_sdk_deg", "worlds_xyzuvw"):
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(field=field, value=value):
                    report = small_report()
                    report["dense_ik_samples"][0][field][0] = value
                    with self.assertRaises(ValueError):
                        g.unpack_report(report)
        report = small_report()
        report["endpoint_ik"]["PICK_ASCEND"][0] = float("nan")
        with self.assertRaises(ValueError):
            g.unpack_report(report)

    def test_endpoint_disagreement_is_rejected(self):
        for field, change in (("q_sdk_deg", .01), ("worlds_xyzuvw", .01)):
            with self.subTest(field=field):
                report = small_report()
                report["dense_ik_samples"][0][field][0] += change
                with self.assertRaises(ValueError):
                    g.unpack_report(report)

    def test_joint_interpolation_respects_shared_diagnostic_step(self):
        self.assertLessEqual(g.REPLAY_STEP_DEG, g.d.DIAGNOSTIC_JOINT_STEP_DEG)
        start = np.zeros(7)
        for target in (start.copy(), np.array([1.11, -2.76, .2, 0, -.5, 3.01, -1.0])):
            with self.subTest(target=target.tolist()):
                frames = np.asarray([start] + g.interpolate(start, target))
                self.assertLessEqual(np.max(np.abs(np.diff(frames, axis=0))),
                                     g.d.DIAGNOSTIC_JOINT_STEP_DEG + 1e-9)
                np.testing.assert_allclose(frames[-1], target, atol=1e-12)

    def test_sdk_ee_pose_roundtrip_does_not_add_tcp_offset(self):
        supplied = np.array([700., 250., 550., -72.779, 13.241, -6.634])
        converted = g.pose_dict(supplied)
        self.assertEqual(converted["reference_point"], "EE_POSE")
        np.testing.assert_array_equal(g.pose6(converted), supplied)

    def test_predicted_readout_does_not_reapply_physical_grip_offset(self):
        probe = {"sdk_v2_without_session_t": np.array([10., 20., 30.]),
                 "physical_grip": np.array([500., 500., 500.]),
                 "physical_tcp": np.array([800., 800., 800.]),
                 "rotation": np.eye(3)}
        with mock.patch.object(g.d, "probe", return_value=probe):
            predicted = g.predicted_worlds(None, np.zeros(7), np.array([1., 2., 3.]))
        np.testing.assert_array_equal(predicted[:3], [11., 22., 33.])

    def test_mixed_schema_is_not_accepted_by_old_worlds_validator(self):
        with self.assertRaisesRegex(worlds_executor.PlanError, "schema_version"):
            worlds_executor.validate_plan({"schema_version": g.SCHEMA})


class HybridCapturedReportIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report_path = g.WORK / "frame_calibration/records/20260907_trackA_hybrid_reference/inputs/precheck_worlds_report.json"
        if not cls.report_path.is_file():
            raise unittest.SkipTest("现场只读报告未提供；纯函数测试仍执行")
        cls.parent_report = json.loads(cls.report_path.read_text())
        cls.robot = g.d.xf.load_xifeng(gui=False)
        cls.addClassCleanup(g.d.p.disconnect)
        # 若生成器误连 SDK 或下发命令，测试立即失败；不接管现场执行器。
        with mock.patch.object(g.d.ss, "open_session", side_effect=AssertionError("禁止连接 SDK")), \
             mock.patch.object(g.d.ss, "move_worlds", side_effect=AssertionError("禁止运动")), \
             mock.patch.object(g.d.ss, "move_joints_abs", side_effect=AssertionError("禁止运动")):
            cls.plan = g.build(cls.report_path, cls.robot)

    def test_eight_moves_and_three_gripper_events(self):
        stages = self.plan["stages"]
        self.assertEqual(sum(s["kind"] == "MOVE_WORLDS" for s in stages), 6)
        self.assertEqual(sum(s["kind"] == "MOVE_JOINTS" for s in stages), 2)
        self.assertEqual([s["action"] for s in stages if s["kind"] == "GRIPPER"],
                         ["open", "close", "open"])
        self.assertEqual([s["name"] for s in stages if s["kind"] == "MOVE_JOINTS"],
                         ["TRANSFER_MID", "PLACE_HOVER"])

    def test_source_sdk_pick_poses_preserved_without_second_tcp_conversion(self):
        source = {s["name"]: s for s in self.parent_report["plan_content"]["stages"]}
        actual = {s["name"]: s for s in self.plan["stages"]}
        for name in ("PICK_HOVER", "PICK_DESCEND", "PICK_ASCEND", "RETURN_SAFE"):
            with self.subTest(stage=name):
                np.testing.assert_array_equal(g.pose6(actual[name]["pose"]),
                                              g.pose6(source[name]["pose"]))
                self.assertEqual(actual[name]["pose"], source[name]["pose"])

    def test_no_hardware_authorization_and_old_executor_rejects(self):
        self.assertIs(self.plan["real_motion_authorized"], False)
        self.assertIs(self.plan["debian_execution_allowed"], False)
        self.assertEqual(self.plan["status"], "OFFLINE_CANDIDATE_BLOCKED")
        self.assertTrue(self.plan["blockers"])
        self.assertEqual(self.plan["qualification"]["controller_mixed_path_precheck"], "NOT_RUN")
        with self.assertRaises(worlds_executor.PlanError):
            worlds_executor.validate_plan(self.plan)

    def test_gripper_policy_does_not_reuse_invalid_parent_digest(self):
        policy = self.plan["gripper_policy"]
        self.assertNotIn("source_sha256", policy)
        self.assertEqual(policy["parent_source_sha256"],
                         self.parent_report["plan_content"]["gripper_policy"]["source_sha256"])
        self.assertNotEqual(policy["executor_binding"],
                            self.parent_report["plan_content"]["gripper_policy"]["executor_binding"])

    def test_actual_replay_grip_landing_and_hover_return(self):
        frames = self.plan["offline_replay"]["frames"]
        reached = {
            name: g.d.probe(self.robot, next(
                f["q_sdk_deg"] for f in reversed(frames) if f["stage"] == name))["physical_grip"]
            for name in ("PLACE_DESCEND", "PLACE_ASCEND")
        }
        original_place = g.d.probe(
            self.robot, self.parent_report["endpoint_ik"]["PLACE_DESCEND"])["physical_grip"]
        new_hover_stage = next(s for s in self.plan["stages"] if s["name"] == "PLACE_HOVER")
        new_hover = g.d.probe(self.robot, new_hover_stage["q_sdk_deg"])["physical_grip"]
        landing_delta = reached["PLACE_DESCEND"] - original_place
        return_error = reached["PLACE_ASCEND"] - new_hover
        self.assertLessEqual(landing_delta[1], 0.0)
        self.assertLess(np.linalg.norm(return_error), .2)
        # 指标必须来自实际密集回放终帧，而非另算的理想 qdesc/hover。
        metrics = self.plan["qualification"]["kinematics"]
        np.testing.assert_allclose(metrics["new_place_grip_delta_from_original_mm"],
                                   landing_delta, atol=1e-6, rtol=0)
        np.testing.assert_allclose(metrics["place_ascend_grip_return_error_mm"],
                                   return_error, atol=1e-6, rtol=0)

    def test_dense_frames_and_shared_limits(self):
        frames = [f["q_sdk_deg"] for f in self.plan["offline_replay"]["frames"]
                  if "q_sdk_deg" in f]
        qs = np.asarray(frames)
        self.assertTrue(np.isfinite(qs).all())
        self.assertLessEqual(np.max(np.abs(np.diff(qs, axis=0))),
                             g.d.DIAGNOSTIC_JOINT_STEP_DEG + 1e-6)
        metrics = self.plan["qualification"]["kinematics"]
        self.assertGreaterEqual(min(metrics["minimum_margin_per_joint_deg"]),
                                g.d.ss.LIMIT_MARGIN_DEG - 1e-6)
        self.assertLessEqual(metrics["max_new_worlds_model_position_residual_mm"], .2)
        self.assertLessEqual(metrics["max_new_worlds_model_orientation_residual_deg"], .1)


class OptimizedPoseContractTest(unittest.TestCase):
    def test_long_axis_is_derived_from_measured_profile_not_euler_guess(self):
        axis, provenance = g.endpoint_long_axis()
        np.testing.assert_allclose(axis, [1., 0., 0.], atol=1e-10, rtol=0)
        self.assertAlmostEqual(g.long_axis_tilt_deg([-72.779, 13.241, -6.634]), -13.241, places=8)
        self.assertAlmostEqual(g.long_axis_tilt_deg([-72.779, 0., -6.634]), 0., places=8)
        self.assertIn("grip_center_status", provenance)

    def test_unchanged_endpoint_pose_is_exact_passthrough(self):
        pose = g.pose_dict([700., 250., 410., -72.779, 13.241, -6.634])
        self.assertEqual(g.retarget_endpoint_orientation(pose, pose["sdk_world_uvw_deg"]), pose)

    def test_rotating_grasp_keeps_sdk_grasp_center_and_changes_endpoint_xyz(self):
        pose = g.pose_dict([700., 250., 410., -72.779, 13.241, -6.634])
        new = g.retarget_endpoint_orientation(pose, [-70., 12., -8.])
        T, provenance = g.grasp_pose.sdk_endpoint_to_grasp_transform("left")
        before = np.array(pose["position_mm"]) + g.grasp_pose.sdk_uvw_deg_to_matrix(pose["sdk_world_uvw_deg"]) @ T[:3, 3]
        after = np.array(new["position_mm"]) + g.grasp_pose.sdk_uvw_deg_to_matrix(new["sdk_world_uvw_deg"]) @ T[:3, 3]
        np.testing.assert_allclose(after, before, atol=1e-10, rtol=0)
        self.assertFalse(np.allclose(new["position_mm"], pose["position_mm"]))
        self.assertEqual(new["grasp_transform_provenance"], provenance)

    def test_object_pose_is_not_silently_treated_as_endpoint(self):
        for role in ("OBJECT_POSE", "GRASP_POSE", None):
            pose = g.pose_dict([700., 250., 410., -72.779, 13.241, -6.634])
            pose["reference_point"] = role
            with self.assertRaises(ValueError):
                g.retarget_endpoint_orientation(pose, [-70., 12., -8.])


class OptimizedFieldPassIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.evidence = g.WORK / "frame_calibration/records/20260907_trackA_hybrid_reference"
        if not cls.evidence.is_dir():
            raise unittest.SkipTest("现场成功交接包未提供；纯函数测试仍执行")
        cls.parent_path = g.DEFAULT_OUTPUT
        cls.log_path = cls.evidence / "logs/hybrid_trial_run_20260907_175015_346600.json"
        cls.executor_path = cls.evidence / "artifacts/execute_tabletop_hybrid_trial.py"
        cls.parent_sha = g.sha256_file(cls.parent_path)
        cls.parent = json.loads(cls.parent_path.read_text())
        cls.robot = g.d.xf.load_xifeng(gui=False)
        cls.addClassCleanup(g.d.p.disconnect)
        with mock.patch.object(g.d.ss, "open_session", side_effect=AssertionError("禁止连接 SDK")), \
             mock.patch.object(g.d.ss, "move_worlds", side_effect=AssertionError("禁止运动")), \
             mock.patch.object(g.d.ss, "move_joints_abs", side_effect=AssertionError("禁止运动")):
            cls.plan = g.build_optimized(cls.parent_path, cls.log_path, cls.executor_path, cls.robot)
            cls.horizontal_plan = g.build_optimized(cls.parent_path, cls.log_path, cls.executor_path, cls.robot,
                                                    horizontal_long_axis=True)
            cls.taught_plan = g.build_optimized(cls.parent_path, cls.log_path, cls.executor_path, cls.robot,
                                               templates=[[-93.110, 8.776, -5.238]],
                                               placement_follow_grasp_tilt=True)

    def test_field_home_exact_and_one_transfer_movej(self):
        _, _, home = g.load_success_reference(self.parent_path, self.log_path, self.executor_path)
        self.assertEqual(self.plan["schema_version"], g.OPTIMIZED_SCHEMA)
        self.assertEqual(self.plan["home_joints_sdk_deg"], home.tolist())
        movej = [s for s in self.plan["stages"] if s["kind"] == "MOVE_JOINTS"]
        self.assertEqual([s["name"] for s in movej], ["PLACE_HOVER", "RETURN_SAFE"])
        self.assertEqual(movej[-1]["q_sdk_deg"], home.tolist())
        self.assertEqual(sum(s["kind"] == "MOVE_WORLDS" for s in self.plan["stages"]), 5)

    def test_reviewed_start_preserves_matched_field_snapshot_and_first_gui_frame(self):
        run = json.loads(self.log_path.read_text())
        for plan in (self.plan, self.horizontal_plan, self.taught_plan):
            self.assertEqual(plan["reviewed_start"], run["final_snapshot"])
            self.assertEqual(plan["reviewed_start"]["joints"], plan["offline_replay"]["frames"][0]["q_sdk_deg"])
            self.assertEqual(len(plan["reviewed_start"]["worlds"]), 6)

    def test_original_pick_place_targets_and_file_preserved(self):
        original = {s["name"]: s for s in self.parent["stages"]}
        actual = {s["name"]: s for s in self.plan["stages"]}
        for name in ("PICK_HOVER", "PICK_DESCEND", "PICK_ASCEND", "PLACE_DESCEND", "PLACE_ASCEND"):
            np.testing.assert_array_equal(g.pose6(actual[name]["pose"]), g.pose6(original[name]["pose"]))
        np.testing.assert_array_equal(actual["PLACE_HOVER"]["q_sdk_deg"], original["PLACE_HOVER"]["q_sdk_deg"])
        self.assertEqual(g.sha256_file(self.parent_path), self.parent_sha)
        self.assertFalse(self.plan["pose_selection"]["orientation_changed"])
        self.assertEqual(self.plan["pose_selection"]["grip_center_uncertainty_mm"], 10.)

    def test_hash_mismatch_refuses_input(self):
        with mock.patch.object(g, "sha256_file", return_value="wrong"):
            with self.assertRaisesRegex(ValueError, "SHA"):
                g.load_success_reference(self.parent_path, self.log_path, self.executor_path)

    def test_dense_path_shared_limits_and_explicit_height_tradeoff(self):
        metrics = self.plan["qualification"]["kinematics"]
        self.assertGreaterEqual(min(metrics["minimum_margin_per_joint_deg"]), g.d.ss.LIMIT_MARGIN_DEG)
        self.assertLessEqual(metrics["max_dense_joint_step_deg"], g.REPLAY_STEP_DEG + 1e-6)
        self.assertTrue(metrics["transfer_y_monotone_negative"])
        self.assertLessEqual(metrics["max_new_worlds_model_position_residual_mm"], .2)
        self.assertEqual(len(metrics["field_start_direct_transfer_regression"]), 5)
        for case in metrics["field_start_direct_transfer_regression"]:
            self.assertEqual(case["start_source"], "PICK_ASCEND.after.joints_SETTLED")
            self.assertTrue(case["diagnostic"]["transfer_y_monotone_negative"])
            self.assertGreaterEqual(min(case["diagnostic"]["minimum_margin_per_joint_deg"]), g.d.ss.LIMIT_MARGIN_DEG)
        # 数值范围是现有五次回读的回归锚点，不是允许贴桌/碰框的安全阈值。
        self.assertGreater(abs(metrics["transfer_grip_delta_xyz_min_mm"][2]), metrics["prior_two_movej_model_max_drop_mm"])

    def test_not_authorized_by_parent_hardware_pass_or_gui_playback(self):
        self.assertFalse(self.plan["real_motion_authorized"])
        self.assertFalse(self.plan["debian_execution_allowed"])
        self.assertEqual(self.plan["motion_qualification"]["status"], "BLOCKED")
        self.assertEqual(self.plan["qualification"]["gui_review"], "PENDING")
        self.assertEqual(self.plan["qualification"]["robot_environment_collision"], "NOT_QUALIFIED")

    def test_horizontal_candidate_preserves_sdk_grip_for_all_six_changed_poses(self):
        plan = self.horizontal_plan
        metrics = plan["qualification"]["kinematics"]
        self.assertTrue(plan["pose_selection"]["orientation_changed"])
        self.assertAlmostEqual(plan["pose_selection"]["selected_endpoint_uvw_deg"][1], 0., places=8)
        self.assertEqual(len(plan["pose_selection"]["templates"]), 4)
        self.assertEqual(len(metrics["sdk_grasp_center_preservation_error_mm"]), 6)
        self.assertLess(max(metrics["sdk_grasp_center_preservation_error_mm"].values()), 1e-8)
        self.assertEqual(plan["home_joints_sdk_deg"], self.plan["home_joints_sdk_deg"])
        self.assertGreaterEqual(min(metrics["minimum_margin_per_joint_deg"]), g.d.ss.LIMIT_MARGIN_DEG)
        self.assertTrue(metrics["transfer_y_monotone_negative"])
        self.assertLess(max(abs(v) for v in metrics["transfer_tool_long_axis_tilt_range_deg"]), .5)
        self.assertEqual(plan["motion_qualification"]["status"], "BLOCKED")
        self.assertIn("NOT a physical-scene calibration PASS", metrics["sdk_vs_pb_physical_grasp_warning"])

    def test_taught_uvw_is_exact_and_placement_keeps_field_yaw(self):
        plan = self.taught_plan
        stages = {s["name"]: s for s in plan["stages"]}
        parent = {s["name"]: s for s in self.parent["stages"]}
        for name in ("PICK_HOVER", "PICK_DESCEND", "PICK_ASCEND"):
            self.assertEqual(stages[name]["pose"]["sdk_world_uvw_deg"], [-93.110, 8.776, -5.238])
        for name in ("PLACE_HOVER", "PLACE_DESCEND", "PLACE_ASCEND"):
            key = "expected_endpoint" if name == "PLACE_HOVER" else "pose"
            self.assertEqual(stages[name][key]["sdk_world_uvw_deg"][:2], [-93.110, 8.776])
            self.assertEqual(stages[name][key]["sdk_world_uvw_deg"][2], parent[name][key]["sdk_world_uvw_deg"][2])
        self.assertEqual(plan["home_joints_sdk_deg"], self.plan["home_joints_sdk_deg"])
        self.assertEqual(plan["pose_selection"]["placement_orientation_policy"],
                         "FOLLOW_SELECTED_GRASP_UV_PRESERVE_FIELD_PLACEMENT_W")

    def test_taught_pose_preserves_six_grasp_targets_and_requalifies_path(self):
        plan = self.taught_plan
        metrics = plan["qualification"]["kinematics"]
        self.assertEqual(len(metrics["sdk_grasp_center_preservation_error_mm"]), 6)
        self.assertLess(max(metrics["sdk_grasp_center_preservation_error_mm"].values()), 1e-8)
        self.assertGreaterEqual(min(metrics["minimum_margin_per_joint_deg"]), g.d.ss.LIMIT_MARGIN_DEG)
        self.assertLessEqual(metrics["max_dense_joint_step_deg"], g.REPLAY_STEP_DEG + 1e-6)
        self.assertTrue(metrics["transfer_y_monotone_negative"])
        self.assertEqual(plan["qualification"]["gui_review"], "PENDING")
        self.assertEqual(plan["motion_qualification"]["status"], "BLOCKED")
        self.assertFalse(plan["real_motion_authorized"])
        self.assertEqual(g.sha256_file(self.parent_path), self.parent_sha)

    def test_placement_follow_tilt_requires_explicit_template(self):
        with self.assertRaisesRegex(ValueError, "显式姿态模板"):
            g.build_optimized(self.parent_path, self.log_path, self.executor_path, self.robot,
                              placement_follow_grasp_tilt=True)


if __name__ == "__main__":
    unittest.main()
