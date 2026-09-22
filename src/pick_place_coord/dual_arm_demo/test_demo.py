"""双臂 GUI Demo 的离线契约与物体绑定回归；不加载 SDK。"""
from __future__ import annotations

import copy
import importlib.util
import json
from pathlib import Path
import unittest
from unittest import mock

import numpy as np
import pybullet as p


HERE = Path(__file__).resolve().parent
SPEC = importlib.util.spec_from_file_location("dual_arm_sim_demo", HERE / "demo.py")
demo = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(demo)

# 此字节身份是被复用的现场 A 父轨迹；Demo 构建不得修改它。
A_PARENT_SHA256 = "e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59"


class ContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not demo.PLAN.is_file() or not demo.REPORT.is_file():
            raise RuntimeError("先执行 demo.py --build，测试仅读取构建结果")
        cls.plan = json.loads(demo.PLAN.read_text())

    def candidate(self):
        return copy.deepcopy(self.plan)

    def test_current_artifacts_validate_and_parent_bytes_are_unchanged(self):
        plan, report = demo.load_validated()
        self.assertEqual(demo.sha(demo.SOURCE), A_PARENT_SHA256)
        self.assertEqual(plan["provenance"]["source_sha256"], A_PARENT_SHA256)
        self.assertEqual(report["plan_sha256"], demo.sha(demo.PLAN))
        self.assertEqual(report["dense_frames_checked"], len(plan["frames"]))
        self.assertEqual(plan["scene"]["layout"], "SEPARATE_TABLETOP_TARGETS_NO_BIN")
        self.assertFalse(any("bin" in key.lower() for key in plan["scene"]))
        self.assertFalse(plan["real_motion_authorized"])
        self.assertFalse(plan["debian_execution_allowed"])

    def test_original_left_pick_prefix_is_preserved_and_transfer_is_replanned(self):
        source = json.loads(demo.SOURCE.read_text())["offline_replay"]["frames"]
        close_index = next(i for i, row in enumerate(source) if row.get("action") == "close")
        by_source = {}
        for frame in self.plan["frames"]:
            if frame["source_index"] is not None:
                by_source.setdefault(frame["source_index"], []).append(frame)
        self.assertEqual(set(by_source), set(range(close_index + 1)))
        for index, row in enumerate(source[:close_index + 1]):
            with self.subTest(index=index, stage=row["stage"]):
                generated = by_source[index]
                self.assertTrue(all(f["derivation"] == "PRESERVED_A_PICK_PREFIX" for f in generated))
                self.assertTrue(all(f["stage"] == row["stage"] for f in generated))
                if "q_sdk_deg" in row:
                    np.testing.assert_allclose(generated[-1]["q_sdk_deg"]["left"],
                                               row["q_sdk_deg"], atol=1e-12, rtol=0)
                else:
                    events = [f for f in generated if "event" in f]
                    self.assertEqual(len(events), 1)
                    self.assertEqual(events[0]["event"], row["action"])
                    self.assertEqual(events[0]["event_arms"], ["left", "right"])
        replanned = [frame for frame in self.plan["frames"] if frame["source_index"] is None]
        self.assertTrue(replanned)
        required_stages = {"PICK_ASCEND", "PLACE_HOVER", "PLACE_DESCEND", "OPEN_AT_PLACE",
                           "PLACE_ASCEND", "RETREAT_CLEAR_OBJECT", "RETURN_SAFE"}
        self.assertTrue(required_stages.issubset({f["stage"] for f in replanned}))
        first_new = next(i for i, f in enumerate(self.plan["frames"]) if f["source_index"] is None)
        self.assertTrue(all(f["source_index"] is None for f in self.plan["frames"][first_new:]))

    def test_actual_demo_arms_move_together_not_serially(self):
        arrays = demo.validate_contract(self.plan)
        moving = {arm: np.max(np.abs(np.diff(q, axis=0)), axis=1) > 1e-8
                  for arm, q in arrays.items()}
        either = moving["left"] | moving["right"]
        both = moving["left"] & moving["right"]
        self.assertGreater(np.count_nonzero(either), 0)
        self.assertGreater(np.count_nonzero(both) / np.count_nonzero(either), 0.95)
        report = json.loads(demo.REPORT.read_text())
        self.assertEqual(report["simultaneously_moving_intervals"], int(np.count_nonzero(both)))

    def test_dense_report_has_no_cross_arm_or_environment_penetrations(self):
        report = json.loads(demo.REPORT.read_text())
        self.assertEqual(report["plan_sha256"], demo.sha(demo.PLAN))
        counts = report["collision_frames_by_category"]
        for category in ("cross_arm_mesh", "cross_arm_envelope", "environment"):
            with self.subTest(category=category):
                self.assertEqual(counts.get(category, 0), 0)
                self.assertFalse(any(row["category"] == category for row in report["collision_pairs"]))
        self.assertEqual(report["task_checks"]["cross_arm"], "PASS_SAMPLED_MODEL")
        self.assertEqual(report["task_checks"]["environment"], "PASS_SAMPLED_MODEL")

    def test_return_after_clear_retreat_does_not_recontact_released_objects(self):
        report = json.loads(demo.REPORT.read_text())
        stages = {frame["stage"] for frame in self.plan["frames"]}
        return_stages = {stage for stage in stages if stage.startswith("RETURN_")}
        self.assertTrue({"RETURN_HOME_HOVER", "RETURN_HOME_DESCEND", "RETURN_SAFE"}.issubset(return_stages))
        for stage in return_stages | {"RETREAT_CLEAR_OBJECT"}:
            with self.subTest(stage=stage):
                counts = report["collision_frames_by_stage"].get(stage, {})
                self.assertEqual(counts.get("own_object_robot", 0), 0)
                self.assertEqual(counts.get("unheld_object_gripper", 0), 0)
        # 这些局部 PASS 不得把仍有模型交叠的整体碰撞检查升级为通过。
        self.assertNotEqual(report["task_checks"]["complete_collision_qualification"], "PASS")
        self.assertEqual(report["status"], "SIMULATION_DEMO_ONLY_HARDWARE_BLOCKED")

    def test_independent_place_targets_are_inside_their_table_half(self):
        scene = self.plan["scene"]
        center = np.asarray(scene["table_center_xy_m"], float)
        half = np.asarray(scene["table_size_m"][:2], float) / 2
        radius = scene["bottle_radius_m"]
        left = np.asarray(scene["place_grip_m"]["left"], float)
        right = np.asarray(scene["place_grip_m"]["right"], float)
        self.assertGreater(left[1] - radius, center[1])
        self.assertLess(right[1] + radius, center[1])
        self.assertGreater(np.linalg.norm(left - right), 2 * radius)
        for arm in demo.ARMS:
            pick = np.asarray(scene["pick_grip_m"][arm], float)
            place = np.asarray(scene["place_grip_m"][arm], float)
            with self.subTest(arm=arm):
                self.assertGreater(np.linalg.norm(place[:2] - pick[:2]), 2 * radius)
                self.assertTrue(np.all(place[:2] - radius > center - half))
                self.assertTrue(np.all(place[:2] + radius < center + half))
                self.assertGreaterEqual(place[2] - scene["bottle_height_m"] / 2,
                                        scene["table_top_m"] - 1e-6)

    def test_rejects_hardware_flags_and_wrong_coordinate_frame(self):
        for field, value in [("real_motion_authorized", True),
                             ("debian_execution_allowed", True),
                             ("coordinate_frame", "SDK_WORLD")]:
            with self.subTest(field=field):
                plan = self.candidate()
                plan[field] = value
                with self.assertRaises(ValueError):
                    demo.validate_contract(plan)

    def test_rejects_wrong_arm_identity_and_mapping(self):
        for arm in demo.ARMS:
            for key in ("arm_id", "gripper_id", "joint_ids", "ee_link_id"):
                with self.subTest(arm=arm, key=key):
                    plan = self.candidate()
                    plan["meta"][arm][key] = plan["meta"]["right" if arm == "left" else "left"][key]
                    with self.assertRaises(ValueError):
                        demo.validate_contract(plan)

    def test_rejects_nonfinite_or_wrong_length_joint_vectors(self):
        for arm in demo.ARMS:
            for value in (float("nan"), float("inf"), -float("inf")):
                with self.subTest(arm=arm, value=value):
                    plan = self.candidate()
                    plan["frames"][0]["q_sdk_deg"][arm][0] = value
                    with self.assertRaises(ValueError):
                        demo.validate_contract(plan)
            plan = self.candidate()
            for frame in plan["frames"]:
                frame["q_sdk_deg"][arm] = frame["q_sdk_deg"][arm][:6]
            with self.assertRaises(ValueError):
                demo.validate_contract(plan)

    def test_rejects_bad_step_period_and_both_arm_discontinuities(self):
        for step in (0, -0.1, float("nan"), float("inf"), demo.SHARED["pulse_step_deg"] + 0.01):
            with self.subTest(step=step):
                plan = self.candidate()
                plan["playback"]["joint_step_deg"] = step
                with self.assertRaises(ValueError):
                    demo.validate_contract(plan)
        plan = self.candidate()
        plan["playback"]["period_s"] *= 2
        with self.assertRaises(ValueError):
            demo.validate_contract(plan)
        for arm in demo.ARMS:
            with self.subTest(arm=arm):
                plan = self.candidate()
                plan["frames"][1]["q_sdk_deg"][arm][0] += 2
                with self.assertRaises(ValueError):
                    demo.validate_contract(plan)

    def test_rejects_missing_reordered_and_one_arm_gripper_events(self):
        indices = [i for i, f in enumerate(self.plan["frames"]) if "event" in f]
        mutations = (lambda fs: fs[indices[1]].pop("event"),
                     lambda fs: fs[indices[1]].update(event="open"),
                     lambda fs: fs[indices[1]].update(event_arms=["left"]),
                     lambda fs: fs[indices[1]].update(stage="OPEN_AT_PLACE"))
        for index, mutation in enumerate(mutations):
            with self.subTest(mutation=index):
                plan = self.candidate()
                mutation(plan["frames"])
                with self.assertRaises(ValueError):
                    demo.validate_contract(plan)

    def test_rejects_carrying_state_changes_without_gripper_event(self):
        for wanted in (False, True):
            plan = self.candidate()
            frame = next(f for f in plan["frames"] if f["carrying"] is wanted and "event" not in f)
            frame["carrying"] = not wanted
            with self.subTest(was_carrying=wanted), self.assertRaises(ValueError):
                demo.validate_contract(plan)

    def test_rejects_motion_at_gripper_event_even_below_dense_step(self):
        plan = self.candidate()
        event = next(f for f in plan["frames"] if f.get("event") == "close")
        event["q_sdk_deg"] = copy.deepcopy(event["q_sdk_deg"])
        event["q_sdk_deg"]["right"][0] += 0.01
        with self.assertRaisesRegex(ValueError, "未停止"):
            demo.validate_contract(plan)

    def test_loading_rejects_changed_source_without_writing_source(self):
        original_sha = demo.sha
        def changed_sha(path):
            return "0" * 64 if Path(path) == demo.SOURCE else original_sha(path)
        with mock.patch.object(demo, "sha", side_effect=changed_sha):
            with self.assertRaisesRegex(ValueError, "依赖已变化"):
                demo.load_validated()
        self.assertEqual(original_sha(demo.SOURCE), A_PARENT_SHA256)


class SynchronizationTests(unittest.TestCase):
    def test_both_arms_share_fraction_and_arrive_at_exact_same_frame(self):
        before = {"left": [0.0] * 7, "right": [0.0] * 7}
        after = {"left": [0.5] * 7, "right": [-1.25] * 7}
        frames = demo.densify_pair(before, after, 0.25)
        self.assertEqual(len(frames), 5)
        self.assertEqual(frames[-1], after)
        previous = before
        for i, frame in enumerate(frames, 1):
            for arm in demo.ARMS:
                np.testing.assert_allclose(frame[arm], np.array(after[arm]) * i / 5)
                self.assertLessEqual(float(np.max(np.abs(np.array(frame[arm]) - previous[arm]))), 0.25 + 1e-12)
            previous = frame
        self.assertEqual(before, {"left": [0.0] * 7, "right": [0.0] * 7})

    def test_stationary_arm_remains_fixed_while_other_moves(self):
        before = {"left": [2.0] * 7, "right": [0.0] * 7}
        after = {"left": [2.0] * 7, "right": [0.7] * 7}
        frames = demo.densify_pair(before, after, 0.25)
        for arm in demo.ARMS:
            np.testing.assert_allclose(frames[-1][arm], after[arm], atol=1e-12, rtol=0)
        self.assertTrue(all(f["left"] == before["left"] for f in frames))


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.plan = json.loads(demo.PLAN.read_text())
        self.robot = demo.load_robot()

    def tearDown(self):
        if p.isConnected():
            p.disconnect()

    def test_right_solution_matches_mirrored_full_tool_pose(self):
        left_q = self.plan["frames"][0]["q_sdk_deg"]["left"]
        left_pos, left_r, _, _ = demo.tool(self.robot, "left", left_q)
        right_q, _ = demo.right_ik(self.robot, left_q)
        right_pos, right_r, _, _ = demo.tool(self.robot, "right", right_q)
        self.assertLess(np.linalg.norm(right_pos - demo.MIRROR_M @ left_pos), 0.0002)
        error = right_r @ (demo.MIRROR_M @ left_r @ demo.SWAP_P).T
        angle = np.degrees(np.arccos(np.clip((np.trace(error) - 1) / 2, -1, 1)))
        self.assertLess(angle, 0.2)
        self.assertGreater(np.max(np.abs(np.array(right_q) + left_q)), 0.01)

    def test_release_detaches_objects_during_retreat_and_reset_restores_pick(self):
        scene = demo.Scene(self.robot, self.plan)
        self.assertEqual(set(scene.obstacles), {"table"})
        release = None
        previous_z = {}
        release_q = None
        for frame in self.plan["frames"]:
            scene.update(frame)
            if frame.get("event") == "close":
                self.assertEqual(set(scene.attachments), set(demo.ARMS))
            if frame["carrying"]:
                for arm in demo.ARMS:
                    _, _, origin, quat = demo.tool(self.robot, arm)
                    expected = p.multiplyTransforms(origin, quat, *scene.attachments[arm])
                    actual = p.getBasePositionAndOrientation(scene.objects[arm])
                    np.testing.assert_allclose(actual[0], expected[0], atol=1e-7, rtol=0)
                    self.assertAlmostEqual(abs(float(np.dot(actual[1], expected[1]))), 1, places=6)
            if frame.get("event") == "open" and frame["stage"] == "OPEN_AT_PLACE":
                self.assertFalse(scene.attachments)
                self.assertEqual(scene.released, set(demo.ARMS))
                release = {arm: p.getBasePositionAndOrientation(scene.objects[arm]) for arm in demo.ARMS}
                release_q = frame["q_sdk_deg"]
                previous_z = {arm: release[arm][0][2] for arm in demo.ARMS}
            if release is not None:
                for arm in demo.ARMS:
                    pos, orn = p.getBasePositionAndOrientation(scene.objects[arm])
                    np.testing.assert_allclose(pos[:2], release[arm][0][:2], atol=1e-9, rtol=0)
                    np.testing.assert_allclose(orn, release[arm][1], atol=1e-9, rtol=0)
                    self.assertLessEqual(pos[2], previous_z[arm] + 1e-9)
                    previous_z[arm] = pos[2]
        self.assertIsNotNone(release)
        for arm in demo.ARMS:
            self.assertGreater(np.max(np.abs(np.array(self.plan["frames"][-1]["q_sdk_deg"][arm]) - release_q[arm])), 1.0)
            self.assertLess(previous_z[arm], release[arm][0][2] - 0.005)
        scene.reset()
        self.assertFalse(scene.attachments)
        self.assertFalse(scene.released)
        for arm in demo.ARMS:
            pos, orn = p.getBasePositionAndOrientation(scene.objects[arm])
            np.testing.assert_allclose(pos, self.plan["scene"]["pick_grip_m"][arm], atol=1e-12, rtol=0)
            np.testing.assert_allclose(orn, [0, 0, 0, 1], atol=1e-12, rtol=0)


if __name__ == "__main__":
    unittest.main()
