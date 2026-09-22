#!/usr/bin/env python3
from __future__ import annotations

import copy
import math
import unittest
from pathlib import Path

import numpy as np

from robot_mission.contracts import load_and_validate, validate_document
from robot_mission.tabletop_scene_snapshot import (
    analyze_scene_snapshot,
    merge_ab_pose,
    quaternion_shortest_arc_midpoint,
    validate_snapshot_semantics,
)


ROOT = Path(__file__).resolve().parent.parent
SNAPSHOT_PATH = (ROOT / "pick_place_coord" / "tasks" /
                 "task_tabletop_vision_ab_20260902.json")


def snapshot():
    return load_and_validate(SNAPSHOT_PATH, "tabletop_scene_snapshot.v1")


class SceneSnapshotTests(unittest.TestCase):
    def test_saved_snapshot_preserves_raw_ab_and_recomputes_processed_pose(self):
        value = snapshot()
        raw = value["bottle"]["raw_observations"]
        self.assertEqual([item["observation_id"] for item in raw], ["A1", "B1"])
        merged = merge_ab_pose(raw[0]["pose"], raw[1]["pose"])
        self.assertTrue(np.allclose(
            merged["position_mm"],
            [952.7394104003906, 262.6982421875, 377.65773010253906],
            atol=1e-12, rtol=0))
        self.assertTrue(np.allclose(
            merged["quaternion_xyzw"],
            value["bottle"]["processed_pose"]["quaternion_xyzw"],
            atol=1e-12, rtol=0))

    def test_quaternion_midpoint_uses_shortest_arc_and_is_sign_invariant(self):
        q_a = [0, 0, math.sin(math.pi / 8), math.cos(math.pi / 8)]
        q_b = [0, 0, math.sin(3 * math.pi / 8), math.cos(3 * math.pi / 8)]
        midpoint = quaternion_shortest_arc_midpoint(q_a, q_b)
        same_with_negative_input = quaternion_shortest_arc_midpoint(
            q_a, [-v for v in q_b])
        expected = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        self.assertTrue(np.allclose(midpoint, expected, atol=1e-12))
        self.assertTrue(np.allclose(midpoint, same_with_negative_input,
                                    atol=1e-12))

    def test_feature_pose_cannot_be_promoted_to_grasp_pose(self):
        value = snapshot()
        value["bottle"]["processed_pose_role"] = "GRASP_POSE"
        value["bottle"]["may_be_used_as_grasp_pose"] = True
        with self.assertRaisesRegex(ValueError, "feature|抓取点"):
            validate_snapshot_semantics(value)
        with self.assertRaisesRegex(ValueError, "VISUAL_FEATURE_POSE|expected"):
            validate_document(value, "tabletop_scene_snapshot.v1")

    def test_snapshot_can_never_authorize_real_motion(self):
        value = snapshot()
        self.assertFalse(value["real_motion_authorized"])
        value["real_motion_authorized"] = True
        with self.assertRaisesRegex(ValueError, "false|False"):
            validate_document(value, "tabletop_scene_snapshot.v1")

    def test_safe_uvw_quaternion_and_gripper_empty_interface_are_locked(self):
        value = snapshot()
        self.assertFalse(value["safe_candidate"]["may_be_used_as_transfer_corridor"])
        self.assertIsNone(value["gripper_policy"]["open"]["position"])
        bad = copy.deepcopy(value)
        bad["safe_candidate"]["pose"]["quaternion_xyzw"] = [0, 0, 0, 1]
        with self.assertRaisesRegex(ValueError, "UVW"):
            validate_snapshot_semantics(bad)
        bad = copy.deepcopy(value)
        bad["gripper_policy"]["close"]["force"] = 1000
        with self.assertRaisesRegex(ValueError, "空接口"):
            validate_snapshot_semantics(bad)

    def test_analysis_is_explicitly_blocked_and_contains_geometry_deltas(self):
        analysis = analyze_scene_snapshot(snapshot())
        self.assertEqual(analysis["status"], "BLOCKED_FOR_TRAJECTORY")
        self.assertFalse(analysis["real_motion_authorized"])
        self.assertAlmostEqual(
            analysis["safe_to_bottle"]["distance_3d_mm"],
            287.77695794945845, places=6)
        self.assertIn("bottle_feature_to_grasp_transform",
                      analysis["blockers"])
        self.assertIn("frame_calibration_gate_pass", analysis["blockers"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
