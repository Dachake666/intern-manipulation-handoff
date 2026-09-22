#!/usr/bin/env python3
from __future__ import annotations

import unittest
from pathlib import Path
import sys

import numpy as np
import pybullet as p

from robot_mission.contracts import load_and_validate

# 同目录还有pick_place_coord.py；pytest下不能把它误当成同名包。
sys.path.insert(0, str(Path(__file__).resolve().parent))
from demo_tabletop_scene_snapshot import (
    _single_safe_pose_robot_anchor,
    analyze_preview,
    xf,
)


WORK = Path(__file__).resolve().parent.parent
SNAPSHOT = (WORK / "pick_place_coord" / "tasks" /
            "task_tabletop_vision_ab_20260902.json")


class ScenePreviewTests(unittest.TestCase):
    def test_preview_reports_geometry_risk_without_authorizing_motion(self):
        snapshot = load_and_validate(SNAPSHOT, "tabletop_scene_snapshot.v1")
        report = analyze_preview(snapshot)
        self.assertFalse(report["real_motion_authorized"])
        self.assertEqual(report["status"],
                         "SCENE_PREVIEW_ONLY_NOT_A_TRAJECTORY")
        self.assertAlmostEqual(
            report["metrics"]["safe_proxy_clearance_over_box_top_mm"],
            -21.4591628459, places=8)
        self.assertAlmostEqual(
            report["metrics"]["minimum_endpoint_z_with_profile_clearance_mm"],
            485.2341628459, places=8)
        self.assertEqual(
            report["gates"]["safe_candidate_as_transfer_corridor"],
            "BLOCKED_PROXY_BELOW_ASSUMED_RIM_CLEARANCE")
        self.assertEqual(report["gates"]["pb_sdk_frame_gate"],
                         "NOT_USED_FOR_SCENE_ONLY_PREVIEW")

    def test_raw_pair_and_fixed_orientation_deltas_are_visible(self):
        snapshot = load_and_validate(SNAPSHOT, "tabletop_scene_snapshot.v1")
        metrics = analyze_preview(snapshot)["metrics"]
        self.assertAlmostEqual(metrics["bottle_raw_ab_distance_mm"],
                               30.8640786803, places=8)
        self.assertAlmostEqual(metrics["bottle_raw_ab_orientation_delta_deg"],
                               11.7569375867, places=8)
        self.assertAlmostEqual(
            metrics["bottle_to_safe_endpoint_orientation_delta_deg"],
            118.0754998848, places=8)
        self.assertAlmostEqual(
            metrics["safe_joint_min_margin_after_configured_buffer_deg"],
            19.665, places=9)

    def test_robot_overlay_is_single_pose_visual_anchor_only(self):
        snapshot = load_and_validate(SNAPSHOT, "tabletop_scene_snapshot.v1")
        robot = xf.load_xifeng(gui=False)
        try:
            shift, evidence = _single_safe_pose_robot_anchor(robot, snapshot)
        finally:
            p.disconnect()
        self.assertTrue(np.allclose(
            shift, [-0.2249415286, 0.0061832266, 0.7348735904], atol=1e-8))
        self.assertEqual(evidence["mode"],
                         "VISUAL_ONLY_SINGLE_SAFE_POSE_ANCHOR")
        self.assertIn("NOT_A_FRAME_GATE", evidence["qualification_status"])
        self.assertAlmostEqual(evidence["safe_orientation_residual_deg"],
                               0.5481369553, places=8)
        self.assertGreater(
            evidence["difference_from_in_use_model_a_t_norm_mm"], 50.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
