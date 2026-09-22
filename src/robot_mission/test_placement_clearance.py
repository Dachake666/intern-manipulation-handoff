#!/usr/bin/env python3
"""合成几何测试；不代表现场腕部包络、物料框或轨迹已验证。"""
from __future__ import annotations

import copy
import importlib.util
import itertools
import unittest

import numpy as np

from robot_mission.grasp_pose import (
    matrix_to_quaternion_xyzw, pose_matrix, sdk_uvw_deg_to_matrix)
from robot_mission.placement_clearance import compute_placement_clearance


def _arguments():
    return {
        "T_world_endpoint": pose_matrix([720, 100, 495], [0, 0, 0, 1]),
        "envelope_points_endpoint_mm": [[-30, -30, -50], [30, 30, 60]],
        "T_endpoint_object": pose_matrix([0, 0, -45], [0, 0, 0, 1]),
        "object_dimensions_mm": [70, 70, 200],
        "bin_rim_z_mm": 430,
        "table_surface_z_mm": 280,
        "rim_clearance_mm": 10,
        "held_clearance_mm": 10,
        "hover_z_mm": 550,
        "minimum_hover_gap_mm": 30,
        "support_z_mm": 350,
        "release_mode": "SUPPORTED_ONLY",
        "max_drop_mm": None,
        "support_tolerance_mm": 1,
    }


class PlacementClearanceTests(unittest.TestCase):
    def test_hover_stays_550_when_carried_bottom_is_clear(self):
        args = _arguments()
        args["bin_rim_z_mm"] = 390
        result = compute_placement_clearance(**args)
        self.assertEqual(result["release_z_mm"], 495)
        self.assertEqual(result["hover_z_mm"], 550)
        self.assertEqual(result["release_to_hover_mm"], 55)
        self.assertEqual(result["release_gap_mm"], 0)
        self.assertEqual(result["status"], "CANDIDATE")
        self.assertFalse(result["real_motion_authorized"])

    def test_collision_raises_release_but_never_lowers_target(self):
        args = _arguments()
        args["T_world_endpoint"][2, 3] = 370
        args["support_z_mm"] = 345
        result = compute_placement_clearance(**args)
        self.assertEqual(result["minimum_release_z_mm"], 490)
        self.assertEqual(result["release_z_mm"], 490)
        self.assertEqual(result["release_raise_mm"], 120)
        self.assertEqual(result["envelope_min_z_mm"], 440)
        self.assertEqual(args["T_world_endpoint"][2, 3], 370)
        self.assertEqual(result["release_gap_mm"], 0)

    def test_hover_uses_held_object_bottom_not_endpoint_height(self):
        result = compute_placement_clearance(**_arguments())
        self.assertEqual(result["release_z_mm"], 495)
        self.assertEqual(result["minimum_hover_z_mm"], 585)
        self.assertEqual(result["hover_z_mm"], 585)
        self.assertEqual(result["hover_held_object_bottom_z_mm"], 440)

    def test_release_above_requested_hover_raises_hover(self):
        args = _arguments()
        args["T_world_endpoint"][2, 3] = 600
        args["release_mode"] = "ALLOW_BOUNDED_DROP"
        args["max_drop_mm"] = 120
        result = compute_placement_clearance(**args)
        self.assertEqual(result["release_z_mm"], 600)
        self.assertEqual(result["hover_z_mm"], 630)
        self.assertEqual(result["release_to_hover_mm"], 30)

    def test_rotation_changes_lowest_envelope_and_object_offsets(self):
        args = _arguments()
        args["T_world_endpoint"][:3, :3] = sdk_uvw_deg_to_matrix([90, 0, 0])
        args["envelope_points_endpoint_mm"] = [[0, -100, 0], [0, 10, -50]]
        args["support_z_mm"] = 350
        args["release_mode"] = "ALLOW_BOUNDED_DROP"
        args["max_drop_mm"] = 155
        result = compute_placement_clearance(**args)
        self.assertAlmostEqual(result["envelope_lowest_offset_world_z_mm"], -100)
        self.assertAlmostEqual(result["minimum_release_z_mm"], 540)
        self.assertAlmostEqual(result["held_object_lowest_offset_world_z_mm"], -35)
        self.assertAlmostEqual(result["held_object_bottom_z_mm"], 505)
        self.assertAlmostEqual(result["release_gap_mm"], 155)
        np.testing.assert_array_equal(result["T_world_release_endpoint"][:3, :3],
                                      args["T_world_endpoint"][:3, :3])

    def test_object_local_rotation_and_center_offset_are_applied(self):
        args = _arguments()
        args["T_endpoint_object"][:3, :3] = sdk_uvw_deg_to_matrix([90, 0, 0])
        args["T_endpoint_object"][2, 3] = -110
        result = compute_placement_clearance(**args)
        self.assertAlmostEqual(result["held_object_lowest_offset_world_z_mm"], -145)
        self.assertAlmostEqual(result["release_gap_mm"], 0)

    def test_supported_mode_blocks_raised_release(self):
        args = _arguments()
        args["bin_rim_z_mm"] = 450
        result = compute_placement_clearance(**args)
        self.assertEqual(result["release_z_mm"], 510)
        self.assertEqual(result["release_gap_mm"], 15)
        self.assertEqual(result["status"], "BLOCKED")
        self.assertTrue(result["blockers"][0].startswith("release_not_supported:"))

    def test_support_penetration_is_blocked(self):
        args = _arguments()
        args["support_z_mm"] = 355
        result = compute_placement_clearance(**args)
        self.assertEqual(result["release_gap_mm"], -5)
        self.assertEqual(result["status"], "BLOCKED")
        self.assertTrue(result["blockers"][0].startswith("release_object_below_support:"))

    def test_explicit_bounded_drop_and_excess(self):
        args = _arguments()
        args["support_z_mm"] = 335
        args["release_mode"] = "ALLOW_BOUNDED_DROP"
        args["max_drop_mm"] = 15
        result = compute_placement_clearance(**args)
        self.assertEqual(result["release_gap_mm"], 15)
        self.assertEqual(result["status"], "CANDIDATE")
        args["max_drop_mm"] = 14.9
        result = compute_placement_clearance(**args)
        self.assertEqual(result["status"], "BLOCKED")
        self.assertTrue(result["blockers"][0].startswith("release_drop_exceeds_max:"))

    def test_drop_permission_must_be_explicit(self):
        args = _arguments()
        args["release_mode"] = "ALLOW_BOUNDED_DROP"
        with self.assertRaisesRegex(ValueError, "显式"):
            compute_placement_clearance(**args)
        args = _arguments()
        args["max_drop_mm"] = 20
        with self.assertRaisesRegex(ValueError, "SUPPORTED_ONLY"):
            compute_placement_clearance(**args)

    def test_invalid_scalars_and_dimensions_rejected(self):
        for name in ("bin_rim_z_mm", "table_surface_z_mm", "rim_clearance_mm",
                     "held_clearance_mm", "hover_z_mm", "minimum_hover_gap_mm",
                     "support_z_mm", "support_tolerance_mm"):
            for value in (float("nan"), float("inf"), True):
                with self.subTest(name=name, value=value):
                    args = _arguments()
                    args[name] = value
                    with self.assertRaises(ValueError):
                        compute_placement_clearance(**args)
        for value in ([], [70, 70, 0], [70, float("nan"), 200], [70, 70]):
            with self.subTest(dimensions=value):
                args = _arguments()
                args["object_dimensions_mm"] = value
                with self.assertRaises(ValueError):
                    compute_placement_clearance(**args)

    def test_empty_bad_or_nonfinite_envelope_rejected(self):
        for value in ([], [[], []], [1, 2, 3], [[0, 0]], [[0, 0, float("nan")]]):
            with self.subTest(value=value):
                args = _arguments()
                args["envelope_points_endpoint_mm"] = value
                with self.assertRaisesRegex(ValueError, "点云"):
                    compute_placement_clearance(**args)

    def test_bad_transforms_and_nonpositive_hover_gap_rejected(self):
        args = _arguments()
        args["T_world_endpoint"][0, 0] = 2
        with self.assertRaises(ValueError):
            compute_placement_clearance(**args)
        args = _arguments()
        args["T_endpoint_object"][1, 3] = float("nan")
        with self.assertRaises(ValueError):
            compute_placement_clearance(**args)
        args = _arguments()
        args["minimum_hover_gap_mm"] = 0
        with self.assertRaises(ValueError):
            compute_placement_clearance(**args)

    def test_inputs_not_mutated(self):
        args = _arguments()
        before = copy.deepcopy(args)
        compute_placement_clearance(**args)
        for name in ("T_world_endpoint", "T_endpoint_object"):
            np.testing.assert_array_equal(args[name], before[name])


@unittest.skipUnless(importlib.util.find_spec("pybullet"), "需 Mac 离线 PyBullet")
class ToolProxyBulletTests(unittest.TestCase):
    def test_rotated_proxy_corners_match_bullet_aabb(self):
        """只验证局部工具几何；不使用失效的全局标定、不代表手腕/整场景 PASS。"""
        import pybullet as pb
        import arm_profiles
        from frame_calibration.analysis import calib_common as cc
        profile = arm_profiles.arm_profile("left")
        half = np.asarray(profile["gripper_collision_box_m"]) * 500.0
        center = np.asarray(profile["gripper_collision_center_link_mm"], float)
        tcp = np.asarray(cc.CONFIRMED_R_TCP_LINK11_MM, float)
        R_link = (sdk_uvw_deg_to_matrix([-72.779, 13.241, -6.634]) @
                  sdk_uvw_deg_to_matrix([0, 0, -90]))
        corners = np.asarray(list(itertools.product((-1, 1), repeat=3))) * half
        relative = (corners + center - tcp) @ R_link.T
        client = pb.connect(pb.DIRECT)
        try:
            shape = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=(half / 1000).tolist(),
                                             physicsClientId=client)
            body = pb.createMultiBody(0, shape, physicsClientId=client)
            for z in (370, 495, 520, 550):
                endpoint = np.asarray([720., 100., float(z)])
                proxy_center = endpoint + R_link @ (center - tcp)
                pb.resetBasePositionAndOrientation(
                    body, (proxy_center / 1000).tolist(),
                    matrix_to_quaternion_xyzw(R_link).tolist(), physicsClientId=client)
                lo, hi = np.asarray(pb.getAABB(body, physicsClientId=client)) * 1000
                np.testing.assert_allclose(lo, endpoint + relative.min(axis=0), atol=1e-8)
                np.testing.assert_allclose(hi, endpoint + relative.max(axis=0), atol=1e-8)
        finally:
            pb.disconnect(client)


if __name__ == "__main__":
    unittest.main()
