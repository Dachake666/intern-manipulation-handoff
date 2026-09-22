#!/usr/bin/env python3
import math
import unittest

import numpy as np

from fit_eye_to_hand import build_result, quat_matrix


class CalibrationTests(unittest.TestCase):
    def test_recovers_known_transform_with_holdout(self):
        yaw = math.radians(20)
        R = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                      [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
        t = np.array([.2, -.1, .5])
        q_world_camera = [0, 0, math.sin(yaw/2), math.cos(yaw/2)]
        samples = []
        for i in range(25):
            pc = np.array([.1 + i*.003, -.05 + (i % 4)*.01, .7 + (i % 3)*.02])
            pw = R @ pc + t
            samples.append({"sample_id": str(i), "sync_delta_ms": 10,
                "camera_target": {"position_m": pc.tolist(), "quaternion_xyzw": [0,0,0,1]},
                "sdk_world_target": {"position_m": pw.tolist(),
                                     "quaternion_xyzw": q_world_camera}})
        data = {"samples": samples, "robot_pose_direction": "base_to_hand",
                "ros_position_unit": "meter", "tcp_id": "left_tcp_v1"}
        got = build_result(data, "cal-1", "left", "cam", "robot")
        T = np.asarray(got["T_sdk_world_camera"])
        self.assertLess(np.max(np.abs(T[:3, :3] - R)), 1e-9)
        self.assertLess(np.max(np.abs(T[:3, 3] - t)), 1e-9)
        self.assertEqual(got["fit_samples"], 20)
        self.assertEqual(got["validation_samples"], 5)

    def test_rejects_too_few_samples(self):
        with self.assertRaisesRegex(ValueError, "25"):
            build_result({"samples": []}, "c", "left", "cam", "robot")


if __name__ == "__main__":
    unittest.main(verbosity=2)
