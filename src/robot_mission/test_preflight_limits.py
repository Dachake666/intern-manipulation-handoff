from __future__ import annotations

import math
import unittest

import arm_profiles
from robot_mission.preflight import pybullet_planning_custom_limits


class PybulletPlanningLimitTests(unittest.TestCase):
    def test_left_controller_j2_and_j7_override_urdf_defaults(self):
        profile = arm_profiles.arm_profile("left")
        mapped = pybullet_planning_custom_limits(profile)
        j2 = profile["joint_ids"][1]
        j7 = profile["joint_ids"][6]
        self.assertAlmostEqual(math.degrees(mapped[j2][0]), -10.0)
        self.assertAlmostEqual(math.degrees(mapped[j7][1]), 76.0)

    def test_sdk_to_urdf_sign_flip_preserves_order(self):
        profile = arm_profiles.arm_profile("left")
        profile["controller_limits_deg"][5] = [-30.0, 10.0]
        mapped = pybullet_planning_custom_limits(profile)
        j6 = profile["joint_ids"][5]
        self.assertAlmostEqual(math.degrees(mapped[j6][0]), -10.0)
        self.assertAlmostEqual(math.degrees(mapped[j6][1]), 30.0)


if __name__ == "__main__":
    unittest.main()
