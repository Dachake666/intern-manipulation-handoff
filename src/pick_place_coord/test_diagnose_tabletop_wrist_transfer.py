"""腕部诊断回归：非运动文件、固定上游关节、共享限位及参考点语义。"""
import json
from pathlib import Path
import sys
import unittest

import numpy as np
import pybullet as p

sys.path.insert(0, str(Path(__file__).resolve().parent))
import diagnose_tabletop_wrist_transfer as d


class WristDiagnosticTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = d.xf.load_xifeng(gui=False)
        cls.snapshot = json.loads((d.HERE / "tasks/task_tabletop_vision_ab_20260902.json").read_text())
        cls.q = np.asarray(cls.snapshot["safe_candidate"]["joints_sdk_deg"], float)

    @classmethod
    def tearDownClass(cls):
        p.disconnect()

    def test_input_validation(self):
        with self.assertRaises(ValueError):
            d.finite([0] * 6, 7)
        with self.assertRaises(ValueError):
            d.finite([0] * 6 + [float("nan")], 7)
        self.assertEqual(len(d.grid(0, 0, .5)), 1)

    def test_j7_origin_is_not_tcp(self):
        before = d.probe(self.robot, self.q)
        after_q = self.q.copy()
        after_q[6] += 5
        after = d.probe(self.robot, after_q)
        np.testing.assert_allclose(before["link_origin"], after["link_origin"], atol=0.001)
        self.assertGreater(np.linalg.norm(after["physical_grip"] - before["physical_grip"]), 10)

    def test_fixed_upstream_and_margin(self):
        report, best = d.sweep(self.robot, self.q, d.ss.LIMITS_DEG[1], 5,
                               np.array([20, -150, 0]), step=2)
        np.testing.assert_array_equal(best[:5], self.q[:5])
        self.assertLessEqual(abs(best[6] - self.q[6]), 5)
        self.assertEqual(d.ss.check_limits(1, best, margin=d.ss.LIMIT_MARGIN_DEG), [])
        self.assertTrue(report["not_a_collision_check"])
        self.assertLessEqual(report["selected_segment_diagnostic"]["max_adjacent_joint_step_deg"], .25)

    def test_relative_v2_correction_cancels_with_fixed_j1_j5(self):
        before = d.probe(self.robot, self.q)
        after_q = self.q.copy()
        after_q[5:] += [5, -5]
        after = d.probe(self.robot, after_q)
        np.testing.assert_allclose(
            after["sdk_v2_without_session_t"] - before["sdk_v2_without_session_t"],
            after["physical_tcp"] - before["physical_tcp"], atol=0.001)


if __name__ == "__main__":
    unittest.main()
