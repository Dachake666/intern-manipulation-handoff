#!/usr/bin/env python3
"""move_worlds_once.py 离线安全测试；不导入 pypilot，不连接机器人。"""
import importlib.util
from pathlib import Path
import unittest


PATH = Path(__file__).with_name("move_worlds_once.py")
SPEC = importlib.util.spec_from_file_location("move_worlds_once", PATH)
mod = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mod)


SAFE_Q = [-30.0, 30.0, 0.0, -60.0, 0.0, 0.0, 0.0]
SAFE_WORLD = [600.0, 100.0, 600.0, 0.0, 0.0, 0.0]


class FakeSS:
    def __init__(self):
        self.moves = []
        self.enable = None
        self.closed = False

    def open_session(self, *args, **kwargs):
        self.enable = kwargs["enable"]
        return object()

    def close_session(self, sdk):
        self.closed = True

    def read_worlds(self, sdk, arm_id):
        return SAFE_WORLD[:]

    def read_joints(self, sdk, arm_id):
        return SAFE_Q[:]

    def read_axis_limits(self, sdk, arm_id):
        return [(-180.0, 180.0)] * 7

    def axis_limits_look_valid(self, limits):
        return True, []

    def try_worlds(self, sdk, arm_id, pose):
        return SAFE_Q[:]

    def check_limits(self, arm_id, joints, margin):
        return []

    def check_soft_stop(self, sdk, label):
        pass

    def move_worlds(self, sdk, arm_id, target, interpolation_en):
        self.moves.append((list(target), interpolation_en))

    def wait_until_worlds(self, sdk, arm_id, target, **kwargs):
        return list(target)

    def clear_robot_route(self, sdk, arm_id, emergency_stop):
        pass


class MoveWorldsOnceTests(unittest.TestCase):
    def setUp(self):
        self.original_enable = mod.ENABLE_REAL_MOTION

    def tearDown(self):
        mod.ENABLE_REAL_MOTION = self.original_enable

    def test_blank_target_is_refused(self):
        with self.assertRaisesRegex(SystemExit, "填写完整"):
            mod._pose6([None] * 6)

    def test_default_is_readonly_and_sends_no_motion(self):
        ss = FakeSS()
        mod.ENABLE_REAL_MOTION = False
        self.assertEqual(mod.run(ss, SAFE_WORLD[:]), 0)
        self.assertFalse(ss.enable)
        self.assertEqual(ss.moves, [])
        self.assertTrue(ss.closed)

    def test_real_mode_sends_exactly_one_unblended_move(self):
        ss = FakeSS()
        mod.ENABLE_REAL_MOTION = True
        target = [602.0, 100.0, 600.0, 0.0, 0.0, 0.0]
        self.assertEqual(mod.run(ss, target, input_fn=lambda _: ""), 0)
        self.assertEqual(ss.moves, [(target, False)])
        self.assertTrue(ss.closed)


if __name__ == "__main__":
    unittest.main(verbosity=2)
