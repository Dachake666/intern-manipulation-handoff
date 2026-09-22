#!/usr/bin/env python3
"""execute_worlds_10x 的纯离线测试；不导入 pypilot、不连接机器人。"""
from __future__ import annotations

import tempfile
from pathlib import Path
import sys
import unittest
from unittest import mock

import execute_worlds_10x as mod

SAFE_Q = [-60.0, 30.0, 20.0, -60.0, 0.0, 0.0, 20.0]


class FakeSS:
    def __init__(self):
        self.__file__ = mod.__file__
        self.moves = []
        self.waits = 0
        self.world_reads = 0
        self.soft_stop_checks = 0
        self.try_calls = 0
        self.limits = [(-100.0, 100.0)] * 7
        self.unreachable_at = None
        self.fail_wait_at = None
        self.open_calls = []
        self.closed = False

    def open_session(self, robot_ip, local_ip, arm_ip, arm_port, speed,
                     arm_ids=(1,), enable=True):
        self.open_calls.append({
            "robot_ip": robot_ip,
            "local_ip": local_ip,
            "arm_ip": arm_ip,
            "arm_port": arm_port,
            "speed": speed,
            "arm_ids": tuple(arm_ids),
            "enable": enable,
        })
        return object()

    def close_session(self, sdk):
        self.closed = True

    def read_axis_limits(self, sdk, arm_id):
        return self.limits

    @staticmethod
    def axis_limits_look_valid(limits):
        bad = [i for i, (lo, hi) in enumerate(limits) if hi - lo < 10.0]
        return not bad, bad

    @staticmethod
    def check_limits(arm_id, q, margin=5.0):
        return []

    def try_worlds(self, sdk, arm_id, pose):
        self.try_calls += 1
        if self.unreachable_at == self.try_calls:
            return None
        return [
            SAFE_Q[0] + pose[0] * 0.1,
            SAFE_Q[1] + pose[1] * 0.1,
            SAFE_Q[2] + pose[2] * 0.1,
            SAFE_Q[3], SAFE_Q[4], SAFE_Q[5], SAFE_Q[6],
        ]

    def check_soft_stop(self, sdk, label):
        self.soft_stop_checks += 1

    def move_worlds(self, sdk, arm_id, target, interpolation_en):
        self.moves.append((list(target), interpolation_en))

    def wait_until_worlds(self, sdk, arm_id, target, **kwargs):
        self.waits += 1
        if self.fail_wait_at == self.waits:
            raise RuntimeError("simulated stop")
        return list(target)

    def read_worlds(self, sdk, arm_id):
        self.world_reads += 1
        return list(self.moves[-1][0]) if self.moves else [0.0] * 6

    @staticmethod
    def read_joints(sdk, arm_id):
        return list(SAFE_Q)


class PatternTests(unittest.TestCase):
    def test_operator_defaults_match_simple_wrist_style(self):
        self.assertEqual(mod.ARM_ID, 1)
        self.assertEqual(mod.ARM_IP, mod.ROBOT_IP)
        self.assertEqual(mod.STEP_MM, 20.0)
        self.assertFalse(mod.ENABLE_REAL_MOTION)
        self.assertTrue(mod.CONFIRM_EACH_STEP)

    def test_pattern_has_exactly_ten_moves_and_returns(self):
        offsets = mod.build_offsets(mod.STEP_MM)
        self.assertEqual(len(offsets), 10)
        self.assertEqual(offsets[-1], [0, 0, 0, 0, 0, 0])
        self.assertTrue(all(p[2] >= 0 for p in offsets))
        self.assertEqual({p[3:] for p in map(tuple, offsets)}, {(0, 0, 0)})

    def test_targets_keep_orientation_fixed(self):
        anchor = [300, 200, 900, -20, 90, 0]
        targets = mod.build_targets(anchor, 10)
        self.assertEqual(len(targets), 10)
        self.assertTrue(all(p[3:] == anchor[3:] for p in targets))
        self.assertEqual(targets[-1], anchor)

    def test_step_is_bounded(self):
        for bad in (0, 0.9, mod.MAX_STEP_MM + 0.1, float("inf")):
            with self.assertRaises(ValueError):
                mod.build_offsets(bad)

    def test_densify_includes_end_not_start(self):
        pts = mod.densify_segment([0] * 6, [10, 0, 0, 0, 0, 0], sample_mm=2)
        self.assertEqual(len(pts), 5)
        self.assertEqual(pts[-1], [10, 0, 0, 0, 0, 0])
        self.assertNotEqual(pts[0], [0] * 6)


class PrecheckTests(unittest.TestCase):
    def test_valid_plan_checks_every_dense_point(self):
        ss = FakeSS()
        anchor = [0.0] * 6
        targets = mod.build_targets(anchor, 10)
        report = mod.precheck_plan(ss, object(), 1, anchor, targets, SAFE_Q)
        self.assertEqual(len(report["segments"]), 10)
        self.assertEqual(report["dense_points"], ss.try_calls)
        self.assertGreater(report["dense_points"], 10)

    def test_invalid_controller_limits_block_motion(self):
        ss = FakeSS()
        ss.limits = [(0.0, 0.0)] * 7
        with self.assertRaisesRegex(mod.PrecheckError, "限位读数无效"):
            mod.precheck_plan(
                ss, object(), 1, [0.0] * 6,
                mod.build_targets([0.0] * 6, 10), SAFE_Q)

    def test_limit_margin_blocks_near_limit(self):
        ss = FakeSS()
        ss.limits = [(-10.0, 10.0)] * 7
        with self.assertRaisesRegex(mod.PrecheckError, "当前姿态已经过于接近限位"):
            mod.precheck_plan(
                ss, object(), 1, [0.0] * 6,
                mod.build_targets([0.0] * 6, 10), [3.0] * 7,
                limit_margin_deg=8.0)

    def test_unreachable_dense_point_blocks_motion(self):
        ss = FakeSS()
        ss.unreachable_at = 3
        with self.assertRaisesRegex(mod.PrecheckError, "不可达或奇异"):
            mod.precheck_plan(
                ss, object(), 1, [0.0] * 6,
                mod.build_targets([0.0] * 6, 10), SAFE_Q)

    def test_right_arm_is_explicitly_blocked(self):
        ss = FakeSS()
        with self.assertRaisesRegex(mod.PrecheckError, "只允许 arm-id=1"):
            mod.precheck_plan(
                ss, object(), 2, [0.0] * 6,
                mod.build_targets([0.0] * 6, 10), SAFE_Q)


class ExecutionTests(unittest.TestCase):
    def test_default_run_is_readonly_and_sends_no_motion(self):
        ss = FakeSS()
        with mock.patch.dict(sys.modules, {"sdk_session": ss}), \
                mock.patch.multiple(mod, ROBOT_IP="192.0.2.10",
                                    LOCAL_IP="192.0.2.11", ARM_IP="192.0.2.10"):
            result = mod.run_robot()
        self.assertEqual(result, 0)
        self.assertEqual(len(ss.open_calls), 1)
        self.assertFalse(ss.open_calls[0]["enable"])
        self.assertEqual(ss.moves, [])
        self.assertTrue(ss.closed)

    def test_exactly_ten_moves_and_ten_feedback_records(self):
        ss = FakeSS()
        targets = mod.build_targets([0.0] * 6, 10)
        records = mod.execute_plan(ss, object(), 1, targets)
        self.assertEqual(len(ss.moves), 10)
        self.assertEqual(ss.waits, 10)
        self.assertEqual(ss.world_reads, 10)
        self.assertEqual(ss.soft_stop_checks, 10)
        self.assertEqual(len(records), 10)
        self.assertTrue(all(blend is False for _, blend in ss.moves))
        self.assertEqual([r["move"] for r in records], list(range(1, 11)))

    def test_wrong_move_count_is_refused(self):
        with self.assertRaisesRegex(ValueError, "运动数量"):
            mod.execute_plan(FakeSS(), object(), 1, [[0.0] * 6])

    def test_completed_feedback_survives_later_failure(self):
        ss = FakeSS()
        ss.fail_wait_at = 4
        records = []
        with self.assertRaisesRegex(RuntimeError, "simulated stop"):
            mod.execute_plan(
                ss, object(), 1, mod.build_targets([0.0] * 6, 10),
                records=records)
        self.assertEqual(len(records), 3)

    def test_q_stops_before_next_move_and_keeps_completed_record(self):
        ss = FakeSS()
        answers = iter(["", "q"])
        records = mod.execute_plan(
            ss, object(), 1, mod.build_targets([0.0] * 6, 10),
            confirm_each_step=True,
            input_fn=lambda _prompt: next(answers))
        self.assertEqual(len(records), 1)
        self.assertEqual(len(ss.moves), 1)
        self.assertEqual(ss.world_reads, 1)

    def test_record_writer_never_overwrites(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = mod.save_record(Path(tmp), {"moves": []})
            self.assertTrue(path.exists())
            self.assertIn("worlds_10x_record_", path.name)

    def test_svg_preview_has_no_external_dependency(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = mod.render_preview(Path(tmp) / "preview.svg", 10)
            text = path.read_text(encoding="utf-8")
            self.assertIn("<svg", text)
            self.assertIn("S / 10", text)
            self.assertIn("1,3,5,7,9", text)


if __name__ == "__main__":
    unittest.main(verbosity=2)
