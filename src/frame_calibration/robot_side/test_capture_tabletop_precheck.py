import contextlib
import io
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import capture_tabletop_precheck as c


class SDK:
    def __init__(self):
        self.stopped = False

    def stop(self):
        self.stopped = True


class SS:
    def __init__(self, q=None):
        self.sdk = SDK()
        self.q = [0.0] * 7 if q is None else q
        self.opened = 0

    def open_read_only_session(self, *args):
        self.opened += 1
        return self.sdk

    def read_worlds(self, *args):
        return [0.0] * 6

    def read_joints(self, *args):
        return [0.0] * 7

    def try_worlds(self, *args):
        return self.q


def finite(values, size, label):
    if len(values) != size or not all(math.isfinite(v) for v in values):
        raise ValueError(label)
    return list(values)


def executor(precheck):
    return SimpleNamespace(ARM_ID=1, _finite_vector=finite, precheck_full_path=precheck,
                           movement_targets=lambda plan: [("PICK_ASCEND", [0.0] * 6)])


CONFIG = dict(robot_ip="test", local_ip="test", arm_ip="test", arm_port=8080, limit_margin_deg=5)


class CaptureTests(unittest.TestCase):
    def test_success_exports_and_stops(self):
        def check(ss, sdk, plan, margin):
            q = ss.try_worlds(sdk, 1, [0.0] * 6)
            return {"segments": [{"stage": "PICK_ASCEND", "target_joints_deg": q}]}
        ss, report = SS(), {}
        self.assertEqual(c.capture(executor(check), ss, CONFIG, {}, report), 0)
        self.assertTrue(ss.sdk.stopped)
        self.assertFalse(report["real_motion_authorized"])
        self.assertEqual(len(report["dense_ik_samples"]), 1)
        self.assertEqual(report["endpoint_ik"]["PICK_ASCEND"], [0.0] * 7)

    def test_failure_keeps_partial_and_endpoint_diagnostic(self):
        def check(ss, sdk, plan, margin):
            ss.try_worlds(sdk, 1, [0.0] * 6)
            raise ValueError("limit")
        ss, report = SS(), {}
        self.assertEqual(c.capture(executor(check), ss, CONFIG, {}, report), 2)
        self.assertEqual(report["status"], "PRECHECK_FAILED")
        self.assertEqual(len(report["dense_ik_samples"]), 1)
        self.assertIn("PICK_ASCEND", report["standalone_endpoint_queries"])
        self.assertTrue(ss.sdk.stopped)

    def test_motion_methods_denied(self):
        for name in ("armMoveWorlds", "armMoveJoints", "armWriteCom", "armSetGlobalSpeed", "armEnable"):
            with self.assertRaises(RuntimeError):
                getattr(c.ReadOnlySDK(SDK()), name)
        with self.assertRaises(RuntimeError):
            c.Recorder(SS(), []).move_worlds

    def test_accidental_motion_in_executor_is_blocked(self):
        def check(ss, sdk, plan, margin):
            sdk.armMoveJoints(1, [])
        ss, report = SS(), {}
        self.assertEqual(c.capture(executor(check), ss, CONFIG, {}, report), 2)
        self.assertTrue(ss.sdk.stopped)
        self.assertIn("只读预检拒绝", report["error"])

    def test_nonfinite_ik_does_not_poison_json(self):
        rows = []
        with self.assertRaises(ValueError):
            c.Recorder(SS([float("nan")] * 7), rows).try_worlds(None, 1, [0.0] * 6)
        self.assertIsNone(rows[0]["q_sdk_deg"])
        self.assertTrue(rows[0]["invalid_joint_response"])

    def test_interrupt_stops_without_more_ik(self):
        def check(*args, **kwargs):
            raise KeyboardInterrupt()
        ss, report = SS(), {}
        self.assertEqual(c.capture(executor(check), ss, CONFIG, {}, report), 130)
        self.assertTrue(ss.sdk.stopped)
        self.assertNotIn("standalone_endpoint_queries", report)

    def test_stop_error_is_not_pass(self):
        def check(*args, **kwargs):
            return {"segments": []}
        ss, report = SS(), {}
        def bad_stop():
            raise RuntimeError("stop failed")
        ss.sdk.stop = bad_stop
        c.capture(executor(check), ss, CONFIG, {}, report)
        self.assertEqual(report["status"], "SESSION_STOP_FAILED")

    def test_no_run_cli(self):
        with contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaises(SystemExit) as caught:
                c.main(["ignored.json", "--run"])
        self.assertNotEqual(caught.exception.code, 0)


if __name__ == "__main__":
    unittest.main()
