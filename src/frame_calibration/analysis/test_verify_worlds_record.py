#!/usr/bin/env python3
import datetime as dt
import unittest

from verify_worlds_record import evaluate_gate


class GateTests(unittest.TestCase):
    def test_pass_inside_all_strict_thresholds(self):
        now = dt.datetime(2026, 8, 21, 12)
        got = evaluate_gate([1, 2, 3], [4, 0, 7], now - dt.timedelta(days=7), now)
        self.assertEqual(got["verdict"], "PASS")

    def test_axis_threshold_is_strict(self):
        now = dt.datetime(2026, 8, 21)
        got = evaluate_gate([0, 0, 0], [5, 0, 0], now, now)
        self.assertEqual(got["verdict"], "BLOCKED")
        self.assertFalse(got["checks"]["axis_drift"])

    def test_stale_blocks_even_when_drift_is_zero(self):
        now = dt.datetime(2026, 8, 21)
        got = evaluate_gate([0, 0, 0], [0, 0, 0], now - dt.timedelta(days=8), now)
        self.assertEqual(got["verdict"], "BLOCKED")
        self.assertFalse(got["checks"]["freshness"])

    def test_timezone_aware_now_accepts_legacy_naive_capture_time(self):
        captured = dt.datetime(2026, 7, 13, 18, 0, 23)
        now = dt.datetime(2026, 8, 27, 12, 0, 0,
                          tzinfo=dt.timezone(dt.timedelta(hours=8)))
        got = evaluate_gate([0, 0, 0], [0, 0, 0], captured, now)
        self.assertAlmostEqual(got["age_days"], 44.7497337962963)
        self.assertEqual(got["verdict"], "BLOCKED")


if __name__ == "__main__":
    unittest.main(verbosity=2)
