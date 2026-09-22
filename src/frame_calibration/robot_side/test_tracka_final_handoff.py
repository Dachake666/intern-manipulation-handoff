"""[20260909] 收到的现场最终版原样归档；只做文件/模拟测试，禁止真实SDK连接。"""
from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

WORK = Path(__file__).resolve().parents[2]
SIDE = WORK / "frame_calibration/robot_side"
RECORD = WORK / "frame_calibration/records/20260908_trackA_hybrid_final"
sys.path.insert(0, str(SIDE))
spec = importlib.util.spec_from_file_location("tracka_field_received", SIDE / "execute_tabletop_hybrid_trial_reviewfix_field.py")
field = importlib.util.module_from_spec(spec)
spec.loader.exec_module(field)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class FinalHandoffTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.manifest = json.loads((RECORD / "as_run.json").read_text())
        cls.plan_path = WORK / cls.manifest["runtime_artifacts"]["tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json"]["path"]
        cls.plan = json.loads(cls.plan_path.read_text())

    def test_every_received_runtime_hash_is_preserved(self):
        for item in self.manifest["runtime_artifacts"].values():
            self.assertEqual(digest(WORK / item["path"]), item["sha256"])

    def test_both_success_logs_have_matching_identity_and_full_cleanup(self):
        self.assertEqual(len(self.manifest["runs"]), 2)
        for record in self.manifest["runs"]:
            path = RECORD / record["file"]
            self.assertEqual(digest(path), record["scrubbed_log_sha256"])
            log = json.loads(path.read_text())
            self.assertEqual(log["status"], "PASS_RETURNED_SAFE")
            self.assertEqual(log["plan_sha256"], digest(self.plan_path))
            self.assertEqual(log["executor_sha256"], digest(Path(field.__file__)))
            self.assertEqual([e["stage"] for e in log["events"]], [s["name"] for s in self.plan["stages"]])
            self.assertEqual(log["cleanup"]["status"], "PASS")
            self.assertTrue(log["cleanup"]["sdk_stopped"])
            self.assertTrue(log["cleanup"]["protection_enabled"])
            self.assertEqual(log["cleanup"]["speed_restored_readback"], log["cleanup"]["speed_expected"])
            self.assertFalse(log["cleanup"]["errors"])
            self.assertLessEqual(log["events"][-1]["assert_error"]["position_mm"], field.WORLD_TOL_MM)

    def test_sdk_alignment_preserves_cartesian_targets_and_one_transfer(self):
        parent = json.loads((WORK / "pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED.json").read_text())
        old = {s["name"]: s for s in parent["stages"]}
        for stage in self.plan["stages"]:
            for key in ("pose", "expected_endpoint"):
                if key in stage:
                    self.assertEqual(stage[key], old[stage["name"]][key])
        movej = [s for s in self.plan["stages"] if s["kind"] == "MOVE_JOINTS"]
        self.assertEqual([s["name"] for s in movej], ["PLACE_HOVER", "RETURN_SAFE"])
        self.assertEqual(movej[0]["q_sdk_deg"], [-33.131, 2.908, 19.904, -90.809, -23.508, -7.728, 40.384])
        self.assertEqual(self.plan["gripper_policy"]["open"]["settle_s"], 1.0)
        self.assertEqual(self.plan["gripper_policy"]["close"]["settle_s"], 2.0)

    def test_received_executor_dry_run_never_connects_or_moves(self):
        with tempfile.TemporaryDirectory(prefix=".field-handoff-test-", dir=WORK) as folder, \
                patch.object(field.ss, "open_read_only_session", side_effect=AssertionError("no SDK")), \
                patch.object(field.ss, "open_session", side_effect=AssertionError("no SDK")), \
                patch.object(field.ss, "move_worlds", side_effect=AssertionError("no motion")), \
                patch.object(field.ss, "move_joints_abs", side_effect=AssertionError("no motion")):
            output = Path(folder) / "dry_run.json"
            self.assertEqual(field.main([str(self.plan_path), "--dry-run", "--log", str(output)]), 0)
            report = json.loads(output.read_text())
            self.assertFalse(report["sdk_connected"])
            self.assertEqual(report["status"], "FILE_CHECKS_PASS_NOT_MOTION_AUTHORIZED")


if __name__ == "__main__":
    unittest.main()
