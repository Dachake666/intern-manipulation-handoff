"""离线假SDK回归：不连接机器人、不导入厂商Linux wheel。"""
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parent
WORK = HERE.parent.parent
sys.path.insert(0, str(HERE))
import execute_tabletop_hybrid_trial as e
from test_precheck_tabletop_hybrid import fixture


def plan_fixture(v2=False):
    plan = fixture()
    plan["gripper_policy"] = {"gripper_id": 2, "open_command": 0x11, "close_command": 0x10,
                              "open": {"position": 500, "settle_s": 0},
                              "close": {"speed": 50, "force": 50, "settle_s": 0}}
    if v2:
        plan["schema_version"] = e.hp.SCHEMA_V2
        plan["home_joints_sdk_deg"] = [0.] * 7
        plan["reviewed_start"] = {"joints": [0.] * 7, "worlds": [800., -100., 650., 0., 0., 0.]}
        plan["motion_qualification"] = {"status": "BLOCKED"}
        plan["stages"] = [s for s in plan["stages"] if s["name"] != "TRANSFER_MID"]
        plan["offline_replay"]["frames"] = [f for f in plan["offline_replay"]["frames"] if f["stage"] != "TRANSFER_MID"]
        stage = next(s for s in plan["stages"] if s["name"] == "RETURN_SAFE")
        stage.update(kind="MOVE_JOINTS", q_sdk_deg=[0.] * 7, expected_endpoint=stage.pop("pose"))
    return plan


class FakeSDK:
    def __init__(self):
        self.world = [800., -100., 650., 0., 0., 0.]  # 明显远离nominal Safe
        self.q = [0.] * 7
        self.speed = 10.
        self.protected = True
        self.calls = []
        self.stop_result = None
        self.fail_restore = False
        self.drift_on_enable = False

    def armGetWorlds(self, *args): return 0, list(self.world)
    def armGetJoints(self, *args): return 0, list(self.q)
    def armGetRobotMoveState(self, *args): return 0, False
    def getSoftStopSwitch(self): return 0, "CLOSE"
    def armGetRobotProtectStatus(self, *args): return 0, self.protected
    def armGetGlobalSpeed(self): return 0, self.speed
    def armClearAlarm(self): self.calls.append("clear"); return 0
    def armRobotEnableOrNot(self, flag):
        self.calls.append("enable")
        if self.drift_on_enable: self.q[0] += 0.5
        return 0
    def armGetRobotEnableStatus(self): return 0, True
    def armServoIsOpOrNot(self): return 0, True
    def armGetSingleRobotEnableStatus(self, *args): return 0, True
    def armSetGlobalSpeed(self, speed):
        self.calls.append(("speed", speed))
        if self.fail_restore and speed == 10: return 7
        self.speed = speed
        return 0
    def armOpenCom(self, *args): self.calls.append("com"); return 0
    def armWriteCom(self, *args): self.calls.append("gripper"); return 0
    def armSetRobotProtectStatus(self, arm, value): self.protected = value; return 0
    def stop(self): self.calls.append("stop"); return self.stop_result


class ExecutorTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=WORK, prefix=".hybrid-executor-test-")
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.sdk = FakeSDK()

    def qualified(self, plan):
        plan["motion_qualification"] = {"status": "STAGED_TEST_READY"}
        plan["meta"]["t_session_used"] = "FAKE_SDK_ONLY"
        path = self.folder / "plan.json"
        path.write_text(json.dumps(plan))
        digest = e.sha256(path)
        evidence = {"schema_version": "tabletop_hybrid_qualification.v1", "result": "STAGED_TEST_READY",
                    "trajectory_sha256": digest, "checks": {}}
        for key in ("geometry", "calibration", "dense_collision", "controller_interpolation"):
            proof = self.folder / f"{key}_proof.json"
            content = {"schema_version": f"tabletop_hybrid_{key}_evidence.v1", "result": "PASS",
                       "trajectory_sha256": digest, "arm_id": 1, "blockers": [],
                       "verified_components": {k: True for k in ("table", "bin", "object", "gripper", "wrist", "full_robot")},
                       "t_session_used": "FAKE_SDK_ONLY", "sample_captured_at": e.utc_now(),
                       "sample_after_last_chassis_or_waist_motion": True, "max_axis_delta_mm": 0,
                       "euclidean_delta_mm": 0, "dense_step_deg": .5, "world_step_mm": 2, "world_step_deg": 1,
                       "reviewed_motion_stages": [s["name"] for s in e.effective_stages(plan) if s["kind"].startswith("MOVE_")],
                       "coverage": {k: "PASS" for k in ("self_collision", "environment", "held_object", "open_gripper", "wrist")},
                       "matches_sdk_moveworlds_and_movejoints": True}
            proof.write_text(json.dumps(content))
            evidence["checks"][key] = {"status": "PASS", "evidence_files": [{"path": proof.name, "sha256": e.sha256(proof)}]}
        ep = self.folder / "qualification.json"
        ep.write_text(json.dumps(evidence))
        gui = {"schema_version": "pybullet_gui_review.v1", "result": "PASS", "reviewer": "FAKE SDK TEST ONLY",
               "trajectory_sha256": digest, "arm_id": 1, "dense_step_deg": .5,
               "full_replay_completed": True,
               "reviewed_motion_stages": [s["name"] for s in e.effective_stages(plan) if s["kind"].startswith("MOVE_")]}
        gp = self.folder / "gui.json"
        gp.write_text(json.dumps(gui))
        return ["--qualification-report", str(ep), "--gui-review", str(gp)]

    def run_main(self, *, plan=None, extra=(), answer=None, ik=None, dry=False):
        plan = plan or plan_fixture()
        p = self.folder / "plan.json"
        p.write_text(json.dumps(plan))
        logpath = self.folder / "log.json"
        def move_worlds(sdk, arm, target, **kw):
            sdk.calls.append("worlds")
            sdk.world = list(target)
        def move_joints(sdk, arm, target):
            sdk.calls.append("joints")
            sdk.q = list(target)
            sdk.world = [600., 100., 550., 0., 0., 0.]
        args = [str(p), "--dry-run" if dry else "--run", "--accept-blockers", "--auto-continue",
                "--robot-ip", "192.0.2.1", "--local-ip", "192.0.2.2", "--log", str(logpath), *extra]
        with contextlib.ExitStack() as stack:
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(patch.dict(e.os.environ, {"XIFENG_ALLOW_REAL_MOTION": "1"}))
            stack.enter_context(patch.object(e, "LEGACY_PLAN_SHA256", e.sha256(p)))
            opened = stack.enter_context(patch.object(e.ss, "open_read_only_session", return_value=self.sdk))
            stack.enter_context(patch.object(e.robot_lock, "acquire", return_value=contextlib.nullcontext()))
            stack.enter_context(patch.object(e.time, "sleep", return_value=None))
            stack.enter_context(patch.object(e.ss, "read_axis_limits", return_value=e.ss.LIMITS_DEG[1]))
            stack.enter_context(patch.object(e.ss, "try_worlds", side_effect=ik or (lambda sdk, arm, pose: list(sdk.q))))
            stack.enter_context(patch.object(e.ss, "joint_out_limit", return_value=(False, False)))
            stack.enter_context(patch.object(e.ss, "move_worlds", side_effect=move_worlds))
            stack.enter_context(patch.object(e.ss, "move_joints_abs", side_effect=move_joints))
            stack.enter_context(patch.object(e.ss, "wait_until_worlds", side_effect=lambda sdk, *a, **kw: sdk.world))
            stack.enter_context(patch.object(e.ss, "wait_until_joints", side_effect=lambda sdk, *a, **kw: sdk.q))
            stack.enter_context(patch.object(e.ss, "clear_robot_route", side_effect=lambda *a, **kw: self.sdk.calls.append("emergency")))
            code = e.main(args, input_fn=answer or (lambda prompt: ""))
        return code, json.loads(logpath.read_text()), opened.call_count

    def test_v1_far_start_full_run_with_fixed_joint_home(self):
        code, log, _ = self.run_main()
        self.assertEqual(code, 0)
        self.assertEqual(log["status"], "PASS_RETURNED_SAFE")
        self.assertEqual(self.sdk.q, e.SAFE_HOME_JOINTS_DEG)
        self.assertEqual(self.sdk.speed, 10)
        self.assertEqual(log["cleanup"]["status"], "PASS")
        self.assertEqual(len(log["events"]), 13)
        self.assertEqual(log["config"]["limit_margin_deg"], 5)
        self.assertTrue(log["operator_confirmation"]["empty_gripper_confirmed"])
        self.assertEqual(len(log["source_hashes"]), 5)
        for event in log["events"]:
            if event["kind"].startswith("MOVE_"):
                self.assertLessEqual(event["command_issued_at"], event["arrival_tolerance_at"])
                self.assertLessEqual(event["arrival_tolerance_at"], event["controller_stopped_at"])
        self.assertEqual(len(log["readonly_precheck"]["segments"]), 8)

    def test_nominal_safe_far_rejected_without_enable(self):
        code, log, _ = self.run_main(extra=("--start-policy", "nominal-safe"))
        self.assertEqual(code, 2)
        self.assertNotIn("enable", self.sdk.calls)

    def test_pre_enable_operator_drift_rejected(self):
        def answer(prompt): self.sdk.q[0] += .4; return ""
        code, log, _ = self.run_main(answer=answer)
        self.assertEqual(code, 2)
        self.assertIn("drifted", log["error"])
        self.assertNotIn("enable", self.sdk.calls)

    def test_after_enable_drift_never_opens_com_or_moves(self):
        self.sdk.drift_on_enable = True
        code, log, _ = self.run_main()
        self.assertEqual(code, 2)
        self.assertIn("after enable", log["error"])
        for command in ("com", "worlds", "joints"): self.assertNotIn(command, self.sdk.calls)
        self.assertIn("emergency", self.sdk.calls)

    def test_disabled_protection_blocks_before_enable(self):
        self.sdk.protected = False
        code, log, _ = self.run_main()
        self.assertEqual(code, 2)
        self.assertNotIn("enable", self.sdk.calls)
        self.assertIn("protection", log["error"])

    def test_first_ik_jump_compares_actual_live_joint(self):
        code, log, _ = self.run_main(ik=lambda *a: [7.] + [0.] * 6)
        self.assertEqual(code, 2)
        self.assertIn("branch jump", log["error"])
        self.assertNotIn("enable", self.sdk.calls)

    def test_no_limit_margin_bypass(self):
        code, log, opened = self.run_main(extra=("--limit-margin-deg", "0"))
        self.assertEqual(code, 2)
        self.assertEqual(opened, 0)

    def test_unknown_v1_hash_has_no_field_trial_exception(self):
        with self.assertRaisesRegex(RuntimeError, "byte-identical"):
            e.qualification_gate(plan_fixture(), "0" * 64, type("Args", (), {"accept_blockers": True})())

    def test_v2_changed_reviewed_start_rejected(self):
        current = {"joints": [.3] + [0.] * 6, "worlds": [0.] * 6}
        with self.assertRaisesRegex(RuntimeError, "requalify"):
            e.require_qualified_start(plan_fixture(True), current)

    def test_v2_same_q_different_worlds_does_not_reuse_gui(self):
        current = {"joints": [0.] * 7, "worlds": [810., -100., 650., 0., 0., 0.]}
        with self.assertRaisesRegex(RuntimeError, "requalify"):
            e.require_qualified_start(plan_fixture(True), current)

    def test_v2_blockers_cannot_be_accepted(self):
        code, log, opened = self.run_main(plan=plan_fixture(True))
        self.assertEqual(code, 2)
        self.assertEqual(opened, 0)
        self.assertIn("cannot override", log["error"])

    def test_v2_missing_gui_rejected_before_sdk(self):
        plan = plan_fixture(True)
        refs = self.qualified(plan)
        code, log, opened = self.run_main(plan=plan, extra=refs[:2])
        self.assertEqual(code, 2)
        self.assertEqual(opened, 0)
        self.assertIn("--gui-review", log["error"])

    def test_v2_one_movej_transfer_and_explicit_home(self):
        plan = plan_fixture(True)
        refs = self.qualified(plan)
        code, log, _ = self.run_main(plan=plan, extra=refs)
        self.assertEqual(code, 0, log.get("error"))
        self.assertEqual(self.sdk.calls.count("joints"), 2)
        self.assertEqual(self.sdk.q, [0.] * 7)
        self.assertEqual(log["config"]["handoff_pos_tol_mm"], 5)
        self.assertEqual(len(log["events"]), 12)

    def test_v2_gui_sha_mismatch_rejected(self):
        plan = plan_fixture(True)
        refs = self.qualified(plan)
        gp = Path(refs[-1]); gui = json.loads(gp.read_text()); gui["trajectory_sha256"] = "0" * 64
        gp.write_text(json.dumps(gui))
        code, log, opened = self.run_main(plan=plan, extra=refs)
        self.assertEqual(code, 2)
        self.assertEqual(opened, 0)

    def test_blocked_evidence_cannot_be_wrapped_in_pass_report(self):
        plan = plan_fixture(True)
        refs = self.qualified(plan)
        proof = self.folder / "dense_collision_proof.json"
        content = json.loads(proof.read_text()); content["result"] = "BLOCKED"
        proof.write_text(json.dumps(content))
        ep = Path(refs[1]); report = json.loads(ep.read_text())
        report["checks"]["dense_collision"]["evidence_files"][0]["sha256"] = e.sha256(proof)
        ep.write_text(json.dumps(report))
        code, log, opened = self.run_main(plan=plan, extra=refs)
        self.assertEqual(code, 2)
        self.assertEqual(opened, 0)

    def test_invalid_protection_readback_blocks_before_enable(self):
        self.sdk.protected = "False"
        code, log, _ = self.run_main()
        self.assertEqual(code, 2)
        self.assertNotIn("enable", self.sdk.calls)

    def test_v2_mismatched_home_rejected(self):
        plan = plan_fixture(True); plan["home_joints_sdk_deg"][0] = 1
        with self.assertRaises(ValueError): e.hp.validate(plan)

    def test_v2_recorded_final_home_frame_is_not_called_live(self):
        plan = plan_fixture(True)
        plan["offline_replay"]["frames"][0]["stage"] = "RECORDED_FINAL_HOME_NOT_CURRENT_LIVE_START"
        self.assertEqual(len(e.hp.validate(plan)), 7)

    def test_v2_blocked_dryrun_never_connects(self):
        code, log, opened = self.run_main(plan=plan_fixture(True), dry=True)
        self.assertEqual(code, 0)
        self.assertEqual(opened, 0)
        self.assertEqual(log["qualification"]["status"], "BLOCKED")

    def test_restore_failure_never_writes_pass(self):
        self.sdk.fail_restore = True
        code, log, _ = self.run_main()
        self.assertEqual(code, 2)
        self.assertEqual(log["status"], "CLEANUP_FAILED")
        self.assertTrue(log["motion_completed"])
        self.assertEqual(log["cleanup"]["speed_restore_code"], 7)
        self.assertIn("stop", self.sdk.calls)

    def test_sdk_stop_failure_never_writes_pass(self):
        self.sdk.stop_result = False
        code, log, _ = self.run_main()
        self.assertEqual(code, 2)
        self.assertEqual(log["status"], "CLEANUP_FAILED")
        self.assertFalse(log["cleanup"]["sdk_stopped"])

    def test_emergency_route_failure_never_increases_speed(self):
        self.sdk.speed = 3
        log = {"enable": {"speed_before": 50}}
        with patch.object(e.ss, "clear_robot_route", side_effect=RuntimeError("stop failed")):
            self.assertFalse(e.checked_cleanup(self.sdk, log, enabled=True, completed=False))
        self.assertEqual(self.sdk.speed, 3)
        self.assertEqual(log["cleanup"]["speed_restore_status"], "DEFERRED_STOP_UNCONFIRMED")
        self.assertNotIn(("speed", 50), self.sdk.calls)

    def test_still_moving_never_increases_speed(self):
        self.sdk.speed = 3
        self.sdk.armGetRobotMoveState = lambda *args: (0, True)
        log = {"enable": {"speed_before": 50}}
        with patch.object(e.ss, "clear_robot_route", return_value=None):
            self.assertFalse(e.checked_cleanup(self.sdk, log, enabled=True, completed=False))
        self.assertEqual(self.sdk.speed, 3)

    def test_boolean_numeric_rejected(self):
        with self.assertRaises(ValueError): e.finite_vec([True] * 7, 7, "joints")


if __name__ == "__main__":
    unittest.main()
