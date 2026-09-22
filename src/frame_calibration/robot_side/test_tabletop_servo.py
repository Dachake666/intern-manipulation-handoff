"""A 转 Servo 的离线回归；假 SDK，不连接真实机器人。"""
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

SIDE = Path(__file__).resolve().parent
WORK = SIDE.parents[1]
sys.path.insert(0, str(SIDE))
import execute_tabletop_servo as e
import tabletop_servo_contract as c
from test_execute_tabletop_hybrid_trial import FakeSDK

PARENT = WORK / "pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json"


class ContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = c.build(PARENT)
        cls.parent = json.loads(PARENT.read_text())

    def test_every_source_knot_retained_in_order_without_sign_or_tcp_change(self):
        stream = iter(w["q_sdk_deg"] for w in self.plan["waypoints"] if "q_sdk_deg" in w)
        for frame in self.parent["offline_replay"]["frames"]:
            if "q_sdk_deg" in frame:
                self.assertTrue(any(q == frame["q_sdk_deg"] for q in stream))
        self.assertEqual(self.plan["home_joints_sdk_deg"], self.parent["home_joints_sdk_deg"])
        self.assertEqual(self.plan["source_stages"], self.parent["stages"])
        self.assertEqual(c.digest(PARENT), c.SOURCE_SHA)

    def test_all_events_and_native_gripper_parameters_preserved(self):
        self.assertEqual(self.plan["gripper_policy"], self.parent["gripper_policy"])
        events = [w for w in self.plan["waypoints"] if "gripper" in w]
        self.assertEqual([w["gripper"] for w in events], ["open", "close", "open"])
        self.assertEqual([w["settle_s"] for w in events], [1, 2, 1])

    def test_exported_metrics_and_shared_limits(self):
        metrics = c.validate(self.plan)
        self.assertEqual(metrics["motion_frame_count"], 2484)
        self.assertLessEqual(metrics["max_step_deg"], .1)
        self.assertLessEqual(metrics["max_velocity_deg_s"], 5)
        self.assertGreaterEqual(min(metrics["minimum_margin_per_joint_deg"]), 5)
        self.assertEqual(self.plan["qualification"]["verdict"], "BLOCKED")

    def test_new_parent_is_not_implicitly_accepted(self):
        with patch.object(c, "digest", return_value="0"*64):
            with self.assertRaisesRegex(ValueError, "父轨迹"):
                c.build(PARENT)

    def test_reject_corrupt_contract_or_arrays(self):
        changes = [
            lambda p: p["meta"].update(arm_id=True),
            lambda p: p["meta"].update(j6_flipped=False),
            lambda p: p["meta"].update(arm_profiles_sha256="0"*64),
            lambda p: p["stream_policy"].update(period_s=.001),
            lambda p: p["waypoints"][2]["q_sdk_deg"].__setitem__(0, float("nan")),
            lambda p: p["waypoints"][2]["q_sdk_deg"].__setitem__(6, 80),
            lambda p: p["waypoints"].pop(1),
            lambda p: p["waypoints"][1].update(gripper="close"),
            lambda p: p["waypoints"][2].update(carrying=True),
            lambda p: p["gripper_policy"]["close"].update(force=-1),
            lambda p: p["gripper_policy"].update(open_command=99),
            lambda p: p["kinematics"].update(motion_frame_count=1),
        ]
        for index, mutate in enumerate(changes):
            with self.subTest(index=index):
                plan = copy.deepcopy(self.plan)
                mutate(plan)
                with self.assertRaises((ValueError, KeyError)):
                    c.validate(plan)

    def test_live_limits_must_be_intersected(self):
        bounds = c.limits()
        bounds[1] = (0, 208)
        with self.assertRaisesRegex(ValueError, "有效关节限位"):
            c.validate(self.plan, bounds)


class Clock:
    def __init__(self): self.now = 0.
    def read(self): return self.now
    def sleep(self, duration): self.now += duration


class TimingTests(unittest.TestCase):
    def test_no_burst_after_small_delay(self):
        clock = Clock()
        pacer = e.sc.StrictPacer(.02, clock.read, clock.sleep)
        pacer.before_send()
        clock.now += .025
        pacer.before_send()
        pacer.before_send()
        self.assertAlmostEqual(clock.now, .045)
        self.assertEqual(pacer.report()["send_slots"], 3)

    def test_large_delay_rejects_next_command(self):
        clock = Clock()
        pacer = e.sc.StrictPacer(.02, clock.read, clock.sleep)
        pacer.before_send()
        clock.now += .2
        with self.assertRaisesRegex(RuntimeError, "不补发"):
            pacer.before_send()
        self.assertEqual(pacer.frames, 1)

    def test_gripper_pause_resets_schedule(self):
        clock = Clock()
        pacer = e.sc.StrictPacer(.02, clock.read, clock.sleep)
        pacer.before_send()
        clock.now += 2
        pacer.reset()
        pacer.before_send()
        self.assertEqual(pacer.report()["max_late_ms"], 0)

    def test_invalid_period_rejected(self):
        for p in [0, -1, float("nan"), float("inf")]:
            with self.assertRaises(ValueError): e.sc.StrictPacer(p)


class ExecutorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls): cls.plan = c.build(PARENT)

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(dir=WORK, prefix=".tabletop-servo-test-")
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.path = self.folder / "candidate.json"
        self.path.write_text(json.dumps(self.plan))
        self.report = self.folder / "report.json"
        self.sdk = FakeSDK()
        self.sdk.q = self.plan["first_point_q_sdk_deg"][:]
        self.sdk.world = json.loads(PARENT.read_text())["reviewed_start"]["worlds"]
        self.sdk.armPluseToServo = self.pulse
        self.pulses = 0
        self.fail_frame = None
        self.endpoints = {}
        for s in self.plan["source_stages"]:
            if not s["kind"].startswith("MOVE_"): continue
            q = next(w["q_sdk_deg"] for w in reversed(self.plan["waypoints"]) if w["stage"] == s["name"])
            pose = s.get("pose", s.get("expected_endpoint"))
            self.endpoints[tuple(q)] = pose["position_mm"]+pose["sdk_world_uvw_deg"]

    def pulse(self, arm, q):
        self.pulses += 1
        if self.pulses == self.fail_frame: return 7
        self.sdk.calls.append("pulse")
        self.sdk.q = list(q)
        self.sdk.world = self.endpoints.get(tuple(q), self.sdk.world)
        return 0

    def invoke(self, *extra, approved=False, answer=None):
        clock = Clock()
        real_pacer = e.sc.StrictPacer
        with contextlib.ExitStack() as stack:
            stack.enter_context(contextlib.redirect_stdout(io.StringIO()))
            stack.enter_context(patch.dict(e.os.environ, {"XIFENG_ALLOW_REAL_MOTION": "1"}))
            stack.enter_context(patch.object(e.ss, "open_read_only_session", return_value=self.sdk))
            stack.enter_context(patch.object(e.ss, "read_axis_limits", return_value=c.limits()))
            stack.enter_context(patch.object(e.ss, "make_float_vector", side_effect=list))
            stack.enter_context(patch.object(e.ss, "clear_robot_route", side_effect=lambda *a, **kw: self.sdk.calls.append("emergency")))
            stack.enter_context(patch.object(e.robot_lock, "acquire_robot", return_value=contextlib.nullcontext()))
            sleeps = stack.enter_context(patch.object(e.time, "sleep", side_effect=clock.sleep))
            stack.enter_context(patch.object(e.sc, "StrictPacer", side_effect=lambda p: real_pacer(p, clock.read, clock.sleep)))
            if approved: stack.enter_context(patch.object(e, "require_reviews", return_value={"FAKE_TEST_ONLY": True}))
            code = e.main([str(self.path), "--report", str(self.report), "--robot-ip", "192.0.2.1",
                           "--local-ip", "192.0.2.2", *extra], input_fn=answer or (lambda prompt: ""))
        return code, json.loads(self.report.read_text()), sleeps

    def test_dry_run_never_connects(self):
        with patch.object(e.ss, "open_read_only_session", side_effect=AssertionError("SDK")):
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(e.main([str(self.path), "--dry-run"]), 0)

    def test_readonly_has_no_write_and_closes_session(self):
        code, log, _ = self.invoke("--precheck-only")
        self.assertEqual(code, 0, log.get("error"))
        self.assertEqual(self.sdk.calls, ["stop"])
        self.assertEqual(log["motion_commands_sent"], 0)

    def test_wrong_start_does_not_auto_movej(self):
        self.sdk.q = self.plan["home_joints_sdk_deg"][:]
        code, log, _ = self.invoke("--precheck-only")
        self.assertEqual(code, 2)
        self.assertIn("首点不匹配", log["error"])
        self.assertEqual(self.sdk.calls, ["stop"])

    def test_missing_new_qualification_blocks_before_sdk(self):
        code, log, _ = self.invoke("--run")
        self.assertEqual(code, 2)
        self.assertIn("--gui-review", log["error"])
        self.assertEqual(self.sdk.calls, [])

    def reviews_fixture(self):
        sha = c.digest(self.path)
        gui = {"schema_version": "pybullet_gui_review.v1", "result": "PASS", "arm_id": 1,
               "trajectory_sha256": sha, "full_replay_completed": True, "frame_count": 2484,
               "dense_step_deg": .1}
        qp, gp = self.folder / "qualification.json", self.folder / "gui.json"
        qualification = {"schema_version": "tabletop_servo_qualification.v1", "verdict": "PASS",
                         "trajectory_sha256": sha, "executor_sha256": c.digest(e.__file__),
                         "servo_common_sha256": c.digest(e.sc.__file__), "contract_sha256": c.digest(c.__file__),
                         "runtime_hashes": e.runtime_hashes(), "scene_id": "FAKE UNIT TEST ONLY",
                         "frame_count": 2484, "stream_policy": self.plan["stream_policy"], "checks": {}}
        for key in ("fk_path", "self_collision", "robot_environment_collision",
                    "held_and_released_object_collision", "entry_region", "servo_dynamics"):
            evidence = {"schema_version": f"tabletop_servo_{key}_evidence.v1", "status": "PASS",
                        "blockers": [], "arm_id": 1, "trajectory_sha256": sha,
                        "frame_count": 2484, "stream_policy": self.plan["stream_policy"],
                        "scene_id": qualification["scene_id"]}
            ep = self.folder / (key + ".json")
            ep.write_text(json.dumps(evidence))
            qualification["checks"][key] = {"status": "PASS", "evidence_files": [
                {"path": ep.name, "sha256": c.digest(ep)}]}
        gp.write_text(json.dumps(gui))
        qp.write_text(json.dumps(qualification))
        return types.SimpleNamespace(trajectory=self.path, gui_review=gp, qualification_report=qp)

    def test_sha_bound_review_and_evidence_contract(self):
        args = self.reviews_fixture()
        self.assertIn("qualification_sha256", e.require_reviews(args, self.plan))

    def test_gui_from_old_mode_is_rejected(self):
        args = self.reviews_fixture()
        gui = json.loads(args.gui_review.read_text())
        gui["trajectory_sha256"] = c.SOURCE_SHA
        args.gui_review.write_text(json.dumps(gui))
        with self.assertRaisesRegex(ValueError, "GUI"):
            e.require_reviews(args, self.plan)

    def test_changed_dependency_is_rejected(self):
        args = self.reviews_fixture()
        with patch.object(e, "runtime_hashes", return_value={}):
            with self.assertRaisesRegex(ValueError, "资格报告"):
                e.require_reviews(args, self.plan)

    def test_missing_or_modified_evidence_is_rejected(self):
        args = self.reviews_fixture()
        path = self.folder / "held_and_released_object_collision.json"
        path.write_text("{}")
        with self.assertRaisesRegex(ValueError, "证据文件 SHA"):
            e.require_reviews(args, self.plan)

    def test_hash_bound_but_blocked_evidence_cannot_be_promoted(self):
        args = self.reviews_fixture()
        ep = self.folder / "self_collision.json"
        body = json.loads(ep.read_text()); body["blockers"] = ["UNRESOLVED MESH"]
        ep.write_text(json.dumps(body))
        qp = args.qualification_report
        report = json.loads(qp.read_text())
        report["checks"]["self_collision"]["evidence_files"][0]["sha256"] = c.digest(ep)
        qp.write_text(json.dumps(report))
        with self.assertRaisesRegex(ValueError, "证据未完整通过"):
            e.require_reviews(args, self.plan)

    def test_complete_fake_servo_only_with_gripper_waits_and_cleanup(self):
        code, log, sleeps = self.invoke("--run", approved=True)
        self.assertEqual(code, 0, log.get("error"))
        self.assertEqual(log["motion_commands_sent"], 2484)
        self.assertEqual(self.sdk.calls.count("pulse"), 2484)
        self.assertEqual(self.sdk.calls.count("gripper"), 3)
        self.assertEqual(self.sdk.q, self.plan["home_joints_sdk_deg"])
        self.assertTrue(self.sdk.protected)
        self.assertEqual(self.sdk.speed, 10)
        self.assertEqual(log["cleanup"]["status"], "PASS")
        self.assertGreaterEqual(sleeps.call_args_list.count(unittest.mock.call(1.0)), 3)
        self.assertIn(unittest.mock.call(2.0), sleeps.call_args_list)
        self.assertNotIn("emergency", self.sdk.calls)

    def test_sdk_failure_counts_successful_sends_and_restores(self):
        self.fail_frame = 4
        code, log, _ = self.invoke("--run", approved=True)
        self.assertEqual(code, 2)
        self.assertEqual(log["motion_commands_sent"], 3)
        self.assertIn("emergency", self.sdk.calls)
        self.assertEqual(self.sdk.calls.count("gripper"), 1)  # 失败不自动开爪丢物
        self.assertTrue(self.sdk.protected)
        self.assertEqual(self.sdk.speed, 10)

    def test_operator_cancel_never_enables(self):
        code, log, _ = self.invoke("--run", approved=True, answer=lambda _: "q")
        self.assertEqual(code, 2)
        self.assertNotIn("enable", self.sdk.calls)
        self.assertEqual(log["motion_commands_sent"], 0)

    def test_post_enable_drift_stops_before_com(self):
        self.sdk.drift_on_enable = True
        code, log, _ = self.invoke("--run", approved=True)
        self.assertEqual(code, 2)
        self.assertNotIn("com", self.sdk.calls)
        self.assertIn("emergency", self.sdk.calls)

    def test_cleanup_error_cannot_be_success(self):
        self.sdk.fail_restore = True
        code, log, _ = self.invoke("--run", approved=True)
        self.assertEqual(code, 2)
        self.assertEqual(log["status"], "CLEANUP_FAILED")


if __name__ == "__main__": unittest.main()
