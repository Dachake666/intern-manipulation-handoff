"""混合轨迹只读交付回归；无机器人连接，SDK白名单假件验证。"""
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
import precheck_tabletop_hybrid as h


def fixture():
    pose = {"position_mm": [600., 100., 550.], "sdk_world_uvw_deg": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]}
    stages, frames = [], [{"stage": "CAPTURED_LIVE_START_NOT_ASSERTED_SAFE", "q_sdk_deg": [0.] * 7}]
    for name, kind in h.STAGES:
        stage = {"name": name, "kind": kind}
        if kind == "GRIPPER":
            stage["action"] = "close" if name == "CLOSE_AT_PICK" else "open"
            frames.append({"stage": name, "action": stage["action"]})
        elif kind == "MOVE_JOINTS":
            stage.update(q_sdk_deg=[0.] * 7, expected_endpoint=copy.deepcopy(pose))
        else:
            stage["pose"] = copy.deepcopy(pose)
        if kind.startswith("MOVE_"):
            frames.append({"stage": name, "q_sdk_deg": [0.] * 7})
        stages.append(stage)
    return {"schema_version": h.SCHEMA, "status": "OFFLINE_CANDIDATE_BLOCKED",
            "real_motion_authorized": False, "debian_execution_allowed": False,
            "meta": {"arm_id": 1, "gripper_id": 2},
            "contract": {"arm": "left", "arm_id": 1, "gripper_id": 2, "world_frame": "sdk_world",
                         "length_unit": "millimeter", "dynamic_transport": "MOVE_WORLDS_AND_MOVE_JOINTS"},
            "stages": stages, "offline_replay": {"frames": frames}, "expected_pick_ascent_joints_sdk_deg": [0.] * 7,
            "blockers": ["test collision not verified"]}


class FakeSDK:
    def __init__(self):
        self.stopped = False
        self.moving = False
        self.ik = [0.] * 7
        self.q = [0.] * 7

    def armGetWorlds(self, arm, mode): return 0, [600., 100., 550., 0., 0., 0.]
    def armGetJoints(self, arm): return 0, list(self.q)
    def armGetRobotMoveState(self, arm): return 0, self.moving
    def getSoftStopSwitch(self): return 0, "CLOSE"
    def armTryWorlds(self, arm, pose):
        if self.ik is None: return 1, []
        if isinstance(self.ik, BaseException): raise self.ik
        return 0, self.ik
    def stop(self): self.stopped = True


class Helper:
    LIMITS_DEG = h.ss.LIMITS_DEG
    def __init__(self):
        self.sdk = FakeSDK()
        self.limits = copy.deepcopy(self.LIMITS_DEG[1])
        self.opens = 0
    def open_read_only_session(self, *args):
        self.opens += 1
        return self.sdk
    def read_worlds(self, sdk, arm): return sdk.armGetWorlds(arm, True)[1]
    def read_joints(self, sdk, arm): return sdk.armGetJoints(arm)[1]
    def read_axis_limits(self, sdk, arm): return self.limits
    def try_worlds(self, sdk, arm, pose):
        code, q = sdk.armTryWorlds(arm, pose)
        return q if code == 0 else None
    check_soft_stop = staticmethod(h.ss.check_soft_stop)
    read_move_state = staticmethod(h.ss.read_move_state)
    intersect_axis_limits = staticmethod(h.ss.intersect_axis_limits)
    axis_limits_look_valid = staticmethod(h.ss.axis_limits_look_valid)


CONFIG = dict(robot_ip="192.0.2.10", local_ip="192.0.2.11", arm_ip="192.0.2.10", arm_port=8080)


class HybridPrecheckTests(unittest.TestCase):
    def run_capture(self, helper=None, plan=None):
        helper = helper or Helper()
        report = {}
        with contextlib.redirect_stdout(io.StringIO()):
            code = h.capture(helper, CONFIG, plan or fixture(), report)
        return helper, code, report

    def test_good_queries_never_authorize_motion(self):
        helper, code, report = self.run_capture()
        self.assertEqual(code, 0)
        self.assertTrue(helper.sdk.stopped)
        self.assertFalse(report["real_motion_authorized"])
        self.assertEqual(len(report["segments"]), 8)
        self.assertIn("NOT_MOTION_AUTHORIZED", report["status"])
        self.assertTrue(all("NOT_FK_OR_CONTROLLER_PATH_VERIFIED" in s["move_j_verdict"]
                            for s in report["segments"] if s["kind"] == "MOVE_JOINTS"))

    def test_motion_methods_denied(self):
        for name in ("armMoveWorlds", "armMoveJoints", "armWriteCom", "armClearRobotRoute", "armSetGlobalSpeed", "armEnable", "armSetRobotProtectStatus"):
            with self.assertRaises(RuntimeError): getattr(h.ReadOnlySDK(FakeSDK()), name)

    def test_missing_and_reordered_frames_rejected(self):
        p = fixture()
        p["offline_replay"]["frames"] = p["offline_replay"]["frames"][:-1]
        with self.assertRaises(ValueError): h.validate(p)
        p = fixture()
        p["stages"][2], p["stages"][3] = p["stages"][3], p["stages"][2]
        with self.assertRaises(ValueError): h.validate(p)

    def test_wrong_arm_units_authorization_rejected(self):
        for key, value in (("arm_id", 2), ("length_unit", "meter")):
            p = fixture(); p["contract"][key] = value
            with self.assertRaises(ValueError): h.validate(p)
        for key in ("real_motion_authorized", "debian_execution_allowed"):
            p = fixture(); p[key] = True
            with self.assertRaises(ValueError): h.validate(p)

    def test_nonfinite_or_bool_frame_rejected(self):
        for v in (float("nan"), True):
            p = fixture(); p["offline_replay"]["frames"][0]["q_sdk_deg"][0] = v
            with self.assertRaises(ValueError): h.validate(p)

    def test_move_j_frame_endpoint_must_match(self):
        p = fixture(); p["stages"][6]["q_sdk_deg"][0] = 1
        with self.assertRaises(ValueError): h.validate(p)

    def test_moving_arm_is_rejected_and_stopped(self):
        helper = Helper(); helper.sdk.moving = True
        helper, code, r = self.run_capture(helper)
        self.assertEqual(code, 2); self.assertTrue(helper.sdk.stopped)
        self.assertEqual(r["segments"], [])

    def test_unreachable_queries_are_preserved(self):
        helper = Helper(); helper.sdk.ik = None
        _, code, r = self.run_capture(helper)
        self.assertEqual(code, 2); self.assertEqual(len(r["segments"]), 8)
        self.assertIsNone(r["segments"][0]["queries"][0]["q_sdk_deg"])

    def test_nan_ik_never_enters_report(self):
        helper = Helper(); helper.sdk.ik = [float("nan")] * 7
        _, code, r = self.run_capture(helper)
        self.assertEqual(code, 2)
        json.dumps(r, allow_nan=False)

    def test_controller_limits_do_not_expand_j7(self):
        helper = Helper(); helper.limits[6] = [-88., 88.]
        _, _, r = self.run_capture(helper)
        self.assertEqual(r["effective_limits_deg"][6][1], h.ss.LIMITS_DEG[1][6][1])

    def test_move_j_checks_interior_not_just_endpoints(self):
        helper = Helper(); helper.limits[5] = [-40., 7.]
        p = fixture(); frames = p["offline_replay"]["frames"]
        index = next(i for i,f in enumerate(frames) if f["stage"] == "TRANSFER_MID")
        interior = copy.deepcopy(frames[index]); interior["q_sdk_deg"][5] = 3.
        frames.insert(index, interior)
        _, code, r = self.run_capture(helper, p)
        segment = next(s for s in r["segments"] if s["stage"] == "TRANSFER_MID")
        self.assertEqual(code, 2)
        self.assertEqual(segment["dense_joint_frames_checked"], 2)
        self.assertEqual(segment["joint_limit_errors"][0]["sample"], 0)

    def test_changed_live_pose_invalidates_queries(self):
        helper = Helper(); count = 0
        def read_joints(sdk, arm):
            nonlocal count
            count += 1
            return [2. if count > 1 else 0.] + [0.] * 6
        helper.read_joints = read_joints
        helper, code, r = self.run_capture(helper)
        self.assertEqual(code, 2); self.assertTrue(helper.sdk.stopped)
        self.assertIn("关节已改变", r["error"])

    def test_new_ik_branch_is_reported(self):
        helper = Helper(); helper.sdk.ik = [3., 0., 0., 0., 0., 0., 0.]
        _, code, r = self.run_capture(helper)
        self.assertEqual(code, 2)
        self.assertIn("ENDPOINT_IK_DIFFERS_FROM_OFFLINE_BRANCH", r["segments"][0]["issues"])

    def test_interrupt_and_stop_failure(self):
        helper = Helper(); helper.sdk.ik = KeyboardInterrupt()
        helper, code, _ = self.run_capture(helper)
        self.assertEqual(code, 130); self.assertTrue(helper.sdk.stopped)
        helper = Helper()
        def fail(): raise RuntimeError("cleanup")
        helper.sdk.stop = fail
        _, code, r = self.run_capture(helper)
        self.assertEqual(code, 2); self.assertEqual(r["status"], "SESSION_STOP_FAILED")

    def test_explicit_stop_rejection_is_not_success(self):
        helper = Helper()
        helper.sdk.stop = lambda: False
        _, code, report = self.run_capture(helper)
        self.assertEqual(code, 2)
        self.assertFalse(report["session_stopped"])
        self.assertEqual(report["status"], "SESSION_STOP_FAILED")

    def test_report_scrubs_secret_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory(dir=WORK, prefix=".hybrid-test-") as folder:
            path = Path(folder) / "report.json"
            # 合成测试数据；避免静态扫描把测试夹具当成归档的控制器凭据。
            fixture_token = "test-secret"
            h.write_report(path, {"token": fixture_token})
            self.assertNotIn(fixture_token, path.read_text())
            with self.assertRaises(FileExistsError): h.write_report(path, {})

    def test_no_run_or_margin_bypass_cli(self):
        for args in (("unused.json", "--run"), ("unused.json", "--limit-margin-deg", "0")):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit): h.main(list(args))

    def test_real_candidate_dryrun_never_opens_sdk(self):
        source = WORK / "pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_CANDIDATE.json"
        with tempfile.TemporaryDirectory(dir=WORK, prefix=".hybrid-test-") as folder:
            output = Path(folder) / "report.json"
            with patch.object(h.ss, "open_read_only_session", side_effect=AssertionError("SDK called")), contextlib.redirect_stdout(io.StringIO()):
                code = h.main([str(source), "--dry-run", "--output", str(output)])
            self.assertEqual(code, 0)
            r = json.loads(output.read_text())
            self.assertFalse(r["sdk_connected"]); self.assertEqual(r["motion_stages"], 8)


if __name__ == "__main__":
    unittest.main()
