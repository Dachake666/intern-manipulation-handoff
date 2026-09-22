"""双臂只读包离线回归：仅假 SDK，绝不连接硬件。"""
from __future__ import annotations

import builtins
import contextlib
import copy
import json
import os
from pathlib import Path
import sys
import types

import pytest

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
sys.path.insert(0, str(HERE))
import precheck_dual_arm as m

PLAN = ROOT / "pick_place_coord/dual_arm_demo/dual_arm_simulation.json"
CONFIG = {"robot_ip": "192.0.2.1", "local_ip": "192.0.2.2",
          "arm_ip": "192.0.2.1", "arm_port": 8080}


@pytest.fixture
def plan():
    return json.loads(PLAN.read_text())


class FakeSDK:
    """只提供读 API 和连接清理；任何未知/写 API 都令测试失败。"""
    def __init__(self, plan):
        self.calls = []
        self.q = {m.PROFILES[a]["arm_id"]: list(plan["frames"][0]["q_sdk_deg"][a])
                  for a in m.PROFILES}
        self.bounds = {m.PROFILES[a]["arm_id"]: copy.deepcopy(m.nominal_limits(a))
                       for a in m.PROFILES}
        self.limit_reads = {1: 0, 2: 0}
        self.joint_reads = {1: 0, 2: 0}
        self.moving = False
        self.raise_on_read = None
        self.limit_drift = False
        self.joint_drift = False
        self.stop_result = None
        self.stop_exception = None

    def __getattr__(self, name):
        raise AssertionError(f"假 SDK 不允许该调用：{name}")

    def getSoftStopSwitch(self):
        self.calls.append("getSoftStopSwitch")
        return 0, "CLOSE"

    def armGetRobotMoveState(self, aid):
        self.calls.append(("armGetRobotMoveState", aid))
        return 0, self.moving

    def armGetJoints(self, aid, fb=True):
        self.calls.append(("armGetJoints", aid, fb))
        if self.raise_on_read is not None:
            raise self.raise_on_read
        self.joint_reads[aid] += 1
        result = list(self.q[aid])
        if self.joint_drift and self.joint_reads[aid] > 1:
            result[0] += 1
        return 0, result

    def armGetWorlds(self, aid, fb=True):
        self.calls.append(("armGetWorlds", aid, fb))
        return 0, [600, 200 if aid == 1 else -200, 500, 0, 0, 0]

    def armGetAxisParameter(self, aid):
        # 假 read_axis_limits 聚合回调；实际 SDK 的每轴解析另由 SDK 测试覆盖。
        self.calls.append(("armGetAxisParameter", aid))
        self.limit_reads[aid] += 1
        result = copy.deepcopy(self.bounds[aid])
        if self.limit_drift and self.limit_reads[aid] > 1:
            result[0][0] += 1
        return result

    def stop(self):
        self.calls.append("stop")
        if self.stop_exception:
            raise self.stop_exception
        return self.stop_result


class FakeHelper(types.ModuleType):
    def __init__(self, sdk):
        super().__init__("sdk_session")
        self.sdk = sdk
        self.open_args = None

    def open_read_only_session(self, *args):
        self.open_args = args
        return self.sdk

    def close_session(self, *args):
        raise AssertionError("只读采集不应调用会修改设置的 close_session")

    def check_soft_stop(self, ro, operation):
        assert ro.getSoftStopSwitch() == (0, "CLOSE")

    def read_move_state(self, ro, aid):
        return ro.armGetRobotMoveState(aid)[1]

    def read_joints(self, ro, aid):
        return ro.armGetJoints(aid, True)[1]

    def read_worlds(self, ro, aid):
        return ro.armGetWorlds(aid, True)[1]

    def read_axis_limits(self, ro, aid):
        return ro.armGetAxisParameter(aid)

    @staticmethod
    def axis_limits_look_valid(bounds):
        return all(hi - lo >= 10 for lo, hi in bounds), "fake diagnostic"

    @staticmethod
    def intersect_axis_limits(controller, operational):
        result = [(max(a, c), min(b, d)) for (a, b), (c, d) in zip(controller, operational)]
        if any(hi <= lo for lo, hi in result):
            raise ValueError("empty intersection")
        return result


def capture(plan, sdk=None):
    sdk = sdk or FakeSDK(plan)
    helper = FakeHelper(sdk)
    report = {}
    result = m.capture(helper, CONFIG, plan, report)
    return result, report, sdk, helper


def test_valid_candidate_and_all_frames(plan):
    assert m.validate(plan) is plan["frames"]
    code, report, sdk, helper = capture(plan)
    assert code == 0
    assert report["status"] == "READONLY_LIMITS_AND_START_PASS_NOT_MOTION_AUTHORIZED"
    assert report["real_motion_authorized"] is False
    assert report["session_stopped"] is True
    assert helper.open_args == (*CONFIG.values(), (1, 2))
    for arm in ("left", "right"):
        assert report["arms"][arm]["dense_joint_limit_check"]["frames_checked"] == len(plan["frames"])
        assert report["arms"][arm]["start_aligned"] is True
    assert sdk.calls.count("stop") == 1


@pytest.mark.parametrize("key,value", [
    ("schema_version", "single_arm.v1"), ("coordinate_frame", "SDK"),
    ("real_motion_authorized", True), ("debian_execution_allowed", True),
    ("real_motion_authorized", 0), ("units", {"joint": "radian"}),
])
def test_reject_wrong_top_level_contract(plan, key, value):
    plan[key] = value
    with pytest.raises(ValueError):
        m.validate(plan)


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "1", None])
def test_reject_bad_joint_number(plan, bad):
    plan["frames"][0]["q_sdk_deg"]["right"][0] = bad
    with pytest.raises(ValueError):
        m.validate(plan)


@pytest.mark.parametrize("mutation", [
    lambda p: p["meta"]["right"].update(arm_id=1),
    lambda p: p["meta"]["left"].update(joint_ids=[1, 2, 3, 4, 5, 6, 7]),
    lambda p: p["playback"].update(period_s=0.03),
    lambda p: p["playback"].update(joint_step_deg=0.5),
    lambda p: p["frames"][1]["q_sdk_deg"]["left"].__setitem__(0, 170),
    lambda p: p["frames"][0].update(carrying=True),
    lambda p: p["frames"][0].update(stage=""),
    lambda p: p["frames"][0]["q_sdk_deg"].update(left=[0] * 6),
])
def test_reject_mappings_timing_steps_and_state(plan, mutation):
    mutation(plan)
    with pytest.raises(ValueError):
        m.validate(plan)


@pytest.mark.parametrize("field,value", [("event", "release"), ("event_arms", ["left"]), ("carrying", True)])
def test_reject_bad_event(plan, field, value):
    event = next(row for row in plan["frames"] if row.get("event") == "open")
    event[field] = value
    with pytest.raises(ValueError):
        m.validate(plan)


def test_reject_event_while_moving(plan):
    event = next(row for row in plan["frames"] if row.get("event") == "close")
    event["q_sdk_deg"]["left"][0] += 0.1
    with pytest.raises(ValueError, match="停稳"):
        m.validate(plan)


def test_left_j7_cannot_be_widened_by_controller(plan):
    sdk = FakeSDK(plan)
    sdk.bounds[1][6] = [-88, 88]
    code, report, *_ = capture(plan, sdk)
    assert code == 0
    assert report["arms"]["left"]["dense_joint_limit_check"]["limits_deg"][6][1] == 76


def test_actual_right_limits_narrower_than_nominal_fail(plan):
    sdk = FakeSDK(plan)
    sdk.bounds[2][1] = [-150, -10]
    code, report, *_ = capture(plan, sdk)
    metrics = report["arms"]["right"]["dense_joint_limit_check"]
    assert code == 2
    assert metrics["hard_violation_count"] > 0
    assert metrics["frames_checked"] == len(plan["frames"])
    assert report["session_stopped"] is True


@pytest.mark.parametrize("kind", ["limits", "joints"])
def test_state_change_during_capture_rejects(plan, kind):
    sdk = FakeSDK(plan)
    sdk.limit_drift = kind == "limits"
    sdk.joint_drift = kind == "joints"
    code, report, *_ = capture(plan, sdk)
    assert code == 2
    assert report["status"] == "READONLY_FAILED"
    assert report["session_stopped"] is True
    assert "error" in report


def test_start_mismatch_is_reported_not_recovered(plan):
    sdk = FakeSDK(plan)
    sdk.q[1][0] += 2
    code, report, *_ = capture(plan, sdk)
    assert code == 2
    assert report["arms"]["left"]["start_aligned"] is False
    assert report["arms"]["left"]["start_action"] == "REPORT_ONLY_NO_MOTION_NO_HOME_RECOVERY"


@pytest.mark.parametrize("failure,expected", [(RuntimeError("read failed"), 2), (KeyboardInterrupt(), 130)])
def test_read_failure_and_interrupt_stop_session(plan, failure, expected):
    sdk = FakeSDK(plan)
    sdk.raise_on_read = failure
    code, report, *_ = capture(plan, sdk)
    assert code == expected
    assert report["session_stopped"] is True
    assert sdk.calls.count("stop") == 1


@pytest.mark.parametrize("bad", [False, -1, RuntimeError("stop failure")])
def test_cleanup_failure_is_not_pass(plan, bad):
    sdk = FakeSDK(plan)
    if isinstance(bad, Exception):
        sdk.stop_exception = bad
    else:
        sdk.stop_result = bad
    code, report, *_ = capture(plan, sdk)
    assert code == 2
    assert report["status"] == "SESSION_STOP_FAILED"
    assert report["session_stopped"] is False


def test_moving_or_unready_robot_is_rejected(plan):
    for moving, bounds in [(True, None), (False, [[0, 0]] * 7)]:
        sdk = FakeSDK(plan)
        sdk.moving = moving
        if bounds:
            sdk.bounds[2] = bounds
        code, report, *_ = capture(plan, sdk)
        assert code == 2
        assert report["session_stopped"] is True


@pytest.mark.parametrize("method", ["armMoveJoints", "armMoveWorlds", "armToServo", "armTryWorlds",
                                    "armRobotEnableOrNot", "armClearAlarm", "armSetGlobalSpeed",
                                    "setSoftStopSwitch", "armSetCollisionLevel", "close_session", "stop"])
def test_read_only_proxy_rejects_writes_before_underlying_lookup(plan, method):
    with pytest.raises(RuntimeError, match="拒绝"):
        getattr(m.ReadOnlySDK(FakeSDK(plan)), method)


def test_default_does_not_import_sdk_and_forces_motion_off(tmp_path, monkeypatch):
    original = builtins.__import__

    def deny_sdk(name, *args, **kwargs):
        if name in {"sdk_session", "pypilot"}:
            raise AssertionError("默认离线入口导入了 SDK")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", deny_sdk)
    monkeypatch.setenv("XIFENG_ALLOW_REAL_MOTION", "1")
    output = tmp_path / "report.json"
    assert m.main([str(PLAN), "--output", str(output)]) == 0
    report = json.loads(output.read_text())
    assert report["status"] == "FILE_CHECKS_PASS_NOT_MOTION_AUTHORIZED"
    assert report["sdk_connected"] is False
    assert report["plan_sha256"] == m.digest(PLAN)
    assert report["source_hashes"]["precheck_dual_arm.py"] == m.digest(Path(m.__file__))
    assert os.environ["XIFENG_ALLOW_REAL_MOTION"] == "0"
    assert not output.with_name("report_sdk.log").exists()


def test_no_run_switch(tmp_path):
    with pytest.raises(SystemExit) as exc:
        m.main([str(PLAN), "--run", "--output", str(tmp_path / "report.json")])
    assert exc.value.code == 2


def test_native_output_redaction_and_fd_restoration(tmp_path):
    path = tmp_path / "sdk.log"
    before = (os.fstat(1), os.fstat(2))
    with m.captured_sdk_log(path):
        os.write(1, b'{"token":"test-private-value"}\n')
        os.write(2, b'Authorization: Bearer test-private-bearer\n')
    after = (os.fstat(1), os.fstat(2))
    assert [(s.st_dev, s.st_ino) for s in before] == [(s.st_dev, s.st_ino) for s in after]
    text = path.read_text()
    assert "test-private-value" not in text
    assert "test-private-bearer" not in text
    assert text.count("<REDACTED>") == 2
    assert list(tmp_path.iterdir()) == [path]
    with pytest.raises(ValueError, match="不覆盖"):
        with m.captured_sdk_log(path):
            pass


def configure_live_main(monkeypatch, plan):
    sdk = FakeSDK(plan)
    helper = FakeHelper(sdk)
    monkeypatch.setitem(sys.modules, "sdk_session", helper)
    monkeypatch.setattr(m.robot_lock, "acquire_robot", lambda *args: contextlib.nullcontext())
    return sdk, helper


def live_args(output):
    return [str(PLAN), "--precheck-only", "--robot-ip", CONFIG["robot_ip"],
            "--local-ip", CONFIG["local_ip"], "--output", str(output)]


def test_live_mock_records_report_and_scrubbed_log(tmp_path, monkeypatch, plan):
    sdk, helper = configure_live_main(monkeypatch, plan)
    original_open = helper.open_read_only_session

    def open_with_native_log(*args):
        os.write(1, b'{"token":"fake-only-secret"}\n')
        return original_open(*args)

    helper.open_read_only_session = open_with_native_log
    output = tmp_path / "report.json"
    assert m.main(live_args(output)) == 0
    report = json.loads(output.read_text())
    log = output.with_name(report["sdk_log_file"])
    assert report["sdk_log_sha256"] == m.digest(log)
    assert "fake-only-secret" not in log.read_text()
    assert report["real_motion_authorized"] is False
    assert report["runtime"] == CONFIG
    assert sdk.calls[-1] == "stop"


def test_sdk_import_failure_still_binds_scrubbed_log(tmp_path, monkeypatch, plan):
    configure_live_main(monkeypatch, plan)
    original = builtins.__import__

    def fail_sdk(name, *args, **kwargs):
        if name == "sdk_session":
            os.write(2, b'{"password":"fake-import-secret"}\n')
            raise ImportError("SDK unavailable")
        return original(name, *args, **kwargs)

    monkeypatch.setattr(builtins, "__import__", fail_sdk)
    output = tmp_path / "report.json"
    assert m.main(live_args(output)) == 2
    report = json.loads(output.read_text())
    log = tmp_path / "report_sdk.log"
    assert report["sdk_log_file"] == log.name
    assert report["sdk_log_sha256"] == m.digest(log)
    assert "fake-import-secret" not in log.read_text()


def test_explicit_zero_port_is_rejected(tmp_path, monkeypatch, plan):
    sdk, helper = configure_live_main(monkeypatch, plan)
    output = tmp_path / "report.json"
    assert m.main(live_args(output) + ["--arm-port", "0"]) == 2
    assert helper.open_args is None


def test_existing_outputs_never_overwritten(tmp_path):
    for name in ("report.json", "report_sdk.log"):
        old = tmp_path / name
        old.write_text("original")
        assert m.main([str(PLAN), "--output", str(tmp_path / "report.json")]) == 2
        assert old.read_text() == "original"
        old.unlink()


def test_validation_report_must_bind_plan(tmp_path, monkeypatch, plan):
    candidate = tmp_path / "candidate.json"
    candidate.write_text(json.dumps(plan))
    (tmp_path / "validation_report.json").write_text(json.dumps({"plan_sha256": "0" * 64}))
    output = tmp_path / "report.json"
    assert m.main([str(candidate), "--output", str(output)]) == 2
    assert "SHA" in json.loads(output.read_text())["error"]


def make_manifest(root):
    required = set(m.RUNTIME_FILES) | {"arm_profiles.py", "arm_profiles.v1.json",
                                      "dual_arm_simulation.json", "validation_report.json"}
    for name in required:
        (root / name).write_text("test-content:" + name)
    manifest = root / "SHA256SUMS"
    manifest.write_text("\n".join(f"{m.digest(root / name)}  {name}" for name in sorted(required)) + "\n")
    return manifest


def test_bundle_manifest_detects_drift_missing_duplicate_and_path(tmp_path, monkeypatch):
    monkeypatch.setattr(m, "HERE", tmp_path)
    manifest = make_manifest(tmp_path)
    original = manifest.read_text()
    assert m.check_bundle_manifest() == "MATCH_LOCAL_MANIFEST_NOT_A_SIGNATURE"
    changes = [original.splitlines()[0] + "\n", original + original.splitlines()[0] + "\n",
               original + "0" * 64 + "  ../escape\n", original.replace(original[:64], "0" * 64, 1)]
    for changed in changes:
        manifest.write_text(changed)
        with pytest.raises((ValueError, FileNotFoundError)):
            m.check_bundle_manifest()


def test_manifest_failure_prevents_live_connection(tmp_path, monkeypatch, plan):
    sdk, helper = configure_live_main(monkeypatch, plan)
    monkeypatch.setattr(m, "check_bundle_manifest", lambda: (_ for _ in ()).throw(ValueError("changed bundle")))
    output = tmp_path / "report.json"
    assert m.main(live_args(output)) == 2
    assert helper.open_args is None
    assert "changed bundle" in json.loads(output.read_text())["error"]
