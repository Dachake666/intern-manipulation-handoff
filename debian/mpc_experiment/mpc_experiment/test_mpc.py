"""MPC 数学、故障停机、参考契约、真实 PB 电机反馈的离线回归。"""
from dataclasses import replace
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from controller import JointMPC, MPCError, Settings
from run_experiment import DEFAULT_REFERENCE, BulletPlant, load_reference, sha, audit_urdf_inertias, cc


def solver(n=1, **kwargs):
    return JointMPC([[-80, 80]]*n, 10, 60, Settings(**kwargs))


def test_mpc_actually_uses_feedback():
    ref = np.ones((15, 1))*0.01
    nominal = solver().step([0], [0], [0], ref)
    behind = solver().step([-0.005], [0], [0], ref)
    assert behind["command_deg"][0] > nominal["command_deg"][0]


@pytest.mark.parametrize("alpha", [0.25, 0.4, 1.0])
def test_prediction_matches_forward_model(alpha):
    m = solver(response_alpha=alpha)
    result = m.step([1], [1.1], [0], np.ones((15, 1))*2)
    q = np.array([1.0])
    predictions = []
    for command in result["predicted_command_deg"]:
        q = (1-alpha)*q + alpha*command
        predictions.append(q.copy())
    np.testing.assert_allclose(predictions, result["predicted_q_deg"], atol=1e-10)


def test_constraints_and_terminal_stop_with_unreachable_target():
    m = solver(horizon=25, response_alpha=1.0)
    q, v = np.array([79.0]), np.array([0.0])
    for _ in range(150):
        result = m.step(q, q, v, np.full((25, 1), 90))
        c = result["predicted_command_deg"]
        assert np.max(c) <= 80 + 2e-5
        assert abs(result["velocity_deg_s"][0]) <= 10 + 2e-5
        assert abs(result["velocity_deg_s"][0]-v[0]) <= 60*0.02 + 2e-5
        # 最后一步速度为零，未来指令的最后两点重合。
        np.testing.assert_allclose(c[-1], c[-2], atol=2e-5)
        q, v = result["command_deg"], result["velocity_deg_s"]
    assert abs(q[0]-80) < 0.01


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf")])
def test_rejects_nonfinite_feedback(bad):
    with pytest.raises(ValueError):
        solver().step([bad], [0], [0], np.zeros((15, 1)))


def test_rejects_bad_reference_shape():
    with pytest.raises(ValueError):
        solver().step([0], [0], [0], np.zeros((14, 1)))


def test_rejects_outside_initial_state_not_silent_clipping():
    with pytest.raises(MPCError, match="越过"):
        solver().step([81], [0], [0], np.zeros((15, 1)))


def test_infeasible_cannot_stop_is_rejected():
    m = JointMPC([[-80, 80]], 20, 5, Settings())
    with pytest.raises(MPCError):
        m.step([79.9], [79.9], [20], np.full((15, 1), 80))


@pytest.mark.parametrize("status", [2, 3, 7, 8])
def test_solver_inaccurate_infeasible_timeout_never_reuses_solution(status):
    m = solver()
    m.solver = SimpleNamespace(update=lambda **kw: None,
        solve=lambda **kw: SimpleNamespace(info=SimpleNamespace(status_val=status, status="fake"),
                                           x=np.zeros(15)))
    with pytest.raises(MPCError, match="不执行旧解"):
        m.step([0], [0], [0], np.zeros((15, 1)))


def test_solver_claiming_success_but_violating_constraints_is_rejected():
    m = solver()
    m.solver = SimpleNamespace(update=lambda **kw: None,
        solve=lambda **kw: SimpleNamespace(info=SimpleNamespace(status_val=1, status="solved"),
                                           x=np.full(15, 10000)))
    with pytest.raises(MPCError, match="残差"):
        m.step([0], [0], [0], np.zeros((15, 1)))


@pytest.mark.parametrize("field,value", [("dt", 0), ("horizon", 1), ("response_alpha", 1.1),
                                          ("velocity_weight", -1), ("dt", float("nan"))])
def test_settings_reject_invalid(field, value):
    with pytest.raises(ValueError):
        replace(Settings(), **{field: value})


def test_reference_identity_events_and_timing_unchanged():
    before = sha(DEFAULT_REFERENCE)
    plan, blocks, source = load_reference(DEFAULT_REFERENCE, 0.02)
    assert before == sha(DEFAULT_REFERENCE)
    assert source["motion_frame_count"] == 2484
    assert [b["event"]["action"] for b in blocks if b["event"]] == ["open", "close", "open"]
    assert [b["event"]["minimum_hold_s"] for b in blocks if b["event"]] == [0.5, 1, 0.5]
    assert sum(len(b["q"]) for b in blocks) == 2484+25+50+25+25
    np.testing.assert_equal(blocks[-1]["q"][-1], plan["home_joints_sdk_deg"])


def test_reference_period_mismatch_rejected():
    with pytest.raises(ValueError, match="周期"):
        load_reference(DEFAULT_REFERENCE, 0.01)


def test_pybullet_position_motor_not_teleport(monkeypatch):
    import pybullet as pb
    plan, _, _ = load_reference(DEFAULT_REFERENCE, 0.02)
    q0 = np.asarray(plan["first_point_q_sdk_deg"])
    plant = BulletPlant(q0, 0.02)
    try:
        def no_reset(*args, **kwargs):
            raise AssertionError("step 必须来自物理电机，不许 resetJointState")
        monkeypatch.setattr(pb, "resetJointState", no_reset)
        target = q0.copy()
        target[0] += 0.5
        samples = plant.step(target)
        assert len(samples) == 8
        q1, v1 = samples[-1]
        assert 0.01 < q1[0]-q0[0] < 0.49
        assert np.max(np.abs(v1)) > 0.01
        for _ in range(60):
            last = plant.step(target)[-1][0]
        assert np.max(np.abs(last-target)) < 0.01
    finally:
        plant.close()


def test_no_hardware_cli_switch():
    # 实验只输出与 SDK 轨迹不同的 schema；无 run/enable/serial 入口。
    source = (HERE/"run_experiment.py").read_text()
    assert 'parser.add_argument("--run"' not in source
    assert '"schema_version": "offline_joint_mpc_experiment.v1"' in source
    assert '"real_motion_authorized": False' in source


def test_invalid_vendor_inertias_are_reported_not_silently_used():
    issues = {x["link"] for x in audit_urdf_inertias(cc.DEFAULT_URDF)}
    assert {"left_elbow_pitch_link", "right_elbow_pitch_link"} <= issues
