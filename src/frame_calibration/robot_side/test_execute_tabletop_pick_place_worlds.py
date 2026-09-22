#!/usr/bin/env python3
"""execute_tabletop_pick_place_worlds.py 的纯离线安全测试。"""
from __future__ import annotations

import contextlib
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


PATH = Path(__file__).with_name("execute_tabletop_pick_place_worlds.py")
SPEC = importlib.util.spec_from_file_location("execute_tabletop_pick_place_worlds", PATH)
executor = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(executor)


SAFE = [600.0, 100.0, 500.0, 0.0, 0.0, 0.0]
PICK_HOVER = [620.0, 110.0, 500.0, 0.0, 0.0, 0.0]
PICK = [620.0, 110.0, 450.0, 0.0, 0.0, 0.0]
PLACE_HOVER = [650.0, 0.0, 520.0, 0.0, 0.0, 0.0]
PLACE = [650.0, 0.0, 460.0, 0.0, 0.0, 0.0]
SAFE_Q = [-20.0, 30.0, 0.0, -70.0, 0.0, 0.0, 0.0]
CONFIG = {
    "robot_ip": "192.0.2.10",
    "local_ip": "192.0.2.20",
    "arm_ip": "192.0.2.10",
    "arm_port": 8080,
    "global_speed": 1.0,
}


def pose(values):
    return {
        "position_mm": list(values[:3]),
        "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
        "sdk_world_uvw_deg": list(values[3:]),
    }


def make_plan(*, blockers=None):
    blockers = list(blockers or [])
    policy = {
        "policy_id": "left-gripper2-test-v1",
        "gripper_id": 2,
        "tuning_status": "SITE_TUNABLE",
        "protocol_encoding": "EB90_UINT16_LITTLE_ENDIAN",
        "open_command": 17,
        "close_command": 16,
        "command_contract": "OPEN_POSITION_CLOSE_SPEED_FORCE",
        "value_unit": "device_native_uint16",
        "executor_binding": "execute_tabletop_pick_place_worlds.py/v1",
        "open": {"position": 500, "settle_s": 0.0},
        "close": {"speed": 500, "force": 800, "settle_s": 0.0},
    }
    policy["source_sha256"] = hashlib.sha256(
        executor._canonical_bytes(policy)).hexdigest()
    resolved = {
        "pick_endpoint": pose(PICK),
        "pick_hover_endpoint": pose(PICK_HOVER),
        "place_endpoint": pose(PLACE),
        "place_hover_endpoint": pose(PLACE_HOVER),
        "place_object_after_release": pose(PLACE),
        "safe_endpoint": pose(SAFE),
    }

    def move(name, key, carrying):
        return {
            "name": name,
            "kind": "MOVE_WORLDS",
            "pose": copy.deepcopy(resolved[key]),
            "carrying": carrying,
            "interpolation_en": False,
            "wait_until_worlds": True,
            "critical": True,
        }

    stages = [
        {"name": "START_ASSERT_SAFE", "kind": "ASSERT_ENDPOINT",
         "pose": copy.deepcopy(resolved["safe_endpoint"]), "carrying": False},
        {"name": "OPEN_BEFORE_PICK", "kind": "GRIPPER", "action": "open",
         "policy_id": policy["policy_id"], "carrying_after": False,
         "wait_until_done": True},
        move("PICK_HOVER", "pick_hover_endpoint", False),
        move("PICK_DESCEND", "pick_endpoint", False),
        {"name": "CLOSE_AT_PICK", "kind": "GRIPPER", "action": "close",
         "policy_id": policy["policy_id"], "carrying_after": True,
         "wait_until_done": True},
        move("PICK_ASCEND", "pick_hover_endpoint", True),
        move("PLACE_HOVER", "place_hover_endpoint", True),
        move("PLACE_DESCEND", "place_endpoint", True),
        {"name": "OPEN_AT_PLACE", "kind": "GRIPPER", "action": "open",
         "policy_id": policy["policy_id"], "carrying_after": False,
         "wait_until_done": True},
        move("PLACE_ASCEND", "place_hover_endpoint", False),
        move("RETURN_SAFE", "safe_endpoint", False),
        {"name": "END_ASSERT_SAFE", "kind": "ASSERT_ENDPOINT",
         "pose": copy.deepcopy(resolved["safe_endpoint"]), "carrying": False},
    ]
    return {
        "schema_version": "tabletop_plan.v1",
        "plan_id": "unit-test-plan",
        "status": ("OFFLINE_CANDIDATE_BLOCKED" if blockers else
                   "OFFLINE_CANDIDATE_PRECHECK_REQUIRED"),
        "real_motion_authorized": False,
        "contract": {
            "arm": "left", "arm_id": 1, "gripper_id": 2,
            "world_frame": "sdk_world", "length_unit": "millimeter",
            "quaternion_order": "xyzw",
            "sdk_uvw_convention": "Rz(W)*Ry(V)*Rx(U)",
            "dynamic_transport": "MOVE_WORLDS_ONLY",
            "move_j_policy": "VERIFIED_FIXED_SEGMENTS_ONLY_NOT_USED_IN_THIS_PLAN",
            "runtime_authorization": "RUN_FLAG_PLUS_ENV_GATE_PLUS_OPERATOR_CONFIRMATION",
        },
        "resolved": resolved,
        "stages": stages,
        "gripper_policy": policy,
        "safety": {"input_blockers": blockers, "input_geometry_gate": "BLOCKED" if blockers else "PASS"},
    }


class FakeSDK:
    def __init__(self, owner, enable):
        self.owner = owner
        self.enable = enable

    def armOpenCom(self, name, baud, bits, parity, stop):
        self.owner.events.append(("open_com", name, baud, bits, parity, stop))
        return 0

    def armWriteCom(self, name, payload):
        self.owner.gripper_frames.append(bytes(payload))
        action = "open" if payload[4] == 0x11 else "close"
        self.owner.events.append(("gripper", action))
        return 0


class FakeSS:
    def __init__(self):
        self.events = []
        self.sessions = []
        self.moves = []
        self.gripper_frames = []
        self.current = SAFE[:]
        self.try_calls = 0
        self.fail_try_call = None
        self.fail_move_number = None
        self.route_clears = 0

    def open_session(self, robot_ip, local_ip, arm_ip, arm_port,
                     speed, arm_ids, enable):
        self.events.append(("session", enable))
        sdk = FakeSDK(self, enable)
        self.sessions.append(sdk)
        return sdk

    def close_session(self, sdk):
        self.events.append(("close_session", sdk.enable))

    def read_worlds(self, sdk, arm_id):
        return self.current[:]

    def read_joints(self, sdk, arm_id):
        return SAFE_Q[:]

    def read_axis_limits(self, sdk, arm_id):
        return [(-180.0, 180.0)] * 7

    def axis_limits_look_valid(self, limits):
        return True, []

    def check_limits(self, arm_id, joints, margin):
        return []

    def try_worlds(self, sdk, arm_id, target):
        self.try_calls += 1
        if self.fail_try_call == self.try_calls:
            return None
        return SAFE_Q[:]

    def check_soft_stop(self, sdk, label):
        self.events.append(("soft_stop", label))

    def move_worlds(self, sdk, arm_id, target, interpolation_en):
        self.moves.append((list(target), interpolation_en))
        self.events.append(("move", list(target), interpolation_en))
        if self.fail_move_number == len(self.moves):
            raise RuntimeError("injected move failure")

    def wait_until_worlds(self, sdk, arm_id, target, **kwargs):
        self.current = list(target)
        self.events.append(("wait", list(target)))
        return self.current[:]

    def clear_robot_route(self, sdk, arm_id, emergency_stop):
        self.route_clears += 1
        self.events.append(("clear_route", emergency_stop))

    @staticmethod
    def require_code_zero(code, operation):
        if code != 0:
            raise RuntimeError(f"{operation}: {code}")


class FakeLock:
    @staticmethod
    @contextlib.contextmanager
    def acquire(robot_ip, arm_id, owner):
        yield None


def recovery_plan(*, confirmed=True):
    plan = make_plan()
    plan["startup_policy"] = {
        "mode": "RECOVER_TO_TASK_SAFE_IF_NEEDED",
        "clearance_z_mm": 550.0,
        "allowed_start_xyz_bounds_mm": [[550, 700], [-50, 150], [450, 600]],
        "maximum_rotation_deg": 20.0,
        "safe_joints_deg": SAFE_Q[:],
        "safe_joint_tolerance_deg": 2.0,
        "endpoint_exclusion_boxes_mm": [
            {"name": "synthetic_table", "min_mm": [0, -1000, 0], "max_mm": [2000, 1000, 400]}],
        "qualification": {"status": "CONFIRMED" if confirmed else "PENDING",
            "scope": "FULL_ARM_EMPTY_GRIPPER_RECOVERY_VOLUME",
            "source": "SYNTHETIC_UNIT_TEST_ONLY", "scene_revision": "synthetic"},
    }
    return plan


class ExecutorTests(unittest.TestCase):
    def test_horizontal_pick_independent_ascend_and_legacy_compatibility(self):
        plan = make_plan()
        executor.validate_plan(plan)
        plan["pick_mode"] = "HORIZONTAL_X_POSITIVE"
        plan["resolved"]["pick_hover_endpoint"] = pose([570, 110, 450, 0, 0, 0])
        plan["resolved"]["pick_ascend_endpoint"] = pose(PICK_HOVER)
        plan["stages"][2]["pose"] = copy.deepcopy(plan["resolved"]["pick_hover_endpoint"])
        executor.validate_plan(plan)
        plan["resolved"]["pick_hover_endpoint"]["position_mm"][1] += 1
        plan["stages"][2]["pose"] = copy.deepcopy(plan["resolved"]["pick_hover_endpoint"])
        with self.assertRaisesRegex(executor.PlanError, "Y/Z"):
            executor.validate_plan(plan)

    def test_recovery_four_moves_hold_orientation_then_rotate_at_height(self):
        plan = recovery_plan()
        current = [620, 80, 510, 10, 0, 0]
        targets = executor.build_startup_recovery(current, plan)
        self.assertEqual([name for name, _ in targets],
            ["RECOVERY_LIFT", "RECOVERY_ROTATE", "RECOVERY_TRANSIT", "RECOVERY_DESCEND"])
        self.assertEqual(targets[0][1], [620, 80, 550, 10, 0, 0])
        self.assertEqual(targets[1][1], [620, 80, 550, 0, 0, 0])
        self.assertEqual(targets[-1][1], SAFE)
        high = executor.build_startup_recovery([620,80,580,0,0,0], plan)
        self.assertNotIn("RECOVERY_LIFT", [name for name, _ in high])
        self.assertEqual(high[0][1][2], 580)

    def test_recovery_at_safe_is_skipped_but_legacy_50mm_tolerance_is_not_used(self):
        plan = recovery_plan()
        self.assertEqual(executor.build_startup_recovery(SAFE, plan), [])
        offset = SAFE[:]; offset[0] += 10
        self.assertTrue(executor.build_startup_recovery(offset, plan))

    def test_recovery_outside_region_or_large_rotation_rejected(self):
        plan = recovery_plan()
        for current in ([620,80,300,0,0,0], [620,80,510,30,0,0]):
            with self.assertRaises(executor.PrecheckError):
                executor.build_startup_recovery(current, plan)

    def test_continuous_recovery_segment_intersection_blocks_thin_obstacle(self):
        plan = recovery_plan()
        plan["startup_policy"]["endpoint_exclusion_boxes_mm"].append(
            {"name":"thin_wall", "min_mm":[609.01, -100, 540], "max_mm":[609.02,150,560]})
        with self.assertRaisesRegex(executor.PrecheckError, "thin_wall"):
            executor.build_startup_recovery([620,80,510,0,0,0], plan)

    def test_pending_recovery_permits_readonly_diagnostic_but_never_enable(self):
        ss = FakeSS(); ss.current = [620,80,510,0,0,0]
        plan = recovery_plan(confirmed=False)
        self.assertEqual(executor.run_with_sdk(ss, plan, CONFIG, real_motion=False), 0)
        with self.assertRaisesRegex(executor.PrecheckError, "净空尚未确认"):
            executor.run_with_sdk(ss, plan, CONFIG, real_motion=True,
                                  input_fn=lambda _: self.fail("未确认区域不能询问使能"))
        self.assertFalse(any(sdk.enable for sdk in ss.sessions))
        self.assertEqual(ss.moves, [])

    def test_recovery_requires_explicit_empty_gripper_then_operator_enter(self):
        ss = FakeSS(); ss.current = [620,80,510,0,0,0]
        self.assertEqual(executor.run_with_sdk(ss, recovery_plan(), CONFIG,
            real_motion=True, input_fn=lambda _: ""), 2)
        self.assertEqual(ss.moves, [])
        self.assertEqual([sdk.enable for sdk in ss.sessions], [False])

    def test_recovery_only_moves_before_any_com_or_gripper(self):
        ss = FakeSS(); ss.current = [620,80,510,10,0,0]
        answers = iter(["EMPTY", ""])
        result = executor.run_with_sdk(ss, recovery_plan(),
            dict(CONFIG, recovery_only=True), real_motion=True, input_fn=lambda _: next(answers))
        self.assertEqual(result, 0)
        self.assertEqual(len(ss.moves), 4)
        self.assertEqual(ss.current, SAFE)
        self.assertFalse(any(event[0] == "open_com" for event in ss.events))
        self.assertEqual(ss.gripper_frames, [])

    def test_recovery_connects_to_original_task_and_rechecks_after_arrival(self):
        ss = FakeSS(); ss.current = [620,80,510,0,0,0]
        answers = iter(["EMPTY", ""])
        self.assertEqual(executor.run_with_sdk(ss, recovery_plan(), CONFIG,
            real_motion=True, input_fn=lambda _: next(answers)), 0)
        self.assertEqual(len(ss.moves), 10)
        open_com_index = next(i for i,e in enumerate(ss.events) if e[0] == "open_com")
        self.assertEqual(sum(e[0]=="move" for e in ss.events[:open_com_index]), 3)

    def test_recovery_failure_does_not_open_gripper_or_retry(self):
        ss = FakeSS(); ss.current = [620,80,510,0,0,0]; ss.fail_move_number = 1
        answers = iter(["EMPTY", ""])
        with self.assertRaisesRegex(RuntimeError, "injected move failure"):
            executor.run_with_sdk(ss, recovery_plan(), CONFIG, real_motion=True,
                                  input_fn=lambda _: next(answers))
        self.assertEqual(len(ss.moves), 1)
        self.assertEqual(ss.gripper_frames, [])
        self.assertEqual(ss.route_clears, 1)

    def test_session_start_drift_is_rejected_before_motion(self):
        ss = FakeSS(); ss.current = [620,80,510,0,0,0]
        answers = iter(["EMPTY", ""])
        def confirm(_):
            answer = next(answers)
            if answer == "": ss.current[0] += 10
            return answer
        with self.assertRaises(executor.PrecheckError):
            executor.run_with_sdk(ss, recovery_plan(), CONFIG, real_motion=True, input_fn=confirm)
        self.assertEqual(ss.moves, [])
        self.assertEqual(ss.gripper_frames, [])

    def test_wrong_safe_joint_branch_and_nonfinite_feedback_are_rejected(self):
        ss = FakeSS()
        plan = recovery_plan(); plan["startup_policy"]["safe_joints_deg"][0] += 20
        with self.assertRaisesRegex(executor.PrecheckError, "分支"):
            executor.precheck_full_path(ss, None, plan)
        ss.current[0] = float("nan")
        with self.assertRaisesRegex(executor.PlanError, "非有限"):
            executor.precheck_full_path(ss, None, make_plan())

    def test_zero_margin_still_checks_live_controller_limits(self):
        ss = FakeSS()
        ss.read_axis_limits = lambda *_: [(-10,180)] + [(-180,180)]*6
        with self.assertRaisesRegex(executor.PrecheckError, "控制器安全区间"):
            executor.precheck_full_path(ss, None, make_plan(), margin=0.0)

    def test_invalid_recovery_policy_rejected_and_no_confirmed_empty_geometry(self):
        for mutation in (lambda p:p.update(mode="TYPO"),
                         lambda p:p.update(clearance_z_mm=float("nan")),
                         lambda p:p.update(endpoint_exclusion_boxes_mm=[])):
            plan = recovery_plan(); mutation(plan["startup_policy"])
            with self.assertRaises(executor.PlanError): executor.validate_plan(plan)

    def test_recovery_only_does_not_authorize_blocked_grasp_task(self):
        plan = recovery_plan()
        plan["status"] = executor.BLOCKED_STATUS
        plan["safety"] = {"input_blockers": ["task collision unknown"], "input_geometry_gate": "BLOCKED"}
        self.assertIsNone(executor.run_gate_error(plan, True, True, recovery_only=True))
        self.assertIsNotNone(executor.run_gate_error(plan, True, True))
        ss = FakeSS()
        with self.assertRaises(executor.PrecheckError):
            executor.run_with_sdk(ss, plan, CONFIG, real_motion=True)
        self.assertEqual(ss.sessions, [])

    def test_after_recovery_failed_task_recheck_never_opens_com(self):
        ss = FakeSS(); ss.current = [620,80,510,0,0,0]
        count = executor.precheck_full_path(ss, None, recovery_plan())["dense_points"]
        ss.try_calls = 0; ss.fail_try_call = 2 * count + 1
        answers = iter(["EMPTY", ""])
        with self.assertRaises(executor.PrecheckError):
            executor.run_with_sdk(ss, recovery_plan(), CONFIG, real_motion=True,
                                  input_fn=lambda _: next(answers))
        self.assertEqual(len(ss.moves), 3)
        self.assertEqual(ss.gripper_frames, [])
        self.assertFalse(any(e[0] == "open_com" for e in ss.events))

    def test_default_dry_run_is_readonly_and_sends_nothing(self):
        ss = FakeSS()
        result = executor.run_with_sdk(
            ss, make_plan(), CONFIG, real_motion=False,
            input_fn=lambda _: self.fail("dry-run 不应询问执行确认"))
        self.assertEqual(result, 0)
        self.assertEqual([sdk.enable for sdk in ss.sessions], [False])
        self.assertEqual(ss.moves, [])
        self.assertEqual(ss.gripper_frames, [])
        self.assertFalse(any(event[0] == "open_com" for event in ss.events))

    def test_precheck_failure_is_fail_closed_before_any_motion_or_gripper(self):
        ss = FakeSS()
        ss.fail_try_call = 1
        with self.assertRaisesRegex(executor.PrecheckError, "不可达"):
            executor.run_with_sdk(
                ss, make_plan(), CONFIG, real_motion=True, input_fn=lambda _: "")
        self.assertEqual([sdk.enable for sdk in ss.sessions], [False])
        self.assertEqual(ss.moves, [])
        self.assertEqual(ss.gripper_frames, [])
        self.assertEqual(ss.route_clears, 0)

    def test_success_uses_strict_unblended_move_and_gripper_order(self):
        ss = FakeSS()
        result = executor.run_with_sdk(
            ss, make_plan(), CONFIG, real_motion=True, input_fn=lambda _: "")
        self.assertEqual(result, 0)
        self.assertEqual([sdk.enable for sdk in ss.sessions], [False, True])
        self.assertTrue(all(interpolation is False for _, interpolation in ss.moves))
        actions = []
        for event in ss.events:
            if event[0] == "gripper":
                actions.append(event[1])
            elif event[0] == "move":
                actions.append(tuple(event[1][:3]))
        self.assertEqual(actions, [
            "open", tuple(PICK_HOVER[:3]), tuple(PICK[:3]), "close",
            tuple(PICK_HOVER[:3]), tuple(PLACE_HOVER[:3]), tuple(PLACE[:3]),
            "open", tuple(PLACE_HOVER[:3]), tuple(SAFE[:3]),
        ])
        self.assertEqual(ss.route_clears, 0)

    def test_run_requires_both_cli_and_environment_gate(self):
        ss = FakeSS()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "plan.json"
            path.write_text(json.dumps(make_plan()), encoding="utf-8")
            result = executor.main(
                [str(path), "--run"], ss_module=ss, lock_module=FakeLock,
                environ={}, input_fn=lambda _: "")
        self.assertEqual(result, 2)
        self.assertEqual(ss.sessions, [])

    def test_explicit_precheck_only_is_readonly(self):
        ss = FakeSS()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "plan.json"
            path.write_text(json.dumps(make_plan()), encoding="utf-8")
            result = executor.main(
                [str(path), "--precheck-only"],
                ss_module=ss, lock_module=FakeLock,
                environ={
                    "XIFENG_ROBOT_IP": "192.0.2.10",
                    "XIFENG_LOCAL_IP": "192.0.2.20",
                }, input_fn=lambda _: self.fail("预检模式不应询问执行确认"))
        self.assertEqual(result, 0)
        self.assertEqual([sdk.enable for sdk in ss.sessions], [False])
        self.assertEqual(ss.moves, [])
        self.assertEqual(ss.gripper_frames, [])

    def test_run_and_precheck_only_are_mutually_exclusive(self):
        with self.assertRaises(SystemExit) as caught:
            executor.main(["plan.json", "--run", "--precheck-only"])
        self.assertEqual(caught.exception.code, 2)

    def test_run_rejects_plan_input_blocker_before_session(self):
        ss = FakeSS()
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "plan.json"
            path.write_text(json.dumps(make_plan(blockers=["geometry missing"])),
                            encoding="utf-8")
            result = executor.main(
                [str(path), "--run"], ss_module=ss, lock_module=FakeLock,
                environ={
                    "XIFENG_ALLOW_REAL_MOTION": "1",
                    "XIFENG_ROBOT_IP": "192.0.2.10",
                    "XIFENG_LOCAL_IP": "192.0.2.20",
                }, input_fn=lambda _: "")
        self.assertEqual(result, 2)
        self.assertEqual(ss.sessions, [])

    def test_operator_must_confirm_with_blank_enter(self):
        ss = FakeSS()
        result = executor.run_with_sdk(
            ss, make_plan(), CONFIG, real_motion=True, input_fn=lambda _: "q")
        self.assertEqual(result, 2)
        self.assertEqual([sdk.enable for sdk in ss.sessions], [False])
        self.assertEqual(ss.moves, [])
        self.assertEqual(ss.gripper_frames, [])

    def test_exception_clears_route_without_automatic_release(self):
        ss = FakeSS()
        # 第四条 MoveWorlds 在闭爪以后失败；异常处理不得再发送放置处 open。
        ss.fail_move_number = 4
        with self.assertRaisesRegex(RuntimeError, "injected move failure"):
            executor.run_with_sdk(
                ss, make_plan(), CONFIG, real_motion=True, input_fn=lambda _: "")
        gripper_actions = [event[1] for event in ss.events if event[0] == "gripper"]
        self.assertEqual(gripper_actions, ["open", "close"])
        self.assertEqual(ss.route_clears, 1)

    def test_gripper_frame_matches_eb90_little_endian_contract(self):
        frame = executor.build_gripper_frame(2, 0x10, [500, 800])
        self.assertEqual(frame[:5], bytes.fromhex("eb 90 02 05 10"))
        self.assertEqual(frame[5:9], bytes.fromhex("f4 01 20 03"))
        self.assertEqual(frame[-1], sum(frame[2:-1]) & 0xFF)

    def test_tampered_policy_hash_is_rejected(self):
        plan = make_plan()
        plan["gripper_policy"]["close"]["force"] += 1
        with self.assertRaisesRegex(executor.PlanError, "source_sha256"):
            executor.validate_plan(plan)


if __name__ == "__main__":
    unittest.main(verbosity=2)
