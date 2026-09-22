#!/usr/bin/env python3
"""P2 轨迹执行选项的离线单元测试（无需连接机器人）。"""
from __future__ import annotations

import importlib
import os
import sys
import tempfile
import types
import unittest


class _FloatVector(list):
    pass


# Mac 开发环境没有容器内的 pypilot；执行器导入阶段只需要 FloatVector 类型存在。
if "pypilot" not in sys.modules:
    fake_pypilot = types.ModuleType("pypilot")
    fake_pypilot.FloatVector = _FloatVector
    sys.modules["pypilot"] = fake_pypilot

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
executor = importlib.import_module("execute_trajectory")


def _q(value):
    return [float(value)] * 7


class ExecuteTrajectoryOptionsTest(unittest.TestCase):
    def test_cli_defaults_do_not_create_optional_artifacts(self):
        args = executor.build_arg_parser().parse_args(["traj.json"])
        self.assertFalse(args.continuous)
        self.assertFalse(args.record_worlds)
        self.assertIsNone(args.save_log)

    def test_continuous_mode_still_settles_at_boundaries(self):
        waypoints = [
            {"seg": "HOME->READY", "q_sdk_deg": _q(0)},
            {"seg": "HOME->READY", "q_sdk_deg": _q(1)},
            {"seg": "PICK_ASCEND", "q_sdk_deg": _q(2)},
            {"seg": "PICK_ASCEND", "gripper": "close"},
            {"seg": "READY->HOME", "q_sdk_deg": _q(3)},
        ]
        self.assertFalse(executor.waypoint_requires_full_settle(waypoints, 0))
        self.assertTrue(executor.waypoint_requires_full_settle(waypoints, 1))
        self.assertTrue(executor.waypoint_requires_full_settle(waypoints, 2))
        self.assertTrue(executor.waypoint_requires_full_settle(waypoints, 4))

    def test_multi_pair_transfer_segments_are_handoff_eligible(self):
        """多点任务的编号段名必须自动纳入交接; 近物链默认仍逐点停稳。"""
        def wp(seg, v):
            return {"seg": seg, "q_sdk_deg": _q(v)}

        transfers = ["HOME->READY", "READY->PICK1_HOVER",
                     "PICK1_HOVER->PLACE1_HOVER", "PLACE1_HOVER->PICK2_HOVER",
                     "PLACE2_HOVER->READY", "READY->HOME"]
        for seg in transfers:
            wps = [wp(seg, 0), wp(seg, 1), wp(seg, 2)]
            self.assertFalse(executor.waypoint_requires_full_settle(wps, 1),
                             msg=f"{seg} 段内中间点应可交接")
        for seg in ["PICK1_DESCEND", "PLACE2_ASCEND", "PLACE1_SETTLE"]:
            wps = [wp(seg, 0), wp(seg, 1), wp(seg, 2)]
            self.assertTrue(executor.waypoint_requires_full_settle(wps, 1),
                            msg=f"{seg} 默认必须完整停稳")

    def test_near_object_handoff_is_opt_in(self):
        """开关打开后竖直链中间点参与交接, 链末仍强制停稳。"""
        wps = [{"seg": "PICK1_DESCEND", "q_sdk_deg": _q(v)} for v in (0, 1, 2)]
        old = executor.CONTINUOUS_INCLUDE_NEAR_OBJECT
        try:
            executor.CONTINUOUS_INCLUDE_NEAR_OBJECT = True
            self.assertFalse(executor.waypoint_requires_full_settle(wps, 1))
            self.assertTrue(executor.waypoint_requires_full_settle(wps, 2))
        finally:
            executor.CONTINUOUS_INCLUDE_NEAR_OBJECT = old

    def test_handoff_tolerance_is_adaptive_and_bounded(self):
        self.assertEqual(executor.continuous_handoff_tol(_q(0), _q(2)), 0.8)
        self.assertEqual(executor.continuous_handoff_tol(_q(0), _q(5)), 1.5)
        self.assertEqual(executor.continuous_handoff_tol(_q(0), _q(20)), 1.5)

    def test_sharp_joint_space_turn_becomes_hard_stop(self):
        waypoints = [
            {"seg": "HOME->READY", "q_sdk_deg": [0, 0, 0, 0, 0, 0, 0]},
            {"seg": "HOME->READY", "q_sdk_deg": [1, 0, 0, 0, 0, 0, 0]},
            {"seg": "HOME->READY", "q_sdk_deg": [1, 1, 0, 0, 0, 0, 0]},
        ]
        self.assertAlmostEqual(executor.waypoint_turn_deg(waypoints, 1), 90.0)
        self.assertTrue(executor.waypoint_requires_full_settle(waypoints, 1))

    def test_sdk_motion_state_and_route_clear_wrappers(self):
        class FakeSdk:
            def __init__(self, state):
                self.state = state
                self.cleared = None

            def armGetRobotMoveState(self, arm_id):
                return 0, self.state

            def armClearRobotRoute(self, arm_id, emergency_stop):
                self.cleared = (arm_id, emergency_stop)
                return 0

        # 真机 pypilot 返回 int 0/1(20260715 实测), 部分版本返回 bool —— 都要兼容
        self.assertTrue(executor.ss.read_move_state(FakeSdk(True), 1))
        self.assertFalse(executor.ss.read_move_state(FakeSdk(False), 1))
        self.assertTrue(executor.ss.read_move_state(FakeSdk(1), 1))
        self.assertFalse(executor.ss.read_move_state(FakeSdk(0), 1))
        with self.assertRaises(RuntimeError):
            executor.ss.read_move_state(FakeSdk(2), 1)
        sdk = FakeSdk(True)
        executor.ss.clear_robot_route(sdk, 1, emergency_stop=True)
        self.assertEqual(sdk.cleared, (1, True))

    def test_gripper_frame_builder_reproduces_legacy_and_new_force(self):
        # 旧 PDF 全部 4 条命令 = speed500/force100/open500, 必须逐字节复现
        self.assertEqual(executor.build_gripper_cmd(1, 0x10, [500, 100]),
                         "eb 90 01 05 10 f4 01 64 00 6f")
        self.assertEqual(executor.build_gripper_cmd(2, 0x10, [500, 100]),
                         "eb 90 02 05 10 f4 01 64 00 70")
        self.assertEqual(executor.build_gripper_cmd(1, 0x11, [500]),
                         "eb 90 01 03 11 f4 01 0a")
        self.assertEqual(executor.build_gripper_cmd(2, 0x11, [500]),
                         "eb 90 02 03 11 f4 01 0b")
        # 当前默认闭合命令(force=500): 校验和随数据变化
        self.assertEqual(executor.build_gripper_cmd(2, 0x10, [500, 500]),
                         "eb 90 02 05 10 f4 01 f4 01 01")

    def test_key_segment_confirmation_patterns(self):
        # 20260721: 确认点 = 夹爪闭合事件所在段(已下探到底待夹取) + 回起始点。
        # 下探段本身不确认 —— 那时还没到抓取位, 确认早了看不到实际夹取位置。
        cases = [("PICK1", True), ("PICK2", True), ("PICK", True),
                 ("READY->HOME", True),
                 ("PICK1_DESCEND", False), ("PICK2_DESCEND", False),
                 ("PICK1_ASCEND", False), ("READY->PICK1_HOVER", False),
                 ("PLACE1", False), ("PLACE1_DESCEND", False),
                 ("PLACE1_SETTLE", False), ("HOME->READY", False),
                 ("PICK1_HOVER->PLACE1_HOVER", False),
                 ("PLACE1_HOVER->PICK2_HOVER", False),
                 ("PLACE2_HOVER->READY", False)]
        for seg, need in cases:
            self.assertEqual(bool(executor.KEY_CONFIRM_RE.search(seg)), need,
                             msg=seg)

    def test_multi_pair_alternating_gripper_sequence_accepted(self):
        import json

        def make_traj(actions):
            wps = [{"seg": "HOME", "q_sdk_deg": [10, 10, 0, -15, 0, 0, 0]}]
            for k, act in enumerate(actions):
                wps.append({"seg": f"S{k}",
                            "q_sdk_deg": [10, 10, 0, -15, 0, 0, 0]})
                wps.append({"seg": f"S{k}", "gripper": act})
            data = {"meta": {"arm_id": 1, "j6_flipped": True},
                    "waypoints": wps}
            f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
            json.dump(data, f)
            f.close()
            return f.name

        good = make_traj(["close", "open", "close", "open"])
        bad = make_traj(["close", "open", "close"])
        try:
            meta, wps = executor.load_and_check(good)
            self.assertEqual(
                [w["gripper"] for w in wps if "gripper" in w],
                ["close", "open", "close", "open"])
            with self.assertRaises(SystemExit):
                executor.load_and_check(bad)
        finally:
            os.unlink(good)
            os.unlink(bad)

    def test_minimal_waypoint_step_gate_requires_both_meta_flags(self):
        """少点位大步长只在 meta 双重声明时放行; 缺任一声明仍按 6° 拒绝。"""
        import json

        def make(meta_extra):
            meta = {"arm_id": 1, "j6_flipped": True}
            meta.update(meta_extra)
            wps = [{"seg": "A", "q_sdk_deg": [0, 10, 0, -15, 0, 0, 0]},
                   {"seg": "B", "q_sdk_deg": [0, 32, 0, -15, 0, 0, 0]}]  # 步长22°
            f = tempfile.NamedTemporaryFile("w", suffix=".json", delete=False)
            json.dump({"meta": meta, "waypoints": wps}, f)
            f.close()
            return f.name

        both = make({"minimal_waypoints": True, "path_verified_dense": True})
        only_one = make({"minimal_waypoints": True})
        neither = make({})
        try:
            executor.load_and_check(both)              # 双重声明 -> 放行
            for path, label in ((only_one, "仅声明 minimal"), (neither, "无声明")):
                with self.assertRaises(SystemExit, msg=label):
                    executor.load_and_check(path)
        finally:
            for path in (both, only_one, neither):
                os.unlink(path)

    def test_declared_turn_limit_requires_verification_and_is_capped(self):
        """轨迹可声明更高的转角门限, 但必须同时声明 path_verified_dense,
        且不得超过绝对上限 —— 否则退回默认 20°。"""
        default = executor.CONTINUOUS_MAX_TURN_DEG
        cap = executor.MAX_DECLARED_TURN_DEG
        # 双重声明 -> 采用声明值
        self.assertEqual(executor.effective_turn_limit(
            {"handoff_turn_limit_deg": 51.7, "path_verified_dense": True}), 51.7)
        # 缺验证声明 -> 退回默认
        self.assertEqual(executor.effective_turn_limit(
            {"handoff_turn_limit_deg": 51.7}), default)
        # 无声明 -> 默认
        self.assertEqual(executor.effective_turn_limit({}), default)
        # 超上限 -> 封顶
        self.assertEqual(executor.effective_turn_limit(
            {"handoff_turn_limit_deg": 999, "path_verified_dense": True}), cap)

    def test_turn_limit_changes_stop_count(self):
        """同一条轨迹, 门限不同 -> 停顿次数不同(证明门限真的被用上了)。"""
        def wp(seg, a, b):
            return {"seg": seg, "q_sdk_deg": [a, b, 0, 0, 0, 0, 0]}
        # 中间点是 45° 转角: 20° 门限下必停, 50° 门限下可交接
        wps = [wp("A->B", 0, 0), wp("A->B", 10, 0), wp("A->B", 20, 10),
               wp("A->B", 20, 20)]
        self.assertTrue(executor.waypoint_requires_full_settle(wps, 1, 20.0))
        self.assertFalse(executor.waypoint_requires_full_settle(wps, 1, 50.0))

    def test_console_only_log_does_not_create_file(self):
        old_path = executor._LOG_PATH
        try:
            with tempfile.TemporaryDirectory() as tmp:
                would_be_log = os.path.join(tmp, "exec.log")
                executor._LOG_PATH = None
                executor.log("console only")
                self.assertFalse(os.path.exists(would_be_log))
        finally:
            executor._LOG_PATH = old_path


if __name__ == "__main__":
    unittest.main()
