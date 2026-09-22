#!/usr/bin/env python3
"""透传相关新增逻辑的离线回归测试(不连机器人)。

20260731 复查指出: 这些逻辑当时都只用一次性脚本验过, 没进测试套件 —— 改一次就
可能悄悄退化。这里把它们钉死。

覆盖:
  1. FollowMonitor every<=0 -> 零 SDK 读取(上次的坑: max(1,...) 把 0 变成 1)
  2. FollowMonitor 给了 commanded -> 每次采样只读一次
  3. Pacer 把故意的暂停排除在漂移之外
  4. Pacer 真跟不上时如实计数
  5. make_double_vector 用 DoubleVector, make_float_vector 用 FloatVector
  6. 机器人互斥锁: 第二个进程被拒 / 释放后可重入
  7. open_session(enable=False) 只读会话: 不清报警不使能不设速度
  8. close_session 还原全局速度
  9. 关节速度校验拦下"位置连续但速度过快"
 10. finally 里不 return —— 初始化异常必须能传播出来

运行: python3 test_servo_safety.py
"""
from __future__ import annotations

import io
import os
import subprocess
import sys
import time
import types
import unittest
import contextlib

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)


def _optional(mod):
    """按包部署时两条线各自只有自己的执行器 —— 缺了就跳过对应用例, 不算失败。"""
    try:
        return __import__(mod)
    except ImportError:
        return None


def _find_traj(name):
    """在几个可能的位置找轨迹: 同目录(按包部署) 或 仓库的 trajectories/。"""
    for c in (os.path.join(_HERE, name),
              os.path.normpath(os.path.join(_HERE, "..", "..",
                                            "pick_place_coord", "trajectories",
                                            name))):
        if os.path.exists(c):
            return c
    return None


def _stub_pypilot():
    """装一个假 pypilot, 记录 FloatVector / DoubleVector 各被用了几次。"""
    m = types.ModuleType("pypilot")

    class FloatVector(list):
        def append(self, v): list.append(self, float(v))

    class DoubleVector(list):
        def append(self, v): list.append(self, float(v))

    m.FloatVector = FloatVector
    m.DoubleVector = DoubleVector
    sys.modules["pypilot"] = m
    return m


_stub_pypilot()
import robot_lock                                    # noqa: E402
import sdk_session as ss                             # noqa: E402
import servo_common as sc                            # noqa: E402

# unittest discover 可能先导入其他测试留下功能不全的 pypilot 壳；sdk_session 若已被
# 缓存，也仍指向旧壳。显式绑定本文件的完整假模块，保证测试顺序不影响结果。
ss.pypilot = sys.modules["pypilot"]


class FakeJointSDK:
    def __init__(self):
        self.reads = []
    def armGetJoints(self, arm, fb=True):
        self.reads.append(fb)
        return (0, [0.0] * 7)


class TestFollowMonitor(unittest.TestCase):
    def test_disabled_does_zero_sdk_reads(self):
        """every<=0 必须彻底关闭。曾经写成 max(1,...), 设 0 反而每帧都采。"""
        for every in (0, -1):
            sdk = FakeJointSDK()
            fm = sc.FollowMonitor(1, every)
            self.assertFalse(fm.enabled)
            for _ in range(50):
                self.assertIsNone(fm.maybe_sample(sdk, commanded=[0.1] * 7))
            self.assertEqual(sdk.reads, [], f"every={every} 竟然读了 SDK")

    def test_default_is_disabled(self):
        """默认必须是关的 —— 阻塞读进热循环会造成 5Hz 微顿(20260731 真机实证)。"""
        self.assertFalse(sc.FollowMonitor(1).enabled)

    def test_commanded_costs_one_read(self):
        """给了 commanded 就只读一次反馈, 不再额外读指令值。"""
        sdk = FakeJointSDK()
        fm = sc.FollowMonitor(1, 5)
        for _ in range(20):
            fm.maybe_sample(sdk, commanded=[0.1] * 7)
        self.assertEqual(len(fm.samples), 4)
        self.assertEqual(len(sdk.reads), 4, "每次采样应只有 1 次读")
        self.assertTrue(all(fb is True for fb in sdk.reads))

    def test_without_commanded_costs_two_reads(self):
        """不给 commanded 才退回两次读(worlds 通道), 明确记录这个代价。"""
        sdk = FakeJointSDK()
        fm = sc.FollowMonitor(1, 5)
        for _ in range(10):
            fm.maybe_sample(sdk)
        self.assertEqual(len(sdk.reads), 4)


class TestPacer(unittest.TestCase):
    def test_pause_excluded_from_drift(self):
        """夹爪等待不能被算成漂移(曾经 625 帧凭空报 +4.87s)。"""
        p = sc.Pacer(0.02)
        p.reset()
        for _ in range(10):
            p.tick()
        time.sleep(0.3)
        p.reset()
        for _ in range(10):
            p.tick()
        r = p.report()
        self.assertEqual(r["frames"], 20)
        self.assertLess(abs(r["drift_s"]), 0.10,
                        f"0.3s 暂停被算进漂移了: {r['drift_s']:.3f}s")

    def test_late_frames_counted(self):
        """真跟不上时必须如实计数, 不能被 max(0.0, ...) 静默吞掉。"""
        p = sc.Pacer(0.001)
        p.reset()
        for _ in range(20):
            time.sleep(0.004)
            p.tick()
        r = p.report()
        self.assertEqual(r["late_frames"], 20)
        self.assertGreater(r["max_late_ms"], 0.0)


class TestVectorTypes(unittest.TestCase):
    def test_double_vs_float(self):
        """armTryWorlds 要 DoubleVector, ToServo 要 FloatVector —— 厂商不统一。"""
        import pypilot
        self.assertIsInstance(ss.make_double_vector([1, 2, 3]), pypilot.DoubleVector)
        self.assertIsInstance(ss.make_float_vector([1, 2, 3]), pypilot.FloatVector)

    def test_try_worlds_passes_double(self):
        import pypilot
        seen = {}

        class SDK:
            def armTryWorlds(self, arm, vec):
                seen["type"] = type(vec)
                return (0, [0.0] * 7)
        ss.try_worlds(SDK(), 1, [1, 2, 3, 4, 5, 6])
        self.assertIs(seen["type"], pypilot.DoubleVector)


class TestRobotLock(unittest.TestCase):
    def test_second_process_refused_and_released(self):
        """同一台机械臂同时只能一个进程; 持有者退出后必须能重入。"""
        code = ("import sys,time; sys.path.insert(0,%r);"
                "import robot_lock as rl\n"
                "with rl.acquire('1.2.3.4', 9, 'holder'): time.sleep(3)" % _HERE)
        holder = subprocess.Popen([sys.executable, "-c", code],
                                  stdout=subprocess.DEVNULL)
        try:
            time.sleep(1.0)
            with self.assertRaises(SystemExit):
                with robot_lock.acquire("1.2.3.4", 9, "second"):
                    pass
        finally:
            holder.wait()
        with contextlib.redirect_stdout(io.StringIO()):
            with robot_lock.acquire("1.2.3.4", 9, "third"):
                pass


class TestSession(unittest.TestCase):
    def _sdk(self, log):
        class SDK:
            speed = 50.0
            def __init__(self, ip): pass
            def start(self): return True
            def getSoftStopSwitch(self): return (0, "CLOSE")
            def armInitData(self, *a): return 0
            def armGetLinkStatus(self): return (0, True)
            def armGetWorlds(self, a, fb=True):     # 只读会话的就绪探测要用
                return (0, [0.0] * 6)
            def armClearAlarm(self): log.append("清报警"); return 0
            def armRobotEnableOrNot(self, x): log.append("使能"); return 0
            def armGetRobotEnableStatus(self): return (0, True)
            def armServoIsOpOrNot(self): return (0, True)
            def armGetGlobalSpeed(self): return (0, SDK.speed)
            def armSetGlobalSpeed(self, v):
                SDK.speed = v; log.append(f"设速度{v}"); return 0
            def armGetSingleRobotEnableStatus(self, a): return (0, True)
            def stop(self): pass
        return SDK

    def test_readonly_not_ready_is_refused(self):
        """只读会话读不到数据时必须【明确报错】, 不能带着空数据往下跑。
        20260803 真机实测: link 一通就返回的话, 轴极限会读到全 0、
        armGetWorlds 返回 -1, 那种状态下做逆解预检毫无意义。"""
        log = []
        import pypilot
        SDK = self._sdk(log)
        SDK.armGetWorlds = lambda self, a, fb=True: (-1, [])   # 模拟数据未就绪
        pypilot.PilotSDK = SDK
        with self.assertRaises(RuntimeError):
            with contextlib.redirect_stdout(io.StringIO()):
                ss.open_session("ip", "l", "a", 1, 3.0, (1,), enable=False)

    def test_readonly_session_touches_nothing(self):
        """--precheck-only 用的只读会话: 不清报警/不使能/不设速度。"""
        log = []
        import pypilot
        pypilot.PilotSDK = self._sdk(log)
        with contextlib.redirect_stdout(io.StringIO()):
            ss.open_session("ip", "l", "a", 1, 3.0, (1,), enable=False)
        self.assertEqual(log, [], f"只读会话不该有副作用, 实际: {log}")

    def test_global_speed_restored(self):
        """全局速度是控制器上的持久设置, 收尾必须还原。"""
        log = []
        import pypilot
        SDK = self._sdk(log)
        pypilot.PilotSDK = SDK
        with contextlib.redirect_stdout(io.StringIO()):
            sdk = ss.open_session("ip", "l", "a", 1, 3.0, (1,))
            self.assertEqual(SDK.speed, 3.0)
            ss.close_session(sdk)
        self.assertEqual(SDK.speed, 50.0, "全局速度没还原")


class TestWorldsExecutor(unittest.TestCase):
    def test_joint_speed_guard(self):
        """位置连续 != 速度可行: 每步 4°(跳变门限内) / 20ms = 200°/s 必须拦下。"""
        wsv = _optional("execute_worlds_servo_grasp")
        if wsv is None:
            self.skipTest("本目录没部署 worlds 执行器(pulse 包不含它)")
        js = [[0] * 7, [4, 0, 0, 0, 0, 0, 0], [8, 0, 0, 0, 0, 0, 0]]
        with self.assertRaises(SystemExit):
            wsv.check_joint_speed(js, 0.02, 60.0)
        self.assertAlmostEqual(wsv.check_joint_speed(js, 0.02, 300.0), 200.0, 1)

    def test_exec_via_defaults_to_joints(self):
        """默认必须走"执行预检算出的那条关节序列", 而不是让控制器重新逆解。"""
        wsv = _optional("execute_worlds_servo_grasp")
        if wsv is None:
            self.skipTest("本目录没部署 worlds 执行器(pulse 包不含它)")
        self.assertEqual(wsv.EXEC_VIA, "joints")
        self.assertTrue(wsv.IK_PRECHECK)


class TestCleanupOnInitFailure(unittest.TestCase):
    def test_exception_propagates(self):
        """finally 里不能 return —— 否则初始化失败的原因会被吞掉。"""
        import execute_servo_grasp as ex
        orig_open, orig_enable = ss.open_session, ex.ENABLE_REAL_MOTION
        ex.ENABLE_REAL_MOTION = True
        ss.open_session = lambda *a, **k: (_ for _ in ()).throw(
            RuntimeError("使能失败(模拟)"))
        try:
            traj = _find_traj("traj_minimal_joint.json")
            if traj is None:
                self.skipTest("找不到 traj_minimal_joint.json")
            buf = io.StringIO()
            with self.assertRaises(RuntimeError):
                with contextlib.redirect_stdout(buf):
                    ex.main([traj, "--run", "--robot-ip", "ip",
                             "--local-ip", "local", "--arm-ip", "arm"])
            self.assertIn("会话未建立", buf.getvalue())
        finally:
            ss.open_session, ex.ENABLE_REAL_MOTION = orig_open, orig_enable


if __name__ == "__main__":
    unittest.main(verbosity=2)
