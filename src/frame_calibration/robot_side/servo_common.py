#!/usr/bin/env python3
"""伺服透传的公共件: 节拍器 + 跟随误差监控 + 收尾报告。

pulse(关节透传) 和 worlds(末端透传) 两条线共用这里的东西, 免得两边各写一份然后
各自漂移。本文件不碰任何运动指令, 只负责【控节拍】和【看数据】。

20260731 新增。动机来自真机现场那句"运行的还行, 偶尔卡顿":
  * 原来的节拍循环用 time.sleep(max(0.0, t - now)), 那个 max 会【静默吞掉】
    "这一帧已经超时了"的情况 —— 跟不上从日志里完全看不出来;
  * 原来全程开环, 只在段末查一次误差, 中途伺服跟没跟上没有任何信号。
现在两件事都被量出来: 卡顿要么是节拍没守住(本地问题), 要么是跟随误差发散
(机械臂侧问题), 跑一次就能分清。
"""
from __future__ import annotations

import math
import time

import sdk_session as ss


class StrictPacer:
    """[20260909] 新 Servo 候选专用：单调时钟、绝不追帧、长迟到停住。

    不修改冻结 Track C 的 Pacer 行为。before_send 在 SDK 发送前调用；
    每次发送至少相隔 period，迟到超过两个周期拒绝下一帧。
    """

    def __init__(self, period_s, clock=None, sleep=None):
        self.period = float(period_s)
        if not math.isfinite(self.period) or self.period <= 0:
            raise ValueError("Servo period 必须是有限正数")
        self.clock = clock or time.monotonic
        self.sleep = sleep or time.sleep
        self.frames = 0
        self.max_late_s = 0.0
        self.due = None

    def reset(self):
        self.due = None

    def before_send(self):
        now = self.clock()
        if self.due is not None:
            if now < self.due:
                self.sleep(self.due-now)
                now = self.clock()
            late = max(0.0, now-self.due)
            self.max_late_s = max(self.max_late_s, late)
            if late > 2*self.period:
                raise RuntimeError(f"Servo 下发迟到 {late*1000:.1f}ms，停止，不补发积压帧")
        self.frames += 1
        self.due = now+self.period

    def report(self):
        return {"send_slots": self.frames, "period_s": self.period,
                "max_late_ms": self.max_late_s*1000, "catchup_bursts_allowed": False}


class Pacer:
    """固定周期节拍器 —— 维护绝对时间轴, 并统计"有没有守住节拍"。

    为什么是绝对时间轴而不是 sleep(period): sleep(period) 的真实周期是
    "SDK 调用耗时 + period", 误差【逐帧累积】(实测 5ms 调用 + 20ms 周期 ->
    8 帧就偏了 78ms, 整条轨迹慢 49%)。绝对时间轴把调用耗时吸收进 sleep,
    偏差恒定不累积。

    统计的意义: late_frames 就是"我们跟不上自己设的节拍"的次数。它 > 0 说明
    单帧 SDK 往返已经比 period 还长, 这时候的卡顿是【本地节拍问题】, 调
    STEP_DEG 没用, 要放长 period 或减少每帧的 SDK 调用次数。"""

    def __init__(self, period_s, max_catchup_frames=2):
        """max_catchup_frames: 落后超过这么多个周期就【放弃追赶】, 重新对齐。

        ⚠ 20260804 复查指出的真实风险: 绝对时间轴天生会追赶 —— 这在吸收几毫秒
        抖动时正是我们要的, 但如果 SDK 某一帧阻塞了 200ms, 接下来 10 帧算出来的
        sleep 全是负数, 会【背靠背连发】10 个点。关保护状态下这等于步长瞬间放大
        10 倍。所以追赶要有上限: 落后太多就认了, 从现在重新起算。"""
        self.period = float(period_s)
        self.max_catchup = max(1, int(max_catchup_frames))
        self.realigns = 0
        self.frames = 0
        self.late_frames = 0
        self.max_late_s = 0.0
        self.total_late_s = 0.0
        # 只统计【流内】耗时。夹爪等待、停稳轮询这些故意的暂停必须排除, 否则
        # 每次运行都会凭空多出几秒"漂移"(实测干跑里 625 帧报 +4.87s 漂移, 其实
        # 全是两次夹爪的等待时间), 真正的节拍问题反而被埋掉。
        self.stream_s = 0.0
        self.stream_frames = 0
        self._seg_t0 = None
        self._seg_last = None
        self._seg_frames = 0
        self.t = time.time()

    def _close_segment(self):
        # 段长必须算到【最后一帧结束的那一刻】, 不能算到 reset() 被调用的时刻 ——
        # reset() 通常是在夹爪等待【之后】才调的, 用 time.time() 会把那几秒算进来
        # (实测会让一次 0.3s 暂停凭空变成 0.346s 漂移)。
        if self._seg_t0 is not None and self._seg_frames and self._seg_last:
            self.stream_s += self._seg_last - self._seg_t0
            self.stream_frames += self._seg_frames
        self._seg_t0 = None
        self._seg_last = None
        self._seg_frames = 0

    def reset(self):
        """把时间轴对齐到"现在", 并结束当前统计段。

        故意的暂停(夹爪等待)之后必须调, 否则节拍器会试图把那几秒【追回来】——
        5s 暂停 / 20ms 周期 = 250 帧瞬间全发出去。"""
        self._close_segment()
        self.t = time.time()

    def tick(self):
        """推进一帧并睡到该睡的时刻。返回本帧实际 sleep 秒数(0 表示已超时)。"""
        if self._seg_t0 is None:
            self._seg_t0 = self.t        # reset() 对齐的那个时刻 = 本段起点(精确)
        self.frames += 1
        self._seg_frames += 1
        self.t += self.period
        gap = self.t - time.time()
        if gap > 0:
            time.sleep(gap)
            self._seg_last = time.time()
            return gap
        # gap <= 0: 这一帧已经落后于节拍
        self.late_frames += 1
        self.total_late_s += -gap
        self.max_late_s = max(self.max_late_s, -gap)
        if -gap > self.max_catchup * self.period:
            # 落后太多 -> 放弃把积压帧补发出去, 直接对齐到现在。
            # 宁可这一段整体慢一点, 也不能让伺服吃到一串背靠背的点。
            self.t = time.time()
            self.realigns += 1
        self._seg_last = time.time()
        return 0.0

    def report(self):
        self._close_segment()
        want = self.stream_frames * self.period
        return {
            "frames": self.frames,
            "elapsed_s": self.stream_s,
            "expected_s": want,
            "drift_s": self.stream_s - want,
            "late_frames": self.late_frames,
            "late_ratio": (self.late_frames / self.frames) if self.frames else 0.0,
            "max_late_ms": self.max_late_s * 1000.0,
            "mean_late_ms": ((self.total_late_s / self.late_frames * 1000.0)
                             if self.late_frames else 0.0),
            "realigns": self.realigns,
        }

    def print_report(self, tag=""):
        r = self.report()
        print(f"\n[节拍统计]{(' ' + tag) if tag else ''}")
        print(f"  帧数 {r['frames']}  实际 {r['elapsed_s']:.2f}s  "
              f"预计 {r['expected_s']:.2f}s  偏差 {r['drift_s']:+.2f}s")
        if r["late_frames"]:
            print(f"  ⚠ 跟不上的帧: {r['late_frames']} 帧 "
                  f"({r['late_ratio']*100:.1f}%)  最大超时 {r['max_late_ms']:.1f}ms  "
                  f"平均超时 {r['mean_late_ms']:.1f}ms")
            print(f"    -> 单帧 SDK 往返已超过周期 {self.period*1000:.0f}ms。"
                  f"卡顿在【本地节拍】, 调步长没用: 放长周期, 或减少每帧的 SDK 调用。")
            if r["realigns"]:
                print(f"  ⚠ 放弃追赶并重新对齐 {r['realigns']} 次 "
                      f"(落后超过 {self.max_catchup} 个周期时触发) —— "
                      f"这是保护措施: 宁可慢一点, 也不让积压帧背靠背连发。")
        else:
            print(f"  ✓ 无超时帧, 节拍守住了 (周期 {self.period*1000:.0f}ms)")
        return r


class FollowMonitor:
    """跟随误差监控 —— 透传是开环的, 这是唯一能看见"伺服跟没跟上"的手段。

    ⚠ 20260731 真机复盘后【默认关闭】。教训:
    第一版在热循环里每 10 帧调 read_follow_error, 而它内部是【两次】SDK 往返
    (fb=False 取指令值 + fb=True 取反馈值)。20ms 的预算里凭空多两次网络往返 ——
    实测 625 帧里 8~34 帧迟到, 最大超时 17~182ms, 而采样正好是 5Hz, 于是手臂出现
    5Hz 的周期性微顿。总时长仍是 12.50s(节拍总体守住), 但观感明显不如未优化的
    v1。结论: 【任何阻塞式 SDK 读取都不该进透传热循环】。

    现在的做法:
      * 默认 every=0 = 彻底关闭, 热循环一次额外调用都没有;
      * 打开时只做【一次】读: 拿我们【刚发出去的那个点】当指令值, 只读一次反馈。
        既省一半往返, 量的还是完整链路(我们->控制器->伺服), 比原来的
        "控制器目标->伺服"更有意义;
      * 即使这样也仍是阻塞读 —— 只在诊断时开, 不要在正式跑时开。
        真要常驻遥测, 正解是独立线程或控制器订阅, 不占发送节拍。"""

    def __init__(self, arm_id, every_n_frames=0, warn_deg=3.0):
        self.arm_id = arm_id
        # every<=0 表示【关闭】。原来写 max(1, ...) 是个坑: 设 0 反而变成每帧都采,
        # 想关反而更糟。复查指出后改掉。
        self.every = int(every_n_frames) if int(every_n_frames) > 0 else 0
        self.enabled = self.every > 0
        self.warn_deg = float(warn_deg)
        self.samples = []          # [(相对时刻, 最大跟随误差, 逐关节误差, 标签)]
        self.n = 0
        self.t0 = None
        self.warned = False

    def maybe_sample(self, sdk, commanded=None, label=""):
        """每 every 帧采一次; 关闭时【立即返回】, 不产生任何 SDK 调用。

        commanded = 我们刚发出去的那个点。给了就用它当指令值(只需一次读);
        没给才退回 read_follow_error(两次读, 不推荐在热循环里用)。"""
        if not self.enabled:
            return None
        self.n += 1
        if self.n % self.every:
            return None
        if self.t0 is None:
            self.t0 = time.time()
        try:
            if commanded is not None:
                act = ss.read_joints(sdk, self.arm_id, fb=True)   # 只读一次
                err = [c - a for c, a in zip(commanded, act)]
                emax = max(abs(v) for v in err)
            else:
                err, emax, _cmd, _act = ss.read_follow_error(sdk, self.arm_id)
        except Exception:                     # noqa: BLE001 采样失败不该打断运动
            return None
        self.samples.append((time.time() - self.t0, emax, err, label))
        if emax > self.warn_deg and not self.warned:
            self.warned = True
            worst = max(range(7), key=lambda k: abs(err[k]))
            print(f"\n  ⚠ 跟随误差 {emax:.2f}° 超过 {self.warn_deg}° "
                  f"(j{worst+1} 最大), 伺服开始跟不上{(' @ ' + label) if label else ''}")
        return emax

    def print_report(self):
        if not self.enabled:
            print("\n[跟随误差] 已关闭(FOLLOW_SAMPLE_EVERY=0) —— 热循环零额外 SDK 调用。"
                  "\n  要诊断跟随情况再打开, 但注意它是阻塞读, 会挤占节拍预算。")
            return None
        if not self.samples:
            print("\n[跟随误差] 无采样")
            return None
        emaxes = [s[1] for s in self.samples]
        peak_i = max(range(len(self.samples)), key=lambda i: emaxes[i])
        t, emax, err, label = self.samples[peak_i]
        print(f"\n[跟随误差] 采样 {len(self.samples)} 次 "
              f"(每 {self.every} 帧一次)")
        print(f"  平均 {sum(emaxes)/len(emaxes):.3f}°   最大 {emax:.3f}° "
              f"@ t={t:.1f}s{(' ' + label) if label else ''}")
        print(f"  峰值处逐关节: "
              f"{' '.join(f'j{k+1}:{v:+.2f}' for k, v in enumerate(err))}")
        # 误差随时间的粗略曲线(每 1/8 段取一个)
        step = max(1, len(self.samples) // 8)
        pts = " ".join(f"{s[0]:.0f}s:{s[1]:.2f}" for s in self.samples[::step])
        print(f"  随时间: {pts}")
        if emax > self.warn_deg:
            print(f"  -> 峰值超过 {self.warn_deg}°: 伺服跟不上。"
                  f"减小 STEP_DEG(降速)、或确认负载已用 armSetRobotLoad 告知控制器、"
                  f"或检查过热降载。")
        else:
            print(f"  ✓ 全程跟随良好 (< {self.warn_deg}°)")
        return {"n": len(self.samples), "peak_deg": emax,
                "mean_deg": sum(emaxes) / len(emaxes)}


def describe_axis_limits(real, fallback, adopted=False):
    """把控制器读回的真实轴极限与本地硬编码表逐轴对比并打印差异。

    adopted 必须如实反映【是否真的采纳了控制器值】—— 原来无论如何都打印
    "后续全部改用控制器真值", 但 mode='report' 下根本没改, 这句是假的
    (20260803 复查发现)。谁看日志谁被误导, 必须由调用方传真话。"""
    print("\n[轴极限] 控制器真值 vs 本地表:")
    diff, looser, tighter = 0, [], []
    for k, ((lo, hi), (flo, fhi)) in enumerate(zip(real, fallback), start=1):
        same = abs(lo - flo) < 1e-6 and abs(hi - fhi) < 1e-6
        if not same:
            diff += 1
            if lo < flo - 1e-6 or hi > fhi + 1e-6:
                looser.append(k)
            if lo > flo + 1e-6 or hi < fhi - 1e-6:
                tighter.append(k)
        print(f"  j{k}: 控制器 [{lo:+7.1f}, {hi:+7.1f}]   "
              f"本地 [{flo:+7.1f}, {fhi:+7.1f}]  {'✓' if same else '≠'}")
    if diff:
        if looser:
            print(f"  控制器比本地【宽】的轴: {looser} "
                  f"-> 本地表在这些轴上会造成【假超限】(明明能到却被拒)")
        if tighter:
            print(f"  控制器比本地【紧】的轴: {tighter} "
                  f"-> 本地表在这些轴上过于乐观, 可能放行实际会撞限位的点")
        print(f"  => 判定用: {'控制器真值 ✓' if adopted else '本地表(mode=report, 仅对比不采纳)'}")
    return diff
