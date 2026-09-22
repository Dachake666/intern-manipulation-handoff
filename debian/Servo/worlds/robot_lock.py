#!/usr/bin/env python3
"""机器人级互斥锁 —— 同一台机械臂同时只允许一个脚本操作。

为什么必须有(20260731 复查指出):
  代码可以按目录隔离(新目录里的脚本只 import 新目录里的模块), 但【机器人只有
  一台】。两个进程同时跑会互相覆盖:
      保护状态   —— A 关保护做透传, B 收尾时把它开了 -> A 的密集流被误判触发保护
      全局速度   —— 互相改, 谁都不知道当前是多少
      伺服指令   —— 两条流同时喂给同一个伺服, 行为完全不可预期
  在关保护的透传场景下, 这是能直接把手臂甩出去的事故路径。
  "两个目录不能同时运行"过去只是口头约定, 没有任何东西拦着 —— 现在拦。

实现: 按 (机器人IP, 臂号) 命名一个锁文件, 用 fcntl.flock 排他非阻塞加锁。
  * 进程活着 -> 锁一直持有;
  * 进程退出/被 kill/崩溃 -> 内核自动释放, 不会留下死锁(这是 flock 相比
    "写 pidfile 自己删"的关键优势: 崩溃场景不需要人工清理);
  * 锁文件里写 pid/脚本名/时间, 抢不到时能告诉你是谁在占。

用法:
    with robot_lock.acquire("192.168.8.148", 1, "execute_servo_grasp.py"):
        ...  # 这段期间独占该机械臂
"""
from __future__ import annotations

import contextlib
import errno
import json
import os
import time

try:
    import fcntl
except ImportError:                      # Windows 没有 fcntl
    fcntl = None

LOCK_DIR = os.environ.get("XIFENG_LOCK_DIR", "/tmp/xifeng_robot_locks")


def lock_path(robot_ip, arm_id):
    safe = str(robot_ip).replace(":", "_").replace("/", "_")
    return os.path.join(LOCK_DIR, f"arm_{safe}_{arm_id}.lock")


@contextlib.contextmanager
def acquire(robot_ip, arm_id, owner="", required=True):
    """独占这台机械臂。抢不到就抛 SystemExit 并说明是谁在占。

    required=False 时, 平台不支持 flock 只警告不阻断(便于在 Mac 上跑离线测试)。"""
    path = lock_path(robot_ip, arm_id)
    if fcntl is None:
        msg = f"⚠ 本平台没有 fcntl, 无法加机器人互斥锁 ({path})"
        if required:
            raise SystemExit(msg + " —— 拒绝在无锁保护下操作真机")
        print(msg + " —— 仅离线测试可继续")
        yield None
        return

    os.makedirs(LOCK_DIR, exist_ok=True)
    fh = open(path, "a+")
    try:
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            if exc.errno not in (errno.EACCES, errno.EAGAIN):
                raise
            fh.seek(0)
            try:
                held = json.loads(fh.read() or "{}")
            except Exception:                    # noqa: BLE001
                held = {}
            raise SystemExit(
                f"✗ 机械臂 {robot_ip} #{arm_id} 正被另一个进程占用, 拒绝启动。\n"
                f"    占用者: pid={held.get('pid','?')} "
                f"{held.get('owner','?')} 起于 {held.get('since','?')}\n"
                f"    锁文件: {path}\n"
                f"  同时操作同一台机械臂会互相覆盖保护状态/全局速度/伺服指令 ——\n"
                f"  在关保护的透传场景下这是能把手臂甩出去的事故路径。\n"
                f"  等对方结束, 或确认那个进程已经死了再重试(进程退出锁会自动释放)。")
        fh.seek(0)
        fh.truncate()
        fh.write(json.dumps({"pid": os.getpid(), "owner": owner,
                             "since": time.strftime("%Y-%m-%d %H:%M:%S")},
                            ensure_ascii=False))
        fh.flush()
        print(f"[互斥锁] 已独占 {robot_ip} #{arm_id} (pid={os.getpid()})")
        yield fh
    finally:
        try:
            fcntl.flock(fh.fileno(), fcntl.LOCK_UN)
        except Exception:                        # noqa: BLE001
            pass
        fh.close()
