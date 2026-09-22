# Track A · 相机绝对坐标两-hover桌上抓放(armMoveWorlds)

**状态**: OFFLINE_CANDIDATE_BLOCKED / 仅允许无运动预检。当前 A1/B1 feature→抓点、瓶子尺寸、料框 datum/内尺寸/口沿及框内落点仍含显式仿真假设；补齐输入并重生成后，仍需 live armTryWorlds 全路径预检与人工确认。

**全新部署**用这个目录：整个拷过去就能跑，不用再从别处补文件。
**已经跑过的机器**只需要补送变化的那几个文件，见下面「增量更新」。

## 运行

```bash
XIFENG_ROBOT_IP=<机器人IP> XIFENG_LOCAL_IP=<本机IP> XIFENG_ARM_IP=<机械臂IP> \
  python3 execute_tabletop_pick_place_worlds.py tabletop_pick_place_worlds.json --precheck-only
```

当前计划状态是 `OFFLINE_CANDIDATE_BLOCKED`，执行器只允许
`--precheck-only`；它会连接 SDK 做只读的密集 `armTryWorlds`、
控制器限位和 IK 连续性检查，但不会清报警、使能、开夹爪或下发运动。

以后输入门全部确认并在 Mac 重新生成无 blocker 计划后，真跑仍需
同时给 `--run` 与 `XIFENG_ALLOW_REAL_MOTION=1`，并在预检通过后
由现场人员再次回车确认。IP 一律在运行时传入，不使用历史 `.147/.148`。

## 文件

运行必需（少一个就跑不起来）：

- `execute_tabletop_pick_place_worlds.py`  — 两 hover 桌上抓放执行器；默认只做 live armTryWorlds 预检
- `robot_lock.py`  — 机器人互斥锁 —— 四个执行器都 import, 必需; 同一台臂同时只允许一个进程
- `sdk_session.py`  — SDK 会话/运动封装 —— 执行器 import 它, 必需
- `tabletop_pick_place_worlds.json`  — Mac 端已完成 TCP 补偿的绝对 SDK endpoint 计划

另外还依赖 `pypilot`（厂商 SDK 轮子），那是装在真机容器里的，
不在封装内。

## 增量更新

真机上已经有这一套的话，按 `SHA256SUMS` 只补送哈希变化的文件。
`sdk_session.py` 只在它自己被改过时才需要重送；不要再复制一套新目录。

## 校验

```bash
sha256sum -c SHA256SUMS
```

或在工作区根目录跑 `python3 releases/make_release.py --verify`。

## 重新生成

```bash
python3 releases/make_release.py A
```

封装是从工作区当前代码复制出来的快照，**不要直接改封装里的文件** —— 
改源文件再重新生成，否则下次生成会把你的改动覆盖掉。
