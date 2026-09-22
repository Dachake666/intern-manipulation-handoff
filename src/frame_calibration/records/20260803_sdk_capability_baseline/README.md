# 20260803 SDK 接口能力基线（pypilot 1.0.0422.1）

`probe_sdk.py` 在真机容器里的实测输出，**这是以后写代码的依据**——手册
（V260526.1）比这个轮子新，按手册写等于按一份还没实现的规格写。

## 结论：只有 4 个真缺

```
armUnsetLoad              本机没有
armGetPosReFreshMS        本机没有
armSetPosReFreshMS        本机没有
armRobotSoftEmergencyStop 本机没有
```

其余「失败」全是**我签名写错**，不是接口缺失。

## 签名对账（我写的 → 真机实际）

| 接口 | 真机签名 | 我原来 | 处理 |
|---|---|---|---|
| `armTryWorlds` | `(int, DoubleVector)` | FloatVector | ✅ 已修 |
| `armGetAxisParameter` | `(int, int, **AxisParameterIndex**)` | 传 int `1/2` | ✅ 已修，用枚举 |
| `armSetRobotLoad` | `(int, **int**) -> tuple` | 传 `LoadParameter` | ⚠ 整数语义未知，**默认关闭** |
| `armSetAxisParameter` | `(int, int, AxisParameterIndex, float)` | 未用 | — |
| `armPluseToServo` / `armWorldsToServo` | `(int, FloatVector)` | 一致 | ✅ |
| `armGetGlobalSpeed` | `() -> tuple` | 一致 | ✅ 速度还原可用 |
| `armGetRobotProtectStatus` | `(int) -> tuple` | 一致 | ✅ 回读可用 |
| `armGetSpeedFeedback` / `armGetRobotPaused` / `armGetRobotAxisMoveState` / `armJointOutLimit` / `armSetEnableTolerance` | 与手册一致 | 一致 | ✅ |

**注意厂商 API 自己不统一**：`armTryWorlds` 收 `DoubleVector`，
`armPluseToServo`/`armWorldsToServo`/`armMoveWorlds` 收 `FloatVector`。不能全局换。

## 顺带发现：52 个手册没登记的接口

其中这几个值得后续查：

| 接口 | 为什么值得看 |
|---|---|
| `armSetLoadParam` / `armGetLoadParam` | 很可能才是 `LoadParameter` 版的负载接口——手册把它写成 `armSetRobotLoad` 了 |
| `armSetMoveJointsEndCallback` / `armSetMoveWorldsEndCallback` | **运动结束回调**。现在的「停稳」靠轮询，有回调就不用猜 |
| `armReadCom` | 读串口——夹爪目前是只写不读，有它就能确认夹爪真实状态 |
| `armGetPulseFeedback` | 脉冲级反馈 |
| `armSwitchRunMode` / `armGetCurrentRunMode` | 运行模式切换，可能与透传相关 |

## 已按这份基线改的代码

- `read_axis_limits()` 改用 `pypilot.AxisParameterIndex` 枚举（按名字取
  positive/negative，取不到就按顺序取前两个并打印提醒人工核对）
- **删掉** `set_pos_refresh_ms` / `read_pos_refresh_ms` 封装和两个执行器里的相关代码
  ——留着只会让调用方以为可用。含义：**反馈固定 关节 40ms / 世界 50ms，改不了**，
  任何比这更密的采样只会读到重复值
- `set_robot_load()` 改成 `(arm_id, int)` 签名，但 `USE_PAYLOAD` **默认 False**，
  因为那个整数代表什么（kg？克？档位？）未知，传错会让动力学补偿更离谱
- 两个执行器的 `OPTIONAL_CAPS` 按实测重列

## 什么时候要重跑

**只在换 SDK 轮子时。** 平时加新接口先查这份基线表，查不到再探。
