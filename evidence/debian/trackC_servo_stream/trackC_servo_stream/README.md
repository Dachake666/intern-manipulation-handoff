# Track C · 关节伺服透传(armPluseToServo 密集流式)

**状态**: 左臂真机已验收 —— 20260804 双抓放(traj_multi_latest.json, 133 路点/4 次夹爪)连跑 5 轮全过, 推荐基线 0.4°/20ms: 32.7s, 终点误差 0.048°, 迟到帧 1.4%。轨迹优化空间见 records/20260804_pulse_multi_grasp_verified/。右臂 traj_multi_right_latest.json 为左臂轨迹的镜像重定向, 仿真通过(残差 0.99mm, 余量剖面与左臂逐项相同), 真机未跑 —— 上机前先量右臂 j7 物理停止位, 见 pick_place_coord/RIGHT_ARM.md

**全新部署**用这个目录：整个拷过去就能跑，不用再从别处补文件。
**已经跑过的机器**只需要补送变化的那几个文件，见下面「增量更新」。

## 运行

```bash
python3 execute_servo_grasp.py traj_multi_latest.json --step-deg 0.4 --speed 15
```

跑之前确认执行器顶部的开关：`ENABLE_REAL_MOTION` 默认 `False`
(只打印不下发)，确认打印无误后再改 `True`。真跑人守急停。

## 文件

运行必需（少一个就跑不起来）：

- `execute_servo_grasp.py`
- `robot_lock.py`  — 机器人互斥锁 —— 四个执行器都 import, 必需; 同一台臂同时只允许一个进程
- `sdk_session.py`  — SDK 会话/运动封装 —— 执行器 import 它, 必需
- `servo_common.py`  — 透传公共件: 节拍器 + 跟随误差监控 —— 两条透传线都 import, 必需
- `traj_minimal_joint.json`
- `traj_multi_latest.json`
- `traj_multi_right_latest.json`

另外还依赖 `pypilot`（厂商 SDK 轮子），那是装在真机容器里的，
不在封装内。

附带工具（不装也能跑）：

- `diag_chassis.py`  — 底盘(AGV)状态诊断; 全程只读不发运动指令, 不碰机械臂, 可与手臂脚本同时跑
- `diag_enable.py`  — 使能专项诊断; 单次使能后只读轮询, 不发运动/夹爪命令
- `probe_sdk.py`  — 探测本机 SDK 实际有哪些接口/什么签名; 不连机器人, 随时可跑
- `scrub_log.py`  — 日志脱敏工具 —— 没人 import, 可不装; 日志外发前过一遍
- `test_gripper_right.py`
- `test_servo_safety.py`  — 离线回归测试(不连机器人): 遥测关闭/节拍/向量类型/互斥锁/只读会话/速度还原

## 增量更新

真机上已经有这一套的话，日常只需要送 **变化了的** 文件——
通常就是 `execute_servo_grasp.py` 和 `robot_lock.py` 和 `servo_common.py` 和 `traj_minimal_joint.json` 和 `traj_multi_latest.json` 和 `traj_multi_right_latest.json`。
`sdk_session.py` 只在它自己被改过时才需要重送（改没改，看提交说明）。

## 校验

```bash
shasum -a 256 -c SHA256SUMS
```

或在工作区根目录跑 `python3 releases/make_release.py --verify`。

## 重新生成

```bash
python3 releases/make_release.py C
```

封装是从工作区当前代码复制出来的快照，**不要直接改封装里的文件** —— 
改源文件再重新生成，否则下次生成会把你的改动覆盖掉。
