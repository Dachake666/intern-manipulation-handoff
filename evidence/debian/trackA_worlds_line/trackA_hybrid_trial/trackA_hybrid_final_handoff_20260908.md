# TrackA Hybrid 抓取-放置轨迹最终交接总结

**日期：2026-09-08**  
**工作目录：** `~/workspace/trackA_worlds_line/trackA_hybrid_trial`

---

## 1. 最终结果

从上游 ZIP 包中的 `tabletop_pick_place_hybrid_OPTIMIZED.json` 出发，完成了轨迹构型修复、真实控制器预检、真机多轮验证、运行逻辑清理和运行时延迟优化。

最终流程：

```text
START
  ↓
OPEN_BEFORE_PICK
  ↓
PICK_HOVER        MoveWorlds
  ↓
PICK_DESCEND      MoveWorlds
  ↓
CLOSE_AT_PICK
  ↓
PICK_ASCEND       MoveWorlds
  ↓
PLACE_HOVER       单次 MoveJ
  ↓
PLACE_DESCEND     MoveWorlds
  ↓
OPEN_AT_PLACE
  ↓
PLACE_ASCEND      MoveWorlds
  ↓
RETURN_SAFE       固定 MoveJ Home
  ↓
PASS_RETURNED_SAFE
```

当前版本只在程序开始前保留一次 Enter 确认，运动过程中不再需要人工回车。

最终版本已在真机上以 **5% 和 10%** 速度完整跑通并返回 Home。

---

## 2. 最终核心文件

### 轨迹文件

```text
tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json
```

当前最终 Plan SHA-256：

```text
e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59
```

夹爪固定等待时间：

```text
open settle  = 1.0 s
close settle = 2.0 s
```

### 执行器

```text
execute_tabletop_hybrid_trial_reviewfix_field.py
```

最终真机 PASS 日志记录的执行器 SHA 前缀：

```text
36a0f659122e...
```

交接前建议重新执行：

```bash
sha256sum execute_tabletop_hybrid_trial_reviewfix_field.py
```

并将完整 SHA 写入 `SHA256SUMS`。

---

## 3. 上游 OPTIMIZED 轨迹发现的问题

原始优化轨迹采用新的近水平抓取姿态：

```text
UVW = [-93.110, 8.776, -5.238] deg
```

主要结构已经简化为单次 MoveJ 转运，没有原来的 `TRANSFER_MID`。

真实控制器只读 IK 检查发现：

- 原 `PLACE_HOVER` 指定七关节构型与后续 `PLACE_DESCEND MoveWorlds` 的控制器 IK 分支不一致；
- MoveJ → MoveWorlds handoff 的最大关节构型差约 **9.405°**；
- 原 `PLACE_HOVER` J2 离硬限位仅约 **5.577°**，距离 5°运行 margin 太近。

这是典型 7DOF 冗余机械臂问题：

> 相同 XYZUVW 不等于相同七关节冗余构型。

---

## 4. SDK-aligned 修复

保持抓取/放置的 XYZUVW 和整体动作结构不变，仅重新对齐 `PLACE_HOVER` 的七关节目标。

最终 `PLACE_HOVER` q：

```text
[-33.131,
   2.908,
  19.904,
 -90.809,
 -23.508,
  -7.728,
  40.384]
```

修复效果：

```text
MoveJ PLACE_HOVER → MoveWorlds PLACE_DESCEND
branch mismatch:
约 9.405° → 约 0.413°
```

同时：

```text
PLACE_HOVER J2 hard-limit margin:
约 5.577° → 约 12.908°
```

之后真实控制器上的 7 个运动段均通过 IK / hard-limit 预检。

---

## 5. 真机验证

代表性真机 PASS：

```text
hybrid_trial_run_20260908_145807_733922.json
speed = 1%
PASS_RETURNED_SAFE
```

证明：

```text
PICK_ASCEND
→ PLACE_HOVER 单次 MoveJ
→ PLACE_DESCEND MoveWorlds
```

可以在真机上实际执行。

随后：

```text
hybrid_trial_run_20260908_151742_088250.json
speed = 3%
PASS_RETURNED_SAFE
```

进一步证明 SDK-aligned handoff 稳定。

---

## 6. 运行逻辑清理

删除测试阶段已经没有运行价值的旧限制：

```text
field-trial-v2
旧 reviewed_start gate
Recorded/GUI start comparison
历史 qualification runtime blocker
旧 GUI 起点绑定
```

最终采用：

> 每次直接读取机器人当前真实停止状态作为运行起点。

保留的运行保护：

- 真实关节 hard-limit 检查；
- 固定 5° joint margin；
- MoveJ 当前 q → 目标 q 的实时 dense limit check；
- MoveWorlds IK branch / limit 检查；
- controller soft-stop / protection；
- 到位检查；
- 异常退出；
- 固定 MoveJ 返回 Home；
- `END_ASSERT_SAFE`。

---

## 7. 人工确认优化

测试阶段曾有三次人工 Enter：

```text
启动前
PICK_ASCEND 后
PLACE_HOVER 后
```

最终删除两个中途确认，只保留启动前唯一一次 Enter。

启动后整套 pick-place 自动完成。

---

## 8. 速度接口

旧版本人为限制：

```text
0.1% ~ 5%
```

最终扩展为：

```text
0.1% ~ 20%
```

速度作为运行参数传入：

```text
--speed 5.0
--speed 10.0
```

---

## 9. 10% 运行时 settling 修复

10% 测试中曾出现：

```text
pre-command/PICK_DESCEND:
reviewed state drifted
```

典型变化约：

```text
1~2 mm
0.1~0.2° UVW
0.1~0.2° joint
```

问题来自：

```text
上一运动结束
→ 大量 armTryWorlds 运行时检查
→ servo / SDK readback 小范围 settling
→ 原 1 mm strict threshold 误判
```

最终增加专门的 pre-command settling envelope：

```text
3.0 mm
0.5° UVW
0.5° joint
```

如果超过原 strict threshold、但仍处于正常 settling 范围：

```text
读取最新真实状态
→ 重新执行 live precheck
→ 再发送下一运动命令
```

启动阶段原来的严格状态检查没有放宽。

之后 10% 多次完整运行成功。

---

## 10. waypoint 停顿优化

通过 PASS 日志测得旧运行方式存在明显的软件空档：

```text
OPEN → PICK_HOVER          1.707 s
PICK_HOVER → PICK_DESCEND  2.540 s
CLOSE → PICK_ASCEND        1.260 s
PLACE_HOVER → DESCEND      0.822 s
OPEN → PLACE_ASCEND        0.831 s
```

而部分 MoveJ 切换只有：

```text
PICK_ASCEND → PLACE_HOVER   0.002 s
PLACE_ASCEND → RETURN_SAFE  0.002 s
```

确认主要延迟不是 `wait_robot_idle()`，而是每个 MoveWorlds 前重复执行整段几十到上百点的 `armTryWorlds` dense IK。

最终改成：

### 运行前

仍然执行：

```text
整条轨迹完整 dense IK
完整 hard-limit 检查
完整 continuity 检查
```

### 运行过程中

MoveWorlds 改为：

```text
当前真实 q
→ 下一段最前面的 3 个 IK 点
→ 检查 branch continuity + hard limit
```

即运行时使用快速 local handoff probe，而不是重新扫描完整 50~130 个 IK 点。

MoveJ 仍保留：

```text
当前真实 q → target q
完整 dense joint limit precheck
```

---

## 11. 最终代表性真机 PASS

目前保留的代表性日志：

```text
hybrid_trial_run_20260908_145807_733922.json
# 1% 最早真机 PASS

hybrid_trial_run_20260908_151742_088250.json
# 3% SDK-aligned PASS

hybrid_trial_run_20260908_171421_385917.json
# 10% fast-runtime 优化前对照

hybrid_trial_run_20260908_172336_575481.json
# 最终 fast-runtime，5% PASS

hybrid_trial_run_20260908_172435_229744.json
# 最终 fast-runtime，10% PASS
```

最终两个日志使用同一个执行器版本：

```text
executor SHA prefix = 36a0f659122e...
```

并使用最终 Plan：

```text
e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59
```

因此最终版本已经至少在 **5% 和 10%** 下完成完整真机验证。

---

## 12. 日志策略

最终运行逻辑：

> 只有完整执行成功、返回 Home 并得到 `PASS_RETURNED_SAFE` 才保存 `hybrid_trial_run_*.json`。

以下情况不保存 run JSON：

```text
SDK 数据未 ready
运行前检查失败
中途异常
operator abort
未成功返回 Home
```

---

## 13. 最终运行命令

例如 10%：

```bash
cd ~/workspace/trackA_worlds_line/trackA_hybrid_trial

XIFENG_ALLOW_REAL_MOTION=1 python3 execute_tabletop_hybrid_trial_reviewfix_field.py   tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json   --run   --start-policy current-reviewed   --robot-ip 192.168.8.147   --local-ip 192.168.8.185   --arm-ip 192.168.8.147   --arm-port 8080   --speed 10.0   --limit-margin-deg 5
```

运行行为：

```text
完整预检
→ 开始前唯一一次 Enter
→ 自动执行完整 pick-place
→ 自动返回 Home
→ PASS_RETURNED_SAFE
→ 保存 run JSON
```

---

## 14. 建议交接文件

### A. 必须发送：最终可运行版本

```text
execute_tabletop_hybrid_trial_reviewfix_field.py
tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json
sdk_session.py
execute_tabletop_pick_place_worlds.py
precheck_tabletop_hybrid.py
robot_lock.py
scrub_log.py
```

### B. 必须发送：最终验证证据

```text
hybrid_trial_run_20260908_172336_575481.json
hybrid_trial_run_20260908_172435_229744.json
```

分别对应：

```text
5%  PASS_RETURNED_SAFE
10% PASS_RETURNED_SAFE
```

### C. 建议发送

```text
trackA_hybrid_final_handoff_20260908.md
SHA256SUMS
```

推荐重新生成 `SHA256SUMS`：

```bash
sha256sum   execute_tabletop_hybrid_trial_reviewfix_field.py   tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json   sdk_session.py   execute_tabletop_pick_place_worlds.py   precheck_tabletop_hybrid.py   robot_lock.py   scrub_log.py   hybrid_trial_run_20260908_172336_575481.json   hybrid_trial_run_20260908_172435_229744.json   > SHA256SUMS
```

---

## 15. 不需要发送

除非对方明确要求开发历史，否则不需要：

```text
execute_tabletop_hybrid_trial.py
execute_tabletop_hybrid_trial_reviewfix.py

execute_tabletop_hybrid_trial_reviewfix_field.py.bak_*
tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json.bak_*

tabletop_pick_place_hybrid_CANDIDATE.json
tabletop_pick_place_hybrid_OPTIMIZED.json
tabletop_pick_place_hybrid_OPTIMIZED_gui_review.json

hybrid_trial_precheck_*.json
hybrid_teach_feedback_*.json
hybrid_teach_feedback_*.jsonl
record_teach_feedback.py

旧的重复 run JSON
失败 run JSON
__pycache__/
uploads/
```

旧 `README.md` 如果仍然描述早期 `BLOCKED` / GUI qualification / field-trial-v2 流程，也不要直接作为最终 README 发送，应先更新。

---

## 16. 最终交接包建议结构

```text
trackA_hybrid_final/
├── trackA_hybrid_final_handoff_20260908.md
├── SHA256SUMS
│
├── execute_tabletop_hybrid_trial_reviewfix_field.py
├── tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json
│
├── sdk_session.py
├── execute_tabletop_pick_place_worlds.py
├── precheck_tabletop_hybrid.py
├── robot_lock.py
├── scrub_log.py
│
├── hybrid_trial_run_20260908_172336_575481.json
└── hybrid_trial_run_20260908_172435_229744.json
```

这是目前最适合作为最终工程交接的最小可运行、可追溯版本。
