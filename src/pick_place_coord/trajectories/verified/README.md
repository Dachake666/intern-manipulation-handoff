# 真机验证通过的轨迹(冻结, 只读)

这个目录里的 JSON **不要改**。每一份都在真机上跑成功过，是出问题时的回退基准。
改进版本一律另存到 `trajectories/` 下，验证通过后再往这里加新文件。

| 文件 | SHA-256(前16) | 真机战绩 | 日志 |
|---|---|---|---|
| `world_grasp_arc12_20260729_REALVERIFIED.json` | `947b7bc6fcbb3813` | Track A，20260729 连续 3 次成功 | [记录](../../../frame_calibration/records/20260729_world_grasp_arc12_success/) |
| `traj_minimal_joint_8pt_REALVERIFIED.json` | `702a198ebcc35848` | Track B，8 路点，3 次成功 | — |
| `traj_multi_2grasp_20260804_REALVERIFIED.json` | `ac5c6fd7715c81fb` | **Track C pulse 双抓放，5 轮全过**（0.4°/20ms，32.7s，终点 0.048°） | [记录](../../../frame_calibration/records/20260804_pulse_multi_grasp_verified/) |

## Track A 这份是什么

18 个拐角：`START → PICK_DESCEND → [close] → PICK_ASCEND → ARC01..ARC12 →
PLACE_DESCEND → [open] → PLACE_ASCEND`，全部走 `armMoveWorlds` 直线，
执行器 `MODE="point"`。

三次运行的到位误差 0.8~3.9mm，限位预检最小余量 9.5°(RUN2 ARC09 j4)。

**已知缺点(20260729 现场评价)**：ARC 那 12 个点每个都是一条独立的
`armMoveWorlds` + 等停稳，转运段又慢又顿；PICK/PLACE 的 ascend/descend
是单条长直线，又快又顺。这就是下一版改混合模式的直接动机。

## 用法

```bash
python3 execute_world_grasp.py world_grasp_arc12_20260729_REALVERIFIED.json
```

执行器侧需要 `MODE="point"`、`ENABLE_REAL_MOTION=True`。
