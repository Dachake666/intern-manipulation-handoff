# 真机执行与历史实验

真机执行开发源在 `src/frame_calibration/robot_side/`；`src/robot_mission/mission_runner.py` 编排任务并调用执行器 `--run`。先选择具体成果和运行记录，再恢复完整组合。只恢复文件不会连接设备；历史脚本的启动行为需要逐项核实。

## 选择成果

| 节点 | 执行入口 | 已完成成果与归档情况 |
|---|---|---|
| B2 Track A 固定场景 | `execute_tabletop_hybrid_trial_reviewfix_field.py` + `tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json` | 5% 一轮、10% 四轮完整成功记录；只支持相同文件组合、任务与场景 |
| B3 Track C 双次抓放 | `execute_servo_grasp.py` + `traj_multi_latest.json` | 已跑出实机结果：原始说明记载双次抓放连续 5 轮成功（133 运动点 + 4 夹爪事件）；完整逐轮日志与源码绑定待补，归档证据标记 LIMITED |
| B4 Servo | `_20ms.py`、`_25ms.py`、`_vaj20_v3.py` 等 field wrapper | 61 份非 MPC 现代 JSON；各轮依赖不同。正式资格绕过与 VAJ3 算法哈希缺口仍在 |
| B5 MPC | `execute_tabletop_servo_field_mpc_20ms.py`、`_mpc_active_20ms.py`、各自 base 与 controller | 已有 6 次 Shadow、4 次 Active 实机运行报告；待补当时 controller 的版本 SHA，以核对实际运行的算法源码 |
| B6 MoveJ 基础 | 99 点冻结组合、153 点运行后安全快照 | 99/88/153 点历史完成日志；不具备统一的现代源码与收尾证据 |
| B7 瓶子示教 | 238 点与带 Home 收尾的 275 点轨迹 | 五轮完整抓放日志缺失；使能/8080 诊断日志不能替代运动日志，LIMITED |
| D1 / D2 / D3 | Servo 长迟到 / MPC 早期失效 / MoveJ 限位与跳点 | D1 有精确对照源码；D2/D3 部分仅有报告，不承诺重新触发故障 |

Track C 与 MPC 的实机成果已经产生；下文的 LIMITED、missing 和未绑定项描述归档缺口，不表示未运行或运行失败。Track C 待补完整逐轮日志，MPC 已有运行报告，待补 controller 的历史版本身份。

Shadow 仍发送原参考轨迹到机器人，只是不应用 MPC 修正，因此属于运动实验。

## 恢复整个节点

```bash
python3 tools/export_version.py B2 .runtime/track-a-reference
python3 tools/export_version.py B3 .runtime/track-c-reference
```

目标目录必须不存在。主要路径如下：

| 节点 | 导出目录中的路径 |
|---|---|
| B2 | `debian/trackA_worlds_line/trackA_hybrid_trial/` |
| B3 | `debian/trackC_servo_stream/trackC_servo_stream/` |
| B4 | `debian/trackA_worlds_line/trackA_servo/` |
| B5 | field/base 在 `debian/trackA_worlds_line/trackA_servo/`；controller 在 `debian/mpc_experiment/mpc_experiment/` |
| B6 | `debian/sdk_tests/frame_calibration/records/` |
| B7 | `debian/trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/` |

`debian/` 是历史来源布局，不是运行类别。`SNAPSHOT.json` 给出来源和 SHA；同名 `sdk_session.py`、`servo_common.py`、wrapper 不能跨节点互换。

## 按一轮报告装配

```bash
python3 tools/restore_run.py --list --node B4
python3 tools/restore_run.py --list --node B5

python3 tools/restore_run.py trackA_worlds_line/trackA_servo/tabletop_servo_20260909_182932_719907228.json .runtime/servo-run-reference
```

工具根据 `docs/debian_selection.json` 的 `run_bindings` 读取精确源码、验证 SHA 并按 `restore_name` 装配，写出 `RESTORE_MANIFEST.json`。装配不执行脚本。必须阅读 manifest 的缺失/未绑定项；MPC controller 没有历史哈希时不会自动选择同名实现，缺口保留为 missing。

B5 field/base 顶层导入 `controller`，节点导出保留它们各自的来源目录。直接在 Servo 目录执行 field，即使传 `--dry-run`，也可能先发生导入错误。装配与路径核对不应绕过算法身份缺口。只需要复核历史结果时，使用 [仿真说明](../sim/README.md) 中的标准库复算工具。

## Track A 的离线开始入口

先恢复 B2，再在其完整目录执行显式 dry-run：

```bash
(cd .runtime/track-a-reference/debian/trackA_worlds_line/trackA_hybrid_trial && python3 execute_tabletop_hybrid_trial_reviewfix_field.py tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json --dry-run)
```

该历史最终组合的关键身份：

- 轨迹 SHA：`e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59`。
- 执行器 SHA：`36a0f659122eb492f63616f5c34a7e3b1f342c67f28e9906e2b9d6ad5b04e7ee`。
- SDK XYZUVW 已是补偿后的 `EE_POSE`，不能再次补 TCP；`PLACE_HOVER` 关节构型对齐关系必须保留。

历史成功不授予新目标、起点、物体或场景的执行资格。新运行包由 `src/releases/make_release.py` 生成；可用模式以其 `--help` 为准，退役实验通过历史节点恢复。**AT 模式仅锁定计划与执行器，其他依赖取当前开发源码，因此不等于 B2 的完整历史组合；当前依赖的重新构建不能自动继承 B2 的实跑结论。**

## 现场执行前后

1. 确认 CPython 3.10 / Linux x86_64 SDK 环境、机器人身份、运行时网络地址，以及当前姿态、工具与标定。环境细节见 [ENVIRONMENT_REFERENCE](../docs/ENVIRONMENT_REFERENCE.md)。
2. 对精确候选完成离线全路径检查和人工 GUI 回放；按该入口实际行为区分 dry-run 与连接 SDK 的只读预检。
3. 清报警、使能、夹爪和运动由现场人员明确确认。持有机器人锁；检查首点跳变、限位、完整场景/持物碰撞与释放回撤。
4. 结束和异常路径验证保护、全局速度恢复及 SDK 关闭。保留成功和失败终端日志与结构化报告，先对副本脱敏，再归档 SHA。

历史 B5 离线实验还包含固定模型路径 `/home/dev/workspace/XF0112048/robot.urdf`。精确复现需匹配挂载布局；修改路径的开发副本应登记新 SHA。其简化 `calib_common.py` 只服务该实验，不能覆盖共享标定源。更多限制见 [KNOWN_ISSUES](../docs/KNOWN_ISSUES.md)。
