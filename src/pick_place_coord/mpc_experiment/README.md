# 7轴位置指令 MPC 仿真实验

本模块在固定关节参考附近，根据反馈优化下一步位置指令。`controller.py` 是纯算法，`run_experiment.py` 组织 PyBullet 对照、GUI 和报告，`test_mpc.py` 覆盖模型、约束、失败拒绝和事件边界。它不是避障规划器，也不是动力学 NMPC。

## 复跑

先按根 `docs/ENVIRONMENT_REFERENCE.md` 建立仿真环境。以下从仓库根导出实验副本，避免覆盖参考结果：

```bash
python3 tools/export_version.py B1 .runtime/mpc-work
(cd .runtime/mpc-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/mpc_experiment/run_experiment.py)
(cd .runtime/mpc-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/mpc_experiment/run_experiment.py --gui)
```

批量模式写副本中的 `results/report.json`、`traces.npz`、`comparison.png`；`--gui` 运行 nominal MPC 动态展示，不覆盖批量结果。B1 导出不含 Git 历史。

## 输入、控制与结果

默认参考由 `run_experiment.py:DEFAULT_REFERENCE` 指定，SHA 为 `71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338`，包含2484个关节参考点、open/close/open事件及等待。替换参考使用 `--reference PATH`，必须通过同一 `tabletop_servo_contract`；相机 XYZ 不能直接作为本模块输入。

控制模型为位置指令积分器与一阶响应：

```text
c[k+1] = c[k] + dt * u[k]
q[k+1] = (1-alpha) * q[k] + alpha * c[k+1]
```

使用 OSQP、20ms周期、15步预测，实验响应系数 alpha=0.4。限制来自共享臂配置和 Servo 策略；约束位置、指令速度和加速度，并处理事件到位与终端停止。坏输入、不可行、非准确求解、超时或残差超限均应中止，不复用旧解。

已有六组对照为正常、响应变慢和短时外力三种工况，各比较原位置流与 MPC。参考报告中正常 RMSE 为 0.064503°→0.003943°，响应失配为 0.149176°→0.057536°。具体数值和来源 SHA 以 `results/report.json` 为准；重新运行应登记新结果，不覆盖参考。

## 边界与在线实现

PyBullet 使用估算惯量和零重力，未辨识真实机器人动力学；没有完整场景/持物碰撞、力矩约束或 SDK 网络时序验收。指令加速度变小不代表反馈加速度和 jerk 都改善。改变参考附近的指令也会改变路径，不能继承父轨迹的碰撞或 GUI 批准。

在线 Shadow/Active 实现与运行记录保存在 B5，使用根 `tools/restore_run.py` 按逐轮身份恢复。Shadow 会向机器人发送原参考运动。旧现场报告缺 controller SHA 的部分保持未知，不用本目录的当前算法倒填。完整职责与复算命令见根 `sim/README.md`、`robot/README.md`。

修改后先运行 `tools/offline_checks.py`。验证记录与新实验登记见根 `docs/MAINTENANCE.md`。
