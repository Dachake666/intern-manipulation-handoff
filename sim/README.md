# 仿真与离线验证

这部分负责目标与轨迹计算、PyBullet、历史指标复算、接口和安全回归。运行主机的位置不改变实验类别。所有命令默认从仓库根执行，并使用已建立的仿真环境。

## 代码职责

| 代码 | 输入 → 输出 |
|---|---|
| `src/pick_place_coord/pick_place_coord.py` | task/pick/place → IK、路径与仿真；`left_arm_ik.py` 解逆解，`xifeng_pb.py` 载入模型 |
| `verify_left_track_c.py` | 冻结双次轨迹 → 密集点流和摘要 |
| `gen_tabletop_hybrid_candidate.py`、`gen_taught_tcp_candidate.py` | 对应任务/反馈/场景 → 新候选与检查，不继承旧真机资格 |
| `mpc_experiment/controller.py` | 当前反馈、前一指令、未来参考 → 受约束的位置指令 |
| `mpc_experiment/run_experiment.py` | 固定参考 → 六组动态仿真、报告、数据、GUI |
| `dual_arm_demo/demo.py` | 双臂候选 → 同步动画与联合碰撞报告 |
| `src/vision/`、`src/robot_mission/` 的任务/预检模块 | 图像深度/位姿 → 观测、任务、候选及检查；不包括真机入口 `mission_runner.py` |

## 先复核，再实验

```bash
python3 tools/offline_checks.py
(cd src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/verify_left_track_c.py)
```

Track C 预期为 0.4°步长 1633 帧、0.2°步长 3232 帧。这只核对历史点流；没有专用入口完整 GUI 回放该冻结双次轨迹。

新实验先导出开发工作副本，避免覆盖参考结果：

```bash
python3 tools/export_version.py B1 .runtime/simulation-work
(cd .runtime/simulation-work/src/pick_place_coord && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord.py --task tasks/task_2pairs_table72.json --seed 7)
```

此命令重新规划双次任务，不保证生成历史 JSON。需要保存候选时使用程序的 `--out` 参数。导出目录没有 `.git`；版本恢复工具仍从完整仓库调用。

## MPC 与双臂

```bash
# 批量对照，写入实验副本的 results/
(cd .runtime/simulation-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/mpc_experiment/run_experiment.py)

# 动态展示
(cd .runtime/simulation-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/mpc_experiment/run_experiment.py --gui)
(cd .runtime/simulation-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/dual_arm_demo/demo.py --gui --exit-after-replay)
```

MPC 结果在 `src/pick_place_coord/mpc_experiment/results/`：`report.json`、`traces.npz`、`comparison.png`。采用零重力、估算惯量和模拟位置反馈，不包含完整场景/持物碰撞资格。双臂为 1624 帧运动学展示，完整工具碰撞资格仍为 REJECT。GUI 播放成功不能替代人工资格确认。

B5 另含一组历史离线 MPC 实验，报告绑定 `controller.py.pre_rt_diagnostics`；不能用在线 controller 代替。历史模型路径和导入布局需要单独恢复，详见 [真机与历史版本说明](../robot/README.md)。

## 不安装 SDK 也能复算现场指标

```bash
python3 tools/export_version.py B5 .runtime/mpc-reference
python3 evidence/mpc_review/recalculate_mpc_reports.py .runtime/mpc-reference/debian
```

该工具只读 JSON 和文件哈希，预期固定 6 Shadow / 4 Active，平均 RMSE 改善约 34.45%。这是特定十轮记录的统计复算，不是新真机实验或成功率估计。

## 修改后的验证

依赖清单与安装边界见 [环境参考](../docs/ENVIRONMENT_REFERENCE.md)。对应模块的 `test_*.py` 是回归资产，不因文件名视为临时实验。使用明确的测试列表，避免自动运行硬件探针。新增实验记录输入/代码 SHA、参数、输出位置与未验证范围；不要覆盖参考 `results/`。维护步骤见 [MAINTENANCE](../docs/MAINTENANCE.md)。
