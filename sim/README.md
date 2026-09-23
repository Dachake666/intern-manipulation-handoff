# 仿真与离线验证

按实际行为分类：PyBullet、轨迹计算、历史指标复算与离线接口测试都在这一类；在机器人主机上运行也不会因此成为实机成果。职责总览见 [START_HERE](../START_HERE.md)。

## 入口一览

| 内容 | 入口 | 输入与输出 |
|---|---|---|
| 规划/IK/场景 | `src/pick_place_coord/pick_place_coord.py`、`left_arm_ik.py`、`xifeng_pb.py` | task/pick/place → 轨迹与仿真 |
| 双次抓放任务 | `pick_place_coord.py --task tasks/task_2pairs_table72.json --seed 7` | 重新规划两对抓放；默认不导出 JSON |
| 冻结 Track C 点流 | `src/pick_place_coord/verify_left_track_c.py` | 冻结133点轨迹 → 1633/3232帧及固定摘要，无 GUI |
| 桌面/瓶子候选 | `gen_tabletop_hybrid_candidate.py`、`gen_taught_tcp_candidate.py` | 对应任务与场景 → 候选；不自动继承旧现场资格 |
| MPC | `src/pick_place_coord/mpc_experiment/run_experiment.py`、`controller.py` | 默认参考 → 六组仿真/报告；`--gui` 为 nominal MPC 动态展示 |
| 双臂 | `src/pick_place_coord/dual_arm_demo/demo.py` | 冻结仿真 JSON → 1624 帧；`--build` 会重建，资格仍为 REJECT |
| 视觉与任务 | `src/vision/vision_system/replay.py`、`src/robot_mission/{create_task,task_adapter,tabletop_plan,preflight}.py` | RGB/深度与位姿 → 观测、候选、离线预检；同目录 mission_runner.py 会调用真机执行器，不属本行 |

## 可直接核对的离线入口

以下从交接根目录执行，解释器需先具备相应依赖：

```bash
python3 tools/offline_checks.py
(cd src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/verify_left_track_c.py)
```

冻结 Track C 应得到：0.4°步长1633帧，0.2°步长3232帧。当前没有直接加载冻结双次轨迹的完整 GUI 专用入口；`demo_three_tracks.py --track C` 是单次 pick/place 演示，`demo_worlds_10x_pybullet.py` 只取一个姿态生成十步图案，都不能替代双次冻结回放。

重新生成双次任务并观看规划过程：

```bash
(cd src/pick_place_coord && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord.py --task tasks/task_2pairs_table72.json --seed 7)
```

这是重新规划，不保证重现历史冻结 JSON 的每个字节；需要保存新候选时显式指定 `--out` 到工作目录，保持原候选不变。上述 GUI 命令本轮仅核对入口，未进行人工观看验收。

## MPC 与双臂复跑

批量 MPC 会写覆盖其所在副本的 `results/`，先从交接根目录恢复 B1 到新目录，再运行副本：

```bash
python3 tools/export_version.py B1 .runtime/simulation-work
(cd .runtime/simulation-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/mpc_experiment/run_experiment.py)
```

B1导出副本没有`.git`，只用于独立实验；恢复其他历史节点仍从原交接根目录调用工具。

GUI 演示仍从交接根目录调用同一副本：

```bash
(cd .runtime/simulation-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/mpc_experiment/run_experiment.py --gui)
(cd .runtime/simulation-work/src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/dual_arm_demo/demo.py --gui --exit-after-replay)
```

MPC 仿真采用零重力/估算惯量，不是完整场景或持物碰撞资格；双臂当前碰撞资格 REJECT。GUI 动画存在不等于已有人工 PASS。

收到的 B5 也包含另一组 MPC 仿真。其报告绑定 `controller.py.pre_rt_diagnostics`，原模型路径及导入布局需按来源恢复；它与 B5 在线 Shadow/Active 是不同结果，不能默认用最新同名 controller 复现。仅复算十轮现场 JSON，可从交接根目录运行：

```bash
python3 tools/export_version.py B5 .runtime/mpc-reference
python3 evidence/mpc_review/recalculate_mpc_reports.py .runtime/mpc-reference/debian
```

预期固定6 Shadow/4 Active，平均RMSE改善约34.45%。工具只读 JSON/文件哈希，不导入 SDK 或发送控制指令。

## 环境与资格边界

当前已测依赖版本见 `src/requirements/mac-tested.txt`；它记录已有环境，并非新机器的完全安装锁。历史 `planning-py310.lock.txt` 自述 Linux x86_64/Python3.10，不与当前实测清单混装。完整环境来源与 SDK/ROS ABI 差异见 [环境参考](../docs/ENVIRONMENT_REFERENCE.md)。

共享模型位于 `src/XF0112048/`；臂配置、schemas 与权威 `calib_common.py` 不复制到各实验里。视觉和手眼能做离线模块检查，但完整检测服务、标定质量门和现场接口仍有缺口，见 [已知问题](../docs/KNOWN_ISSUES.md)。
