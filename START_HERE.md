# 从这里开始

本仓库保存机器人抓取放置的开发代码、仿真结果和可恢复的真机实验版本。**先按职责找到代码，再按成果选择复现入口。** 默认 `src/` 用于开发；历史现场组合从版本节点或逐轮绑定恢复。

## 负责人先看哪部分

| 负责内容 | 开发入口 | 需要维护的输出 |
|---|---|---|
| 规划与仿真 | `src/pick_place_coord/` | task、IK、轨迹、场景、PyBullet 回放和资格报告 |
| Servo 与执行器 | `src/frame_calibration/robot_side/` | SDK 会话、节拍、夹爪、首点与限位检查、保护/速度收尾 |
| MPC 算法 | `src/pick_place_coord/mpc_experiment/` | `controller.py` 控制器、`run_experiment.py` 对照实验；在线集成使用 B5 |
| 视觉与任务接口 | `src/vision/`、`src/robot_mission/` | 观测、坐标语义、任务转换、候选和预检；`mission_runner.py` 属真机编排 |
| 相机与手眼工程 | `extensions/debian_vision/` | 设备、检测、ROS 接口、手眼节点和配置 |
| 共享配置与发布 | `src/arm_profiles*`、`src/schemas/`、`src/frame_calibration/analysis/`、`src/releases/` | 统一契约、标定、模型引用与可追溯运行包 |

处理链为：目标观测 → 坐标/TCP 语义转换 → IK/路径规划 → 仿真与资格检查 → 参考轨迹 → Servo/MPC 执行 → 运行记录。MPC 调整参考轨迹附近的位置指令，不替代视觉、避障规划或全路径资格检查。

## 要复现什么

| 成果 | 使用入口 | 预期结果与限制 |
|---|---|---|
| 双次抓放任务仿真 | `pick_place_coord.py` + `tasks/task_2pairs_table72.json` | 两组抓放规划与动画；重新规划不保证等于历史冻结轨迹 |
| 冻结 Track C 点流 | `verify_left_track_c.py` | 0.4°步长 1633 帧、0.2°步长 3232 帧；纯计算，完整冻结 GUI 回放入口尚缺 |
| MPC 六组离线对照 | `mpc_experiment/run_experiment.py` | 报告、逐周期数据与对照图；不是场景碰撞或真机验收 |
| 双臂同步动画 | `dual_arm_demo/demo.py` | 1624 帧；完整工具碰撞资格 REJECT，不可用于双臂实机执行 |
| Track A 固定场景 | B2 | 五份最终成功 JSON：1 次 5%、4 次 10%；仅限精确文件组合与场景 |
| Track C 双次实机实现 | B3 | 133 运动点 + 4 夹爪事件；五轮汇总原日志不完整，LIMITED |
| Servo 20/25ms、VAJ3、30ms对比 | B4 + 逐轮 `run_bindings` | 61 份非 MPC 现代 JSON；正式资格绕过及部分算法身份缺口保留 |
| MPC Shadow / Active | B5 + 逐轮 `run_bindings` | 6 Shadow + 4 Active，历史指标可复算约 34.45% 平均 RMSE 改善；controller 历史 SHA 缺口保留 |
| 早期 MoveJ / 瓶子抓放 | B6 / B7 | 历史基础与示教轨迹；证据范围见版本表 |
| 视觉→任务→候选 | `vision_system/replay.py`、`robot_mission/tabletop_plan.py` | 离线模块验证；完整视觉自主抓放仍未验收 |

仿真命令见 [sim/README.md](sim/README.md)，真机组合与恢复命令见 [robot/README.md](robot/README.md)。完整目录地图见 [docs/CONTENTS.md](docs/CONTENTS.md)。

## 三个开始入口

以下均在仓库根执行：

```bash
# 完整性；只读文件，不导入 SDK
python3 tools/verify_handoff.py

# 选定离线回归；先按环境说明建立仿真解释器
python3 tools/offline_checks.py

# 查看已登记的 MPC 逐轮组合；不会连接机器人
python3 tools/restore_run.py --list --node B5
```

需要重新运行实验时使用 `.runtime/` 新目录，保留参考报告和冻结输入。环境要求见 [ENVIRONMENT_REFERENCE](docs/ENVIRONMENT_REFERENCE.md)；当前验证范围见 `verification.json`；已知限制见 [KNOWN_ISSUES](docs/KNOWN_ISSUES.md)。

## 版本与维护

- `B1` 导出 `handoff/current` 已登记交付文件，**不含 `.git`**；保存整个仓库才能保留历史版本。手眼大样本随完整包保存。
- B2～B7 为成果组合，D1～D3 为缺陷证据，`history` 为补充历史；按文件 SHA 判断身份。
- `PRE_PRUNE` 保存完整的精简前树，供恢复旧诊断、说明及历史数据，不作为默认开发入口。
- 新功能、实验和运行记录按 [MAINTENANCE](docs/MAINTENANCE.md) 登记；根清单由打包工具更新。

## 仓库工作约定

以下内容与根 [AGENTS.md](AGENTS.md) 同源生成。

### 仓库工作约定

本文件适用于本仓库；仓库根为本文件所在目录。先读 `START_HERE.md`，再按职责查 `docs/CONTENTS.md`、`sim/README.md`、`robot/README.md`。任务的明确要求优先于默认约定。

#### 源码与版本

- 当前开发源码在 `src/`：规划和仿真在 `src/pick_place_coord/`，真机执行在 `src/frame_calibration/robot_side/`，视觉任务接口在 `src/vision/` 与 `src/robot_mission/`。
- `extensions/debian_vision/` 是相机、检测和手眼工程；路径名称标记来源，运行类别由实际行为决定。
- 输出、候选、缓存、恢复目录放在 `.runtime/`。不覆盖冻结轨迹、原日志、历史源码或其他未提交改动。
- `src/frame_calibration/analysis/calib_common.py` 的 `CONFIRMED_*` 是标定常量依据。`CONFIRMED_T_SESSIONS_MM` 用于规划换算，`CONFIRMED_T_SESSIONS_V2_MM` 用于 v2 解释 `armGetWorlds`，不可混用。历史常量不证明当前工位有效。
- `src/arm_profiles.py` / `arm_profiles.v1.json` 统一臂、夹爪、TCP、Home 与限位；`src/schemas/` 统一输入输出契约。禁止在新脚本复制安全常量字面量。
- `src/XF0112048/` 的厂商 URDF/STL 只读。运行发布包只由 `src/releases/make_release.py` 生成，修改源后重建，不手改发布副本。
- 冻结组合使用 `tools/export_version.py` 导出到新目录；有逐轮绑定的实验使用 `tools/restore_run.py` 装配。按 `SNAPSHOT.json`、`RESTORE_MANIFEST.json` 和 SHA 核对，不能按同名文件替换依赖。
- `B1` 为 `handoff/current` 已登记交付版本的导出，不含 `.git`。完整历史必须保留整个仓库和隐藏 `.git/`；`data/handeye/` 大样本随完整包保存，不依赖 Git 恢复。

#### 仿真与真机

- 区分离线计算、连接设备只读、修改控制器设置、运动。`--precheck-only` 是否连接 SDK 以实现为准；MPC Shadow 会发送原参考运动，不是离线模式。
- 新开发的 `ENABLE_REAL_MOTION` 默认关闭。先做离线检查、dry-run 和限位预检；仿真验收不以导入 `pypilot` 为依据。
- 真机前由现场负责人核实机器人身份、网络地址、当前关节/夹爪、标定及工作场景。历史 IP 和位姿只作记录。
- 清故障、使能、夹爪动作和运动必须由现场人员明确确认。历史入口可能默认开启运动，恢复文件不代表允许执行。
- 新生成或重规划的候选，在真机分段测试前必须用执行器一致的密集步长完成 PyBullet 全程 GUI 回放，形成绑定轨迹 SHA 的 `pybullet_gui_review.v1` PASS；headless 碰撞 PASS 不能替代人工 GUI 确认。
- 检查 IK、实际与硬限位、首点跳变、密集完整场景/持物碰撞、开爪包络、放置与撤退空间。j7 实限 76°，不能放宽门限迁就候选。
- 真机互斥使用 `robot_lock`。正常和异常结束均恢复保护与全局速度，并验证收尾，避免持久设置污染后续运行。

#### 证据与维护

- `OBJECT_POSE`、`GRASP_POSE`、`EE_POSE` 必须明确；已补偿的 SDK `EE_POSE` 不再补 TCP。单位、四元数顺序、矩阵方向和标定来源必须进入契约。
- 新日志记录执行器、wrapper、轨迹、动态算法和依赖 SHA、参数、设备身份与时间；成功和失败都留痕。日志使用 `src/frame_calibration/robot_side/scrub_log.py` 脱敏副本，明文 token 不入 Git。
- 不删除原始 `records/` 证据，不改历史日志或冻结源码。目录整理使用可恢复节点与路径映射；不重写证据去消除缺口。
- 只有 README 汇总属于 LIMITED；运行完成、仿真通过、编译通过、哈希一致均不能单独授予通用真机资格。成功结论仅绑定具体文件组合、参数与场景。
- 注释保留 `[待核]`、`[已核实]`、`[重建]` 等置信度标记。改执行器后运行对应离线回归；基础锚点包括 `test_execute_trajectory_options.py`、`test_servo_safety.py`，选定套件入口为 `tools/offline_checks.py`。不要广泛自动收集 `test_*.py`，其中存在硬件探针。
- 新增、移动、退役文件先维护 `docs/DELIVERY_SELECTION.json` 的允许清单和恢复映射，再运行 `tools/offline_checks.py --record` 和 `tools/build_package.py --refresh-manifest`，审阅提交后更新 `handoff/current`，使用 `tools/build_package.py --output 路径.zip` 生成新包；只读校验用 `--check`，不手改根哈希清单。
- 验证结果记录命令、解释器/依赖、范围和未覆盖项；通过的选定套件不等于全仓库全绿。流程见 `docs/MAINTENANCE.md`。
- 提交信息使用简短中文结论句，不做空提交；提交和推送遵循任务授权。本仓库不预设远端。

可选审阅技能包括 `real-robot-run-reviewer`、`run-evidence-archivist`、`frame-calibration-validity-gate`、`motion-variant-scaffolder`、`robot-release-truth-audit`。它们不是运行依赖；未安装时仍执行以上检查。
