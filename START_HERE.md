# 代码目录、职责与成果复现

更新：2026-09-23。按**仿真/离线验证**与**真机执行**划分工作，不按电脑或操作系统划分。旧 workspace 和灵巧手不在本次交接范围。

接手时先选下面的工作内容，找到入口、输入和原结果，再看对应检查。**当前开发代码与历史现场版本分开：`src/` 用于继续开发；复现旧现场组合先恢复 B2～B7，再按具体运行记录核对依赖。**

## 1. 代码之间怎样连接

任务/视觉目标 → 坐标与 TCP 语义转换 → IK/路径规划 → 仿真与资格检查 → 参考轨迹 → 真机执行 → 运行记录。

MPC 是参考轨迹与 Servo 之间的反馈控制层：读取关节反馈、计算下一步位置目标，再交给 Servo。它不负责把相机 XYZ 直接变成完整避障路径。视觉只负责观测与目标；轨迹生成与机器人运动各有独立入口。

## 2. 目录与负责人职责

| 目录/入口 | 负责什么 | 接手人应维护的内容 |
|---|---|---|
| [src/pick_place_coord/](src/pick_place_coord/) | 运动规划与仿真 | 任务、IK、路径、场景、PyBullet 回放及轨迹导出 |
| [src/pick_place_coord/mpc_experiment/](src/pick_place_coord/mpc_experiment/) | MPC 算法与离线实验 | `controller.py` 为算法；`run_experiment.py` 为实验组织和报告；现场接入另看 B5 |
| [src/pick_place_coord/dual_arm_demo/](src/pick_place_coord/dual_arm_demo/) | 双臂仿真 | 双臂轨迹、同步展示和碰撞报告，目前资格 REJECT |
| [src/frame_calibration/robot_side/](src/frame_calibration/robot_side/) | 真机执行开发 | SDK 会话、执行器、Servo、夹爪、互斥、保护/速度恢复及离线测试 |
| [src/vision/](src/vision/) 与 [src/robot_mission/](src/robot_mission/) 的 create_task/task_adapter/tabletop_plan/preflight | 视觉观测与离线任务接口 | 图像/深度回放、观测契约、任务转换、候选生成和无运动预检；不能把整个 robot_mission 目录视为纯仿真 |
| [src/robot_mission/mission_runner.py](src/robot_mission/mission_runner.py) | 真机任务编排 | 调用执行器并传入 `--run`，归真机入口，需要现场授权；不作为离线复现快捷命令 |
| [extensions/debian_vision/](extensions/debian_vision/) | 相机、检测、手眼 ROS/C++ 工程 | 设备驱动、检测服务、手眼节点、接口适配；目录名保留来源，完整链路尚未闭合 |
| [src/frame_calibration/analysis/](src/frame_calibration/analysis/)、[src/arm_profiles.v1.json](src/arm_profiles.v1.json)、[src/schemas/](src/schemas/) | 共享基础 | 标定、坐标/TCP、臂/夹爪配置、数据契约；仿真与执行共同使用 |
| [src/XF0112048/](src/XF0112048/) | 唯一机器人模型 | 只读 URDF/STL，不能为了演示通过而修改厂商模型 |
| [src/releases/make_release.py](src/releases/make_release.py) | 运行发布包生成 | 从开发源码生成指定模式的执行包；不是交接 ZIP 的生成器 |
| [evidence/](evidence/) 与 [src/frame_calibration/records/](src/frame_calibration/records/) | 历史结果与核查 | 运行报告、原结果、脱敏证据；不是另一套当前执行源码 |
| [data/handeye/](data/handeye/) 与 [sdk/](sdk/) | 必要输入与安装物 | 39 组旧手眼样本、厂商 SDK wheel/文档；不代表新环境或标定已验收 |
| [tools/](tools/) 与 [docs/](docs/) | 交接工具与索引 | 文件校验、离线检查、版本恢复、来源映射、已知问题 |
| `.git/` | 可恢复版本 | 精简交付历史；不能单独复制可见源码而丢掉它。大手眼样本随 ZIP 保存，Git 只记录清单 |

角色可以由同一个人兼任。修改入口按职责定位；复现成果按下一节定位。

## 3. 哪段代码复现哪项成果

| 成果 | 代码与输入 | 原结果/验收点 | 复现边界 |
|---|---|---|---|
| 双次抓放任务仿真 | `src/pick_place_coord/pick_place_coord.py` + `tasks/task_2pairs_table72.json` | 两组抓放的规划与动画 | 这是重新规划，不保证重生成历史冻结 JSON |
| Track C 冻结点流 | `src/pick_place_coord/verify_left_track_c.py` + `trajectories/verified/traj_multi_2grasp_20260804_REALVERIFIED.json` | 0.4°为 1633 帧，0.2°为 3232 帧 | 纯计算核对；当前没有直接加载这份冻结双抓轨迹的完整 GUI 专用入口 |
| MPC 六组仿真 | `src/pick_place_coord/mpc_experiment/run_experiment.py` + `controller.py` + 默认参考轨迹 | 同目录 `results/report.json`、`traces.npz`、`comparison.png` | 动态仿真；批跑会覆盖该工作副本的 results，不是旧真机结果 |
| 收到的另一组 MPC 仿真 | B5 的 `debian/mpc_experiment/mpc_experiment/` | 同目录 results | 报告对应 `controller.py.pre_rt_diagnostics`；旧绝对模型路径/依赖需恢复，不能默认以当前 controller 重跑 |
| 双臂同步展示 | `src/pick_place_coord/dual_arm_demo/demo.py` | 同目录 `dual_arm_simulation.json`、`validation_report.json`，1624 帧 | 可展示；碰撞资格仍为 REJECT，不可直接交真机执行 |
| Track A 最终固定场景 | **B2**：`execute_tabletop_hybrid_trial_reviewfix_field.py` + SDKALIGNED 轨迹 | 五份完整成功 JSON：1 次 5%、4 次 10% | 仅覆盖当时场景及精确文件组合 |
| Track C 双次抓放现场实现 | **B3**：`execute_servo_grasp.py` + `traj_multi_latest.json` | 133 运动点 + 4 夹爪事件；8/4 汇总在 records | 收到源码不等于五轮完整 as-run 绑定；成功原日志不足，LIMITED |
| Servo 20/25ms、VAJ3、30ms 历史 | **B4**：对应 `execute_tabletop_servo_field_*` 及依赖 | 61 份非 MPC 现代 JSON 与逐轮绑定 | 按具体报告选择文件；曾绕过正式资格，VAJ3 历史算法哈希缺口仍在 |
| MPC Shadow / Active 现场实验 | **B5**：两种 field 入口、各自 base、controller、参考轨迹 | 固定 6 Shadow + 4 Active，可复算平均 RMSE 改善约 34.45% | 复算历史数据不等于重跑机器人；旧报告未记录 controller 哈希 |
| 早期 MoveJ / 瓶子示教 | **B6 / B7** | B6：99/153 点等；B7：238/275 点轨迹 | 冻结/运行后快照与原日志分别对待；瓶子五轮完整日志缺失 |
| 视觉与任务转换 | `src/vision/vision_system/replay.py`、`src/robot_mission/tabletop_plan.py`、`preflight.py` | 观测 → 任务/候选 → 预检报告；测试在 `vision/test_vision_system.py` | 支持离线模块验证；不是已跑通的视觉自主抓放闭环 |

详细命令见 [仿真与离线验证](sim/README.md)，历史现场根路径与入口见 [真机执行](robot/README.md)。所有状态以本页、这两份说明、[版本表](docs/VERSIONS.md)、[已知问题](docs/KNOWN_ISSUES.md) 为准；内层原 README 保留当时语境，例如早期 MPC README 的“现场源码未收到”已过时。

## 4. 怎样找到一轮历史实验的完整代码

从交接根目录恢复到尚不存在的目录，例如：

```bash
python3 tools/export_version.py B3 .runtime/track-c-reference
```

Track C 执行器与轨迹在 `.runtime/track-c-reference/debian/trackC_servo_stream/trackC_servo_stream/`。B2/B4/B5/B6/B7 用同一恢复工具，完整路径见真机说明。`debian/` 是收到文件的来源布局，不代表其下所有文件都要连机器人。

**工具仅恢复整个节点，并不自动拼装某一轮运行。** 对于有逐轮记录的 Servo/MPC 等，按 `docs/debian_selection.json` 中该 `report_relative` 的 `run_bindings`，核对 `source_relative`、`restore_name` 和 SHA。必要时在新的复现实验目录复制/重命名正确文件，不改冻结来源；`unbound_local_dependency` 仍是未知，不能拿后来收到的同名文件倒填历史身份。

历史缺陷 D1 有精确对照源码；D2/D3 部分只有报告，不能保证重新触发旧故障。B1 为当前开发交付，H_MAC_FROZEN 与 history 为补充历史。详见版本表。

## 5. 最小核查与继续开发

```bash
# 以下命令均在交接根目录执行，不调用机器人
python3 tools/verify_handoff.py
git status --short
git tag --list 'handoff/*'
python3 tools/offline_checks.py
```

最后一步需要已安装相应仿真依赖。原机已测版本在 `src/requirements/mac-tested.txt`；文件名是来源记录，不是按操作系统分组，也不是新设备安装成功承诺。ABI/ROS 差异见 [环境参考](docs/ENVIRONMENT_REFERENCE.md)。

交接检查读 [verification.json](verification.json) 和 [本次复核记录](evidence/handoff_checks/recheck_20260923.json)。根 `HANDOFF_MANIFEST.json` / `SHA256SUMS` 校验当前交付；子目录的历史 SHA 清单保留原归档范围，部分源码已移到 Git 节点或只留原件索引，不能把它当当前目录的完整清单。

继续开发前确认接手的是当前开发版本还是历史复现实验。不要覆盖 `results/`、records 或冻结轨迹；仿真批量实验可先导出 B1 到 `.runtime/` 下的新工作副本。

## 6. 本次整理改动与仍待补齐

- 重写本页，加入目录职责、成果对应、恢复方法和完整交接工作约定；仿真/真机说明不再按操作系统划分。
- 修正文档中含糊的依赖安装指引、重复标题、逐轮恢复说明；补充冻结 Track C GUI 入口缺口及历史子目录校验范围说明。
- 独立核对原包未发现漏拷；两项旧子目录校验值在收到来源中已陈旧（旧 Track A Worlds 执行器、Track C diag_enable.py），保留原表并记录期望/实收差异。
- 更新文档生成器、交付清单、检查记录、Git 当前节点和同名 ZIP；具体文件差异保存在本次复核记录。
- 本轮不改机器人运行源码、轨迹、模型、原日志或历史节点；不删除工作区文件。旁边的 `robot_handoff 2` 属另一个解压副本，未纳入本次更新。
- 未闭合内容仍包括：Track C/瓶子原实跑日志、部分 MPC/VAJ3 身份、视觉手眼质量门与接口、SAM3 服务环境、双臂碰撞资格。详见已知问题；本轮没有真机、GUI 人工验收或新环境安装验收。

## 7. AGENTS.md 工作约定（全文）

下文与根 [AGENTS.md](AGENTS.md) 同源生成，便于负责人一次读完；自动化代理仍从独立 AGENTS.md 读取约定。

### 交接副本工作约定

用户当前指令优先。本约定用于本交接仓库；仓库根是本文件所在目录。先读 `START_HERE.md`，按任务查 `sim/README.md`、`robot/README.md` 和 `docs/VERSIONS.md`。

#### 工作边界与真源

- 当前开发源码在 `src/`。规划与仿真在 `src/pick_place_coord/`，真机执行在 `src/frame_calibration/robot_side/`。同一执行模块的离线测试仍属于仿真/离线验证。
- 原工作区为 `工作1`；其中的源码目录对应本包 `src/` 下同名目录。原工作区父目录是冻结基线，继续在原机工作时仍遵守原 `工作1/AGENTS.md` 的完整边界。迁移后不要照抄历史文档中的个人绝对路径。
- 输出、候选、测试缓存和恢复目录放在本工作区内，默认使用 `.runtime/`。不要覆盖原件、冻结版本或其他人的未提交改动。
- 标定结论只认 `src/frame_calibration/analysis/calib_common.py` 的 `CONFIRMED_*`。`CONFIRMED_T_SESSIONS_MM` 用于规划换算；`CONFIRMED_T_SESSIONS_V2_MM` 用于 v2 解释 `armGetWorlds`，不可混用。
- `src/XF0112048/` 是只读厂商 URDF/STL。运行发布包只由 `src/releases/make_release.py` 再生；改源码后重建，禁止手改发布副本。交接文档由原工作区 `handoff_tools/` 维护，它与运行发布包是两件事。
- 旧 `workspace/` 与灵巧手不在本次范围内。不要把旧快照里的 `sdk_session.py` 当作现行接口。
- 冻结现场组合在 `handoff/*` Git 标签，通过 `tools/export_version.py` 恢复到新目录。恢复只导出节点，不自动按某一轮日志重命名依赖；还需核对 `SNAPSHOT.json` 和 `docs/debian_selection.json` 的 `run_bindings`。历史版本可能默认开启运动，恢复不等于运行授权。

#### 仿真与真机边界

- 按实际行为区分仿真、只读诊断、修改控制器设置、运动。运行机器的系统名称不决定类别；在机器人主机上跑 PyBullet 仍是仿真。
- 交付的厂商 wheel 仅适用于 CPython 3.10 / Linux x86_64。仿真环境不导入 `pypilot` 作为验收，不将视觉 ARM/Humble 构建痕迹当作同一套 SDK 环境。
- 新开发入口 `ENABLE_REAL_MOTION` 默认关闭。先 dry-run、离线预检与限位检查；`--precheck-only` 是否连接 SDK 以具体实现为准。
- 真机使能、运动、夹爪动作、清故障由现场人员确认。机器人 IP、身份、当前关节和标定有效性必须现场核实；历史 IP 不作为当前事实。
- 新生成或重规划的候选，在真机分段测试前需按执行器一致的密集步长完成全程 PyBullet GUI 回放，并形成绑定轨迹 SHA-256 的 `pybullet_gui_review.v1` PASS。headless 碰撞 PASS 不替代人工 GUI 检查。
- 检查 IK、实际/硬限位、首点跳变、完整场景/持物碰撞、放置与撤退空间。j7 实限 76°，不能为通过预检放宽门限。
- 真机互斥使用 `robot_lock`；结束和异常路径必须恢复保护与全局速度，避免控制器持久设置影响其他脚本。

#### 证据与代码约定

- 新运行日志记录轨迹、执行脚本及动态依赖 SHA-256、参数与身份。失败日志也要保留，报告完成不自动代表所有资格检查通过。
- 真机验收需要归档运行日志；只有 README 汇总属于有限证据。仿真通过、编译通过或包完整均不能升级真机验收状态。
- 不删除 `records/`，不修改历史原日志或冻结源码。脱敏只做副本，使用 `src/frame_calibration/robot_side/scrub_log.py`；明文 token 不入 Git。
- 判定日志用 `$real-robot-run-reviewer`，归档用 `$run-evidence-archivist`，上机前标定核查用 `$frame-calibration-validity-gate`，开变体用 `$motion-variant-scaffolder`，发布核对用 `$robot-release-truth-audit`。这些是可选工作环境中的技能名，不是随包安装的依赖；没有技能时仍执行本约定的检查。
- 中文注释保留 `[待核]`、`[已核实]`、`[重建]` 等置信度标记。安全常量（RANGE_BOX、j7、SEED_DEG、HOME_DEG、T）从共享处导入，不在新脚本复制字面量。
- `OBJECT_POSE`、`GRASP_POSE`、`EE_POSE` 明确区分；已补偿的 SDK `EE_POSE` 不重复补 TCP。
- 未跑相应离线测试，不声称执行器修复完成。已有锚点为 `test_execute_trajectory_options.py`、`test_servo_safety.py`；交接选定检查入口为 `tools/offline_checks.py`。
- 提交信息用简短中文结论句。开发修复与冻结历史整理分开，不做空提交；未经用户要求不自行提交或推送。本包未配置远端。
