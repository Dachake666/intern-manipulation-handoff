# 代码地图：职责、状态与修改边界

本页回答三个问题：**这个代码文件做什么、是否属于新任务主流程、接手人是否应该修改它。** 先读 `START_HERE.md`；拿到新抓取/放置点时再读 `NEW_TASK_GUIDE.md`。这里不把“文件存在”解释为“已经进入正式主流程”或“已经完成真机验收”。

## 状态定义

| 状态 | 含义 |
|---|---|
| `ACTIVE` | 当前主流程源码；新任务通常会直接或间接使用 |
| `SUPPORT` | 主流程共享支持代码、校验、配置或诊断；通常复用而不是重写 |
| `OPTIONAL` | 特定路线才需要，例如 MPC、视觉、双臂 |
| `EXPERIMENTAL` | 已实现/已测试部分能力，但不是默认主流程或端到端资格尚不完整 |
| `HISTORICAL` | 历史证据/冻结源码；为复现保留，不作为当前开发副本修改 |
| `TEST` | 自动回归或明确测试；`robot_side/test_*` 中存在硬件探针，不能按文件名批量自动执行 |
| `TOOLING` | 仓库恢复、验证、打包工具 |

## 新点位开发最短阅读集

若场景、机器人、TCP/夹爪定义不变，只更换新的 pick/place 点位，优先看：

1. `tools/new_task_pipeline.py` — 新 XYZ 点位统一入口；调用现有 planner/validator 并生成候选、SHA 报告和后续命令，不自动真机运动。
2. `src/pick_place_coord/pick_place_coord.py` — 任务编排、IK/路径规划与轨迹导出。
3. `src/pick_place_coord/left_arm_ik.py` — 左臂 IK。
4. `src/pick_place_coord/xifeng_pb.py` — PyBullet 机器人/场景与碰撞基础。
5. `src/pick_place_coord/validate_trajectory.py` — 轨迹静态检查。
6. `src/robot_mission/preflight.py` — 候选进入执行前的限位/碰撞/首点等资格检查。
7. `src/frame_calibration/robot_side/tabletop_servo_contract.py`、`execute_tabletop_servo.py` — Servo 候选契约与消费者。
8. `src/arm_profiles.py` / `src/arm_profiles.v1.json` — 臂、夹爪、TCP、Home、限位的共享定义。

**不要从历史 B2/B3/B4/B5 文件反向复制成“新任务源码”。** 历史成果用 `tools/export_version.py` / `tools/restore_run.py` 恢复；新任务在当前 `src/` 开发。

## 顶层共享代码

| 文件 | 状态 | 作用 | 修改建议 |
|---|---|---|---|
| `src/arm_profiles.py` | SUPPORT | 加载/校验统一臂配置，避免各执行器复制限位、TCP、夹爪映射 | 谨慎；改后需跑相关规划/执行回归 |
| `src/execution_authorization.py` | SUPPORT | 将资格/人工批准与具体轨迹身份绑定 | 通常不改 |

## 规划、IK、候选与仿真

| 文件 | 状态 | 作用 | 新任务关系 |
|---|---|---|---|
| `src/pick_place_coord/pick_place_coord.py` | ACTIVE | 单/双抓放任务编排，调用 IK、路径规划、场景检查并导出轨迹 | 核心入口 |
| `src/pick_place_coord/left_arm_ik.py` | ACTIVE | 左臂 FK/IK 与关节映射辅助 | 核心算法 |
| `src/pick_place_coord/xifeng_pb.py` | ACTIVE | 加载 XF0112048 URDF、关节映射、PyBullet 场景/碰撞基础 | 核心模型 |
| `src/pick_place_coord/validate_trajectory.py` | SUPPORT | 对轨迹结构、关节范围/步长等做静态验证 | 新候选应运行 |
| `src/pick_place_coord/gen_tabletop_hybrid_candidate.py` | ACTIVE | 从桌上任务/参考生成 Track A 混合候选并绑定来源 | 特定候选路线 |
| `src/pick_place_coord/gen_taught_tcp_candidate.py` | OPTIONAL | 从示教 TCP/位姿生成候选 | 示教路线 |
| `src/pick_place_coord/gen_bottle_servo_candidate.py` | OPTIONAL | 生成瓶子任务 Servo 候选 | 瓶子路线 |
| `src/pick_place_coord/gen_tabletop_vision_replay_candidate.py` | EXPERIMENTAL | 将视觉回放结果接到桌面候选生成链 | 完整自主抓放尚未验收 |
| `src/pick_place_coord/gen_right_from_left_mirror.py` | EXPERIMENTAL | 左→右臂镜像候选与 FK/IK 检查 | 右臂仍需独立现场资格 |
| `src/pick_place_coord/authorize_servo_candidate.py` | SUPPORT | 将候选资格结果绑定到可执行 Servo 候选 | 执行前支持 |
| `src/pick_place_coord/verify_left_track_c.py` | SUPPORT | 复算/核对冻结 Track C 点流、插值与事件 | 历史/回归检查 |
| `src/pick_place_coord/demo_bottle_trajectory.py` | OPTIONAL | 瓶子轨迹可视化/演示 | 演示 |
| `src/pick_place_coord/demo_tabletop_pick_place_worlds.py` | OPTIONAL | 桌上 Worlds 计划仿真/展示 | 演示与检查 |
| `src/pick_place_coord/demo_tabletop_scene_snapshot.py` | OPTIONAL | 展示/分析桌面场景 snapshot | 场景诊断 |
| `src/pick_place_coord/diagnose_tabletop_wrist_transfer.py` | SUPPORT | 诊断桌面转移段腕部/关节问题 | 异常诊断 |
| `src/pick_place_coord/dual_arm_demo/demo.py` | EXPERIMENTAL | 双臂同步动画与碰撞报告 | 非双臂真机资格 |

### MPC（可选控制层）

| 文件 | 状态 | 作用 |
|---|---|---|
| `src/pick_place_coord/mpc_experiment/controller.py` | OPTIONAL | `JointMPC`/OSQP 控制器，在参考轨迹附近求控制修正 |
| `src/pick_place_coord/mpc_experiment/run_experiment.py` | OPTIONAL | 批量运行 MPC 离线对照实验并产出结果 |
| `src/pick_place_coord/mpc_experiment/test_mpc.py` | TEST | MPC 求解、约束和实验逻辑回归 |

### 规划模块测试

| 文件 | 状态 | 覆盖内容 |
|---|---|---|
| `src/pick_place_coord/test_demo_tabletop_scene_snapshot.py` | TEST | 场景 snapshot 演示/分析 |
| `src/pick_place_coord/test_diagnose_tabletop_wrist_transfer.py` | TEST | 腕部转移诊断 |
| `src/pick_place_coord/test_export_paths.py` | TEST | 规划导出路径与文件行为 |
| `src/pick_place_coord/test_gen_tabletop_hybrid_candidate.py` | TEST | hybrid candidate 生成/拒绝条件 |
| `src/pick_place_coord/test_tabletop_vision_replay_candidate.py` | TEST | 视觉回放候选链 |
| `src/pick_place_coord/test_taught_tcp_candidate.py` | TEST | 示教 TCP 候选 |
| `src/pick_place_coord/dual_arm_demo/test_demo.py` | TEST | 双臂 demo 与碰撞报告 |

## 真机执行与 Servo

| 文件 | 状态 | 作用 |
|---|---|---|
| `src/frame_calibration/robot_side/sdk_session.py` | SUPPORT | PilotSDK 会话初始化、连接/状态/收尾共享逻辑 |
| `src/frame_calibration/robot_side/robot_lock.py` | SUPPORT | 机器人/臂互斥锁，避免多个进程同时控制 |
| `src/frame_calibration/robot_side/servo_common.py` | SUPPORT | Servo 密集插值、节拍/反馈统计等共享逻辑 |
| `src/frame_calibration/robot_side/tabletop_servo_contract.py` | ACTIVE | 桌面 Servo 输入、事件与身份契约 |
| `src/frame_calibration/robot_side/execute_tabletop_servo.py` | ACTIVE | 桌面参考轨迹/Servo 候选的当前开发消费者 |
| `src/frame_calibration/robot_side/execute_servo_grasp.py` | ACTIVE | Track C 关节 Servo 点流 + 夹爪事件执行器 |
| `src/frame_calibration/robot_side/execute_trajectory.py` | ACTIVE | MoveJ 路点执行、日志、暂停/终止基础 |
| `src/frame_calibration/robot_side/execute_tabletop_pick_place_worlds.py` | ACTIVE | 桌面 MoveWorlds 计划与 live 预检 |
| `src/frame_calibration/robot_side/execute_tabletop_hybrid_trial.py` | ACTIVE | Track A 混合执行器开发版 |
| `src/frame_calibration/robot_side/execute_tabletop_hybrid_trial_reviewfix_field.py` | ACTIVE | Track A 固定场景现场版；历史 B2 仍按 SHA 恢复 |
| `src/frame_calibration/robot_side/execute_worlds_servo_grasp.py` | OPTIONAL | Worlds/Servo 抓取实验执行入口 |
| `src/frame_calibration/robot_side/execute_worlds_10x.py` | OPTIONAL | 多目标 Worlds 真机采样/一致性实验 |
| `src/frame_calibration/robot_side/precheck_tabletop_hybrid.py` | SUPPORT | Track A 连接 SDK 的只读预检 |
| `src/frame_calibration/robot_side/capture_tabletop_precheck.py` | SUPPORT | 采集桌面任务 live precheck/当前状态 |
| `src/frame_calibration/robot_side/precheck_dual_arm.py` | EXPERIMENTAL | 双臂 live 预检 |
| `src/frame_calibration/robot_side/probe_sdk.py` | SUPPORT | 核对当前 pypilot SDK 能力，不执行运动 |
| `src/frame_calibration/robot_side/soft_stop_watchdog.py` | SUPPORT | 软件停止状态监视/保护辅助 |
| `src/frame_calibration/robot_side/scrub_log.py` | SUPPORT | 对日志副本做 token/敏感字段脱敏 |

### robot_side 测试与硬件探针

这些文件为回归/诊断保留，**不能仅因为名称以 `test_` 开头就全部自动执行**。

| 文件 | 状态 | 覆盖/风险 |
|---|---|---|
| `src/frame_calibration/robot_side/test_capture_tabletop_precheck.py` | TEST | precheck 采集逻辑 |
| `src/frame_calibration/robot_side/test_execute_tabletop_hybrid_trial.py` | TEST | hybrid 执行器离线逻辑 |
| `src/frame_calibration/robot_side/test_execute_tabletop_pick_place_worlds.py` | TEST | Worlds 执行器逻辑 |
| `src/frame_calibration/robot_side/test_execute_trajectory_options.py` | TEST | MoveJ 执行参数/选项 |
| `src/frame_calibration/robot_side/test_gripper_right.py` | TEST | **硬件探针：可实际操作右夹爪，禁止自动批跑** |
| `src/frame_calibration/robot_side/test_move_worlds_once.py` | TEST | **硬件探针：单次 Worlds 行为，人工使用** |
| `src/frame_calibration/robot_side/test_precheck_dual_arm.py` | TEST | 双臂预检逻辑 |
| `src/frame_calibration/robot_side/test_precheck_tabletop_hybrid.py` | TEST | Track A 预检逻辑 |
| `src/frame_calibration/robot_side/test_servo_safety.py` | TEST | Servo 安全门、拒绝条件与收尾 |
| `src/frame_calibration/robot_side/test_tabletop_servo.py` | TEST | 桌面 Servo 契约/执行逻辑 |
| `src/frame_calibration/robot_side/test_tracka_final_handoff.py` | TEST | Track A 最终 handoff 回归 |
| `src/frame_calibration/robot_side/test_worlds_10x.py` | TEST | Worlds 10-target 采样逻辑 |
| `src/frame_calibration/robot_side/test_worlds_servo_minimal.py` | TEST | Worlds/Servo 最小实验逻辑 |

## 标定与坐标分析

| 文件 | 状态 | 作用 |
|---|---|---|
| `src/frame_calibration/analysis/calib_common.py` | SUPPORT | 标定常量、关节映射和会话变换共享定义 |
| `src/frame_calibration/analysis/fit_eye_to_hand.py` | OPTIONAL | 手眼数据拟合/误差分析 |
| `src/frame_calibration/analysis/verify_worlds_record.py` | SUPPORT | Worlds 采样记录一致性/漂移检查 |
| `src/frame_calibration/analysis/test_fit_eye_to_hand.py` | TEST | 手眼拟合回归 |
| `src/frame_calibration/analysis/test_verify_worlds_record.py` | TEST | Worlds 记录验证回归 |
| `src/frame_calibration/records/20260907_trackA_hybrid_reference/artifacts/execute_tabletop_hybrid_trial.py` | HISTORICAL | 2026-09-07 Track A 参考运行绑定源码快照；为证据保留，不作为当前源码修改 |

## 任务契约、场景与任务编排

| 文件 | 状态 | 作用 |
|---|---|---|
| `src/robot_mission/contracts.py` | ACTIVE | JSON schema/语义校验、SHA 与跨文件链接检查 |
| `src/robot_mission/create_task.py` | ACTIVE | 从输入/观测创建标准任务文档 |
| `src/robot_mission/task_adapter.py` | ACTIVE | observation/task 与 legacy planner 格式互转、坐标转换 |
| `src/robot_mission/grasp_pose.py` | ACTIVE | 位姿/四元数/TCP 变换与抓取姿态资格 |
| `src/robot_mission/tabletop_plan.py` | ACTIVE | 将桌面任务展开成 pick/hover/descend/place/gripper 阶段计划 |
| `src/robot_mission/preflight.py` | ACTIVE | 密集限位、首点、PyBullet 碰撞等执行前资格报告 |
| `src/robot_mission/qualify_candidate.py` | ACTIVE | 汇总候选 precheck/资格并形成绑定结论 |
| `src/robot_mission/mission_runner.py` | ACTIVE | 串行编排已批准计划与执行器；可进入真机执行 |
| `src/robot_mission/approve.py` | SUPPORT | 对候选/资格结果执行显式批准绑定 |
| `src/robot_mission/arm_selector.py` | OPTIONAL | 按资格结果对候选机械臂排序/选择 |
| `src/robot_mission/build_trajectory_v2.py` | EXPERIMENTAL | 构建 v2 轨迹/接口适配；非默认新任务入口 |
| `src/robot_mission/scene_guard.py` | SUPPORT | 比较执行前后观测，判断场景变化 |
| `src/robot_mission/tabletop_scene_snapshot.py` | SUPPORT | 合并/校验桌面 snapshot 并计算场景变化指标 |
| `src/robot_mission/placement_clearance.py` | SUPPORT | 计算放置/容器几何间隙 |
| `src/robot_mission/visual_acceptance.py` | EXPERIMENTAL | 根据前后视觉观测验证抓取跟随和放置结果；完整视觉闭环尚未验收 |
| `src/robot_mission/obstacle_scene_support.py` | EXPERIMENTAL | 桌面计划→障碍场景/容器/物体几何与执行序列门；**待整合，不视为主流程已接入** |
| `src/robot_mission/__init__.py` | SUPPORT | Python package 标记 |

### robot_mission 测试

| 文件 | 状态 | 覆盖内容 |
|---|---|---|
| `src/robot_mission/test_mission.py` | TEST | 契约、臂选择、场景与任务综合逻辑 |
| `src/robot_mission/test_obstacle_scene_support.py` | TEST | 障碍场景扩展 |
| `src/robot_mission/test_placement_clearance.py` | TEST | 放置间隙与工具代理几何 |
| `src/robot_mission/test_preflight_limits.py` | TEST | PyBullet 规划/预检限位 |
| `src/robot_mission/test_tabletop_plan.py` | TEST | 坐标变换、adapter、桌面计划 |
| `src/robot_mission/test_tabletop_scene_snapshot.py` | TEST | 场景 snapshot 语义 |

## 视觉 Python 层

| 文件 | 状态 | 作用 |
|---|---|---|
| `src/vision/setup.py` | SUPPORT | `vision_system` Python 包安装元数据 |
| `src/vision/vision_system/detector.py` | EXPERIMENTAL | RGB-D 颜色/盒体检测、连通域、反投影等离线检测逻辑 |
| `src/vision/vision_system/evidence.py` | SUPPORT | 保存/加载 RGB-D frame bundle 与哈希证据 |
| `src/vision/vision_system/replay.py` | EXPERIMENTAL | 对历史 RGB-D frame 执行离线 replay/检测 |
| `src/vision/vision_system/rgbd_observer_node.py` | OPTIONAL | ROS RGB-D observer 节点入口 |
| `src/vision/vision_system/temporal.py` | SUPPORT | 多帧稳定性/姿态波动 temporal gate |
| `src/vision/vision_system/__init__.py` | SUPPORT | Python package 标记 |
| `src/vision/test_vision_system.py` | TEST | 视觉检测、证据与 temporal gate 回归 |

完整相机驱动、检测和手眼 ROS/C++ 工程位于 `extensions/debian_vision/`，属于 **OPTIONAL 设备扩展**；固定点位规划不需要阅读该目录。

## Release 与仓库工具

| 文件 | 状态 | 作用 |
|---|---|---|
| `src/releases/make_release.py` | TOOLING | 从当前源码生成明确模式的运行发布包 |
| `src/releases/test_make_release_dual.py` | TEST | 双臂 release 生成回归 |
| `src/releases/test_make_release_hybrid.py` | TEST | hybrid release 生成回归 |
| `src/releases/test_release_pruning.py` | TEST | release 文件裁剪/白名单回归 |
| `tools/new_task_pipeline.py` | TOOLING | 新 XYZ 点位统一编排：输入检查→现有 planner→静态验证→SHA/provenance→生成后续 dry-run/precheck 命令；自身不授权/启动真机 |
| `tools/verify_handoff.py` | TOOLING | 按根 manifest/SHA 校验完整交付，不导入 SDK |
| `tools/offline_checks.py` | TOOLING | 运行明确列出的离线回归并可记录结果；不广泛发现所有 `test_*` |
| `tools/export_version.py` | TOOLING | 按 B1–B7/D/history 节点导出历史版本到新目录，不执行代码 |
| `tools/restore_run.py` | TOOLING | 按历史运行报告的 SHA 绑定恢复执行器/轨迹/依赖 |
| `tools/build_package.py` | TOOLING | 检查白名单/manifest/Git 状态并生成完整交接 ZIP |
| `tools/check_history.py` | TOOLING | 核对历史节点、退役文件和逐轮 run bindings |
| `tools/test_package_tools.py` | TEST | export/restore/build 等交接工具回归 |
| `tools/test_vision_model_paths.py` | TEST | 独立编译/测试视觉 C++ 模型路径解析块；不代表完整 ROS/ONNX 验收 |

## 什么不建议删除

以下内容可能看起来“重复/旧”，但承担可追溯性，不按普通源码冗余删除：

- `src/frame_calibration/records/`：历史运行绑定的源码/输入证据。
- `evidence/`：验证报告、原日志或选定证据。
- `data/handeye/`：历史手眼样本；固定点位规划不依赖，但完整项目追溯依赖。
- `.git/`：B2–B7、D、history、PRE_PRUNE 等历史节点与 `handoff/current`。
- `extensions/debian_vision/`：固定点位规划可忽略，但视觉/手眼路线需要。

若以后需要轻量包，应该新增“core/source-only”发布模式，把证据/大数据作为独立归档，而不是从完整 handoff 中直接删除。

## 维护规则

新增、移动或退役代码时，同时更新本页和 `docs/CONTENTS.md`；新增交付文件还要更新 `docs/DELIVERY_SELECTION.json`。文件状态从 `EXPERIMENTAL` 升为 `ACTIVE` 时，应有对应接口、回归和适用范围证据；仅仅“代码能运行”不足以升级状态。
