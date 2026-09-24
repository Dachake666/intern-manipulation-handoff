# 目录地图与阅读顺序

先读 `START_HERE.md` 选择成果，再按下面的职责查看代码。`src/` 是可维护源码；冻结实机代码按 [版本表](VERSIONS.md) 恢复。默认目录不罗列全部旧实验。

## 仿真、规划与数据契约

| 路径 | 实现内容 | 负责人改动后的检查 |
|---|---|---|
| `src/pick_place_coord/pick_place_coord.py` | 单次/双次抓放任务编排、路径生成、仿真 | 对应 task、轨迹格式、限位、场景和密集 GUI 回放 |
| `left_arm_ik.py`、`xifeng_pb.py` | 逆解与 PyBullet 模型加载 | 关节映射、末端定义、FK/IK 残差 |
| `validate_trajectory.py`、`verify_left_track_c.py` | 轨迹静态校验、冻结双抓点流对照 | SHA、点数、插值步长和夹爪事件 |
| `gen_tabletop_hybrid_candidate.py` | 桌上混合候选、SDK 反馈衔接与回放 | 候选/父轨迹身份、首点、密集检查、对应回归 |
| `gen_taught_tcp_candidate.py`、`gen_bottle_servo_candidate.py` | 示教/瓶子候选生成 | 姿态语义、工具与携物几何、任务资格 |
| `gen_right_from_left_mirror.py` | 右臂候选 FK/IK | 右臂配置和实际限位仍需独立验证 |
| `demo_tabletop_scene_snapshot.py`、`gen_tabletop_vision_replay_candidate.py` | 场景观测展示、视觉候选链路 | snapshot/任务契约与拒绝条件 |
| `src/pick_place_coord/tasks/` | 任务输入 | 坐标系、物体尺寸、抓放目标和场景 |
| `src/pick_place_coord/trajectories/` | 核心参考、候选及测试夹具 | 不覆盖冻结输入；文件身份与适用场景绑定 |
| `src/pick_place_coord/mpc_experiment/` | MPC 控制器、实验组织、参考结果与回归 | `test_mpc.py`、相同参考对照、求解失败和事件边界 |
| `src/pick_place_coord/dual_arm_demo/` | 双臂联合动画和碰撞报告 | `test_demo.py`；碰撞 REJECT 不因动画流畅而改变 |
| `src/vision/vision_system/` | RGB-D 回放与观测 | `src/vision/test_vision_system.py`，同步与质量门 |
| `src/robot_mission/{contracts,grasp_pose,task_adapter,tabletop_plan,preflight}.py` | 统一契约、坐标/TCP、任务转换与资格预检 | 对应接口、几何、限位、场景回归 |

上表中省略前缀的规划脚本均位于 `src/pick_place_coord/`。`robot_mission` 其他辅助模块用于场景、批准和候选绑定；`obstacle_scene_support.py` 是待整合扩展，不能仅凭单独存在推定已接入主流程。

## 真机与设备

| 路径 | 实现内容 | 维护边界 |
|---|---|---|
| `src/frame_calibration/robot_side/execute_trajectory.py` | MoveJ 路点回放、日志、暂停/终止 | Track B 基础；不与 Servo 语义混用 |
| `execute_servo_grasp.py` | 关节 Servo、夹爪事件和安全收尾 | Track C 及任务执行消费者 |
| `execute_tabletop_pick_place_worlds.py` | 桌上 MoveWorlds 计划与 live 预检 | 也是上层计划校验的共享依赖 |
| `execute_tabletop_hybrid_trial_reviewfix_field.py` | 固定场景 Track A 最终现场实现 | 冻结身份；修改作为新版本验证 |
| `execute_tabletop_hybrid_trial.py` | 开发混合执行器与候选回归依赖 | 与现场文件同名相似不代表可互删 |
| `execute_tabletop_servo.py`、`tabletop_servo_contract.py` | Track A 参考转 Servo 候选消费者/契约 | 不能替代 B4/B5 各轮 field 组合 |
| `sdk_session.py`、`robot_lock.py`、`servo_common.py` | SDK 会话、互斥、节拍、反馈统计 | 保护与全局速度恢复；历史节点各用自己的依赖 |
| `precheck_tabletop_hybrid.py`、`capture_tabletop_precheck.py`、`precheck_dual_arm.py` | 连接 SDK 的只读预检/反馈采集 | 不属于纯离线检查；不能由只读 PASS 推断运动许可 |
| `probe_sdk.py`、`test_gripper_right.py` | SDK 接口探测、夹爪能力检查 | 后者可实际开合夹爪，不能被当成普通单测自动执行 |
| `src/robot_mission/mission_runner.py`、`src/execution_authorization.py` | 真机任务串行编排与批准绑定 | 会调用运动执行器 |
| `extensions/debian_vision/` | 相机、检测、手眼 ROS/C++ 工程 | 设备链路；接口、环境和标定质量问题见已知问题 |
| `sdk/` | 厂商 SDK 安装物和手册 | 版本/架构核对，安装不授权运动 |

上表中省略前缀的执行脚本均位于 `src/frame_calibration/robot_side/`。名称为 `test_` 的文件既有离线回归也有硬件探针，使用 `tools/offline_checks.py` 的明确列表。

## 共享资源、历史与工具

| 路径 | 用途 |
|---|---|
| `src/arm_profiles.py`、`src/arm_profiles.v1.json` | 唯一臂/夹爪/限位/TCP 配置入口 |
| `src/frame_calibration/analysis/calib_common.py` | 标定常量与关节映射；待核标记保留 |
| `analysis/verify_worlds_record.py`、`analysis/fit_eye_to_hand.py` | 会话漂移门、手眼计算；路径前缀为 `src/frame_calibration/` |
| `src/frame_calibration/data/` | 当前检查依赖的标定参考/候选输入，文件日期不等于现场有效 |
| `src/schemas/` | JSON 数据契约；不能用宽松 schema 替代数值质量门 |
| `src/XF0112048/` | 单份只读机器人 URDF/STL |
| `src/releases/make_release.py` | 从源码生成所选运行发布包 |
| `src/frame_calibration/records/`、`evidence/` | 选定原日志、固定输入与验证报告；历史背景从版本节点恢复 |
| `data/handeye/` | 39 组历史手眼样本，纳入 Git 并随完整包保存 |
| `tools/verify_handoff.py` | 对根 manifest 逐文件校验 |
| `tools/offline_checks.py` | 选定离线回归；`--record` 写验证结果与源身份 |
| `tools/export_version.py` | 导出整个节点，不执行代码，导出副本无 Git |
| `tools/restore_run.py` | 按某一报告的 SHA 绑定装配代码与依赖 |
| `tools/build_package.py` | 白名单校验、清单刷新、完整 Git 历史打包 |
| `tools/check_history.py` | 核对历史节点、退役原字节和逐轮装配，保留身份缺口 |
| `tools/test_vision_model_paths.py` | 独立编译/测试C++模型路径解析块；不代表完整ROS或推理验收 |
| `.git/` | 当前开发与冻结节点；完整迁移时必须保留 |

## 文档索引

| 文档 | 阅读目的 |
|---|---|
| `START_HERE.md`、`AGENTS.md` | 职责入口与仓库约定 |
| `NEW_TASK_GUIDE.md`、`config/site_config.example`、`templates/task_template.json` | 新点位继续开发、现场变量与任务格式模板 |
| `sim/README.md`、`robot/README.md` | 仿真命令、真机成果恢复与现场顺序 |
| `docs/CONTENTS.md` | 本目录地图 |
| `docs/CODE_MAP.md` | 逐个代码文件的职责、ACTIVE/SUPPORT/OPTIONAL/EXPERIMENTAL/HISTORICAL/TEST/TOOLING 状态与修改边界 |
| `docs/VERSIONS.md` | 版本节点、逐轮恢复与身份边界 |
| `docs/ENVIRONMENT_REFERENCE.md` | 新设备环境和安装边界 |
| `docs/KNOWN_ISSUES.md` | 运动、标定、视觉与环境待解决项 |
| `docs/MAINTENANCE.md` | 新增代码/实验、回归、版本与重新打包 |
| `src/frame_calibration/README.md`、`UVW_TO_QUATERNION.md` | 标定/执行模块与坐标约定 |
| `src/PilotSDK_enable_varget_8080_debug_20260901.md` | SDK 监听端口与使能状态的排障边界 |
| `src/pick_place_coord/mpc_experiment/README.md`、`dual_arm_demo/README.md` | 算法/仿真模块入口 |
| `src/pick_place_coord/trajectories/{candidates,verified}/README.md` | 候选与冻结轨迹的身份规则 |
| `src/requirements/README.md`、`src/vision/README.md` | 依赖清单与视觉接口 |
| `extensions/debian_vision/` 内工程 README | 各 C++/ROS 模块的构建与设备职责 |

机器清单：`docs/DELIVERY_SELECTION.json` 控制默认交付文件；`docs/SOURCE_MAP.json` 记录来源；`docs/debian_selection.json` 保存节点/逐轮绑定；`docs/vision_selection.json` 保存视觉源与外部依赖索引；`HANDOFF_MANIFEST.json` / `SHA256SUMS` 校验当前负载，`verification.json` 记录选定验证范围。
