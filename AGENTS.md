# 仓库工作约定

本文件适用于本仓库；仓库根为本文件所在目录。先读 `START_HERE.md`，再按职责查 `docs/CONTENTS.md`、`sim/README.md`、`robot/README.md`。任务的明确要求优先于默认约定。

## 源码与版本

- 当前开发源码在 `src/`：规划和仿真在 `src/pick_place_coord/`，真机执行在 `src/frame_calibration/robot_side/`，视觉任务接口在 `src/vision/` 与 `src/robot_mission/`。
- `extensions/debian_vision/` 是相机、检测和手眼工程；路径名称标记来源，运行类别由实际行为决定。
- 输出、候选、缓存、恢复目录放在 `.runtime/`。不覆盖冻结轨迹、原日志、历史源码或其他未提交改动。
- `src/frame_calibration/analysis/calib_common.py` 的 `CONFIRMED_*` 是标定常量依据。`CONFIRMED_T_SESSIONS_MM` 用于规划换算，`CONFIRMED_T_SESSIONS_V2_MM` 用于 v2 解释 `armGetWorlds`，不可混用。历史常量不证明当前工位有效。
- `src/arm_profiles.py` / `arm_profiles.v1.json` 统一臂、夹爪、TCP、Home 与限位；`src/schemas/` 统一输入输出契约。禁止在新脚本复制安全常量字面量。
- `src/XF0112048/` 的厂商 URDF/STL 只读。运行发布包只由 `src/releases/make_release.py` 生成，修改源后重建，不手改发布副本。
- 冻结组合使用 `tools/export_version.py` 导出到新目录；有逐轮绑定的实验使用 `tools/restore_run.py` 装配。按 `SNAPSHOT.json`、`RESTORE_MANIFEST.json` 和 SHA 核对，不能按同名文件替换依赖。
- `B1` 为 `handoff/current` 已登记交付版本的导出，不含 `.git`。完整历史必须保留整个仓库和隐藏 `.git/`；`data/handeye/` 大样本随完整包保存，不依赖 Git 恢复。

## 仿真与真机

- 区分离线计算、连接设备只读、修改控制器设置、运动。`--precheck-only` 是否连接 SDK 以实现为准；MPC Shadow 会发送原参考运动，不是离线模式。
- 新开发的 `ENABLE_REAL_MOTION` 默认关闭。先做离线检查、dry-run 和限位预检；仿真验收不以导入 `pypilot` 为依据。
- 真机前由现场负责人核实机器人身份、网络地址、当前关节/夹爪、标定及工作场景。历史 IP 和位姿只作记录。
- 清故障、使能、夹爪动作和运动必须由现场人员明确确认。历史入口可能默认开启运动，恢复文件不代表允许执行。
- 新生成或重规划的候选，在真机分段测试前必须用执行器一致的密集步长完成 PyBullet 全程 GUI 回放，形成绑定轨迹 SHA 的 `pybullet_gui_review.v1` PASS；headless 碰撞 PASS 不能替代人工 GUI 确认。
- 检查 IK、实际与硬限位、首点跳变、密集完整场景/持物碰撞、开爪包络、放置与撤退空间。j7 实限 76°，不能放宽门限迁就候选。
- 真机互斥使用 `robot_lock`。正常和异常结束均恢复保护与全局速度，并验证收尾，避免持久设置污染后续运行。

## 证据与维护

- `OBJECT_POSE`、`GRASP_POSE`、`EE_POSE` 必须明确；已补偿的 SDK `EE_POSE` 不再补 TCP。单位、四元数顺序、矩阵方向和标定来源必须进入契约。
- 新日志记录执行器、wrapper、轨迹、动态算法和依赖 SHA、参数、设备身份与时间；成功和失败都留痕。日志使用 `src/frame_calibration/robot_side/scrub_log.py` 脱敏副本，明文 token 不入 Git。
- 不删除原始 `records/` 证据，不改历史日志或冻结源码。目录整理使用可恢复节点与路径映射；不重写证据去消除缺口。
- 只有 README 汇总属于 LIMITED；运行完成、仿真通过、编译通过、哈希一致均不能单独授予通用真机资格。成功结论仅绑定具体文件组合、参数与场景。
- 注释保留 `[待核]`、`[已核实]`、`[重建]` 等置信度标记。改执行器后运行对应离线回归；基础锚点包括 `test_execute_trajectory_options.py`、`test_servo_safety.py`，选定套件入口为 `tools/offline_checks.py`。不要广泛自动收集 `test_*.py`，其中存在硬件探针。
- 新增、移动、退役文件先维护 `docs/DELIVERY_SELECTION.json` 的允许清单和恢复映射，再运行 `tools/offline_checks.py --record` 和 `tools/build_package.py --refresh-manifest`，审阅提交后更新 `handoff/current`，使用 `tools/build_package.py --output 路径.zip` 生成新包；只读校验用 `--check`，不手改根哈希清单。
- 验证结果记录命令、解释器/依赖、范围和未覆盖项；通过的选定套件不等于全仓库全绿。流程见 `docs/MAINTENANCE.md`。
- 提交信息使用简短中文结论句，不做空提交；提交和推送遵循任务授权。本仓库不预设远端。

可选审阅技能包括 `real-robot-run-reviewer`、`run-evidence-archivist`、`frame-calibration-validity-gate`、`motion-variant-scaffolder`、`robot-release-truth-audit`。它们不是运行依赖；未安装时仍执行以上检查。
