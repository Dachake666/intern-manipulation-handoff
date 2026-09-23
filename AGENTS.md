# 交接副本工作约定

用户当前指令优先。本约定用于本交接仓库；仓库根是本文件所在目录。先读 `START_HERE.md`，按任务查 `sim/README.md`、`robot/README.md` 和 `docs/VERSIONS.md`。

## 工作边界与真源

- 当前开发源码在 `src/`。规划与仿真在 `src/pick_place_coord/`，真机执行在 `src/frame_calibration/robot_side/`。同一执行模块的离线测试仍属于仿真/离线验证。
- 原工作区为 `工作1`；其中的源码目录对应本包 `src/` 下同名目录。原工作区父目录是冻结基线，继续在原机工作时仍遵守原 `工作1/AGENTS.md` 的完整边界。迁移后不要照抄历史文档中的个人绝对路径。
- 输出、候选、测试缓存和恢复目录放在本工作区内，默认使用 `.runtime/`。不要覆盖原件、冻结版本或其他人的未提交改动。
- 标定结论只认 `src/frame_calibration/analysis/calib_common.py` 的 `CONFIRMED_*`。`CONFIRMED_T_SESSIONS_MM` 用于规划换算；`CONFIRMED_T_SESSIONS_V2_MM` 用于 v2 解释 `armGetWorlds`，不可混用。
- `src/XF0112048/` 是只读厂商 URDF/STL。运行发布包只由 `src/releases/make_release.py` 再生；改源码后重建，禁止手改发布副本。交接文档由原工作区 `handoff_tools/` 维护，它与运行发布包是两件事。
- 旧 `workspace/` 与灵巧手不在本次范围内。不要把旧快照里的 `sdk_session.py` 当作现行接口。
- 冻结现场组合在 `handoff/*` Git 标签，通过 `tools/export_version.py` 恢复到新目录。恢复只导出节点，不自动按某一轮日志重命名依赖；还需核对 `SNAPSHOT.json` 和 `docs/debian_selection.json` 的 `run_bindings`。历史版本可能默认开启运动，恢复不等于运行授权。

## 仿真与真机边界

- 按实际行为区分仿真、只读诊断、修改控制器设置、运动。运行机器的系统名称不决定类别；在机器人主机上跑 PyBullet 仍是仿真。
- 交付的厂商 wheel 仅适用于 CPython 3.10 / Linux x86_64。仿真环境不导入 `pypilot` 作为验收，不将视觉 ARM/Humble 构建痕迹当作同一套 SDK 环境。
- 新开发入口 `ENABLE_REAL_MOTION` 默认关闭。先 dry-run、离线预检与限位检查；`--precheck-only` 是否连接 SDK 以具体实现为准。
- 真机使能、运动、夹爪动作、清故障由现场人员确认。机器人 IP、身份、当前关节和标定有效性必须现场核实；历史 IP 不作为当前事实。
- 新生成或重规划的候选，在真机分段测试前需按执行器一致的密集步长完成全程 PyBullet GUI 回放，并形成绑定轨迹 SHA-256 的 `pybullet_gui_review.v1` PASS。headless 碰撞 PASS 不替代人工 GUI 检查。
- 检查 IK、实际/硬限位、首点跳变、完整场景/持物碰撞、放置与撤退空间。j7 实限 76°，不能为通过预检放宽门限。
- 真机互斥使用 `robot_lock`；结束和异常路径必须恢复保护与全局速度，避免控制器持久设置影响其他脚本。

## 证据与代码约定

- 新运行日志记录轨迹、执行脚本及动态依赖 SHA-256、参数与身份。失败日志也要保留，报告完成不自动代表所有资格检查通过。
- 真机验收需要归档运行日志；只有 README 汇总属于有限证据。仿真通过、编译通过或包完整均不能升级真机验收状态。
- 不删除 `records/`，不修改历史原日志或冻结源码。脱敏只做副本，使用 `src/frame_calibration/robot_side/scrub_log.py`；明文 token 不入 Git。
- 判定日志用 `$real-robot-run-reviewer`，归档用 `$run-evidence-archivist`，上机前标定核查用 `$frame-calibration-validity-gate`，开变体用 `$motion-variant-scaffolder`，发布核对用 `$robot-release-truth-audit`。这些是可选工作环境中的技能名，不是随包安装的依赖；没有技能时仍执行本约定的检查。
- 中文注释保留 `[待核]`、`[已核实]`、`[重建]` 等置信度标记。安全常量（RANGE_BOX、j7、SEED_DEG、HOME_DEG、T）从共享处导入，不在新脚本复制字面量。
- `OBJECT_POSE`、`GRASP_POSE`、`EE_POSE` 明确区分；已补偿的 SDK `EE_POSE` 不重复补 TCP。
- 未跑相应离线测试，不声称执行器修复完成。已有锚点为 `test_execute_trajectory_options.py`、`test_servo_safety.py`；交接选定检查入口为 `tools/offline_checks.py`。
- 提交信息用简短中文结论句。开发修复与冻结历史整理分开，不做空提交；未经用户要求不自行提交或推送。本包未配置远端。
