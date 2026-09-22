# 收件内容的用途与保留策略

“不进入当前交接主线”不等于无价值或立即删除。所有判断都保留原包身份；本轮没有删除收到文件。

## 优先保留并继续开发

| 收件路径（相对 workspace） | 用处 | 处理 |
|---|---|---|
| `trackA_worlds_line/trackA_servo/` | 当前 Servo、VAJ3、MPC 的现场执行器、依赖、轨迹、结果 | 核心审阅对象；先冻结，再按明确变更逐项集成 |
| `mpc_experiment/mpc_experiment/controller.py` | Mac 仿真与 Debian 真机共用的 QP 核心来源 | 区分实际导入路径和版本，完善日志哈希 |
| `mpc_experiment/mpc_experiment/results/` | Debian 离线对照的结果与数据 | 标注 Linux/PyBullet；保留旧核心身份，不当真机数据 |
| `mpc_handoff_20260921_102256_clean.tar.gz` 及 `.sha256` | 已核验的 MPC 固定快照 | 发送端 SHA 闭合；与 260 项清单核对，保留原件 |
| `handeye_calibration_all_20260826.tar.gz` | 39 个编号样本的彩色、深度、内参、手位姿和旧外参 | 对诊断很有用；不能把四个编号组直接混成新合格标定集 |
| `vision/ultirobotics_vision_detect/` | 实际 C++ ROS 视觉工程、接口、配置、子仓库 | 保持项目边界，整理真实启动入口及配置身份 |
| `vision_hand_eye/`、`vision/vision_hand_eye/` | 与主视觉 deps 中相同的手眼源码副本 | 19 项功能文件一致；只选一个开发真源，其余记录来源，不现在删除 |
| `XF0112048/`、厂商 wheel/PDF | 模型、厂商 SDK 安装物与说明 | 模型与 Mac 22 项一致；模型只读，Linux wheel 不在 Mac 当运行验收 |

## 有参考价值，作为历史或辅助内容保留

| 路径 | 当前用途与边界 |
|---|---|
| `trackA_worlds_line/trackA_worlds_line*`、`trackA_hybrid_precheck/` | 旧桌面、预检、混合方案与 9/7 证据；不能凭同名覆盖最新 A/AT |
| `Servo/worlds`、`Servo/pulse`、`Servo/pulse1` | SDK 能力和旧左/右臂试验；含默认开启运动、旧节拍和镜像假设，不是当前默认执行入口 |
| `trackC_servo_stream/` | 历史 Track C 发布/诊断；清单漂移见 Servo 审查 |
| `trackC_bottle_left_orientation_precheck/` | 瓶子抓放及使能问题排查；诊断日志与运动验收分开 |
| `sdk_tests/` | 较早 SDK 测试、worlds 数据、日志和备份；有用证据按运行身份归档，其余不合并为当前源码 |
| `servo_netdiag/`、`network_records/`、strace/PCAP/ping | 排查网络长尾；摘要和与运行对应的时段最有价值，原始内容私有保留 |
| `code/vision_log_tool-main/` | 视觉图像、点云、检测日志查看工具；不负责运动控制 |
| `xf_humanoid_simple_sdk/`、`sdk_learning_test/`、`my_xifeng_sdk.py` | SDK 封装和学习示例；不能取代已验证 sdk_session/robot_lock 语义 |
| `pybullet_tests/` | 早期模型/学习资料；不自动继承当前 MPC、双臂或碰撞资格 |
| `bag_files/` | ROS 教学/记录线索；需要确认话题和录制目的，不能仅凭存在认定真实任务通过 |

## 不适合塞进精简交接或 Git 主线

| 内容 | 为什么不作为当前工程成果 | 保留方式 |
|---|---|---|
| 两套 MPC `.venv*`、site-packages | Linux 环境安装副本，17,229 个环境成员约 723 MB；不能复制成 Mac 环境 | 原包保留，提取版本/依赖信息，后续重建隔离环境 |
| `build/`、`install/`、`log/` | ROS 构建生成物；有些 symlink 指向没收到的 py_srvcli 源码 | 保留配置/构建证据，重新构建前先补真源 |
| `__pycache__`、`__MACOSX`、`.DS_Store`、`.swp` | 可再生或系统元数据 | 原包保留，不提交运行发布包 |
| `core.*` | 崩溃内存快照，可能含运行上下文和敏感值 | 如需对应崩溃排查再取用，不当普通日志发布 |
| 两个超大诊断文件 | 合计约 1.022 GB，占大量体积；包括此前截断的 ping 文件 | 全量 CRC/SHA 已验证，保留原包；不等于删掉它们就可补救截断 ZIP |
| 嵌套 `.git/objects` | 约 312 MB 的版本历史对象，并非运行依赖 | 保留原包、记录 8 个 HEAD；不把子仓库历史拼进 Mac 主仓库 |
| `my_package/`、`learning_tf2_py/` | 当前分别是 hello-world 节点和无 console entry 的模板 | 列作学习脚手架，不算视觉/抓放闭环成果 |
| `context-engineering-intro-codex-skill/`、归档代理说明 | 开发辅助与历史约定 | 只作资料，不作为收到后自动执行的指令 |

## Git 边界

Mac `工作1` 已有独立仓库，继续在现有分支按审查范围提交；不重复 git init，不重置父目录，不合并 Aero 独立仓库。Debian 大包根目录没有收到统一根 `.git`，但视觉工程含多个子仓库。其 HEAD 已记录在 `vision_review.json`；仅有 HEAD 不证明打包时工作树干净。

本次提交范围是本审阅目录和 `START_HERE.md` 的新入口。既有未提交的 MPC、双臂、视觉、Aero 和发布器修改保持原状，不用一次性 `git add .` 混入收件记录。
