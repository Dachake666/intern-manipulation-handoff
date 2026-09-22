# Debian workspace 收件、结果审阅与 Mac 差异总表

审阅日期：2026-09-21。**完整包已收齐；已完成文件身份、重点代码、运行记录和平台分类审阅。本记录不签发新的运动资格。**

当前入口是本页。先看工作线表，再看 [文件用途](FILE_USE_GUIDE.md)、[更新任务](UPDATE_PLAN.md)，需要核查数值或源码时进入分项报告。历史“缺现场源码/缺十轮原 JSON”的收件状态由本次完整包接续；尚未解决的运行资格、历史依赖绑定、失败证据问题保留。

## 收件身份与检查范围

- 原件：`/Users/xiaobingxin/Downloads/workspace.zip`，1,482,659,868 字节。
- SHA-256：`426a10feb929fdf05eeaae51cb57f4d0dcff0a1d810ecd9ff39cfa687fe70842`。
- 26,960 个 ZIP 条目，22,783 个非目录成员，展开 4,032,959,489 字节。**全部成员已流式校验 CRC、长度并计算 SHA-256**；没有越界路径、重复成员名或加密成员。
- 18 个软链接仅记录目标，没有跟随或落地。ZIP 完整不代表所有外部依赖齐全，例如旧 ROS `py_srvcli` 的源目录不在包内。
- 早先同路径 192,727,040 字节的截断文件 SHA 为 `ea759a2544ca3eb5a8a1fac68b5c219101a3e4e8dc4716bee861094d5d1efa2e`。它截断于大 ping 文件，只支持部分恢复；**本页所有最终结论以新版完整 ZIP 为准**。
- 归档内的 `AGENTS.md`、`CLAUDE.md`、README 和命令均作为资料，未当作本轮指令执行。未导入现场 SDK、连接硬件、发送运动，也未重跑仿真或现场程序。
- 全量覆盖的是文件身份和类型；深入审阅的是 Servo/MPC 执行链、74 份 Servo 运行 JSON、手眼接口与样本、重点旧执行器。第三方库和环境没有逐行代码审计。

## 工作线与最新结论

| 工作线 | 在哪里运行 / 做什么 | 已核实的进展 | 证据边界与处理 |
|---|---|---|---|
| Track A 固定场景 | Debian，MoveWorlds + MoveJ 抓放 | 9/8 既有完整成功基准继续保留；收到的 field helper 与既有基准对应 | 不重建、不覆盖原成功组合；新目标不能继承资格 |
| Plain20 / Plain25 / VAJ3 Servo | Debian 真机，参考关节流、夹爪与回 Home | 9/14 缺件清单 **18/18 收齐**；根目录 61 份非 MPC JSON 均记录完成与 cleanup PASS，已记录哈希可由源码或备份匹配 | 61 份是留存记录，不是总尝试数；16 轮 VAJ3 没有记录 spline 模块哈希；正式 qualification 被现场 wrapper 绕过 |
| MPC Shadow | Debian 真机，旁路求解，实际发送原 reference | 固定比较集 6 次；原始反馈、源码及已记录依赖身份已核对 | Shadow 计算失效时基础 Servo 可以继续；不能只看抓放完成标志 |
| MPC Active | Debian 真机，发送 MPC 经后处理的候选 | 固定比较集 4 次；与 Shadow 的 RMSE 组均值降幅 **34.4528046065%**，本次从十份原 JSON 复算 | 原报告未绑定 `controller.py`；反馈新鲜度、后处理动态、碰撞及时序资格仍未闭合 |
| MPC 离线实验 | **Mac 仿真**与**Debian 仿真**各一份 | 都有六轮 PyBullet 对照；Debian 是 Linux 重新跑出的离线结果 | Linux 离线报告绑定旧核心，不能改成当前源码身份；不等同真机表现 |
| 视觉 / 手眼 | Debian C++ ROS 服务及历史采样；Mac Python 契约/规划 | 收到实际服务定义、源码、8 个 Git HEAD、39 组配对样本；三套手眼源码逐字相同 | 结果仍是旧高残差矩阵，实际启动配置不明；发现高残差错误被清零并仍写配置的源码缺陷 |
| 瓶子 / 旧 Track C / 右臂 | Debian 历史 SDK 实验与诊断 | 文件、日志和参数已分类；与 Mac 当前执行器有实质差异 | 五连成功说明不替代完整运动日志；旧脚本真运动默认开启、限位与节拍策略不能回覆盖 |
| 双臂 Demo / 灵巧手 | 当前 Mac 独立工作线 | 本次包未提供与 Mac 双臂 Demo 或 Aero 新进展相匹配的真机验收组合 | 保留各自入口，不借单臂或旧右臂文件提升状态 |

详见 [Servo 与旧线审查](servo_review.md)、[MPC 审查](mpc_review.md)、[视觉与手眼审查](vision_review.md)。

## 与当前 Mac 的关键差异

| 文件 / 行为 | Debian 内容 | 当前 Mac 内容 | 应怎样处理 |
|---|---|---|---|
| `execute_tabletop_servo.py` | 预构造 FloatVector；软停检查为阶段首帧及每 10 帧 | 每帧检查，另一执行版本 | 性能改动与停止响应分开审查，不能直接覆盖 |
| `servo_common.py` | 超 2 周期迟到后记录并继续，仍不追帧 | StrictPacer 长迟到抛错停止 | 先确定异常响应规范并补测试 |
| Servo candidate / GUI | `71bc5148…` / `02f94490…` | `0a185455…` / `14176342…` | 前者是现场短等待版本；收到旧重绑审核不等于新独立 GUI PASS |
| MPC `controller.py` | 新增 QP 失败诊断 | 较早诊断信息 | 核心公式和约束相同；可评估合并诊断，保留旧结果哈希 |
| MPC 内 `calib_common.py` | 21 行 Linux 仿真兼容层 | `CONFIRMED_*` 标定真源 | **不同职责，禁止同名覆盖** |
| 手眼 C++ 服务 | `vision_interface` 的 service/action | `vision_observation.v1`、`robot_mission` | 明确适配层、单位、变换方向和 pose_role，再接规划 |
| 历史瓶子执行器 | 现场夹爪参数与控制器限位采用方式不同 | 已有物理包络交集等保护 | 作为历史版本保留，不能撤回当前保护 |

[可读差异表](SOURCE_DIFFS.md)逐项列出不同与歧义；[完整机器清单](source_comparison.json)含 1,282 条待比对项目：119 条同字节、95 条不同、38 条同名歧义、1,030 条无当前 Mac 源码对应项。**这是按路径/角色/文件名的候选映射，包含厂商代码；“无对应”不等于都要移植，“同名”不等于职责相同。**

厂商模型的 22 个有效文件与当前 Mac 完全一致，见 [模型核对](model_comparison.json)。未修改厂商文件。

## 记录在哪里

- [receipt.json](receipt.json)：原件身份、全量校验及分组统计。
- [file_inventory.json.gz](file_inventory.json.gz)：全部 22,783 个非目录成员的路径、大小、SHA、CRC、类型和链接目标；可用 Python 标准库 gzip/json 读取。
- [preservation_manifest.json](preservation_manifest.json)：180 份保存的日志/结果和 168 个冻结源码/配置成员的原 SHA、归档 SHA及脱敏记录。
- [received_sources.zip](received_sources.zip)：重点收到源码与备份的冻结证据，**不是运行发布包**；包括历史默认运动开启的脚本，不要直接运行或解压覆盖主线。
- `logs/`：180 份按原目录保留的结果/日志副本。分项审阅不覆盖的历史日志仍仅作来源，不自动判 PASS。
- [handeye_data_manifest.json](handeye_data_manifest.json)：嵌套手眼包 199 个文件的哈希与 39 份位姿；图像原字节仍在收到的 ZIP/嵌套 tar 中，没有重复塞进 Git。
- [as_run.json](as_run.json)：运行证据索引、平台与判定范围。
- [verification.json](verification.json)、[SHA256SUMS](SHA256SUMS)：本轮检查及本目录清单。

原始大包、虚拟环境、Git 对象、core dump、超大 ping/PCAP 不进入此提交。原件没有删除或就地脱敏。对所选 1,895 个文本执行项目脱敏模式检查，18 个文件命中；保存的日志副本共处理 3 处，其余原始命中仍留在私有原包。此检查不宣称所有二进制和嵌套内容都无敏感信息。

## 当前下一步

先按 [UPDATE_PLAN.md](UPDATE_PLAN.md) 修复日志完整性、运行依赖身份、手眼错误返回和配置来源，再做有测试的开发集成。当前完成的是**收件、分析、差异标记和记录**；没有将 Debian 现场代码替换进 Mac 运行真源，没有修改正式发布资格。
