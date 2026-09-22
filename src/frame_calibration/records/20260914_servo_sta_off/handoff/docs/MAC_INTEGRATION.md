# Mac 集成流程（离线，不覆盖既有工程）

历史 9 月 12 日报告引用了 `/Users/xiaobingxin/Desktop/工作/工作1`、`START_HERE.md`、`TRACKC_SERVO_SMOOTHING_RESEARCH_20260910.md` 与 `frame_calibration/robot_side/servo_common.py`。这是历史线索，不是本次远程读到 Mac 的现状。报告还提示当时现场依赖未全部回收、Mac 与 field pacer 行为不同。来源摘要见 `../evidence/MAC_baseline_context_excerpt.md`。

建议将本包解压到当前活动仓库中的独立归档位置，例如 `frame_calibration/records/20260914_servo_sta_off/handoff/`；目录不存在或与仓库约定不一致时，让 Codex 先检查已有结构，不在父目录冻结基线上改动。

接收分两步：

**现在：知识与证据同步。**校验本包，读取新主文档，把原来“等待三模式复测”的状态更新为“四轮新轨迹报告摘要已收到、原 JSON 和 field 源码待补”。更新入口索引，并保留 25ms 与 VAJ3。

**收到 `Servo_FIELD_SOURCE_*.zip` 后：字节级工程同步。**校验字段与原文件 SHA，先归档到独立 `field_snapshot/`，再与 Mac 的文件做差异分析。候选与 reviewer 绑定、场景模型、入口 Home bridge、pacer 策略、初始化/cleanup、夹爪等待消费路径和动态 import 都要审。不要把全目录覆盖到 `robot_side/`。

Mac 这一步不需要 SDK 或连接真机。直接读取文件、JSON 和 AST 足够。需要离线模型回放时，先核验模型与所需程序确实存在，并说明这不会证明新现场安全资格。不要把用于 Debian/Humble 的 IP/接口名当成 Mac 物理网络配置。

原 ZIP README 中曾提供 Mac 再生/回放命令，留在 `legacy/README_from_original_Servo_package.md`；它们依赖旧目录、源轨迹和模型，本次没有运行，也不保证对现场 wrapper 适用。
