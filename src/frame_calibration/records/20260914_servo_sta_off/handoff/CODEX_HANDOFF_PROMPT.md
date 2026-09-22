# 给 Mac 端 Codex 的接续任务

请接收这个 Servo 交接包，并把最新现场进度同步到当前 Mac 项目文档。当前任务只包括**离线审阅、证据接收、差异比较、文档更新和允许的离线测试**，不包括机器人运行、路由器操作或远端 Git 推送。

## 必须先读

`00_README_先读.md` → `Servo_真机控制与网络长尾排查完整归档_20260914.md` → `docs/FILE_MATRIX_AND_GAPS.md` → `docs/SAFETY_AND_EVIDENCE_NOTES.md` → `docs/NETWORK_STATE_AND_RESTORE.md` → `state/handoff_state.json`。

然后读取当前仓库实际存在的 `START_HERE.md`、`TRACKC_SERVO_SMOOTHING_RESEARCH_20260910.md`、相关 AGENTS/README 与 `frame_calibration/robot_side/servo_common.py`。这些路径来自历史报告，不保证仍存在；先定位，不另造第二个工作区。历史工作区线索为 `/Users/xiaobingxin/Desktop/工作/工作1`；不得改其父目录冻结基线。

## 当前事实基线

- Debian/Humble：`~/workspace/trackA_worlds_line/trackA_servo`，robot/arm `.148`，local `.244`，左臂 1，夹爪 2，arm port 8080。
- 机器人经路由器 `wlan1` 的 5 GHz AP 接入，不是端到端有线。
- 同一 5 GHz `phy#1` 上的并发对象是 `wlan-sta0`（上游 `Ultirobotics_5G`）和 `wlan1`（机器人 AP）；`wlan0` 是另一张 2.4 GHz AP。
- 简单 `ifconfig wlan-sta0 down` 没有可靠保持；后续实际发现接口 UP、重新关联。不能把期间每轮都标记为严格 STA_OFF。
- 后续执行 `uci set wireless.sta.disabled='1'` 和 `wifi reload`，未见 `uci commit`；用户贴出的状态确认 STA 消失、机器人仍在 AP 上。未有最终恢复验收。
- 9 月 14 日真实轨迹四轮合计 2780 Pulse，加权均值 4.006274722 ms、最大 55.311642122 ms，>20ms 21 次、>40ms 1 次，realigns 总计 0。
- 保留 Plain20、Plain25、VAJ3 全部版本。Plain20 可作为效率导向默认建议；不能删除 25ms，不把 0.3ms 均值差异当成算法优劣证明。
- 夹爪仅 5 个 `settle_s` 从开 1→0.5 秒、闭 2→1 秒；q 轨迹未改。名义等待减少 2 秒，不冒充实测总时长减少 2 秒。

## 文件证据规则

1. `legacy/trackA_servo_20260909_original.zip` 是旧原件；不是最新 field 运行版本。其 `StrictPacer` 长迟到停止，现场报告是 REALIGN_AND_CONTINUE_NEVER_CATCH_UP；不能静默互换。
2. `reconstructed/` 的新 candidate 是离线重建件，字节 SHA 与现场报告相符；不是现场取回证明，也不是新 GUI PASS。
3. `state/*summary.json` 是用户输出转录/提取摘要；绝不重命名成四份 `tabletop_servo_20260914_*.json` 原始日志。
4. 如果没有补充 `Servo_FIELD_SOURCE_*.zip`，先完成文档接收并明确列出缺失源码/原始 JSON；不要用旧版依赖拼一个号称已通过现场验证的执行器。
5. 历史上曾直接修改 GUI review 的轨迹 SHA 并同步 allowlist。保留这段历史，但它**不等于对新时序的独立 GUI 回放/审核**。不得为通过检查继续重绑旧 PASS、放宽限位、跳过资格检查或删失败日志。
6. 源码审阅用文本/AST；不要 import `sdk_session` 或运行 SDK 初始化。即使 stationary probe 也会使能/发送真机命令，不是只读工具。

## 收到现场补充包后怎么做

先校验 ZIP/SHA256SUMS 和 FIELD_MANIFEST；原样落入 `frame_calibration/records/20260914_servo_sta_off/field_snapshot/` 或仓库当前同类归档目录。遇已存在路径，先比对并保留版本，不覆盖。

核对三套 field 脚本、`vaj3_spline.py`、共用依赖及 candidate/review 实际哈希，重算四份报告；检查所有动态导入依赖、schema、首点/Home、限位、保护恢复、失败记录、夹爪等待消费路径。不要仅凭文件名判断版本。

将 field 时序策略与 Mac 旧策略逐项比较，输出 diff 摘要；**先归档差异，不自动把现场放宽行为合并进主执行器。**离线验证/回放也需要现有完整模型和场景条件，不能凭软件 PASS 宣布真机安全。

## 应产出

- 更新主项目入口文档，使其指向本次归档，注明状态截止 2026-09-14。
- 更新 Servo 研究/实验记录：网络拓扑、ifconfig 试验局限、UCI 变更、四轮统计、缺口和恢复待办。
- 独立保存两套旧新代码/报告，不污染已冻结 A/AT/C 基线。
- 给出：实际读过/接收的文件、运行过的离线测试及结果、未运行/未验证项、下一步最小工作清单。
- 不运行 `--run`、不连机器人、不开关接口、不执行 UCI/NM 修改、不自动 commit/push、不删除旧三模式或失败日志。任何这些操作都需新的明确授权。
