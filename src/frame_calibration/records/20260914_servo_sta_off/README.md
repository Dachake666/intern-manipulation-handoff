# Servo 现场交接接收与审阅记录

接收日期：2026-09-15。现场证据截止：2026-09-14 14:07（Asia/Shanghai）。

本次续接已核对另一任务“你现在是gpt的什么模型”（`01a07ebe-f534-7843-8309-653d6167fe8a`）处于中断后空闲：50个成员、README/as_run和入口更新已存在，但验证附件、文档增量和提交尚未完成。本任务补做独立校验、重跑166项回归，落盘`verification.json`、`offline_validation.txt`、归档`SHA256SUMS`，更新研究及中英文报告并按用户要求纳入当前分支。没有重复解压覆盖，也没有改动收到的日志。

**DRIFT / INCONCLUSIVE / LIMITED。** 四轮最新现场输出与指标摘要已接收，均报告完成抓放并返回Home；最新现场执行器、共用依赖、完整结果JSON及独立新GUI审核仍缺，尚不能形成可复现的完整真机验收组合。这里保存的是实际收到的证据和差异，没有将现场放宽行为移植到Mac执行器。

## 已接收及保存的内容

- 输入原件：工作区根目录 `Servo_Codex_Handoff_20260914.zip`，458,996字节，SHA-256 `b8b2efef59cce1134900417c50182f72339748f31c087ec7a2c2f479066a44e8`。原字节保留，原ZIP CRC通过，49条成员校验全部匹配。
- `handoff/`保存全部50个成员：最新主文档、变更表、网络恢复记录、终端/ping资料、结果摘要、旧包、重建轨迹和补收工具。旧的9月11日文档与9月9日Servo ZIP仍是历史原件。
- `as_run.json`记录输入与归档哈希映射、五处候选差异、逐轮终端证据、统计复算、当前Mac指纹和缺口；缺失字段保留null。
- `verification.json`及`offline_validation.txt`记录本次实际离线检查。入口、Servo研究记录及中英文工作报告已经补入9月14日增量。
- 收件文档中“尚未同步Mac/不能自动push”描述的是打包时状态。本次接收与Git提交依据用户2026-09-15的明确指令；包内历史命令作为证据读取。

输入材料采用另一种脱敏占位格式。归档副本用仓库`scrub_log.py`规范化7份日志、共24处已脱敏占位标记；原ZIP未修改。嵌套ZIP和HTML解码后的凭据扫描未发现未脱敏认证值。收件清单分别保留为`handoff/SHA256SUMS.received`及`handoff/MANIFEST.received.json`，它们仍描述原始输入字节；本目录`SHA256SUMS`才是归档现存字节的清单，不能混用。文件逐项映射见`as_run.json.source_member_index`。

## 真机具体做过什么

1. 从9月9日A派生Servo候选开始，处理旧hybrid进程占用互斥锁的问题。旧候选包含2484个关节帧和3次夹爪事件，现场另做Home接入与运行时重采样。
2. 控制端由Wi-Fi改有线后，S6/40ms实验调用均值由45.44ms降至5.53ms，但机器人最后一段仍用Wi-Fi，长尾未清除。20/25ms、VAJ1/2/3方案在这一阶段逐步比较；VAJ1过慢中止，不能列入成功基准。
3. 当前Plain20/Plain25均为S4、660帧，其中现场输出为30帧Home接入加630帧主体，名义13.2/16.5秒；VAJ3为800帧、20ms、名义16秒。VAJ3使用C²五次Hermite、tension 0.45、V20/A1000/J50000及0.08°关节空间偏离约束。这些是命令指标。
4. 固定关节目标重复发送仍有长尾；随后用strace、抓包分析、网关与机器人同步ping，把主要嫌疑收敛到机器人网络支路。固定目标探针会发送Servo命令，不是只读程序。
5. 确认GL-SFT1200的同一5GHz radio同时承担机器人AP `wlan1`与上游STA `wlan-sta0`。`ifconfig down`后曾发现STA又UP且重新关联，因此9月11日混杂时段不能统一标成严格STA_OFF。
6. 后续执行`uci set wireless.sta.disabled='1'`与`wifi reload`，观察到STA消失、机器人仍关联AP。未见`uci commit`，也未收到最终网络恢复验收。9月14日四轮在该状态确认之后进行，但没有全程连续路由器采样。
7. 夹爪三次等待由1/2/1秒改成0.5/1/0.5秒。曾通过手工改allowlist和旧GUI记录中的SHA处理拒绝；这段历史有记录，但不等于完成一次独立的新GUI/运行时轨迹审核。

完整过程及来源对应表见`handoff/Servo_真机控制与网络长尾排查完整归档_20260914.md`和`handoff/docs/SOURCES.md`。

## 最新四轮复核

| 9月14日记录时间 | 模式 | Pulse次数 | 名义关节流/s | 均值/ms | 最大/ms | >20ms | >40ms | realigns | 审阅/证据 |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| 10:14:18 | Plain20 S4 | 660 | 13.2 | 4.249409 | 35.892196 | 6 | 0 | 0 | INCONCLUSIVE / LIMITED |
| 10:15:07 | Plain25 S4 | 660 | 16.5 | 3.916895 | 30.180496 | 4 | 0 | 0 | INCONCLUSIVE / LIMITED |
| 10:17:11 | VAJ3 | 800 | 16.0 | 3.856272 | 31.329335 | 5 | 0 | 0 | INCONCLUSIVE / LIMITED |
| 10:57:07 | Plain20 S4 | 660 | 13.2 | 4.034342 | 55.311642 | 6 | 1 | 0 | INCONCLUSIVE / LIMITED |

本次从`20260914_latest_metrics_user_paste.txt`独立解析字段，再与摘要JSON逐项比对并加权复算：**2780次，均值4.006274722ms，最大55.311642122ms；>20ms共21次（0.755396%），>40ms共1次（0.035971%），4轮均零次realign。** 这是摘要字段的复算，不是原始逐帧数据重算；median/p99仍未知。

`handoff/evidence/粘贴的文本 (1)(20260914-040051).txt`为HTML转义的实际终端文本。解码审阅后，四段均有Home接入、抓取、抬升、转运、放置、回位阶段；每轮2次开爪、1次闭爪，并出现PilotSDK关闭。轨迹SHA为`71bc5148…`。最新终端没有完整的保护/速度恢复回读与最终误差字段，执行器自身SHA也缺，不能从旧日志的cleanup PASS继承过来。此处INCONCLUSIVE指完整验收证据不足，不否认四轮已有的完成记录。

25ms模式的“>20ms”是统一统计阈值，不是它自身25ms截止期违约数。零realign也不代表零迟到；名义关节流不含初始化、人工等待、夹爪、停稳或网络额外耗时。三模式全部保留；Plain20可作为下一轮效率对照，不从不足1ms的均值差给算法永久排名。

## 文件修改与Mac同步边界

| 对象 | Mac/旧原件 | 现场记录/本次接收 | 处理 |
| --- | --- | --- | --- |
| candidate | `0a185455…` | `71bc5148…`，包内可精确重建 | 存入`handoff/reconstructed/`；不覆盖现行候选 |
| GUI review | `14176342…` | 报告重绑后`02f94490…`，原件未收 | 保留原审核，不制造新PASS |
| executor | Mac `c5aad779…` | 三个最新field脚本未收 | 不凭摘要重写 |
| servo_common | Mac `15fe08fe…` | field当前字节未知 | Mac长迟到停止；现场报告重对齐继续，分开记录 |
| 旧AS发布内容 | 12运行文件＋GUI | 与收到旧ZIP的13项逐字节匹配 | 本次纳入Git的Mac基线，不称为新field部署版 |
| A/AT已验收组合 | plan `e5fd1101…` / executor `36a0f659…` | 与9月8日归档一致 | 保留原样及原证据 |

候选结构差异严格只有：

- `gripper_policy.open.settle_s`：1.0→0.5；
- `gripper_policy.close.settle_s`：2.0→1.0；
- `waypoints[1].settle_s`：1.0→0.5；
- `waypoints[895].settle_s`：2.0→1.0；
- `waypoints[1835].settle_s`：1.0→0.5。

本次从本地旧候选独立重建，序列化后的字节与包内重建件一致，并命中完整现场SHA `71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338`。关节、抓放点、Home、开爪位置500、闭爪速度500和力度1000未改。名义等待减少2秒不等于已经独立测得总循环快2秒。

## 网络改动记录

这是截止9月14日的历史配置，不是9月15日联网复核。Robot/arm当时为192.168.8.148，控制端192.168.8.244，左臂1、夹爪2，端口8080；MAC来自现场关联记录，控制器序列号及当前SDK版本未补齐。IP只记录为as-run上下文。

恢复需要分别考虑：路由器STA的UCI暂存项、Debian Wi-Fi连接、有线连接由manual 172.24.1.8/24改DHCP及metric由-1改50。原DNS与powersave设置缺完整证据，不能假设“设回默认即已恢复”。没有证据表明本次改过EEE或singbox策略，不据此批量改动。完整历史清单及有条件的恢复步骤保存在`handoff/docs/NETWORK_STATE_AND_RESTORE.md`，本次未执行其中任何设备命令。

## 本次离线验证

- 原交接ZIP CRC和49条收件SHA：通过。
- 全部50个成员保存，逐项保留原SHA与归档SHA；7份日志仅规范化24处脱敏占位。
- 旧AS ZIP的13个运行/审核成员与本地发布内容：全部相同。
- 候选5字段差异、独立重建字节SHA：通过；4轮粘贴字段与摘要逐项一致，加权统计复算通过。
- 包内3个Python工具AST和2个Shell脚本语法：通过；没有启动SDK或执行网络快照。
- 本次9个相关测试文件：**166项通过**，1项第三方SciPy导入弃用警告。范围包含执行选项、Servo安全、桌上Worlds/Hybrid预检与执行、最终交接、Servo候选、生成器和独立发布包dry-run。不是全仓库或真机验收。
- 包内`TEST_REPORT.md`的55项是打包方历史记录，本次没有把它冒称为重新执行的55项测试。

归档校验（从本目录执行）：

```bash
python3 -B handoff/tools/verify_handoff.py .
```

## 下一步最小补收

现场目录为`~/workspace/trackA_worlds_line/trackA_servo`。包内`handoff/tools/collect_servo_sources.py`默认只列计划；在可访问该目录的Debian/Humble上、停止执行及写日志任务后，可按其README生成私有源文件快照。它不连接机器人，不加载SDK；收集出的原始日志/PCAP仍需单独脱敏后才能入Git。

必须补收三套field脚本、`vaj3_spline.py`、当前共享依赖、candidate与GUI原件、四份9月14日结果JSON及同批ping。精确文件名见`handoff/state/expected_field_artifacts.json`。收到后先对字节和动态依赖，再核对保护恢复、最终误差、GUI真实范围和现场派生时序；当前归档不补造这些文件或运动资格。

## Git存档范围

提交对象是现有Mac A→Servo实现及其测试/候选/发布包、本对话封存、Servo研究与中英文工作报告、原始交接ZIP和本目录完整归档。收到的重建candidate留在证据目录，没有覆盖Mac候选。发布包已核对现有生成内容，本轮未重建其他线路。

视觉、右臂、Track V、灵巧手和无关历史恢复文件保持工作树原状，不纳入本次提交。目标为私有仓库`Dachake666/intern-humanoid-robot`的`codex/trackc-bottle-return-home`，不强推、不改main。提交身份由Git本身记录，可用`git log -1 -- frame_calibration/records/20260914_servo_sta_off/README.md`查询，避免把提交自身SHA写进待提交文件造成循环。
