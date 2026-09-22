# Servo → Mac Codex 交接包｜2026-09-14 状态快照

这是**离线接续与证据归档包**，不是一键部署包，也不表示已经同步到用户的 Mac、Git 仓库或长期记忆。

## 先读顺序

1. `CODEX_HANDOFF_PROMPT.md`：可直接交给 Mac Codex 的任务说明。
2. `Servo_真机控制与网络长尾排查完整归档_20260914.md`：当前主文档，覆盖完整时间线并修订旧结论。
3. `docs/FILE_MATRIX_AND_GAPS.md`、`docs/SAFETY_AND_EVIDENCE_NOTES.md`：区分收到的原件、重建件、摘要和缺失件。
4. `state/results_20260914_reported_summary.json`、`state/handoff_state.json`：机器可读结果与接续状态。

## 现在的关键信息

9 月 14 日在 `wireless.sta.disabled='1'`、执行 `wifi reload` 并确认 STA 接口消失后，完成 20 ms、25 ms、VAJ3、20 ms 复测。用户贴出的 4 份报告摘要共 2780 次 Pulse：加权均值 **4.0063 ms**，最大 **55.3116 ms**，>20 ms **21 次**、>40 ms **1 次**，4 轮均 **0 realign**。

这支持“禁用路由器上游 STA 后，本批真机发送时序显著改善”；不是唯一驱动机制证明，也不是硬实时/完整安全资格证书。

## 包里确实有与没有的东西

**有：**9 月 9 日原始 `trackA_servo.zip`、9 月 11 日原始 MD、相关终端/历史 ping 日志、9 月 14 日新测试摘要、网络和夹爪变更说明。日志中的显式认证 token 已脱敏。原始 ZIP 保持字节不变。

**有一个明确标注的重建件：**从旧 ZIP 的 candidate 仅修改现场确认的 5 个 `settle_s`，按现场格式序列化，其 SHA-256 与现场 `71bc5148…a1db338` 完全一致。放在 `reconstructed/`，不是声称从 Debian 收到的新原件，不携带新执行授权。

**缺：**最新 20ms/25ms/VAJ3 三套 field 源码、`vaj3_spline.py`、现场实际共用依赖、四份原始结果 JSON、新 GUI review 原件、9 月 14 日同步 ping 原件。不能从本包直接还原完整可运行的最新现场版本。

## Mac 怎么接

把整个文件夹放入当前 Mac 项目的独立交接目录，不覆盖旧源码。历史报告中的工作区线索是 `/Users/xiaobingxin/Desktop/工作/工作1`，但本包没有核验该目录现在是否仍是活动工作区。让 Codex 先读交接 prompt，再列差异、更新文档；缺文件就列缺口，不能凭摘要重写 field 执行器冒充原版。

离线校验包内容：

```bash
python3 tools/verify_handoff.py
```

## 从 Debian 补收真正现场文件

将 `tools/collect_servo_sources.py` 放到**能访问现场目录的 Debian 或 humble 环境**。先停止所有真机执行/写日志任务。以下仅打包本地文件，不连机器人、不加载 SDK、不改源文件、不改网络。

```bash
python3 collect_servo_sources.py \
  --root "$HOME/workspace/trackA_worlds_line/trackA_servo"
```

确认列表后收集：

```bash
python3 collect_servo_sources.py \
  --root "$HOME/workspace/trackA_worlds_line/trackA_servo" \
  --collect --include-pcaps
```

输出在 `~/servo_handoff_exports/Servo_FIELD_SOURCE_*.zip`；`FIELD_MANIFEST.json` 会列出缺失/排除项。PCAP 和原始日志可能含认证内容，只能私下交给自己的 Mac/Codex，不公开提交。位于工程目录外的模型、SDK、本次 ping 家目录文件须另补，不能把这份快照叫作自包含运行环境。

**本次打包没有恢复网络。**最后一次已确认状态仍是 UCI 禁用 STA，是否已被其他人改变未知。恢复请按 `docs/NETWORK_STATE_AND_RESTORE.md`，现场停机并协调其他联网用户后操作。
