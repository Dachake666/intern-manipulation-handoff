# 可恢复版本

版本按职责与证据划分。当前源码用于开发，历史组合由不可变节点恢复；提交时间不等于实验时间。目录名或 REALVERIFIED 文件名不能替代运行报告与精确 SHA。

| 节点 | 内容 | Git标签 |
|---|---|---|
| B1 | 当前开发基础：仿真、接口、真机消费者 | `handoff/current` |
| B2 | Track A 最终固定场景 | `handoff/b2` |
| B3 | Track C 133点双次抓放实机成果：原始说明记载连续5轮成功，待补完整逐轮日志 | `handoff/b3` |
| B4 | Servo 20/25ms、VAJ3及30ms历史 | `handoff/b4` |
| B5 | MPC 实机成果与离线参考：已有6次Shadow、4次Active报告，待补controller版本SHA | `handoff/b5` |
| B6 | Track B 99/153点基础 | `handoff/b6` |
| B7 | 瓶子示教与Home收尾 | `handoff/b7` |
| D1 | Servo长迟到缺陷组合 | `handoff/d1` |
| D2 | MPC早期失效证据 | `handoff/d2` |
| D3 | MoveJ限位/跳点缺陷证据 | `handoff/d3` |
| history | 补充运动与诊断历史 | `handoff/history` |
| H_MAC_FROZEN | 既有冻结开发源码 | `handoff/h-mac-frozen` |
| PRE_PRUNE | 精简前完整文件树；旧工具/说明/数据恢复 | `handoff/pre-prune` |

## 导出整个节点

以下命令从仓库根执行，目标目录必须不存在：

```bash
python3 tools/export_version.py B3 .runtime/track-c-reference
python3 tools/export_version.py PRE_PRUNE .runtime/legacy-reference
```

B1 导出 `handoff/current` 已登记交付文件及手眼样本，**没有 `.git`**。其他节点保留各自的 `src/`、`debian/` 来源布局；PRE_PRUNE 对应完整精简前树，也恢复配套手眼数据。导出副本用于独立检查或实验，保留完整仓库才能继续恢复其他节点。

PRE_PRUNE 的固定提交为 `ad272bf210c65ef71410262190e7a10de03a5c6b`。该节点保存旧诊断、被替代的实验和历史说明；使用旧文档时以其时间和场景为边界，不作为当前执行指令。

## 按报告装配

```bash
python3 tools/restore_run.py --list
python3 tools/restore_run.py --list --node B5
```

复制列表中精确的报告路径，作为 `tools/restore_run.py 报告路径 .runtime/新目录` 的第一个参数。工具按 `run_bindings` 的 SHA 选择源文件并映射 `restore_name`，输出 `RESTORE_MANIFEST.json` 和 `recorded_run.json`。只验证文件绑定，不导入 SDK、不执行运动、不补猜未记录的算法依赖。

`RECORDED_BINDINGS_VERIFIED` 表示已登记绑定匹配，不代表全部运行依赖、场景或动作资格已验证；`PARTIAL_SOURCE_IDENTITY` 表示仍有未绑定项。MPC controller 的历史 SHA 缺失保留在 manifest 中；该标记描述源码追溯缺口，MPC 已有实机运行报告。

## 证据使用规则

- B2 的最终固定场景成功记录与早期 9/7 参考分开；5%/10%成功不证明其他速度或新场景。
- B3 原始说明记载8/4双次抓放连续5轮成功；当前缺完整逐轮归档，不是未跑出结果。补档须使用对应实验原日志，不能用7/31八点基础日志替代。B7的使能故障日志不能替代抓放日志。
- B4/B5 不同 wrapper、base、SDK、Servo 辅助件按各轮恢复；不按“最新同名文件”拼接。
- D1 有精确缺陷对照源码；D2/D3 部分只有证据，缺失实现不捏造。
- 当前完整性使用根 `HANDOFF_MANIFEST.json` / `SHA256SUMS`。历史内部清单只描述当时范围；原来源身份保存在 `SNAPSHOT.json` 和机器映射。

新版本登记与打包流程见 [MAINTENANCE.md](MAINTENANCE.md)。
