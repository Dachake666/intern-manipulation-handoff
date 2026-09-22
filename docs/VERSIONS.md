# 可恢复版本

工作1原仓库HEAD为9f65b7619cc3ae70446a258c9276111e5eb14720，当前工作文件也被纳入B1。
本包新建精简Git历史，未继承原仓库全部祖先。提交日期是整理日，不能当作实跑日期。
七类基础节点和三类缺陷之外，history/H_MAC_FROZEN保留必要补充材料，不提供默认运动快捷入口。

| 节点 | 内容 | Git标签 |
|---|---|---|
| B1 | Mac当前开发基础 | `handoff/current` |
| B2 | Track A最终固定场景 | `handoff/b2` |
| B3 | Track C133点双次抓放 | `handoff/b3` |
| B4 | Servo20/25/VAJ3与30ms历史 | `handoff/b4` |
| B5 | MPC Shadow/Active及离线参考 | `handoff/b5` |
| B6 | Track B99/153点基础 | `handoff/b6` |
| B7 | 瓶子示教与HOME | `handoff/b7` |
| D1 | Servo长迟到缺陷组合 | `handoff/d1` |
| D2 | MPC两次早期失效证据 | `handoff/d2` |
| D3 | MoveJ限位/跳点缺陷证据 | `handoff/d3` |
| H_MAC_FROZEN | Mac既有冻结源码 | `handoff/h-mac-frozen` |
| history | 其他运动/诊断历史 | `handoff/history` |

`python3 tools/export_version.py 节点 新目录` 可离线恢复。B1恢复整套当前交付，其他节点恢复debian/或src/原布局。
D2/D3只有证据的部分不会捏造缺失源码；D1按报告绑定的原字节保存。

原提交定位：Track A为909b73e；Track C成果698b066，左臂标签指向d82af3a；早期Pulse v1为864b0a4。
这些是来源映射，本包Git不含原提交的完整祖先；具体收到文件以SOURCE_MAP.json、各SNAPSHOT.json的完整SHA为准。
Servo的cc16e65是较早Mac交接阶段，不能替代收到现场源码；9f65b76是收件审阅提交，不是真机执行时提交。

docs/debian_selection.json内74条run_bindings给出执行器/轨迹/GUI/wrapper/依赖的恢复名与来源。
同一文件在多个历史节点使用时Git对象复用；5种sdk_session不强行合并。
9/7五轮与9/8最终五轮分开；7/31八点与8/4双抓放分开；根traj_multi_latest隔离候选不替换Track C。
