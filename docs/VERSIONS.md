# 可恢复版本

初次收件工作区HEAD为9f65b7619cc3ae70446a258c9276111e5eb14720，允许范围内的当前工作文件也纳入B1。
本包为精简Git历史，未继承原仓库全部祖先。整理提交日期不是实跑日期。
handoff/current随本次交接修订前移；B2～B7、D1～D3、history/H_MAC_FROZEN保持原提交与身份。

| 节点 | 内容 | Git标签 |
|---|---|---|
| B1 | 当前开发基础（仿真、接口、真机消费者） | `handoff/current` |
| B2 | Track A最终固定场景 | `handoff/b2` |
| B3 | Track C133点双次抓放 | `handoff/b3` |
| B4 | Servo20/25/VAJ3与30ms历史 | `handoff/b4` |
| B5 | MPC Shadow/Active及离线参考 | `handoff/b5` |
| B6 | Track B99/153点基础 | `handoff/b6` |
| B7 | 瓶子示教与HOME | `handoff/b7` |
| D1 | Servo长迟到缺陷组合 | `handoff/d1` |
| D2 | MPC两次早期失效证据 | `handoff/d2` |
| D3 | MoveJ限位/跳点缺陷证据 | `handoff/d3` |
| H_MAC_FROZEN | 原工作区既有冻结源码 | `handoff/h-mac-frozen` |
| history | 其他运动/诊断历史 | `handoff/history` |

从交接根目录执行 `python3 tools/export_version.py 节点 .runtime/新目录`。目标目录必须不存在。
B1导出当前交付工作文件和手眼数据；其他节点导出debian/或src/来源布局。
导出副本不含.git，适合独立实验；要继续使用所有历史节点，保留原完整交接包。

**导出不是逐轮自动装配。** 对有run_bindings的记录，必须按report_relative选择source_relative、restore_name与SHA，
在新的实验目录组成该轮代码；工具不自动重命名备份，也不会补齐未绑定的算法身份。
D2/D3只有证据的部分不捏造源码；D1按报告绑定的原字节保存。

原提交定位：Track A为909b73e；Track C成果698b066，左臂标签指向d82af3a；早期Pulse v1为864b0a4。
这些是来源映射，不是本包可直接checkout的祖先。以SOURCE_MAP.json、各SNAPSHOT.json的完整SHA为准。
Servo的cc16e65是较早交接阶段，不能替代现场源码；9f65b76是收件审阅提交，不是真机执行时提交。

docs/debian_selection.json内74条run_bindings记录执行器、轨迹、GUI、wrapper及依赖恢复名与来源。
同一文件跨节点时Git对象复用；5种sdk_session不强行合并。
9/7五轮与9/8最终五轮分开；7/31八点与8/4双抓放分开；根traj_multi_latest隔离候选不替换Track C。
历史子目录SHA256SUMS保留原归档范围；当前交付完整性使用根清单，迁移与排除以来源映射为准。
