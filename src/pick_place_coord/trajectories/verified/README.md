# 冻结轨迹参考

本目录保存核心冻结输入。历史文件名中的 `REALVERIFIED` 是来源名称，**不代表当前已具备完整实机验收证据**；使用结论以本说明、版本表和运行身份为准。

| 文件 | 身份 | 当前证据边界 |
|---|---|---|
| `traj_multi_2grasp_20260804_REALVERIFIED.json` | SHA前缀 `ac5c6fd7715c81fb`；133运动点 + 4夹爪事件 | 原始说明记载 Track C 双次抓放连续 5 轮成功；完整逐轮原日志尚未归档到本包，LIMITED 仅描述当前归档证据范围 |

冻结输入只读。点流对照从仓库根运行：

```bash
(cd src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/verify_left_track_c.py)
```

预期0.4°步长1633帧，0.2°步长3232帧。这是纯离线计算，不构成GUI或真机验收。B3保存对应历史实现；精确历史运行身份缺口不能由当前开发执行器填补。

旧Worlds/8点等参考通过PRE_PRUNE、history或对应成果节点恢复。新候选先存独立路径，完成明确范围的验证后登记，不修改已有冻结文件。
