# 候选、固定参考与检查夹具

本目录保留开发和离线回归仍依赖的输入。文件名的 CANDIDATE、RUN 或旧状态字段不是当前运动许可；按精确 SHA、场景、配套报告和执行器共同判断。

| 文件/组 | 用途与限制 |
|---|---|
| `tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json` | Track A最终固定参考，SHA `e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59`；配套现场组合为B2 |
| `tabletop_pick_place_hybrid_CANDIDATE.json`、`_OPTIMIZED.json` | 旧候选仍作为生成器/执行器回归输入，不能因较旧而移除测试依赖 |
| `tabletop_pick_place_SERVO_CANDIDATE.json` | 开发Servo参考，SHA前缀 `0a18545594212ecf`；B4/B5各轮可能用不同候选，按绑定恢复 |
| `traj_bottle_taught_tcp_20260831_RUN.json` | 275运动点及夹爪事件，SHA `cce2a83fae21468f80f0aad487087b61650eac6f92dea65f9560091d0239ad2b`；完整五轮原日志不足，LIMITED |
| 瓶子CANDIDATE、qualification task/report、GUI review | 生成/回放与来源资格证据；不能替代B7现场执行器身份 |
| `tabletop_pick_place_worlds.json` | 逐段MoveWorlds候选，场景与资格需核对；不是通用成功基线 |
| `traj_multi_right_20260806_CANDIDATE.json` | 右臂硬件阻断候选，实际限位、TCP和夹爪闭环未完成 |
| `traj_tabletop_vision_ab_20260902_REPLAY_CANDIDATE.json` 及对应task/report | 视觉场景回放与候选链路，不代表自主抓放已验收 |

新实验输出到 `.runtime/`，登记代码/输入/参数 SHA 和对应验证。需要成为长期参考时先补用途、依赖和证据边界，再加入根 `docs/DELIVERY_SELECTION.json`。不要覆盖历史文件、改JSON状态或改文件名来升级资格。
