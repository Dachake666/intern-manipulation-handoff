# Track A Servo：30 ms 成功基线

## 对应运行
- 成功报告：tabletop_servo_20260910_115323_657498295.json
- 状态：SERVO_COMPLETED_RETURNED_HOME
- 主体发送帧数：1248
- Runtime stride：2
- Runtime period：0.030 s
- 主体名义发送时间：37.44 s，不含起步、确认及夹爪等待
- 全局速度参数：5.0%
- 起步仍为 MoveJ 到 Servo 首点，之后执行 Servo 主体
- 原候选 JSON 保持不变，30 ms 在 wrapper 的 runtime 副本中覆盖

## 本次测量
- Pulse 平均调用耗时：5.5899 ms
- Pulse 最大调用耗时：166.1279 ms
- 平均正迟到：0.3795 ms
- 最大迟到：136.2350 ms
- Realign：1 次；本版本门槛为迟到超过 60 ms
- Cleanup：PASS；保护已恢复，全局速度恢复至 10%

## 已知边界
完整执行和终点检查通过，现场反馈明显顺畅。
仍有偶发时序尖峰，不等于硬实时保证或全程跟随误差验证。

## 环境记录
- Robot / Arm IP：192.168.8.147
- Local IP：192.168.8.185
- Wi-Fi profile：GL-SFT1200-71f-5G
- 主机 NetworkManager powersave：2（disable），此前现场已确认
- Python / PilotSDK 及其共享库沿用原可运行环境，本包不包含这些安装依赖

## 重启后对比
保持代码、轨迹、stride、period 和 speed 不变。
先确认机器人 IP、当前关节位置和现场状态，再按原流程测试。
重启后改善本身不能单独证明过热，因为进程和通信状态也同时被重置。
