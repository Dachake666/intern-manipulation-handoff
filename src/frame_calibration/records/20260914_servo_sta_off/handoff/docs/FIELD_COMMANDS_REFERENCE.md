# 现场命令留档（不要在 Mac/接续任务中运行）

这些是用户曾在 `dev@humble` 执行的命令，不是本交接包的当前运行授权。最新 field 源码、新 review 原件与完整资格均需先核验；不要从旧 ZIP 加重建 candidate 直接拼装执行。任何网络切换、保护设置和真机运动需现场负责人另行确认。

三模式使用同一短夹爪等待 candidate；现场 CLI 中的 `--speed 5.0` 不能单独代表 Servo 关节速度，实际还由关节帧与周期决定。

## Plain20 / stride4

```bash
cd ~/workspace/trackA_worlds_line/trackA_servo || exit 1
XIFENG_ALLOW_REAL_MOTION=1 python3 -u execute_tabletop_servo_field_20ms.py \
  tabletop_pick_place_SERVO_CANDIDATE.json \
  --field-trial \
  --run \
  --gui-review tabletop_pick_place_SERVO_gui_review.json \
  --robot-ip 192.168.8.148 \
  --local-ip 192.168.8.244 \
  --arm-ip 192.168.8.148 \
  --arm-port 8080 \
  --speed 5.0
```

## Plain25 / stride4

```bash
cd ~/workspace/trackA_worlds_line/trackA_servo || exit 1
XIFENG_ALLOW_REAL_MOTION=1 python3 -u execute_tabletop_servo_field_25ms.py \
  tabletop_pick_place_SERVO_CANDIDATE.json \
  --field-trial \
  --run \
  --gui-review tabletop_pick_place_SERVO_gui_review.json \
  --robot-ip 192.168.8.148 \
  --local-ip 192.168.8.244 \
  --arm-ip 192.168.8.148 \
  --arm-port 8080 \
  --speed 5.0
```

## VAJ3 / 20ms

```bash
cd ~/workspace/trackA_worlds_line/trackA_servo || exit 1
XIFENG_ALLOW_REAL_MOTION=1 python3 -u execute_tabletop_servo_field_vaj20_v3.py \
  tabletop_pick_place_SERVO_CANDIDATE.json \
  --field-trial \
  --run \
  --gui-review tabletop_pick_place_SERVO_gui_review.json \
  --robot-ip 192.168.8.148 \
  --local-ip 192.168.8.244 \
  --arm-ip 192.168.8.148 \
  --arm-port 8080 \
  --speed 5.0
```

## 日志分析（离线）

```bash
python3 tools/summarize_servo_logs.py --root /实际路径/field_snapshot \
  --out /一个尚不存在的输出路径/summary.json
```

分析脚本不执行任何项目模块。缺失指标为 null；不会用 0 伪装缺失值，不从少量 slow_events 伪算完整 median/p99，不把 nominal_duration 当动作实际用时。
