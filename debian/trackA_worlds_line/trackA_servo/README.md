# Track A Servo · 最终成功 A 路线派生候选

状态：SOFTWARE_CANDIDATE / MOTION_BLOCKED：关节流与执行器已生成；可离线或只读预检，新 Servo 的完整场景/持物碰撞与动态时序资格尚未通过，不继承 A 的真机 PASS。

## 与原 A 的关系

只更换执行方式：原规划关节折线进一步密化，保留所有旧关节锚点、抓放目标和固定 Home。
全程运动用 armPluseToServo；无 MoveJ、MoveWorlds、自动寻找首点或第二次 TCP/关节符号转换。
来源是 A JSON 内 SDK 预测/插值回放，不是真机逐帧录制；原控制器连续插值与新 Servo 不等价。
原 A 的现场执行器作为初始化/夹爪编码/收尾依赖保留，不调用它的运动 main。

- 2484 个关节点，最大步长 0.100000°，20ms 一帧。
- 名义关节流 49.68s；开爪1s、闭爪2s、放置开爪1s，停稳/网络耗时另计。
- 离散指令最大速度 5.000°/s，差分加速度 379.672°/s²；不是机器人反馈或厂商动力学额定值。
- max_step/period 由 JSON 和共享配置固定；--speed 仅控制器全局速度，不能据此把 Servo 周期任意加速。
- 抓放事件要先停稳并核对实际 EE；转运没有人为插入中途停顿，但折线细分不保证 jerk 连续。
- 单调时钟，不追帧；发送迟到超过两个周期停止。SDK调用阻塞仍受SDK自身超时约束，不是硬实时系统。
- 真机只读限位交集保持5°余量；失败停止，不自动开爪丢物；收尾回读保护和原速度。

## 起点特别注意

首点是成功包保存的历史采集构型，不是结束 Home，二者不能混同。没有从未知位置自动返回这一段。
只有现场已停稳且每轴距首点≤0.4°才继续；只读预检不替你移动，人工审核也不绕过首点突跳限制。
首点 SDK joints deg：`[-16.031999588012695, 23.22800064086914, -0.35608211159706116, -72.66799926757812, -25.340999603271484, -11.798999786376953, 9.605405807495117]`。
最终 Home SDK joints deg：`[-17.711, 29.247, 6.936, -75.605, -13.196, -11.823, 12.235]`。
若当前为 Home，先做只读预检并回传结果；不要单纯放宽起点阈值。需单独规划/审阅 Home 接入段，或者在现场确认净空后用示教器到首点。

## Debian 目前可做的检查（不会运动）

保留原 trackA_hybrid_trial 目录；将本 ZIP 单独解压。使用原厂商 SDK 的 Debian Python 环境，不安装 PyBullet/NumPy。
```bash
cd trackA_servo
sha256sum -c SHA256SUMS
python3 execute_tabletop_servo.py tabletop_pick_place_SERVO_CANDIDATE.json --dry-run
```

连接只读预检时，请使用现场实际IP（以下是上次地址示例，不是实时确认）：

```bash
python3 execute_tabletop_servo.py tabletop_pick_place_SERVO_CANDIDATE.json \
  --precheck-only --robot-ip 192.168.8.147 --local-ip 192.168.8.185 --arm-ip 192.168.8.147
```

默认生成不覆盖旧文件的 tabletop_servo_时间.json；成功/失败均含候选、执行器、全部依赖哈希和只读结果。
请回传这个 JSON。只读 PASS 只表示文件、实时限位、静止首点检查通过，不代表碰撞或真实 Servo 时序通过。

## 为什么此包还不能直接真机 --run

新 Servo 必须有当前 JSON 完整 GUI 人工 PASS，以及独立的 tabletop_servo_qualification.v1 PASS 报告。
当前自碰撞模型出现非相邻腕部 link9/11 重叠，尚未核实；桌框空间配准、真实夹爪/持瓶/释放后扫掠未完成。
不能拿历史 A 现场 PASS、旧GUI或仅限位 PASS 替代这些证据；包内不会提供伪造PASS或跳过检查的开关。
未来资格报告须绑定轨迹/消费者/依赖 SHA、场景、步长/周期/帧数，逐项提供FK、自碰撞、环境、持物/释放、接入区域、Servo动态时序的哈希证据。
合格后执行器支持 --run --gui-review <审核文件> --qualification-report <资格文件>，还需 XIFENG_ALLOW_REAL_MOTION=1 和现场回车；此处不是当前放行命令。

## Mac 再生与 GUI

```bash
/opt/anaconda3/bin/python -B pick_place_coord/gen_tabletop_hybrid_candidate.py --servo-parent pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json
/opt/anaconda3/bin/python -B pick_place_coord/gen_tabletop_hybrid_candidate.py --replay pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_CANDIDATE.json --gui --gui-report pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_gui_playback.json --gui-review pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_gui_review.json --reviewer operator
python3 releases/make_release.py AS
```

以上从工作1根目录运行。AS 仅重建 trackA_servo 和 ZIP，不改 A/AT/C 既有发布包。
夹爪参数仍在 gripper_policy 接口中；改参数或路径后应重新生成、校验并绑定新 SHA，不使用旧审核。

## 文件

- `arm_profiles.py`
- `arm_profiles.v1.json`
- `execute_tabletop_hybrid_trial_reviewfix_field.py`
- `execute_tabletop_pick_place_worlds.py`
- `execute_tabletop_servo.py`
- `precheck_tabletop_hybrid.py`
- `robot_lock.py`
- `scrub_log.py`
- `sdk_session.py`
- `servo_common.py`
- `tabletop_pick_place_SERVO_CANDIDATE.json`
- `tabletop_pick_place_SERVO_gui_review.json`
- `tabletop_servo_contract.py`
