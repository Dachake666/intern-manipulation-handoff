# Track C-bottle optimized · 左臂姿态优化瓶子搬运（只读预检包）

**状态**: OFFLINE_PASS / REAL_MOTION_BLOCKED。0.4deg 密集限位、机器人/桌面/夹爪代理/携带瓶体碰撞及 PyBullet GUI 人工门均 PASS；当前只允许控制器只读预检。轨迹 meta.real_motion_authorized=false，禁止 --run。

**全新部署**用这个目录：整个拷过去就能跑，不用再从别处补文件。
**已经跑过的机器**只需要补送变化的那几个文件，见下面「增量更新」。

## 运行

```bash
python3 execute_servo_grasp.py traj_bottle_left_orientation_optimized_20260827_CANDIDATE.json \
  --controller-precheck-only \
  --controller-precheck-report bottle_left_orientation_controller_precheck.json
```

本包当前只允许 `--controller-precheck-only`。不要设置
`XIFENG_ALLOW_REAL_MOTION=1`，不要添加 `--run`。轨迹自身也带
`real_motion_authorized=false`，会阻止误下发。

运行前必须通过环境变量或命令行提供本次会话的 robot/local/arm IP；
历史 `.147` / `.148` 不能代替现场身份确认。

## 文件

运行必需（少一个就跑不起来）：

- `arm_profiles.py`  — 双臂配置加载器；右臂未测项会强制阻断真机
- `arm_profiles.v1.json`  — 左右臂关节/TCP/HOME/限位/夹爪契约唯一权威
- `bottle_left_orientation_optimized_gui_review.json`  — 绑定轨迹 SHA-256 的 PyBullet GUI 人工 PASS 记录
- `bottle_left_orientation_optimized_qualification_report.json`  — 限位、密集碰撞和 GUI 人工门 PASS 报告
- `execute_servo_grasp.py`
- `execution_authorization.py`  — 校验 preflight、人工批准和场景复核的绑定关系
- `robot_lock.py`  — 机器人互斥锁 —— 四个执行器都 import, 必需; 同一台臂同时只允许一个进程
- `sdk_session.py`  — SDK 会话/运动封装 —— 执行器 import 它, 必需
- `servo_common.py`  — 透传公共件: 节拍器 + 跟随误差监控 —— 两条透传线都 import, 必需
- `task_bottle_left_orientation_optimized_20260827_qualification.json`  — 密集碰撞资格任务；含桌面、瓶体和抓取中心偏移
- `task_bottle_left_orientation_search_20260827.json`  — 固定 XYZ、绕接近轴搜索姿态的生成任务
- `traj_bottle_left_orientation_optimized_20260827_CANDIDATE.json`  — 姿态优化后的左臂瓶子完整候选；仍禁止真机运动

另外还依赖 `pypilot`（厂商 SDK 轮子），那是装在真机容器里的，
不在封装内。

附带工具（不装也能跑）：

- `scrub_log.py`  — 日志脱敏工具 —— 没人 import, 可不装; 日志外发前过一遍
- `test_servo_safety.py`  — 离线回归测试(不连机器人): 遥测关闭/节拍/向量类型/互斥锁/只读会话/速度还原

## 增量更新

真机上已经有这一套的话，日常只需要送 **变化了的** 文件——
通常就是 `arm_profiles.py` 和 `arm_profiles.v1.json` 和 `bottle_left_orientation_optimized_gui_review.json` 和 `bottle_left_orientation_optimized_qualification_report.json` 和 `execute_servo_grasp.py` 和 `execution_authorization.py` 和 `robot_lock.py` 和 `servo_common.py` 和 `task_bottle_left_orientation_optimized_20260827_qualification.json` 和 `task_bottle_left_orientation_search_20260827.json` 和 `traj_bottle_left_orientation_optimized_20260827_CANDIDATE.json`。
`sdk_session.py` 只在它自己被改过时才需要重送（改没改，看提交说明）。

## 校验

```bash
shasum -a 256 -c SHA256SUMS
```

或在工作区根目录跑 `python3 releases/make_release.py --verify`。

## 重新生成

```bash
python3 releases/make_release.py CBO
```

封装是从工作区当前代码复制出来的快照，**不要直接改封装里的文件** —— 
改源文件再重新生成，否则下次生成会把你的改动覆盖掉。
