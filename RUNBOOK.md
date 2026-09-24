# Robot Handoff Runbook

> 面向新同事的最短操作手册。能力边界、实测结果与 MPC 说明见 `docs/HANDOFF_STATUS.md`。

## 0. 进入仓库

进入包含 `START_HERE.md` 与 `tools/` 的仓库根目录，例如：

```bash
cd /home/dev/workspace/robot_handoff_pipeline_test/robot_handoff
```

基础检查：

```bash
python3 --version
python3 tools/verify_handoff.py
```

如果需要 SDK / 真机相关能力，再检查：

```bash
python3 -c "import pypilot; print(pypilot.__file__)"
```

---

## 1. 新 Pick / Place 点位重新规划

### 1.1 准备任务

```bash
mkdir -p .runtime/new-task
cp templates/task_template.json .runtime/new-task/task.json
```

编辑：

```text
.runtime/new-task/task.json
```

用户侧输入规则：

```text
Position XYZ = mm
Orientation   = deg
```

典型字段：

```json
{
  "pairs": [
    {
      "pick":  [320, 260, 1050],
      "place": [300, 160, 1050]
    }
  ]
}
```

Pipeline 会把 mm 自动转换成 PyBullet planner 使用的 m。

### 1.2 运行新任务规划

```bash
python3 tools/new_task_pipeline.py \
  --task .runtime/new-task/task.json \
  --out .runtime/new-task \
  --stage prepare \
  --seed 7 \
  --arm left
```

成功时应看到：

```text
IK PASS
RRT-Connect PASS
PyBullet collision PASS
status = PASS
real_motion_authorized = false
```

主要输出：

```text
.runtime/new-task/
├── task.json
├── input_task.json
├── planned_candidate.json
├── pipeline_report.json
└── COMMANDS.md
```

---

## 2. 新轨迹 Servo dry-run

左臂默认：

```bash
XIFENG_ALLOW_REAL_MOTION=0 \
python3 src/frame_calibration/robot_side/execute_servo_grasp.py \
  .runtime/new-task/planned_candidate.json \
  --step-deg 0.4 \
  --period-ms 20 \
  --speed 5 \
  --arm-id 1 \
  --gripper-id 2
```

成功时重点看：

```text
限位预检: ... 个路点全部通过
透传密集点: ... 个
DRY RUN: 未下发
```

已实测示例：

```text
84 planner waypoints
→ 1194 dense Servo frames
→ 0.4° / 20 ms
→ joint-limit PASS
```

---

## 3. Controller read-only precheck

先设置现场 IP。不要直接沿用历史 `.147/.148/.185/.244`，以现场为准。

```bash
export XIFENG_ROBOT_IP=<current_robot_ip>
export XIFENG_ARM_IP=<current_arm_ip>
export XIFENG_LOCAL_IP=<current_local_ip>
```

执行：

```bash
XIFENG_ALLOW_REAL_MOTION=0 \
python3 src/frame_calibration/robot_side/execute_servo_grasp.py \
  .runtime/new-task/planned_candidate.json \
  --step-deg 0.4 \
  --period-ms 20 \
  --speed 5 \
  --arm-id 1 \
  --gripper-id 2 \
  --robot-ip "$XIFENG_ROBOT_IP" \
  --local-ip "$XIFENG_LOCAL_IP" \
  --arm-ip "$XIFENG_ARM_IP" \
  --controller-precheck-only \
  --controller-precheck-report .runtime/new-task/controller_precheck.json
```

该步骤是只读预检，不授权真机运动。

---

## 4. 稀疏轨迹 validator 的说明

下面这条旧检查：

```bash
python3 src/pick_place_coord/validate_trajectory.py \
  .runtime/new-task/planned_candidate.json
```

检查的是稀疏 planner waypoint。

新任务可能出现相邻稀疏 waypoint 超过旧 6° gate，例如已观察到约 6.67°。

这不等于 Servo 失败，因为 Servo executor 会按：

```text
step = 0.4°
period = 20 ms
```

重新 densify。

当前新任务主要离线验收：

```text
Planner PASS
+
Servo dry-run PASS
```

不要为了通过旧 validator 直接提高 6° 阈值。

---

## 5. 历史版本恢复

### B2 Track A

```bash
python3 tools/export_version.py B2 .runtime/track-a-run
```

### B3 Track C Servo

```bash
python3 tools/export_version.py B3 .runtime/track-c-run
```

历史 Track-C dry-run：

```bash
cd .runtime/track-c-run/debian/trackC_servo_stream/trackC_servo_stream

python3 -u execute_servo_grasp.py \
  traj_multi_latest.json \
  --step-deg 0.4 \
  --period-ms 20 \
  --speed 15 \
  --arm-id 1
```

### B4 Servo

列出历史运行：

```bash
cd <repo-root>
python3 tools/restore_run.py --list --node B4
```

恢复指定运行：

```bash
python3 tools/restore_run.py \
  <run-json-relative-path> \
  .runtime/servo-run
```

### B5 MPC

恢复历史 MPC：

```bash
python3 tools/export_version.py B5 .runtime/mpc-field
```

Shadow / Active 的完整命令见：

```text
robot/README.md
→ MPC：B5 Shadow / Active
```

MPC 依赖：

```bash
python3 -m pip install \
  -r src/pick_place_coord/mpc_experiment/requirements.txt
```

检查：

```bash
python3 -c "import osqp; print(osqp.__version__)"
```

当前验证过：

```text
osqp 1.1.1
```

---

## 6. 新任务 MPC 当前边界

当前已验证：

```text
new planned_candidate.json
→ execute_servo_grasp.py
→ 0.4° / 20 ms Servo stream
```

当前尚未正式接入：

```text
new planned_candidate.json
→ generic new-task SERVO_CANDIDATE
→ B5 MPC
```

历史：

```bash
gen_tabletop_hybrid_candidate.py --servo-parent
```

不是通用 converter。它要求历史已验收的 SDKALIGNED parent，并保留 provenance / SHA gate。

禁止：

```text
把新的 planned_candidate.json
直接改名成 historical tabletop_pick_place_SERVO_CANDIDATE.json
```

详细边界见：

```text
docs/HANDOFF_STATUS.md
```

---

## 7. 代码入口

不知道代码职责时先看：

```text
docs/CODE_MAP.md
```

新任务主线：

```text
templates/task_template.json
tools/new_task_pipeline.py
src/pick_place_coord/pick_place_coord.py
src/frame_calibration/robot_side/execute_servo_grasp.py
```

历史复现：

```text
START_HERE.md
robot/README.md
sim/README.md
docs/version_refs.json
```
