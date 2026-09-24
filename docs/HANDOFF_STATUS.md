# Handoff Status — Servo-Ready v1

## 当前正式支持范围

本版本面向“新同事拿到交接包后，用新的固定工位 Pick / Place 点位重新规划，并进入现有 Servo 执行链做无运动验证”的场景。

当前正式支持链路：

```text
新任务 XYZ（用户输入 mm）
        ↓
tools/new_task_pipeline.py
        ↓
内部 mm → m
        ↓
pick_place_coord.py
        ↓
IK
        ↓
RRT-Connect
        ↓
PyBullet collision check
        ↓
planned_candidate.json
        ↓
execute_servo_grasp.py
        ↓
0.4° / 20 ms Servo densification
        ↓
dry-run / controller precheck
```

## 新任务统一入口

用户侧输入单位：

- Position XYZ：**mm**
- Orientation：**deg**
- Pipeline 内部送给 PyBullet planner 时自动转换为 **m**

准备任务：

```bash
mkdir -p .runtime/new-task
cp templates/task_template.json .runtime/new-task/task.json
```

修改 `.runtime/new-task/task.json` 中的 `pick` / `place` 后运行：

```bash
python3 tools/new_task_pipeline.py \
  --task .runtime/new-task/task.json \
  --out .runtime/new-task \
  --stage prepare \
  --seed 7 \
  --arm left
```

成功后主要输出：

```text
.runtime/new-task/
├── task.json
├── input_task.json
├── planned_candidate.json
├── pipeline_report.json
└── COMMANDS.md
```

其中：

- `task.json`：用户输入，位置单位 mm
- `input_task.json`：Planner 内部输入，位置单位 m
- `planned_candidate.json`：新规划得到的关节路点、夹爪事件及任务元信息
- `COMMANDS.md`：后续 Servo dry-run / controller precheck 操作入口

---

## 已实际验证

### Test 1

输入：

```text
pick  = [320, 260, 1050] mm
place = [300, 160, 1050] mm
```

结果：

- IK PASS
- RRT PASS
- PyBullet collision PASS
- 80 个 planner 路点
- 2 个 gripper events
- `planned_candidate.json` 成功生成

### Test 2

输入：

```text
pick  = [360, 300, 1050] mm
place = [420, 180, 1050] mm
```

结果：

- IK PASS
- RRT PASS
- PyBullet collision PASS
- 84 个 planner 路点
- 2 个 gripper events
- 与 Test 1 生成不同轨迹，证明不是固定 replay

随后将 Test 2 新轨迹直接送入现有 Servo executor：

```bash
XIFENG_ALLOW_REAL_MOTION=0 \
python3 src/frame_calibration/robot_side/execute_servo_grasp.py \
  .runtime/new-task-test-2/planned_candidate.json \
  --step-deg 0.4 \
  --period-ms 20 \
  --speed 5 \
  --arm-id 1 \
  --gripper-id 2
```

实际结果：

```text
84 个 planner waypoints 全部通过关节限位预检
1194 个 dense Servo frames
预计时长 23.9 s
DRY RUN
未下发真机
```

因此以下链路已实际验证：

```text
新 mm 点位
→ 重新规划
→ IK / RRT / collision
→ planned_candidate.json
→ Servo 0.4° / 20 ms densification
→ dry-run PASS
```

---

## 关于 `validate_trajectory.py`

`planned_candidate.json` 是 Planner 输出的稀疏关节路点。

当前测试中，部分相邻稀疏 waypoint 的最大关节变化约为：

```text
6.67°
```

旧 `validate_trajectory.py` 的 sparse-step gate 为：

```text
6.0°
```

因此直接对新的 `planned_candidate.json` 执行该旧 validator 可能返回 FAIL。

这**不等同于 Servo stream 失败**。

Servo executor 会按：

```text
step = 0.4°
period = 20 ms
```

重新 densify，并对 Servo 输入做关节限位检查。

当前新任务主要离线验收应理解为：

```text
Planner PASS
+
Servo dry-run PASS
```

不要通过简单放宽旧 validator 的 6° 阈值来掩盖稀疏轨迹与 Servo dense stream 的语义差异。

---

## Servo 与 MPC 的边界

### 新任务当前已支持

```text
new planned_candidate.json
        ↓
execute_servo_grasp.py
        ↓
0.4° / 20 ms Servo stream
```

已验证。

### 历史 MPC 链路

历史 B5 使用已经通过历史来源约束的 Servo candidate：

```text
historical SDKALIGNED parent
        ↓
historical SERVO_CANDIDATE contract
        ↓
B5 Servo + MPC Shadow / Active
```

历史 MPC 算法、实验与运行记录保留在交接包中。

### 当前未接入的能力

以下链路**尚未正式实现/验收**：

```text
new planned_candidate.json
        ↓
generic new-task SERVO_CANDIDATE
        ↓
B5 MPC
```

现有：

```bash
gen_tabletop_hybrid_candidate.py --servo-parent
```

不是通用转换器。

它故意要求历史已验收的 SDKALIGNED parent，并会拒绝新的 `planned_candidate.json`：

```text
父轨迹不是已收到的 SDKALIGNED 实跑文件；不隐式更换来源
```

不得删除或绕过这一 provenance/source gate，也不得把新的 `planned_candidate.json` 直接改名冒充历史 `tabletop_pick_place_SERVO_CANDIDATE.json`。

如果未来要支持新任务 MPC，应新增独立、可验证的 generic Servo/MPC candidate contract，而不是污染历史 B4/B5 的 SHA/provenance 关系。

---

## 当前冻结版本承诺

### 支持

```text
新 XYZ 点位（mm）
→ mm → m
→ IK
→ RRT-Connect
→ PyBullet collision check
→ 新 joint trajectory
→ Servo 0.4° / 20 ms dry-run
```

### 不承诺

```text
新任务
→ generic SERVO_CANDIDATE
→ MPC
```

### 历史能力仍保留

历史 B2 / B3 / B4 / B5 的恢复、dry-run、precheck 与证据仍按原交接结构保留。

相关入口：

- `START_HERE.md`
- `NEW_TASK_GUIDE.md`
- `robot/README.md`
- `sim/README.md`
- `docs/CODE_MAP.md`
