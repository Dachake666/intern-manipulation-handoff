# Robot Manipulation Handoff

这是机器人抓放、运动规划、Servo 执行与 MPC 历史实验的完整工程交接仓库。

当前冻结版本重点保证两件事：

1. **新同事拿到新的 Pick / Place 点位后，可以重新规划并生成新的机器人关节轨迹。**
2. **新规划轨迹可以进入现有 Servo 0.4° / 20 ms 执行链做 dry-run / controller precheck。**

当前冻结状态：

```text
Git tag: handoff/servo-ready-v1
Offline regression: 285 passed
Delivery verification: 767 files match
```

> 新任务 generic `SERVO_CANDIDATE → MPC` 尚未正式接入；历史 B5 MPC Shadow / Active 仍完整保留。详见 `docs/HANDOFF_STATUS.md`。

---

## 1. 第一次接手，请按这个顺序看

### ① `START_HERE.md`

整个交接包的总入口。

先看这里，了解：

- 包里有什么
- 当前推荐入口
- 历史恢复工具
- 重要文档位置

### ② `docs/HANDOFF_STATUS.md`

**最重要的状态说明。**

这里明确写了：

- 当前正式支持什么
- 实际验证过什么
- 新任务与 Servo 的边界
- MPC 当前做到哪里
- 哪些能力不能宣称已经完成

### ③ `RUNBOOK.md`

**最短操作手册。**

如果你只想知道“该复制哪条 command”，直接看这里。

包括：

- 环境检查
- 新点位规划
- Servo dry-run
- controller read-only precheck
- 历史 B2 / B3 / B4 / B5 恢复
- MPC 依赖与边界

### ④ `NEW_TASK_GUIDE.md`

专门讲：

> 我现在有一个新的 Pick / Place 点位，如何基于现有工程重新规划？

### ⑤ `docs/CODE_MAP.md`

主要代码职责地图。

不知道一个 `.py` 文件是干什么的时，优先看这里，而不是自己猜。

---

# 2. 当前已验证主流程

当前新任务主线：

```text
新的 Pick / Place XYZ（用户输入 mm）
                ↓
templates/task_template.json
                ↓
tools/new_task_pipeline.py
                ↓
内部 mm → m
                ↓
src/pick_place_coord/pick_place_coord.py
                ↓
IK
                ↓
RRT-Connect
                ↓
PyBullet collision checking
                ↓
planned_candidate.json
                ↓
src/frame_calibration/robot_side/execute_servo_grasp.py
                ↓
0.4° / 20 ms Servo densification
                ↓
dry-run / controller precheck
```

这条链路已经使用两组不同新点位重新规划验证。

已验证结果包括：

```text
Test 1:
80 planner waypoints
2 gripper events

Test 2:
84 planner waypoints
2 gripper events

Test 2 Servo dry-run:
84 planner waypoints
→ 1194 dense Servo frames
→ 0.4° / 20 ms
→ joint-limit PASS
→ no real command sent
```

因此这不是固定轨迹 replay，而是实际重新执行：

```text
IK
RRT
collision checking
trajectory generation
```

---

# 3. 新任务快速开始

## 3.1 复制任务模板

```bash
mkdir -p .runtime/new-task

cp templates/task_template.json \
  .runtime/new-task/task.json
```

编辑：

```text
.runtime/new-task/task.json
```

用户侧单位：

```text
Position XYZ = mm
Orientation   = deg
```

示例：

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

Pipeline 内部会将 mm 转换为 PyBullet planner 使用的 m。

---

## 3.2 重新规划

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

```text
task.json
    用户输入；XYZ = mm

input_task.json
    planner 内部输入；XYZ = m

planned_candidate.json
    新规划得到的关节路点 + gripper events + meta

pipeline_report.json
    本次 pipeline 记录

COMMANDS.md
    针对本次 candidate 自动生成的后续 command
```

---

# 4. 新轨迹 Servo dry-run

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

`execute_servo_grasp.py` 会把 planner 的稀疏 waypoint 在运行时 densify 成 Servo stream。

因此：

```text
planned_candidate.json
≠ 已保存好的 dense frame JSON
```

而是：

```text
planned_candidate.json
        ↓
execute_servo_grasp.py
        ↓
0.4° densification
        ↓
20 ms Servo stream
```

---

# 5. Controller read-only precheck

现场 IP 不应直接照抄历史值。

先根据当前现场设置：

```bash
export XIFENG_ROBOT_IP=<current_robot_ip>
export XIFENG_ARM_IP=<current_arm_ip>
export XIFENG_LOCAL_IP=<current_local_ip>
```

然后：

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

此步骤用于当前 controller 状态检查，不授权真机运动。

---

# 6. 为什么旧 `validate_trajectory.py` 可能 FAIL，但 Servo 仍可 PASS？

`planned_candidate.json` 是 planner 的**稀疏关节路点**。

当前测试中观察到：

```text
max sparse waypoint step ≈ 6.67°
```

旧：

```text
src/pick_place_coord/validate_trajectory.py
```

使用：

```text
6.0° sparse-step gate
```

因此对新的 `planned_candidate.json` 直接运行旧 validator 可能 FAIL。

这不等于 Servo stream 失败。

Servo executor 会重新：

```text
6.x° sparse segment
        ↓
densify
        ↓
≤ 0.4° command increments
```

当前新任务主要离线验收口径：

```text
Planner PASS
+
Servo dry-run PASS
```

不要为了让旧 validator 通过而直接提高其 6° 阈值。

完整说明：

```text
docs/HANDOFF_STATUS.md
```

---

# 7. 主要代码在哪里？

## 新任务主线

| 文件 | 职责 |
|---|---|
| `templates/task_template.json` | 新 Pick / Place 输入模板，用户侧 mm |
| `tools/new_task_pipeline.py` | 新任务总入口；编排已有模块，不重新实现 IK/RRT |
| `src/pick_place_coord/pick_place_coord.py` | 规划主程序：IK、RRT、collision、trajectory export |
| `src/pick_place_coord/left_arm_ik.py` | 左臂 IK |
| `src/pick_place_coord/xifeng_pb.py` | PyBullet / URDF / collision 相关 |
| `src/frame_calibration/robot_side/execute_servo_grasp.py` | Servo executor；densification、限位检查、SDK 执行 |
| `src/frame_calibration/robot_side/sdk_session.py` | pypilot / SDK 封装 |
| `src/frame_calibration/robot_side/probe_sdk.py` | SDK 能力检查 |

更完整的代码职责：

```text
docs/CODE_MAP.md
```

---

# 8. 历史 B2 / B3 / B4 / B5 怎么复现？

历史复现与当前新任务开发是两条不同路线。

## 当前新任务开发

看：

```text
RUNBOOK.md
NEW_TASK_GUIDE.md
tools/new_task_pipeline.py
```

## 历史实验复现

看：

```text
robot/README.md
sim/README.md
```

核心工具：

```bash
python3 tools/export_version.py ...
python3 tools/restore_run.py ...
```

不要通过“找一个同名文件复制过来”猜历史版本。

### B2 Track A

```bash
python3 tools/export_version.py B2 .runtime/track-a-run
```

### B3 Track C Servo

```bash
python3 tools/export_version.py B3 .runtime/track-c-run
```

### B4 Servo

```bash
python3 tools/restore_run.py --list --node B4
```

然后按具体 run 恢复。

### B5 MPC

```bash
python3 tools/export_version.py B5 .runtime/mpc-field
```

Shadow / Active 的具体命令见：

```text
robot/README.md
```

---

# 9. MPC 当前状态

必须区分历史 MPC 和新任务 MPC。

## 历史 B5

已有：

```text
historical SDKALIGNED parent
        ↓
historical SERVO_CANDIDATE
        ↓
Servo 20 ms
        ↓
MPC Shadow / Active
```

历史代码、报告和恢复节点保留。

## 新任务

当前已经支持：

```text
new planned_candidate.json
        ↓
Servo 0.4° / 20 ms
```

当前**尚未正式接入**：

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

不是通用 converter。

它有历史 SDKALIGNED provenance / SHA gate，不允许新的 planner output 冒充历史 parent。

禁止简单把：

```text
planned_candidate.json
```

改名成：

```text
tabletop_pick_place_SERVO_CANDIDATE.json
```

来绕过 contract。

详细说明：

```text
docs/HANDOFF_STATUS.md
```

---

# 10. 环境

当前主要验证环境：

```text
Linux x86_64
CPython 3.10
PyBullet
pybullet_planning
NumPy
SciPy
jsonschema
pypilot 1.0.0422.1
OSQP 1.1.1（MPC）
```

检查 SDK：

```bash
python3 -c "import pypilot; print(pypilot.__file__)"
```

MPC 依赖：

```bash
python3 -m pip install \
  -r src/pick_place_coord/mpc_experiment/requirements.txt
```

---

# 11. 完整性与测试

验证 handoff 文件：

```bash
python3 tools/verify_handoff.py
```

当前冻结版本：

```text
767 files match
```

完整离线回归：

```bash
python3 tools/offline_checks.py
```

当前冻结版本：

```text
285 passed
0 failed
2 warnings
```

Package selection / manifest：

```bash
python3 tools/build_package.py --check
```

---

# 12. 文档导航

| 想知道什么 | 看哪里 |
|---|---|
| 第一次拿到仓库从哪里开始 | `START_HERE.md` |
| GitHub 首页 / 项目总览 | `README.md` |
| 当前到底做到什么、没做到什么 | `docs/HANDOFF_STATUS.md` |
| 所有常用 command | `RUNBOOK.md` |
| 新点位怎么继续开发 | `NEW_TASK_GUIDE.md` |
| 每个代码文件干什么 | `docs/CODE_MAP.md` |
| 真机 / Servo / B2-B5 | `robot/README.md` |
| 仿真 / 离线复现 | `sim/README.md` |
| 当前测试记录 | `verification.json` |
| 最终交付 manifest | `HANDOFF_MANIFEST.json` |

---

# 13. 推荐接手顺序

第一次接手：

```text
README.md
   ↓
START_HERE.md
   ↓
docs/HANDOFF_STATUS.md
   ↓
RUNBOOK.md
   ↓
NEW_TASK_GUIDE.md
   ↓
docs/CODE_MAP.md
```

然后自己用一个新的 mm Pick / Place 点位跑一次：

```text
task_template
→ new_task_pipeline
→ planned_candidate
→ Servo dry-run
```

跑通以后，再根据工作需要进入历史 B2/B3/B4/B5、视觉、hand-eye 或 MPC 模块。
