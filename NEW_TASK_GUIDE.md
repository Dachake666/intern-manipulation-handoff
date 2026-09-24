# 新点位继续开发指南

本页回答接手者最常见的问题：**拿到一个新的抓取/放置点后，如何继续使用本仓库，而不是只回放历史轨迹。**

## 先判断输入属于哪一类

### A. 固定工位，只改变抓取/放置 XYZ

这是当前最完整的新任务入口。规划器 `src/pick_place_coord/pick_place_coord.py` 已包含 IK、RRT-Connect(birrt)+shortcut、近物竖直段、碰撞检查、关节映射、夹爪事件和候选 JSON 导出。

任务文件采用 **PyBullet 世界系、米**：

```json
{
  "pairs": [
    {"pick": [0.32, 0.26, 1.05], "place": [0.30, 0.16, 1.05]}
  ],
  "obstacles": [
    {"center": [0.38, 0.235, 1.0475], "half": [0.08, 0.012, 0.0225]}
  ]
}
```

复制模板：

```bash
mkdir -p .runtime/new-task
cp templates/task_template.json .runtime/new-task/task.json
# 编辑 .runtime/new-task/task.json，只填写本次真实任务与场景
```

模板不是现场事实。特别是障碍物、桌面关系和点位必须由本次工位确认。

### B. 手里只有 SDK 世界系 XYZ（mm）

单组任务可以直接使用规划器已有转换，不需要手算：

```bash
cd src/pick_place_coord
python3 -B pick_place_coord.py \
  --pick-sdk  <PICK_X_MM> <PICK_Y_MM> <PICK_Z_MM> \
  --place-sdk <PLACE_X_MM> <PLACE_Y_MM> <PLACE_Z_MM> \
  --seed 7 --out ../../.runtime/new-task/planned_candidate.json
```

当前转换使用 `src/frame_calibration/analysis/calib_common.py` 中 `CONFIRMED_T_SESSIONS_MM["20260707"]`。这是历史确认常量，**不是新工位自动有效的标定**。若机器人/工位/坐标关系变化，先重新确认标定，不要直接复用。

### C. 新点位包含完整姿态，或来自视觉

不要把 XYZ-only task 模板硬扩成 `[x,y,z,u,v,w]`。完整位姿/视觉任务已有另一套契约：

- `src/robot_mission/tabletop_plan.py`：SDK-world 位姿任务，明确 `OBJECT_POSE / GRASP_POSE / EE_POSE`；
- `src/vision/`、`src/robot_mission/`：视觉观测到任务/候选的离线模块；
- `src/pick_place_coord/gen_tabletop_vision_replay_candidate.py`：视觉回放候选。

完整视觉自主抓放仍未作为整链验收成果。先读 `START_HERE.md` 和 `docs/CONTENTS.md` 的视觉/任务接口说明。

## XYZ 新任务：推荐统一入口

当前仓库提供 `tools/new_task_pipeline.py`。它只做 orchestration：调用现有 planner 和 validator，生成候选、SHA 报告与后续命令；**不会实现第二套 IK/RRT/Servo，也不会自动授权或启动真机运动**。

最短流程：

```bash
cd "$HANDOFF_ROOT"
mkdir -p .runtime/new-task-001
cp templates/task_template.json .runtime/new-task-001/task.json
# 编辑 task.json 中本次真实 pick/place/obstacles

python3 tools/new_task_pipeline.py \
  --task .runtime/new-task-001/task.json \
  --out .runtime/new-task-001 \
  --stage prepare --seed 7 --arm left
```

`prepare` 顺序执行：task schema 基础检查 → `pick_place_coord.py` 重新规划 → `validate_trajectory.py` 静态验证 → 写出 provenance/SHA 与 `COMMANDS.md`。输出：

```text
.runtime/new-task-001/
├── input_task.json
├── planned_candidate.json
├── pipeline_report.json
└── COMMANDS.md
```

若只想先检查输入和交接依赖，不运行 PyBullet：

```bash
python3 tools/new_task_pipeline.py \
  --task .runtime/new-task-001/task.json \
  --out .runtime/new-task-001 --stage check
```

也可分阶段用 `--stage plan` / `--stage validate`。`COMMANDS.md` 会给出当前新候选的离线 Track-C dry-run 和 controller read-only precheck 命令；真机执行仍是显式人工步骤。

> 这条统一入口针对现有 XYZ planner 契约（PB world, m）。完整 SDK-world 姿态/视觉任务仍走本页 C 类的 `robot_mission` 契约，不应硬塞进 XYZ task。

## XYZ 新任务：底层手动流程（调试/理解用）

### 1. 建立开发副本

从完整仓库根目录：

```bash
python3 tools/export_version.py B1 .runtime/new-task-work
```

也可以直接在当前 `src/` 开发，但所有新输出仍放 `.runtime/`，不要覆盖 `verified/`、历史轨迹或 `latest` 别名。

### 2. 修改任务

在工作副本中：

```bash
cp .runtime/new-task-work/templates/task_template.json \
   .runtime/new-task-work/src/pick_place_coord/tasks/my_new_task.json
```

字段：

- `pairs[].pick/place`：PyBullet 世界系 XYZ，单位 m；
- `obstacles[].center`：轴对齐障碍物中心，单位 m；
- `obstacles[].half`：障碍物 XYZ 半尺寸，单位 m；
- 不存在的障碍物应删除，新增实体必须进入场景模型后才能声称做过碰撞检查。

左臂当前配置工作空间见 `src/arm_profiles.v1.json`；模板只用于说明格式，不替代现场检查。

### 3. 重新规划并导出候选

```bash
cd .runtime/new-task-work/src/pick_place_coord
XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord.py \
  --task tasks/my_new_task.json \
  --seed 7 \
  --out ../../../new-task-planned.json
```

该命令重新运行 IK / RRT / 碰撞检查并输出新的候选。成功输出**不继承任何历史真机资格**。

静态轨迹检查：

```bash
cd "$HANDOFF_ROOT"
python3 src/pick_place_coord/validate_trajectory.py \
  .runtime/new-task-work/new-task-planned.json
```

### 4. 不要把历史 B4/B5 候选直接换成新 planner JSON

B4/B5 的 `tabletop_pick_place_SERVO_CANDIDATE.json` 是各轮 SHA 绑定的历史输入。`pick_place_coord.py` 新输出与 B4/B5 Servo/MPC field candidate **不是同一个契约**。

`tools/new_task_pipeline.py` 现在把新 XYZ planner 输出直接接到 **Track-C legacy consumer contract**：planner 导出的 `meta + waypoints(q_sdk_deg/gripper)` 正是 `execute_servo_grasp.py` 能读取的格式；执行器在运行时按 `--step-deg/--period-ms` 加密。因此，新同事不再需要人工猜“planner JSON 下一步给谁”。

这**仍不等于**自动生成历史 B4 的 `tabletop_servo_candidate.v1`。B4/B5 是另一套 SHA 绑定的 field/MPC contract；不要改名冒充。需要进入 B4/MPC 路线时，仍按任务类型选择并验证对应 generator/contract；相关代码入口：

- `src/pick_place_coord/gen_tabletop_hybrid_candidate.py`
- `src/robot_mission/qualify_candidate.py`
- `src/pick_place_coord/authorize_servo_candidate.py`
- `src/frame_calibration/robot_side/tabletop_servo_contract.py`

这些模块不能仅凭存在就视为已经对任意新 planner 输出完成端到端适配。

## 新同事必须从现场取得的信息

代码和算法随仓库交付，但以下信息不能从历史记录自动推定：

1. 本次 pick/place 的**坐标系、单位和参考点**；完整姿态还要给旋转表示/顺序；
2. 当前桌面、障碍物、料框、物体尺寸以及机器人 base 与工位的关系；
3. 当前 TCP/夹爪/工具配置以及所用标定是否仍有效；
4. 机器人、机械臂、本机网络地址，以及 arm/gripper 身份；
5. 若来自视觉：本次有效手眼外参、相机配置、目标 pose role；
6. 若要真机：现场负责人确认的当前姿态、环境和执行许可。

历史 IP、历史标定、历史成功轨迹都只是证据，不是新工位默认值。

## 什么算“已经复现”

- `pick_place_coord.py` 重新规划成功：说明新任务在当前离线模型中得到候选；
- `validate_trajectory.py` PASS：说明静态轨迹门通过；
- GUI/qualification PASS：只对其绑定 SHA 的候选成立；
- B2/B3/B4/B5 历史恢复 PASS：只证明历史成果可恢复，不证明新点位已经接入相同执行链；
- 真机结果必须单独产生新日志和 SHA 绑定，不继承旧结果。

历史复现命令见 `sim/README.md`、`robot/README.md`；环境安装见 `docs/ENVIRONMENT_REFERENCE.md`；版本/发布见 `docs/MAINTENANCE.md`。
