# Robot Manipulation Handoff

> **上传状态：当前先发布项目说明。完整源码、验证记录和 Git 历史尚未从 Debian 工作区同步；下面的文件链接在对应内容上传后生效。不要将本页上传等同于完整仓库发布完成。**

机器人抓放、运动规划、Servo 执行及历史 MPC 实验的工程交接项目。本次发布范围是**新任务重新规划 + Servo 离线 dry-run**；不包含新任务直接接入历史 B5 MPC。

## 从哪里开始

| 你的目的 | 文档入口 | 主要内容 |
|---|---|---|
| 第一次接手 | [START_HERE.md](START_HERE.md) | 交接目录、推荐入口、历史恢复方式 |
| 上级 review / 确认边界 | [docs/HANDOFF_STATUS.md](docs/HANDOFF_STATUS.md) | 已验证范围、实测结果、未接入能力 |
| 复制运行命令 | [RUNBOOK.md](RUNBOOK.md) | 新任务规划、Servo dry-run、只读预检、历史恢复 |
| 输入新点位继续开发 | [NEW_TASK_GUIDE.md](NEW_TASK_GUIDE.md) | 任务输入、规划输出、后续开发说明 |
| 查代码职责 | [docs/CODE_MAP.md](docs/CODE_MAP.md) | 核心、辅助、实验、历史与测试代码分类 |
| 复现历史 B2/B3/B4/B5 | [robot/README.md](robot/README.md) | 精确恢复、dry-run、预检及现场运行说明 |
| 仿真与离线复现 | [sim/README.md](sim/README.md) | 离线仿真和历史节点入口 |
| 查看测试证据 | [verification.json](verification.json) | 测试记录和对应源码身份 |
| 检查交付完整性 | [HANDOFF_MANIFEST.json](HANDOFF_MANIFEST.json) | 文件清单与 SHA-256 |

建议新同事按 **START_HERE → HANDOFF_STATUS → RUNBOOK → CODE_MAP** 阅读；review 时先看能力边界和验证证据，不必从全部源码开始。

## 当前新任务主线

```text
任务 JSON：Pick / Place XYZ（用户输入 mm）
        ↓
tools/new_task_pipeline.py：mm → m
        ↓
src/pick_place_coord/pick_place_coord.py
        ↓
IK → RRT-Connect → PyBullet 场景检查
        ↓
planned_candidate.json：关节路点 + 夹爪事件 + meta
        ↓
src/frame_calibration/robot_side/execute_servo_grasp.py
        ↓
运行时按 0.4° 最大步长展开，名义周期 20 ms
        ↓
Servo dry-run（未下发真机）
```

**单位转换不等于坐标系变换。**这轮测试沿用原 PyBullet 场景坐标，将原米制点位用毫米表达后传入；不能据此把任意相机或 SDK world 坐标直接当作该规划场景坐标。现场使用应先核对坐标系、标定和场景假设。当前入口实测的是 XYZ 点位任务，不应把文档中的角度单位说明理解为已经验收任意 UVW 姿态输入。

### 本次已观察到的验证结果

以下来自 Debian 端实际运行记录，不代表任意新任务或真机安全资格：

| 测试 | 输入（同一规划场景，mm） | 观察结果 |
|---|---|---|
| 新任务 1 | pick `[320,260,1050]`；place `[300,160,1050]` | 80 个运动路点、2 个夹爪事件；仿真任务完成 |
| 新任务 2 | pick `[360,300,1050]`；place `[420,180,1050]` | 84 个运动路点、2 个夹爪事件；仿真任务完成 |
| 任务 2 Servo dry-run | 新生成的 `planned_candidate.json` | 84 路点限位检查通过；1194 个密集点；名义约 23.9 s；未下发 |
| 离线回归 | `tools/offline_checks.py --record` | 285 passed，2 warnings |
| 加入首页后的交付检查 | `build_package.py --check` 与 `verify_handoff.py` | 用户本地报告 768 个文件校验通过 |

当前 `--stage prepare` 实测完成规划、候选导出和命令文档生成；**没有自动完成完整 preflight、qualification 或新任务 MPC 验收**。

## 新任务快速开始

以下命令在已配置依赖的 Python 环境、仓库根目录执行；源码尚未上传完成时不要直接运行。

```bash
mkdir -p .runtime/new-task
cp templates/task_template.json .runtime/new-task/task.json
```

修改任务中的 `pairs`，同时核对 `obstacles` 与实际规划场景。示例点位：

```json
{
  "pairs": [
    {
      "pick": [320, 260, 1050],
      "place": [300, 160, 1050]
    }
  ]
}
```

这里仅展示 `pairs` 字段，不替代包含单位、障碍物及说明的完整模板。

```bash
XIFENG_ALLOW_REAL_MOTION=0 \
python3 tools/new_task_pipeline.py \
  --task .runtime/new-task/task.json \
  --out .runtime/new-task \
  --stage prepare \
  --seed 7 \
  --arm left
```

主要输出：

```text
.runtime/new-task/
├── task.json                # 用户输入，位置 mm
├── input_task.json          # 转换后的规划输入，位置 m
├── planned_candidate.json   # 稀疏关节路点及夹爪事件
├── pipeline_report.json
└── COMMANDS.md
```

### 新轨迹 Servo dry-run

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

观察路点限位检查、密集点数量与 `DRY RUN: 未下发`。密集点在执行器内部生成，不一定另存为 JSON；名义周期和预计时长不代表实测硬实时性能。

### 只读控制器预检

这是需要现场机器人连接的独立步骤，本页不把它计入上述两组新轨迹的已完成验收。命令与现场参数见 [RUNBOOK.md](RUNBOOK.md) 和 [robot/README.md](robot/README.md)。不沿用历史 IP，不自动运行真机。

## 稀疏路点检查与 Servo 检查不能混为一谈

任务 1 的稀疏路点曾在 `validate_trajectory.py` 中出现最大相邻变化约 6.67°，超过该检查的 6° 阈值，因而返回 FAIL。这条失败记录应保留，不应标为通过。

Servo 执行器按 0.4° 展开路点属于另一层检查。**插值通过并不自动消除原检查的全部风险，也不证明密集路径无碰撞、动力学合格或真机可安全运行。**不得为了通过检查而修改旧阈值或冒充历史已验收轨迹。当前冻结的是规划示例与 Servo dry-run 的已验证范围。

## 代码职责

| 文件 / 目录 | 职责 |
|---|---|
| `templates/task_template.json` | 新任务输入模板 |
| `tools/new_task_pipeline.py` | 编排输入转换、现有规划器与输出记录 |
| `src/pick_place_coord/pick_place_coord.py` | 抓放任务规划、IK/RRT、仿真和轨迹导出 |
| `src/pick_place_coord/left_arm_ik.py` | 左臂逆运动学 |
| `src/pick_place_coord/xifeng_pb.py` | PyBullet 机器人模型与相关支持 |
| `src/frame_calibration/robot_side/execute_servo_grasp.py` | 关节路点读取、插值和 Servo 执行入口 |
| `src/frame_calibration/robot_side/sdk_session.py` | SDK 会话与操作封装 |
| `src/frame_calibration/robot_side/probe_sdk.py` | SDK 接口能力探针 |
| `src/pick_place_coord/mpc_experiment/` | MPC 控制器、仿真对照和测试 |
| `tools/export_version.py` | 导出历史节点，不执行机器人代码 |
| `tools/restore_run.py` | 按运行记录恢复绑定文件组合 |
| `tools/verify_handoff.py` | 文件完整性检查 |
| `tools/build_package.py` | 交付清单、manifest 与打包 |

更多职责及维护边界见 [docs/CODE_MAP.md](docs/CODE_MAP.md)。历史 `records`、`evidence` 与 Git 对象不是因同名就可以删除的冗余文件。

## 历史复现：与新任务开发分开

在完整仓库根目录执行：

```bash
python3 tools/export_version.py B2 .runtime/track-a-run
python3 tools/export_version.py B3 .runtime/track-c-run
python3 tools/restore_run.py --list --node B4
python3 tools/export_version.py B5 .runtime/mpc-field
```

这些是恢复入口，不会自动复现实机运动。各节点恢复后的目录、参数、dry-run 与现场操作详见 [robot/README.md](robot/README.md)；离线仿真见 [sim/README.md](sim/README.md)。

**B5 wrapper 在历史节点中恢复，不是因为当前 `src/` 下找不到同名文件就代表已遗失。**历史运行报告与算法文件的逐字节来源证明也应分开；既有文档记录了当时 controller SHA 的归档缺口。

## MPC 边界

历史 B5 使用已有 Servo reference，在执行层进行 Shadow / Active 实验，而不是替代 IK/RRT 规划。

```text
历史 SDKALIGNED parent
→ 历史 SERVO_CANDIDATE
→ B5 Servo + MPC
```

新任务当前已验证：

```text
new planned_candidate.json
→ execute_servo_grasp.py
→ Servo dry-run
```

尚未接入：

```text
new planned_candidate.json
→ generic new-task Servo/MPC candidate
→ B5 MPC
```

`gen_tabletop_hybrid_candidate.py --servo-parent` 有历史 SDKALIGNED 来源保护，不是通用转换器；不能通过改名、删除检查或沿用历史 SHA/review 来接入新任务。Shadow 在真实运行模式下也发送原参考运动，不能把 Shadow 当作 dry-run。

## 环境与检查入口

本次已使用的环境包括 Linux x86_64、CPython 3.10、PyBullet、pybullet_planning、NumPy、SciPy、jsonschema、厂商 `pypilot 1.0.0422.1`；MPC 依赖 OSQP，本轮安装确认版本为 1.1.1。

```bash
python3 --version
python3 -c "import pypilot; print(pypilot.__file__)"
python3 tools/verify_handoff.py
python3 tools/build_package.py --check
python3 tools/offline_checks.py
```

MPC 依赖文件入口：

```bash
python3 -m pip install \
  -r src/pick_place_coord/mpc_experiment/requirements.txt
```

具体依赖范围、SDK 来源和平台限制见 [docs/ENVIRONMENT_REFERENCE.md](docs/ENVIRONMENT_REFERENCE.md)。不要把第三方同名包当作公司厂商 SDK 安装。

## 交接结论

已在用户 Debian 工作区保存并测试的是“新任务规划 + Servo dry-run”范围；generic 新任务 MPC 不在该冻结版承诺内。完整 Git 历史、源码、manifest 与测试证据应一并同步后再宣布 GitHub 发布完成。
