# 真机执行与历史实验

真机执行开发源在 `src/frame_calibration/robot_side/`；`src/robot_mission/mission_runner.py` 编排任务并调用执行器 `--run`。先选择具体成果和运行记录，再恢复完整组合。只恢复文件不会连接设备；历史脚本的启动行为需要逐项核实。 本文 B2～B5 主要用于历史成果恢复；**新的抓取/放置点不要直接替换历史绑定 JSON**，先按 [NEW_TASK_GUIDE](../NEW_TASK_GUIDE.md) 重新规划并建立新的候选/资格证据。

## 选择成果

| 节点 | 执行入口 | 已完成成果与归档情况 |
|---|---|---|
| B2 Track A 固定场景 | `execute_tabletop_hybrid_trial_reviewfix_field.py` + `tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json` | 5% 一轮、10% 四轮完整成功记录；只支持相同文件组合、任务与场景 |
| B3 Track C 双次抓放 | `execute_servo_grasp.py` + `traj_multi_latest.json` | 已跑出实机结果：原始说明记载双次抓放连续 5 轮成功（133 运动点 + 4 夹爪事件）；完整逐轮日志与源码绑定待补，归档证据标记 LIMITED |
| B4 Servo | `_20ms.py`、`_25ms.py`、`_vaj20_v3.py` 等 field wrapper | 61 份非 MPC 现代 JSON；各轮依赖不同。正式资格绕过与 VAJ3 算法哈希缺口仍在 |
| B5 MPC | `execute_tabletop_servo_field_mpc_20ms.py`、`_mpc_active_20ms.py`、各自 base 与 controller | 已有 6 次 Shadow、4 次 Active 实机运行报告；待补当时 controller 的版本 SHA，以核对实际运行的算法源码 |
| B6 MoveJ 基础 | 99 点冻结组合、153 点运行后安全快照 | 99/88/153 点历史完成日志；不具备统一的现代源码与收尾证据 |
| B7 瓶子示教 | 238 点与带 Home 收尾的 275 点轨迹 | 五轮完整抓放日志缺失；使能/8080 诊断日志不能替代运动日志，LIMITED |
| D1 / D2 / D3 | Servo 长迟到 / MPC 早期失效 / MoveJ 限位与跳点 | D1 有精确对照源码；D2/D3 部分仅有报告，不承诺重新触发故障 |

Track C 与 MPC 的实机成果已经产生；下文的 LIMITED、missing 和未绑定项描述归档缺口，不表示未运行或运行失败。Track C 待补完整逐轮日志，MPC 已有运行报告，待补 controller 的历史版本身份。

Shadow 仍发送原参考轨迹到机器人，只是不应用 MPC 修正，因此属于运动实验。

## 恢复整个节点

```bash
python3 tools/export_version.py B2 .runtime/track-a-reference
python3 tools/export_version.py B3 .runtime/track-c-reference
```

目标目录必须不存在。主要路径如下：

| 节点 | 导出目录中的路径 |
|---|---|
| B2 | `debian/trackA_worlds_line/trackA_hybrid_trial/` |
| B3 | `debian/trackC_servo_stream/trackC_servo_stream/` |
| B4 | `debian/trackA_worlds_line/trackA_servo/` |
| B5 | field/base 在 `debian/trackA_worlds_line/trackA_servo/`；controller 在 `debian/mpc_experiment/mpc_experiment/` |
| B6 | `debian/sdk_tests/frame_calibration/records/` |
| B7 | `debian/trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/` |

`debian/` 是历史来源布局，不是运行类别。`SNAPSHOT.json` 给出来源和 SHA；同名 `sdk_session.py`、`servo_common.py`、wrapper 不能跨节点互换。

## 按一轮报告装配

```bash
python3 tools/restore_run.py --list --node B4
python3 tools/restore_run.py --list --node B5

python3 tools/restore_run.py trackA_worlds_line/trackA_servo/tabletop_servo_20260909_182932_719907228.json .runtime/servo-run-reference
```

工具根据 `docs/debian_selection.json` 的 `run_bindings` 读取精确源码、验证 SHA 并按 `restore_name` 装配，写出 `RESTORE_MANIFEST.json`。装配不执行脚本。必须阅读 manifest 的缺失/未绑定项；MPC controller 没有历史哈希时不会自动选择同名实现，缺口保留为 missing。

B5 field/base 顶层导入 `controller`，节点导出保留它们各自的来源目录。直接在 Servo 目录执行 field，即使传 `--dry-run`，也可能先发生导入错误。装配与路径核对不应绕过算法身份缺口。只需要复核历史结果时，使用 [仿真说明](../sim/README.md) 中的标准库复算工具。

## Debian 启动前的公共设置

下列命令在 **Debian 的 Bash** 中执行。先激活原来可运行 SDK 的 CPython 3.10 环境，再进入本交接仓库根目录（包含 `START_HERE.md` 和 `tools/`）。SDK wheel 只支持 Linux x86_64；MPC 还需要该环境中的 NumPy、SciPy、OSQP。不要把 macOS 仿真锁直接安装到 SDK 环境。

```bash
export HANDOFF_ROOT="$PWD"
export ROBOT_PY="$(python3 -c 'import sys; print(sys.executable)')"
python3 tools/verify_handoff.py

read -r -p '本次机器人控制器 IP: ' XIFENG_ROBOT_IP
read -r -p '本机连接机器人网卡的 IP: ' XIFENG_LOCAL_IP
read -r -p '本次机械臂服务 IP: ' XIFENG_ARM_IP
export XIFENG_ROBOT_IP XIFENG_LOCAL_IP XIFENG_ARM_IP
set -o pipefail
```

三个 IP 由现场负责人确认，不沿用历史记录中的地址；`ROBOT_PY` 固定为刚激活的解释器。下面每项的恢复目录必须尚不存在；已恢复时从该项的检查步骤继续。各代码块独立执行，**不要把整篇文档作为脚本一次运行**。

| 操作 | 实际行为 |
|---|---|
| `verify_handoff.py` / `export_version.py` / `restore_run.py` | 只读校验或恢复文件，不连接机器人 |
| Track C 原脚本且 `ENABLE_REAL_MOTION=False` | 本地检查，不建立会话，但顶层需要厂商SDK导入 |
| Track A / Servo / MPC 的显式 `--dry-run` | 代码中的离线分支；仍需满足导入依赖 |
| Track A / Servo / MPC 的 `--precheck-only` | 连接 SDK 做当前状态检查；不等于完全离线 |
| 标为“真机运动”的代码块 | 现场确认后才执行，会使能并发送运动/夹爪指令 |

以下命令经过归档源码和参数核对；本次编写未连接机器人，未在 Debian 重新完成运动验收。历史实验结果继续保留，运行时配置与当前场景需另行确认。

## Track A：B2 固定场景

恢复原组合，先执行离线文件检查：

```bash
python3 tools/export_version.py B2 .runtime/track-a-run
(
  cd "$HANDOFF_ROOT/.runtime/track-a-run/debian/trackA_worlds_line/trackA_hybrid_trial" || exit
  "$ROBOT_PY" execute_tabletop_hybrid_trial_reviewfix_field.py \
    tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json --dry-run
)
```

连接 SDK 的不运动检查：

```bash
(
  cd "$HANDOFF_ROOT/.runtime/track-a-run/debian/trackA_worlds_line/trackA_hybrid_trial" || exit
  "$ROBOT_PY" execute_tabletop_hybrid_trial_reviewfix_field.py \
    tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json \
    --precheck-only --speed 5
)
```

**真机运动：** 下面使用历史已有结果的 5% 配置，程序在使能前仍要求现场人员确认；不要用管道自动输入回车。

```bash
(
  cd "$HANDOFF_ROOT/.runtime/track-a-run/debian/trackA_worlds_line/trackA_hybrid_trial" || exit
  XIFENG_ALLOW_REAL_MOTION=1 "$ROBOT_PY" -u execute_tabletop_hybrid_trial_reviewfix_field.py \
    tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json \
    --run --speed 5 2>&1 | tee "terminal_$(date +%Y%m%d_%H%M%S).raw.log"
)
```

这份固定 v2 计划不需要 `--accept-blockers`。不添加 `--auto-continue`，保留程序交互。成功结果应包括 `PASS_RETURNED_SAFE` 和 `cleanup.status=PASS`；失败可能不写运行 JSON，必须保留终端日志。

- 轨迹 SHA：`e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59`。
- 执行器 SHA：`36a0f659122eb492f63616f5c34a7e3b1f342c67f28e9906e2b9d6ad5b04e7ee`。
- SDK XYZUVW 已是补偿后的 `EE_POSE`，不能再次补 TCP；`PLACE_HOVER` 关节构型保持冻结值。
- `src/releases/make_release.py AT` 的依赖来自当前开发源码，与上面恢复的 B2 完整历史组合不同；重新生成的组合单独验证。

## Track C：B3 双次抓放

先恢复冻结组合。归档脚本的 `ENABLE_REAL_MOTION=False`，下面命令仅做本地限位检查和插值；但顶层导入厂商 SDK，仍须使用已安装 SDK 的 Linux 解释器。

```bash
python3 tools/export_version.py B3 .runtime/track-c-run
(
  cd "$HANDOFF_ROOT/.runtime/track-c-run/debian/trackC_servo_stream/trackC_servo_stream" || exit
  "$ROBOT_PY" -u execute_servo_grasp.py traj_multi_latest.json \
    --step-deg 0.4 --period-ms 20 --speed 15 --arm-id 1
)
```

预期133运动点、4夹爪事件、1633帧，纯流式时间32.66秒。该脚本没有 `--run`、`--dry-run` 或 `--precheck-only`，也不读取 IP 环境变量；不要把其他执行器的参数附加给它。

**真机运动：** 以下启动器读取公共设置中的 IP，显式设置这一次运行的模块配置，不改归档脚本字节；先要求操作者输入 `RUN TRACK C`，再允许调用真机入口。原入口会清故障、使能、张开左爪，并可能先移到首点，然后才提示回车开始透传，因此必须在启动器确认前完成现场检查。

```bash
(
  cd "$HANDOFF_ROOT/.runtime/track-c-run/debian/trackC_servo_stream/trackC_servo_stream" || exit
  XIFENG_ALLOW_REAL_MOTION=1 "$ROBOT_PY" -u -c '
import hashlib, ipaddress, json, os, sys, time
from pathlib import Path
if os.environ.get("XIFENG_ALLOW_REAL_MOTION") != "1":
    raise SystemExit("Real motion is not enabled")
addresses = {key: os.environ["XIFENG_" + key] for key in ("ROBOT_IP", "LOCAL_IP", "ARM_IP")}
for value in addresses.values():
    ipaddress.ip_address(value)
import execute_servo_grasp as track
for key, value in addresses.items():
    setattr(track, key, value)
track.ENABLE_REAL_MOTION = True
launcher_source = sys.orig_argv[sys.orig_argv.index("-c") + 1]
record = {"launcher_source": launcher_source,
          "launcher_sha256": hashlib.sha256(launcher_source.encode()).hexdigest(),
          "runtime_overrides": {**addresses, "ARM_PORT": track.ARM_PORT, "ENABLE_REAL_MOTION": True},
          "arguments": sys.argv[1:], "python": sys.executable,
          "source_sha256": {name: hashlib.sha256(Path(name).read_bytes()).hexdigest()
                            for name in ("execute_servo_grasp.py", "sdk_session.py", "servo_common.py", "robot_lock.py", sys.argv[1])}}
Path("trackc_startup_" + str(time.time_ns()) + ".json").write_text(json.dumps(record, indent=2) + "\n")
print(json.dumps(record, indent=2))
if input("现场确认：将使能、开左爪并移首点；输入 RUN TRACK C 才继续：").strip() != "RUN TRACK C":
    raise SystemExit("Operator aborted before opening a robot session")
raise SystemExit(track.main())
' traj_multi_latest.json --step-deg 0.4 --period-ms 20 --speed 15 --arm-id 1 \
    2>&1 | tee "terminal_$(date +%Y%m%d_%H%M%S).raw.log"
)
```

运行结束核对保护回读、原全局速度恢复和 SDK 关闭；旧脚本初始化异常存在收尾缺口，不以退出码代替检查。启动 JSON 记录实际配置与文件 SHA，终端日志记录动作过程。原始说明记载连续5轮成功，完整逐轮日志仍待补；本次新运行单独归档。

轨迹 SHA 为 `ac5c6fd7715c81fbfa2f2fc4c9eca1f676b5a3232dff99761a02c7b4812e4968`，执行器 SHA 为 `808f12e1baa8a3b58053cf6d2b42a5782543a0de643d70987b856c66d4b0e4a1`。B3旧 `SHA256SUMS` 还列有未随节点提供的右臂/诊断文件；本左臂任务使用恢复节点的 `SNAPSHOT.json` 核对已有文件，不拿其他版本补同名依赖。

## Servo：B4 20ms / 25ms / 30ms

每种实验用自己的逐轮装配目录。以下三条恢复命令不运行机器人，均可恢复14个已绑定文件：

```bash
python3 tools/restore_run.py trackA_worlds_line/trackA_servo/tabletop_servo_20260914_151120_671751502.json .runtime/servo-20ms
python3 tools/restore_run.py trackA_worlds_line/trackA_servo/tabletop_servo_20260914_101507_112677088.json .runtime/servo-25ms
python3 tools/restore_run.py trackA_worlds_line/trackA_servo/tabletop_servo_20260910_151044_480345410.json .runtime/servo-30ms
```

| 选择 | `SERVO_DIR` 后缀 | `SERVO_ENTRY` | 节拍与接入 |
|---|---|---|---|
| 20ms | `.runtime/servo-20ms` | `execute_tabletop_servo_field_20ms.py` | stride4，连续Servo桥接 |
| 25ms | `.runtime/servo-25ms` | `execute_tabletop_servo_field_25ms.py` | stride4，连续Servo桥接 |
| 30ms | `.runtime/servo-30ms` | `execute_tabletop_servo_field.py` | stride2，先MoveJ接首点 |

先选一套；下面默认20ms。切换25/30ms时同时替换两个变量，候选和 GUI 文件留在各自恢复目录中，不能交叉复制：

```bash
export SERVO_DIR="$HANDOFF_ROOT/.runtime/servo-20ms"
export SERVO_ENTRY=execute_tabletop_servo_field_20ms.py
```

离线检查：

```bash
(
  cd "$SERVO_DIR" || exit
  XIFENG_ARM_PROFILES="$PWD/arm_profiles.v1.json" XIFENG_ALLOW_REAL_MOTION=0 \
    "$ROBOT_PY" "$SERVO_ENTRY" tabletop_pick_place_SERVO_CANDIDATE.json \
    --field-trial --dry-run --speed 5 --gui-review tabletop_pick_place_SERVO_gui_review.json \
    --report "dryrun_$(date +%Y%m%d_%H%M%S).json"
)
```

连接 SDK 的不运动检查：

```bash
(
  cd "$SERVO_DIR" || exit
  XIFENG_ARM_PROFILES="$PWD/arm_profiles.v1.json" XIFENG_ALLOW_REAL_MOTION=0 \
    "$ROBOT_PY" "$SERVO_ENTRY" tabletop_pick_place_SERVO_CANDIDATE.json \
    --field-trial --precheck-only --speed 5 --gui-review tabletop_pick_place_SERVO_gui_review.json \
    --report "precheck_$(date +%Y%m%d_%H%M%S).json"
)
```

**真机运动：** 保留原程序的现场回车确认；30ms还会在MoveJ接首点后再次确认。

```bash
(
  cd "$SERVO_DIR" || exit
  RUN_ID="$(date +%Y%m%d_%H%M%S)"
  XIFENG_ARM_PROFILES="$PWD/arm_profiles.v1.json" XIFENG_ALLOW_REAL_MOTION=1 \
    "$ROBOT_PY" -u "$SERVO_ENTRY" tabletop_pick_place_SERVO_CANDIDATE.json \
    --field-trial --run --speed 5 --gui-review tabletop_pick_place_SERVO_gui_review.json \
    --report "run_$RUN_ID.json" 2>&1 | tee "terminal_$RUN_ID.raw.log"
)
```

周期和stride由wrapper固定，没有 `--frame-stride` 参数。`--field-trial` 使用历史现场试验路径，不能视为正式资格通过；`--dry-run` / `--precheck-only` 不执行wrapper的完整GUI/候选白名单核验，该核验在 `--run` 阶段执行。

VAJ3历史入口为 `execute_tabletop_servo_field_vaj20_v3.py`，参数形状与上面相同，但逐轮装配缺少已绑定的 `vaj3_spline.py`，连顶层导入也依赖它。先从原Debian确认该轮算法源码并记录SHA，再单独复跑；不能用20/25ms目录或最新同名算法补齐后冒称同一版本。

## MPC：B5 Shadow / Active

恢复完整节点。该节点的10份报告各13项已记录源码/轨迹绑定均与节点一致，固定GUI文件也一致。现场入口不需要PyBullet或URDF；需要SDK、NumPy、SciPy、OSQP。

```bash
python3 tools/export_version.py B5 .runtime/mpc-field
export MPC_SERVO="$HANDOFF_ROOT/.runtime/mpc-field/debian/trackA_worlds_line/trackA_servo"
export MPC_CORE="$HANDOFF_ROOT/.runtime/mpc-field/debian/mpc_experiment/mpc_experiment"
export MPC_ENTRY=execute_tabletop_servo_field_mpc_20ms.py
```

`MPC_ENTRY` 当前选择 **Shadow**；选择 **Active** 时改为：

```bash
export MPC_ENTRY=execute_tabletop_servo_field_mpc_active_20ms.py
```

**controller选择必须显式记录。** 上面的 `MPC_CORE` 使用包内 `controller.py`，SHA 为 `314f5cbe6c43876027bb1d25dd3a24515e4790d054a818281c78f26bc24c515c`。旧10轮报告没有记录controller SHA，以下可作为明确记录当前组合的新复跑，不能据此认定算法与旧10轮逐字节相同。需要精确历史复现时，先核对原Debian当时的controller备份/提交，再将 `MPC_CORE` 指向确认的算法目录；不要默认选择 `pre_rt_diagnostics` 备份。

离线检查（Shadow与Active使用同一命令结构）：

```bash
(
  cd "$MPC_SERVO" || exit
  PYTHONPATH="$MPC_CORE" XIFENG_ARM_PROFILES="$PWD/arm_profiles.v1.json" XIFENG_ALLOW_REAL_MOTION=0 \
    "$ROBOT_PY" "$MPC_ENTRY" tabletop_pick_place_SERVO_CANDIDATE.json \
    --field-trial --dry-run --speed 5 --gui-review tabletop_pick_place_SERVO_gui_review.json \
    --report "mpc_dryrun_$(date +%Y%m%d_%H%M%S).json"
)
```

连接 SDK 的不运动检查：

```bash
(
  cd "$MPC_SERVO" || exit
  PYTHONPATH="$MPC_CORE" XIFENG_ARM_PROFILES="$PWD/arm_profiles.v1.json" XIFENG_ALLOW_REAL_MOTION=0 \
    "$ROBOT_PY" "$MPC_ENTRY" tabletop_pick_place_SERVO_CANDIDATE.json \
    --field-trial --precheck-only --speed 5 --gui-review tabletop_pick_place_SERVO_gui_review.json \
    --report "mpc_precheck_$(date +%Y%m%d_%H%M%S).json"
)
```

**真机运动：Shadow也驱动机器人，只是不应用MPC修正。** 以下显式记录controller身份，保留原程序回车确认；结果应核对 `SERVO_COMPLETED_RETURNED_HOME` 和 `cleanup.status=PASS`。

```bash
(
  cd "$MPC_SERVO" || exit
  RUN_ID="$(date +%Y%m%d_%H%M%S)"
  sha256sum "$MPC_CORE/controller.py" > "mpc_controller_$RUN_ID.sha256" || exit
  PYTHONPATH="$MPC_CORE" XIFENG_ARM_PROFILES="$PWD/arm_profiles.v1.json" XIFENG_ALLOW_REAL_MOTION=1 \
    "$ROBOT_PY" -u "$MPC_ENTRY" tabletop_pick_place_SERVO_CANDIDATE.json \
    --field-trial --run --speed 5 --gui-review tabletop_pick_place_SERVO_gui_review.json \
    --report "mpc_run_$RUN_ID.json" 2>&1 | tee "terminal_$RUN_ID.raw.log"
)
```

本节显式5%速度、20ms周期，与收到的10轮报告参数一致；省略 `--speed` 会回到CLI默认3%。`PYTHONPATH` 只指向本次选择的算法目录，不把其他SDK/执行器目录加进去混用。

## 日志保存与归档

以上运行均在独立 `.runtime/` 目录，报告采用新的文件名。部分历史wrapper会删除失败时自动生成的 `tabletop_servo_*.json`，因此命令使用自定义 `run_` / `mpc_run_` 名称并另存终端日志。每次运行保留启动配置、controller SHA、报告和终端日志；重跑不要覆盖原件。

终端原日志可能包含控制器token。只对副本脱敏，检查通过后再把副本归档进Git；例如在实际运行目录中，将文件名替换为本次日志：

```bash
cp terminal_实际时间.raw.log terminal_实际时间.scrubbed.log
"$ROBOT_PY" "$HANDOFF_ROOT/src/frame_calibration/robot_side/scrub_log.py" terminal_实际时间.scrubbed.log
"$ROBOT_PY" "$HANDOFF_ROOT/src/frame_calibration/robot_side/scrub_log.py" --check terminal_实际时间.scrubbed.log
```

## 现场执行前后

1. 确认 CPython 3.10 / Linux x86_64 SDK 环境、机器人身份、运行时网络地址，以及当前姿态、工具与标定。环境细节见 [ENVIRONMENT_REFERENCE](../docs/ENVIRONMENT_REFERENCE.md)。
2. 对精确候选完成离线全路径检查和人工 GUI 回放；按该入口实际行为区分 dry-run 与连接 SDK 的只读预检。
3. 清报警、使能、夹爪和运动由现场人员明确确认。持有机器人锁；检查首点跳变、限位、完整场景/持物碰撞与释放回撤。
4. 结束和异常路径验证保护、全局速度恢复及 SDK 关闭。保留成功和失败终端日志与结构化报告，先对副本脱敏，再归档 SHA。

历史 B5 离线实验还包含固定模型路径 `/home/dev/workspace/XF0112048/robot.urdf`。精确复现需匹配挂载布局；修改路径的开发副本应登记新 SHA。其简化 `calib_common.py` 只服务该实验，不能覆盖共享标定源。更多限制见 [KNOWN_ISSUES](../docs/KNOWN_ISSUES.md)。
