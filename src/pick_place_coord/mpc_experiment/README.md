# 7轴关节 MPC：已实现的离线实验（2026-09-15）

结论：当前工作1可以接入 MPC，**已经完成代码、真实 PyBullet 电机反馈对照和离线测试**。这是新增的上层局部跟踪器，不是另一种插值，也不是已经可以上机的新轨迹。原 A、Servo、VAJ3 现场基线及发布包未修改。

当前结果：[`results/report.json`](results/report.json)，逐周期/子步数据：[`results/traces.npz`](results/traces.npz)，对照图：[`results/comparison.png`](results/comparison.png)。这些文件不是 SDK 轨迹格式，不要发给真机执行器。

## 一条命令复跑

当前 Mac 已建立工作1内独立 `.venv-mpc`，只新增 OSQP 1.1.1，复用现有 NumPy/SciPy/PyBullet；没有修改全局 Python。

```bash
cd /Users/xiaobingxin/Desktop/工作/工作1
XIFENG_ALLOW_REAL_MOTION=0 .venv-mpc/bin/python -B pick_place_coord/mpc_experiment/run_experiment.py
```

动态 GUI（重新执行 MPC＋物理反馈，不是显示截图；约52秒仿真，不覆盖批量结果）：

```bash
cd /Users/xiaobingxin/Desktop/工作/工作1
XIFENG_ALLOW_REAL_MOTION=0 .venv-mpc/bin/python -B pick_place_coord/mpc_experiment/run_experiment.py --gui
```

本次实际执行的是 DIRECT 动态仿真，**没有登记 GUI 人工 PASS**。GUI 只显示机械臂运动，没有已配准桌框、物理夹爪和瓶子。另一台机器可在隔离环境按 `requirements.txt` 安装；不建议覆盖 Debian 原有 SDK 环境。

## 接的究竟是哪份当前数据

- 输入是9月14日交接包内可恢复的短等待 Servo candidate，SHA `71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338`，路径在 `run_experiment.py:DEFAULT_REFERENCE`。2484个关节参考点、左臂1、夹爪2；只作为参考，不继承硬件资格。
- 保留 open/close/open 三个逻辑事件、0.5/1/0.5秒等待、原首点与原 Home，另加0.5秒最终停稳观察。每条对照都是2609个周期、52.18秒仿真；六次完整运行共15654周期。不是现场 S4/VAJ3 的13～16秒时序。
- `tabletop_servo_contract.validate()` 先核对现有 JSON、来源、参数与依赖。没有对已经补偿的 SDK EE 坐标再次做 TCP 补偿；输入直接是 SDK 关节度，进 PB 时仅按共享配置转弧度/J6符号。
- 最新 Debian 三套 field 脚本和 `vaj3_spline.py` 仍未完整收到，因此**尚未接入精确的现场 VAJ3 执行器，也没有做 MPC 对 VAJ3 的优劣测试**。
- 换一份参考用 `--reference PATH`，仍必须符合现有契约。不能直接传相机 XYZ 给本程序跳过 IK/规划/资格校验。

## 实现了什么

现有下层 Servo 接受位置目标，不是速度或力矩接口。因此把简单积分器扩展为一个线性位置响应模型：

```text
c[k+1] = c[k] + dt * u[k]
q[k+1] = (1-alpha) * q[k] + alpha * c[k+1]
```

`q` 是读到的关节位置，`c` 是位置指令，`u` 是指令变化速度。`dt=0.02s`，预测15步（0.3秒），`alpha=0.4` 是本次仿真实验参数，不是真机辨识值。`alpha=1` 且当前指令等于当前关节位置时，退化成理想运动学积分器。不是动力学 NMPC。

每轮更新当前反馈，最小化未来跟踪误差、指令速度与速度变化三项代价；OSQP 解一个带约束 QP，只应用第一步，下轮再求。代码直接使用 OSQP 稀疏接口和热启动，没有额外安装 CVXPY 建模层。思路与 [OSQP 官方 MPC 示例](https://osqp.org/docs/examples/mpc.html) 一致；增量更新和热启动参考 [OSQP Python 接口](https://osqp.org/docs/interfaces/python.html)。

约束包括：

- 指令位置、预测关节位置：共享控制器限位扣现有5°余量，再与 URDF 仿真硬限位取交集。URDF/控制器的限位分别进报告，不能混称现场实测。
- 指令速度/相邻步长/指令加速度：从现有 Servo policy 导入；本次等效5°/s、0.1°/周期、1000°/s²。它们是**命令包络，不是厂商动态额定值**。
- 预测末端指令速度归零，留出可停止方案，避免有限视野一直冲向边界；不是对任意初态的全局可行性证明。
- 夹爪事件分块：预测不跨过事件预先启动下一段；保留等待，在块结束验证关节误差≤0.25°且速度≤0.5°/s。只核对逻辑/到位条件，没有开真实串口或模拟物品抓取成功。
- 非有限输入、当前关节越界、QP不可行、非准确求解、求解器超时、残差超限，都中止实验，不复用旧解。10ms是求解器内部时间预算；总耗时另记，**不等于具有硬实时保障的20ms执行器**。

底层原本就有位置闭环，不能说“原 Servo 完全没有闭环”。本次新增的是上层利用反馈、预测未来并带约束优化的位置指令生成器。

## 实际结果

相同参考、时间表、模拟电机参数/effort下比较，误差按每个周期结束时实际关节反馈对同一周期参考计算；不是用自己输出的指令作为跟踪真值。

| 工况/指标 | 原插值位置流 | MPC |
| --- | ---: | ---: |
| 正常工况关节 RMSE / ° | 0.064503 | 0.003943 |
| 位置响应变慢一倍的模型失配，RMSE / ° | 0.149176 | 0.057536 |
| 12N短时外力工况，RMSE / ° | 0.064502 | 0.003942 |
| 正常工况峰值指令加速度 / °/s² | 379.672 | 250.000 |
| 正常工况峰值反馈加速度 / °/s² | 902.830 | 960.632 |

正常工况 RMSE 下降约93.9%，响应失配下降约61.4%。**这不是“所有平滑性都提升”：反馈峰值加速度略高约6.4%，反馈加速度 RMS 也没有改善。** 本次优化只约束命令动态，没有直接约束连续时间机械臂反馈加速度或 jerk，因此不能把“指令峰值下降”说成“真实机器人一定更丝滑”。

12N外力在这个强位置控制、零重力实验中影响很小，不构成强抗扰证明；模型失配才是本轮更有区分度的补充测试。两种控制器使用相同的外力/响应参数，MPC在失配工况不重新调参。

三轮7轴 MPC 共7827次求解：本次 Mac 上 p99 约0.58～0.60ms，最大2.65ms以内，记录的总求解耗时没有超过20ms。包括 Python 更新、求解、残差检查，但**不包含真实反馈网络读取、SDK发送与系统调度长尾**；重跑数值以 JSON 为准。

所有角度子步保持在实验限位交集内，三轮 MPC 都完成三次事件到位/停稳检查并回到原参考终点。MPC位置指令相对原参考最大偏离约0.156°（正常）/0.226°（失配），所以它已经是修改后的运动，不可继承原GUI或碰撞批准。

单轴教学例：目标1.2°、上限1°，最终停在1°；速度上限1°/s和0.3°/s时，到0.99°分别约1.10s和3.32s。小角度例与本项目的局部跟踪尺度一致。开发中直接给0→90°大跃迁时，这份凝聚式 QP 的数值收敛曾超时/达迭代上限，程序确实停止；**未把它包装成任意大目标一步可解的规划器**。相机新目标必须先形成可行参考，不能指望 MPC 自动修好不可达目标。

## 当前模型的硬边界

原 URDF 不足以做高保真动力学：完整审计发现多个连杆惯量为零/奇异，两侧肘部 `[0,0,0.01]` 还违反刚体主惯量条件，强制读取时 PB 会把它置零。本程序不改只读厂商文件，改用 PB 从碰撞形状估算的正惯量，保留原质量/effort，采用零重力（理想重力补偿）工况。原运动学回放不依赖惯量，不能因此否定此前真机的几何结果。

本次 `setJointMotorControlArray(POSITION_CONTROL)`＋每周期8个 `stepSimulation()` 子步＋`getJointStates()` 是实际仿真反馈；只有初态使用 `resetJointState`，测试会拦截任何步进期间的重置。不能把 PB 估算惯量和模拟增益当成已经辨识的实机模型。参考 [PyBullet 官方位置/PD 控制示例](https://raw.githubusercontent.com/bulletphysics/bullet3/master/examples/pybullet/examples/pdControl.py)。

没有碰撞求解、完整场景、持瓶动力学、末端水平约束、关节力矩约束、SDK在线闭环或网络抖动测试；**不是避障规划器，也不会因 J2 到限位就自动换另一套 IK 构型**。尚不具备上机资格。

## 文件、验证与后续接入

本轮最后一次合并回归：**192项PASS，46.15秒**（26项新增MPC测试＋166项既有相关测试）。仅有1项第三方SciPy `KDTree` 导入弃用警告，不是全仓库/真机验收。报告的9个源码/输入SHA在运行后逐项复核一致，`git diff --check`通过。

完整回归命令（临时输出也留在工作1）：

```bash
cd /Users/xiaobingxin/Desktop/工作/工作1
task_mpc_tmp="$(mktemp -d "$PWD/.handoff-review-mpc-XXXXXX")"
XIFENG_ALLOW_REAL_MOTION=0 PYTHONDONTWRITEBYTECODE=1 TMPDIR="$task_mpc_tmp" \
.venv-mpc/bin/python -B -m pytest -q -p no:cacheprovider \
  pick_place_coord/mpc_experiment/test_mpc.py \
  frame_calibration/robot_side/test_execute_trajectory_options.py \
  frame_calibration/robot_side/test_servo_safety.py \
  frame_calibration/robot_side/test_execute_tabletop_pick_place_worlds.py \
  frame_calibration/robot_side/test_precheck_tabletop_hybrid.py \
  frame_calibration/robot_side/test_execute_tabletop_hybrid_trial.py \
  frame_calibration/robot_side/test_tracka_final_handoff.py \
  frame_calibration/robot_side/test_tabletop_servo.py \
  pick_place_coord/test_gen_tabletop_hybrid_candidate.py \
  releases/test_make_release_hybrid.py
```

- `controller.py`：可独立调用的 MPC 核心，输入当前关节、上一条位置/速度指令、未来参考，输出第一步位置目标和诊断。无硬件接口。
- `run_experiment.py`：共享契约、PB模型/符号映射、六组完整对照、GUI与报告生成；SDK纯离线契约导入不创建会话。
- `test_mpc.py`：反馈影响、预测模型一致性、位置/速度/加速度限制、可停止末端、坏输入/不可行/超时/伪成功拒绝、事件/来源保留、PB电机不瞬移和惯量异常记录。
- 安全常量不从历史 `world_grasp.py`、`gen_minimal_joint_grasp.py` 的字面量复制：只用 `arm_profiles`、`calib_common`、现行 `tabletop_servo_contract`。Home来自本次任务JSON，不用其他线路的共享默认Home；不使用工作范围常量来冒充碰撞检查。
- `motion-variant-scaffolder` 附带扫描脚本在安装包中缺失，已用 `rg` 核对相关常量；历史不同消费者的5°/6°导出步长没有移植。本实验与成功版本隔离，不创建新发布包。
- 通用轨迹检查器对输入未报错误；其“期望close/open”的警告来自默认夹爪模板，专用现行契约已确认本轨迹open/close/open合法。资格按 `trajectory-candidate-qualifier` 记录为离线控制实验完成，场景碰撞、GUI人工批准、真机与在线时序仍未通过。

以后接真机的顺序：先补齐当前 field/VAJ3 源码 → 辨识真实位置响应/反馈延迟 → MPC只旁路计算、不发送，测总时延和差异 → 对新命令轨迹重新做场景/GUI/首点校验 → 再由人确认受限分段测试。反馈过期、求解超时/不可行时如何停止并恢复保护，须接入现有执行器，不可临时删除门限。

相机链路仍然是：位姿语义/TCP换算 → IK/避障与时间参数化 → 参考关节序列 → MPC反馈跟踪 → Servo。**MPC不负责把XYZ变成完整可行路径，也不能只把结果预先压成一个固定JSON就称作在线MPC。** 后续可保持任务/参考JSON接口稳定，但在线层需要真实、带时间戳的关节反馈。

本轮未连接真机、路由器，未修改URDF/SDK源码/成功候选/release，未commit或push。所有新增与修改只在工作1。
