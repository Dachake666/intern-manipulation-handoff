# Debian / SDK 真机端

当前开发消费者在 ../src/frame_calibration/robot_side；收到现场原字节在Git快照中，避免同名覆盖。
从交接根目录恢复一个版本到不存在的新目录，不会执行任何收到代码：

```bash
python3 tools/export_version.py B5 ../mpc-received-review
```

恢复目录中debian/对应原包workspace/。SNAPSHOT.json记录每个原SHA及脱敏副本SHA。
历史源码可能默认开启运动；恢复不等于运行授权。需要恢复B2/B3/B4/B6/B7或D1时替换节点名。
仅复算MPC历史数据可在恢复后运行：

```bash
python3 evidence/mpc_review/recalculate_mpc_reports.py ../mpc-received-review/debian
```

该工具只读JSON和源码哈希，不导入MPC或SDK；固定10轮RMSE组均值改善约34.45%，不能据此计算成功率。
安装物在sdk/；环境、ABI及视觉ARM区别见docs/ENVIRONMENT_REFERENCE.md。
默认开发端可用src/releases/make_release.py生成AT/AS/DP等明确模式；已有Debian部署只同步SHA变化文件。
本次交接没有调用SDK、网络诊断、使能、清故障或运动；IP仍为IP_UNRESOLVED。

下面为收到各条工作线的精简说明。所有“原相对路径”都相对于恢复目录debian/。

# Debian / SDK 真机端交接

本交付保留收到的现场代码身份、原运行证据及必要历史。**本次没有连接机器人、运行SDK或重新验收。** 当前IP为 `IP_UNRESOLVED`，现场成功只适用于报告对应的目标、起点、姿态及文件组合。

默认工作树用于接手阅读和后续开发；收到的全部field源码由Git历史恢复，保持原字节。旧入口中仍有 `ENABLE_REAL_MOTION=True`、清报警/使能和修改速度的代码，不放入默认工作树，也不提供自动运动启动。开发版若修改默认值或安全逻辑，必须产生新身份和新验证结果，不回填旧运行哈希。

## 从哪里开始

先读总包 `START_HERE.md`，再读 `docs/VERSIONS.md` 中的实际标签/提交。按节点恢复到独立目录，核对manifest和同轮 `run_bindings`，不要把同名 `sdk_session.py`、`servo_common.py`、candidate或wrapper互相覆盖。

| 节点 | 入口与用途 | 已有证据及下一步 |
|---|---|---|
| B2 Track A固定场景 | `execute_tabletop_hybrid_trial_reviewfix_field.py` + SDKALIGNED轨迹 | 9/8—9/9五份成功JSON：1次5%+4次10%，记录依赖身份与cleanup PASS。先显式 `--dry-run`；新目标/场景仍需重新预检与资格检查。 |
| B3 Track C双次抓放 | `execute_servo_grasp.py` + `traj_multi_latest.json`（133运动点+4夹爪事件，SHA ac5c6fd7…） | 8/4五轮只有README。8点基础版有自己的运行文本，不能填补双次抓放原日志。先复核左臂1/夹爪2及Mac双次任务；收到副本不是五轮精确as-run源码。 |
| B4 Servo比较 | `execute_tabletop_servo_field_20ms.py`、`_25ms.py`、`_vaj20_v3.py` | 保留61份非MPC完整JSON及每轮已记录身份映射；9/14七轮全部在内。30ms/stride2另从9/10封存组合恢复。正式Servo资格曾被绕过；VAJ3历史模块哈希未记录。 |
| B5 MPC | `execute_tabletop_servo_field_mpc_20ms.py`、`_mpc_active_20ms.py`及各自base/controller | 固定十轮：6 Shadow+4 Active；另外三轮早期报告归D2。旧真机JSON没记录controller哈希；任务完成与普遍稳定性分开。先显式 `--dry-run`，不自动转Active。 |
| B6 Track B MoveJ基础 | 99点冻结执行器/SDK、153点 `post_run_safe_baseline`；88点轨迹/日志索引 | 99点1份、88点2份、153点3份原完成日志。99点冻结源不具备现代完整as-run哈希；153点是运行后安全快照。旧motion=True入口只恢复阅读。 |
| B7 瓶子示教/TCP | 示教238点+2事件、HOME起步275点+2事件；四种早期变体从索引恢复 | 五轮运动只有说明；四份使能/8080诊断日志不等于抓放日志。契约为 `PYBULLET_PHYSICAL_FK / EE_LINK_ORIGIN`，不套旧T，不重复补TCP。先Mac回放和资格检查。 |
| history 局部能力 | 旧 `execute_world_grasp.py`、`test_worlds_servo_minimal.py`、`execute_worlds_10x.py` | 旧Worlds用anchor+off_mm/fixed UVW；9/3 Z495/Z445参考另保留，旧执行器hash缺口仍在。CW的 `execute_worlds_servo_grasp.py` 当前走Worlds求IK→Pulse。WorldsToServo五轮仅汇总，局限于当前anchor的小平移/W旋转。10/20/50/80mm十步JSON齐，只有80mm执行器/SDK与原报告精确相符。 |
| history 右臂与夹爪 | 右臂latest/mirror/smooth候选、`test_gripper_right.py` | 未找到完整右臂抓放成功日志；右夹爪1、左夹爪2。夹爪工具可动作，不能叫无运动诊断；其成功也不是右臂验收。 |

D1保留9/10长迟到约2794.881ms、73次重对齐但仍报告完成的精确对照。D2早期MPC失效只有报告，旧executor字节缺失，不能拼造可复现版本。D3保留j7/跳过四点的失败日志，不据此放宽当前j7 76°实限。

## 环境与恢复

SDK安装物是一份 `pypilot-1.0.0422.1-cp310-cp310-linux_x86_64.whl`，适用Linux x86_64 / CPython3.10；不在Mac导入SDK作为验收。安装前核对wheel SHA和厂商说明，不复制旧虚拟环境。真实IP/本地IP由现场确认后传入，不能从历史日志推定当前地址。

MPC离线报告记录Python3.10.12、NumPy2.2.6、SciPy1.15.3、OSQP1.1.1、PyBullet3.2.5、Matplotlib3.10.9。开发依赖以对应requirements为准。收到的离线报告绑定 `controller.py.pre_rt_diagnostics`（4341f2fd…），当前现场核心为另一SHA；复现该离线报告时在隔离历史目录映射正确核心。

Debian MPC离线复跑需要保留模型一份，通过显式PYTHONPATH找到同版本Servo契约/臂配置，并显式指定 `--reference` 对应candidate。原字节兼容层的模型路径固定为 `/home/dev/workspace/XF0112048/robot.urdf`，复现需该Debian路径或隔离容器挂载；可移植开发副本修改路径后须登记新身份。其自带简化 `calib_common.py` 只服务这个实验，不能覆盖Mac权威CONFIRMED坐标层。离线 `report.json`、`traces.npz` 属仿真证据，不是真机闭环验收。瓶子Zplus150历史生成器还依赖Mac规划源/模型，原Debian目录本身不构成完整生成环境。

历史99点冻结SDK文件名是 `sdk_session_final_acceptance.py`；恢复时按manifest在隔离目录映射为 `sdk_session.py`，不改字节。备份wrapper/base也按同轮 `restore_name` 恢复；不能仅把整个目录最新同名文件拼在一起。

## 现场交接次序

1. 核对节点、轨迹/姿态语义、模型、参数、源文件SHA和当前标定有效性；先完成Mac离线回放、起点/限位/碰撞检查。
2. 在Debian建立匹配SDK环境并确认IP。显式dry-run只读计划；`--precheck-only`仍可能连接SDK，按对应入口确认不会使能/运动后由现场人员执行。
3. 真机清故障、使能、夹爪与运动均由现场人员确认。使用互斥锁；启动前确认当前姿态/首点、保护及速度，结束必须检查保护/速度恢复与SDK收尾。
4. 保存失败和成功日志，记录执行器、wrapper、轨迹及动态算法依赖的SHA。原件保留；交付日志先脱敏，另记脱敏副本SHA，不用它替代原身份。

诊断工具按动作分类：`read_left_arm_sdk_pose.py`读取位姿；`arm_diag_direct.py`会清报警/使能/设速度；`collect_left_arm_*`、网格/腕姿态/触碰采样会运动；收到的 `capture_tool_down_pose.py` 和十步Worlds已开启运动。`test_sdk_tcp_or_wrist_link11.py`实际是PyBullet离线分析。stationary pulse延迟探针会下发命令，也不是普通ping。

尚需补齐：8/4双抓放及瓶子五轮原运动日志、历史MPC/VAJ3核心身份、失败报告保留、双端节拍与软停策略统一、完整碰撞/动态资格和反馈新鲜度。修复与整理分开提交；本轮整理不升级REALVERIFIED。
