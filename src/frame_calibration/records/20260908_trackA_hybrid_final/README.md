# Track A：2026-09-08 Debian 最终现场版

接收日期：2026-09-09。审查结论：**PASS，证据 FULL，仅限收到的文件组合与这两次现场运行**。

原始交接 ZIP 保留于用户 Downloads，SHA-256：`d7a0ae01e898cb06dce7a96d3394cc2483efd6d79c49145839e8e4f09f5bb880`。
交接清单9项哈希全部匹配。运行记录先复制再经过 `scrub_log.py`，本次未发现敏感串，副本与原始日志哈希一致。

## 当前真源

- 执行器：`frame_calibration/robot_side/execute_tabletop_hybrid_trial_reviewfix_field.py`，SHA `36a0f659122eb492f63616f5c34a7e3b1f342c67f28e9906e2b9d6ad5b04e7ee`。
- 轨迹：`pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json`，SHA `e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59`。
- 上述文件原字节接收，不改源码、参数或文件名来冒充现场版本。5个依赖与Mac已有源文件完全一致。
- 唯一现行最小发布仍为 `releases/trackA_hybrid_trial/`，由 `python3 releases/make_release.py AT` 生成；7个运行文件，不混入旧候选。证据在本目录，完整清单见 `as_run.json`。
- 本目录 `SHA256SUMS` 是原交接清单，包含运行文件的原文件名；不是本归档目录布局的校验命令。归档路径与校验值以 `as_run.json` 为准。

## 两份最终日志核对

日期/速度 | 轨迹/脚本SHA前缀 | 完成 | Home关节最大误差 | 末端最大轴误差 | 保护/速度/关闭 | 判定 | 日志
---|---|---|---|---|---|---|---
09-08 17:23 / 5% | e5fd1101 / 36a0f659 | 12事件、7运动、3夹爪、回Home | 0.231° | 1.859mm / 0.215° | 开启 / 恢复10%回读 / SDK stopped | PASS | [5%日志](logs/hybrid_trial_run_20260908_172336_575481.json)
09-08 17:24 / 10% | e5fd1101 / 36a0f659 | 同上 | 0.205° | 1.869mm / 0.210° | 开启 / 恢复10%回读 / SDK stopped | PASS | [10%日志](logs/hybrid_trial_run_20260908_172435_229744.json)

运动段为5条MoveWorlds、单次转运MoveJ、固定Home MoveJ；不是Servo透传。动作区间约27.400s / 18.563s，不包括启动预检和人工等待，不能按总日志时长误判10%更慢。

通用终端日志扫描器不认识结构化 `tabletop_hybrid_trial_run.v2`，初扫INCONCLUSIVE；本次结论依据JSON字段核对：计划/执行器/全部依赖哈希、完整事件序列、使能/伺服/速度回读、逐段到位和停止、最终断言、cleanup保护/速度/SDK关闭，未仅凭PASS字符串判定。

## 相比上游的实际修改

- 保留原SDK XYZUVW（已经补偿的EE_POSE，不能再次TCP补偿），仅把PLACE_HOVER七关节改为 `[-33.131,2.908,19.904,-90.809,-23.508,-7.728,40.384]`，对齐控制器冗余IK构型。该端点J2距−10°硬限为12.908°，不是全路径余量结论。
- 保留用户近水平抓取UVW `[-93.110,8.776,-5.238]`。夹爪open等待1s、close等待2s。
- 只保留启动前一次确认。全路径dense IK/限位在启动前检查；运行中Worlds检查当前状态接出的前三个IK点，MoveJ仍检查实际q到目标q的密集关节限位。
- pre-command允许3mm/0.5°UVW/0.5°关节范围内的停稳变化并从新状态重检；启动人工确认前后仍用严格门槛。
- 命令行支持0.1–20%，**本包只能证明最终文件在5%和10%的这两次运行**。

## 证据边界与后续

- 交接摘要提到1%、3%、更多10%成功，包内仅有上述两份最终日志，不补造其余证据。
- 摘要约0.413°属于先前测试：最终日志的静止只读参考分支差为4.355°；真实MoveJ到位后下降前三点的最大相邻差为0.422°/0.423°。这两个量不能混用。
- 当前现场执行器main已不调用历史qualification/GUI起点门；文件头仍有旧描述，按实际执行路径理解。原JSON的BLOCKED/旧GUI/旧回放诊断不是最终实跑结论，也不是已重新计算的全场景资格。源文件为保留哈希不作文字修订。
- 成功证明本次固定现场的动作与收尾，不证明任意起点/物体/姿态/场景的避障安全，不升级通用REALVERIFIED，不改标定T。后续相机更新目标仍须重新规划和确认全路径。
- 当前失败运行不保存run JSON；故障排查需保留终端日志并脱敏。该缺口如要修改，应另起有测试的新执行器版本，不能覆盖本次实跑文件。
- 网络地址仅为as-run：机器人/臂192.168.8.147、本机192.168.8.185、臂1/夹爪2。最小JSON未含控制器序列号/完整SDK版本，本次未连接真机重新确认身份。

用户原始总结保存在 [原交接文档](trackA_hybrid_final_handoff_20260908.md)，其中操作建议仅作交接原文，不作为自动执行指令。

## 本次归档的软件回归边界

收到的现场文件/日志/发布检查10项通过；混合执行器、预检、最终交接、基础执行选项、Servo安全基础及生成器/发布的核心回归合计113项通过。扩展历史模块检查为224通过、4失败；另外场景预览3项在修正测试模块同名导入后通过。
4项失败均不在最终7文件运行依赖里，本次不更改其运动/规划逻辑来修复：

- `test_tabletop_vision_replay_candidate.py::test_current_single_anchor_fixed_pose_fails_closed`：旧断言预期在TRANSFER_ABOVE_BIN拒绝，当前实际在PLACE_DESCEND拒绝；仍拒绝生成，未绕过可达性。
- `test_obstacle_scene_support.py`中3项：`test_adapted_task_requires_rigid_object_self_pairs_and_open_bin`、`test_rotated_open_container_is_blocked_until_obb_preflight_exists`、`test_shared_tabletop_plan_is_the_only_pose_contract`；旧场景适配仍引用safe_corridor_endpoint_z_mm / TRANSFER_HIGH / RETURN_HIGH，新tabletop plan接口已经变化。

因此这是带明确待办的任务快照，不宣称全仓库测试全绿；也不把旧实验模块的失败归因于收到的最终现场执行器。
