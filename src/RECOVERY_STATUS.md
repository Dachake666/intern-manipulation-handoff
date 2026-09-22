# 工作区恢复报告 — 2026-07-22

## 发生了什么
原 `工作/` 下的项目文件夹(`pick_place_coord/`、`frame_calibration/` 及其 git 仓库)
被误删。**废纸篓为空、全盘无 git 副本、无原文件备份** —— git 直接恢复不可行。

重建依据三样:
1. `workspace/sdk_tests/` 里"传去机器人测试"的幸存文件(逐字节可信);
2. 本项目的聊天记录(含我编辑各文件的完整/部分内容);
3. 记忆文件与总结中的标定常量。

`workspace/` **原样保留未动**(是恢复源, 且含较早的 SDK 学习内容)。
`workspace.zip`(97MB)也保留作安全网 —— 确认本次重建无误后可删除回收空间。
已清理: 3 个 core 崩溃转储(213MB, 零价值)、重建树的 `__pycache__`。

---

## 逐文件置信度(务必按此对待)

### ✅ 高 — 逐字节幸存 / 逐字重建, 可直接信任
| 文件 | 来源 | 说明 |
|---|---|---|
| `frame_calibration/robot_side/execute_trajectory.py` | 幸存(83dccfb)+聊天升级 | 已升到最新 `9a3fd5f`(连续模式泛化)。力度800/确认点/连续判据全在。`ENABLE_REAL_MOTION` 按 Mac 约定置 False |
| `frame_calibration/robot_side/sdk_session.py` | 幸存逐字节 | 未改 |
| `frame_calibration/robot_side/test_execute_trajectory_options.py` | 聊天逐字重建 | **11/11 测试通过**, 实证执行器逻辑正确 |
| `frame_calibration/robot_side/` 采集脚本 + `容器操作手册.md` | 幸存逐字节 | collect_*/jog/verify_uvw |
| `pick_place_coord/trajectories/traj_multi_latest.json`(133点) | 幸存逐字节 | z+0.05/force环境版, 校验 PASS。**注: 此为 `3b26b661` 版, 无连续meta**(见下"待办") |
| `pick_place_coord/trajectories/traj_three_point_test.json`(47点) | 幸存逐字节 | 校验 PASS |
| `pick_place_coord/tasks/*.json` | 聊天逐字重建 | 两个任务文件原文可信 |
| `pick_place_coord/validate_trajectory.py` | 聊天重建 | **已实跑**: 对两条轨迹输出与原会话完全一致(133点/5.46°/PASS) |
| `frame_calibration/records/`、`realtest_archive/`、`data/`、`trajectories/validated/` | 幸存逐字节 | 全部真机证据(99/88/153点验收、worlds标定数据) |
| `frame_calibration/analysis/calib_common.py` 的 **CONFIRMED_* 常量** | 总结逐字 | R_TCP=[0,168,-39]、D_FOREARM=[-0.08,0.24,-85.06]、T_V2、UVW约定 —— 数值可信 |

### ⚠️ 中 — 按已知行为重建, 逻辑对但细节可能有出入
| 文件 | 缺什么 | 首次真机前怎么核 |
|---|---|---|
| `pick_place_coord/pick_place_coord.py` | 约80%逐字(Traj/多点流程/main/常量), 其余(plan_path/solve_task_poses/单点流程/坐标换算)按行为重建 | 用 `--seed 7 --task tasks/task_2pairs_table72.json --direct` 跑, 应复现 133 路点、方块偏差 10.0/0.3mm、零碰撞 |
| `pick_place_coord/left_arm_ik.py` | 常量可信; DLS求解体的阻尼/步长超参为重建 | 用 `--scan` 与历史可达率对比; 同坐标解出的关节角应与已归档轨迹一致 |

### ❌ 低 — 需人工重点核对(标注不确定, 按你的选择尽力重建了)
| 文件 | 风险 | 必须核对项 |
|---|---|---|
| `pick_place_coord/xifeng_pb.py`(xf加载器) | 几乎只有函数名, 体全靠标准PyBullet补 | URDF路径(已修正为 `XF0112048/robot.urdf`)、基座位姿、载入后目视机器人是否正确、末端三轴是否画在腕部 |
| `calib_common.py` 的 **B类项** | 见文件内 [待核] 标注 | ①`CONFIRMED_T_SESSIONS_MM`(坐标换算T, 可视化曾显示≈[275,-20,-727], 与v2的t不同, 别混用) ②`J6_FLIP_INDEX=5`(腕滚取反下标, 错了腕滚方向会反, 单关节正负测试即可确认) ③`ARM_JOINT_IDS`/`EE_LINK_ID`(依赖URDF结构) ④FK/load_samples/uvw_to_matrix 函数体 |
| `gen_three_point_test.py` | docstring逐字; 生成体重建 | 产出的47点JSON已幸存, 本脚本仅供将来重生成 |

---

## 🔑 重要恢复资源: 原始 PyBullet 祖先脚本(2026-07-22 发现)
`workspace/pybullet_tests/urdf/XF0112048/` 逐字节幸存着 `xf`/`ik`/标定的**祖先脚本**
(独立 demo, 早于 pick_place_coord 重构), 是把重建的胶水代码换成真代码的金矿:
- `gui_test_xf0112048.py` — URDF 加载(真实基座/关节遍历)
- `xf0112048_*_numeric_jacobian_ik_demo.py` — 数值雅可比 DLS IK(与重建版公式一致)
- `xf0112048_left_arm_sdk_fk_compare.py` — 关节映射/SDK-FK 对比
- `xf0112048_left_arm_fit_sdk_world_from_samples.py` — SDK 世界系 T 拟合(可复现坐标换算T)

**已据此修正(commit 0e51183)**: `calib_common.ARM_JOINT_IDS` 左臂
`[3,4,5,6,7,8,10]→[5,6,7,8,9,10,11]`、`EE_LINK_ID` `10→11`(三处脚本佐证);
`J6_FLIP_INDEX=5` 确认为真(腕滚=jid10=下标5); DLS 公式确认一致。
**仍待核**: `basePosition`(祖先里 [0,0,0]最常见/也有 0.5/1.0); 坐标换算 T
可用上面的 fit 脚本重跑得到。→ 这批脚本请勿删。

## 已清理(2026-07-22, Debian 有全量备份兜底)
- `workspace/` 删除与抓取任务无关的学习/构建件(build/install/log/learning_tf2_py/
  bag_files/my_package/context-engineering/sdk_learning_test/冗余whl解包等), 164M→131M。
- 保留: pybullet_tests(祖先)、sdk_tests(证据)、XF0112048(模型)、pypilot whl+手册。

---

## 彻底丢失、需重新产出的东西
- **git 历史**(所有 commit 及其 archaeology)。已在 `工作/` 重新 `git init` 作新起点。
- `frame_calibration/analysis/verify_worlds_record.py`、`visualize_calibration.py`
  —— 只知用途(v2模型验证+t重标+漂移量化 / PyBullet可视化), 未重建, 需要时按
  `calib_common` 现有常量重写。
- 各 `.md` 总结文档(项目进度总结、阶段计划xlsx)—— 内容散见聊天, 未重建。

---

## 恢复后第一件事(强烈建议按序)
1. `cd 工作 && python3 pick_place_coord/validate_trajectory.py pick_place_coord/trajectories/traj_multi_latest.json` — 应 PASS(已验证)。
2. `python3 -m unittest discover -s frame_calibration/robot_side -p "test_*.py"` — 应 11/11(已验证)。
3. **目视核对 xf 加载器**: `python3 -c "import sys;sys.path.insert(0,'pick_place_coord');import xifeng_pb as xf;xf.load_xifeng(gui=True)"` — 确认机器人正确载入。
4. 若要重新生成轨迹(改场景/接相机前): 先按上表 ❌/⚠️ 逐项核对 `calib_common` 与 `left_arm_ik`, 再 `--task ... --direct` 跑, 与已归档 133 点结果对比。
5. 确认一切无误后, 可删 `workspace.zip`(97MB)与整个 `workspace/`(若其中 SDK 学习内容也不再需要)。

## 机器人侧当前该跑的(与删档前一致, 未变)
发 `execute_trajectory.py` + `traj_multi_latest.json` 到容器; 机器人侧设
`ENABLE_REAL_MOTION=True`; 要连续模式加 `--continuous`(注意 133 点这版轨迹
的 `continuous_candidate` 仍是 False, 需用重建的规划器重导出后才能开连续 ——
见下"待办")。

## 已知待办(删档前就存在, 未因恢复改变)
- 133 点轨迹要开 `--continuous`, 需用重建规划器重新导出(带 `continuous_candidate=True`
  + 段名派生), 路点不变仅 meta 变 —— 但这要求先核对通过 ⚠️/❌ 的 Mac 侧链。
- 相机 + 手眼标定阶段(P4): 视觉输出对接 `--task` 的 JSON 格式。
