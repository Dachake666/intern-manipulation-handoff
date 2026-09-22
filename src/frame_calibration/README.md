# frame_calibration — SDK↔PyBullet 标定与真机执行

> 2026-07-22 部分重建。日常工作流与目录说明来自会话/记忆, 标定结论权威在
> `analysis/calib_common.py` 的 `CONFIRMED_*` 常量。整体恢复情况见工作区根
> 目录 `RECOVERY_STATUS.md`。

## 目录
```
analysis/         Mac 侧标定计算(重建件, 部分待核)
  calib_common.py   常量权威 + SDK<->URDF 关节换算 + FK
robot_side/       容器内真机执行(幸存, 已升到最新)
  execute_trajectory.py   低速回放已校验的关节轨迹(逐点/连续)
  sdk_session.py          会话/使能/读关节/软急停封装
  test_execute_trajectory_options.py  离线单测(11/11)
  collect_*.py / jog_* / verify_uvw   标定采样脚本
  容器操作手册.md
data/             worlds_record 标定数据
records/          真机证据归档(99/88/153/133 点各次)
realtest_archive/ 最终验收 99 点 + 夹爪
trajectories/validated/  按 SHA 命名的已验证轨迹快照
```

## 标定结论(权威=calib_common.CONFIRMED_*)
- SDK 腕滚(j6)符号相对 URDF 取反(`sdk_q_to_urdf_q`)。
- TCP: r=[0,168,-39]mm(link11 系, 0.003mm 拟合)。
- UVW = 标准 RPY, `R_sdk = R_link11·Rz(90°)`。
- v2 位置模型: `p_sdk = FK_link11·1000 + R_link9·D_FOREARM + R_link11·r_tcp + t`,
  D_FOREARM=[-0.08,0.24,-85.06]mm(SDK 内部模型前臂比 URDF 长 85mm; 物理真值
  是 URDF, 仿真抓取点无需此修正)。t 是会话级, 腰部上下移动会使其漂移。

## 日常唯一还需要的操作
每次开工(或腰动过后)用 `execute_trajectory.py --record-worlds` 跑一遍逐点轨迹
即可重标 t; 日常运行默认不生成 worlds JSON。把产出的 `worlds_record_*.json`
拷回 `data/`, 运行 `analysis/verify_worlds_record.py <文件>` —— 同时完成链路
验证 + t 重标 + 漂移量化。

> ⚠️ `verify_worlds_record.py` 未随本次恢复重建(仅知用途), 需要时按
> `calib_common` 现有常量重写。

## 场景高度基准(真机实测)
- 任务 z=1.00 ↔ 夹爪底端离地 70cm, 每 +0.01 = +1cm。
- 当前真实桌面 72cm, 物品顶 ~80cm → 用 `task_2pairs_table72.json`(z=1.05,
  夹爪底端约 75cm)。
