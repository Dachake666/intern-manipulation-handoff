# 轨迹候选目录

本目录只保留仍在使用的候选和资格证据，不保存已被当前结果取代的瓶子变体。

## 新桌面左臂瓶子抓放（2026-09-01）

- 生成候选：`traj_bottle_taught_tcp_20260831_CANDIDATE.json`
- 真机测试件：`traj_bottle_taught_tcp_20260831_RUN.json`
- 资格任务：`task_bottle_taught_tcp_20260831_qualification.json`
- GUI 证据：`bottle_taught_tcp_20260831_gui_review.json`
- 资格报告：`traj_bottle_taught_tcp_20260831_qualification_report.json`
- 真机归档：`frame_calibration/records/20260901_taught_tcp_bottle_verified/`

RUN SHA-256：`264808c7da7f8a7f6b9e1844de9cb536c5f39a7c800275b95b5d7a8f1e1a522c`。现场完整执行 PASS；因 Debian 执行脚本只记录 SHA 前缀且与当前发布脚本不同，证据暂为 `LIMITED`，没有迁入 `trajectories/verified/`。

## 右臂候选

`traj_multi_right_20260806_CANDIDATE.json` 仍为 `CANDIDATE_HARDWARE_BLOCKED`；右臂真实限位、TCP、SDK 映射和夹爪会话闭环完成前不得真机执行。
