# 2026-09-07 混合抓放历史参考

2026-09-09按用户“把本任务截至目前提交”的要求，将此前临时检查目录中已核对过的材料保存为可重放测试证据。原ZIP和检查副本不改、不提交。

- 原plan仍在 `pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_CANDIDATE.json`，SHA `3f3e36f370521e4ab815e377ddb491a3cd2adb24d95dcf843451d03d7587c06e`；不复制成新的可运行候选。
- `artifacts/execute_tabletop_hybrid_trial.py` 是对应现场旧执行器，SHA `de5ccc1252a03248cc9a5315a2b2fc180953bd70217176116bf294ca653684c5`。只供来源和Home解析，不是当前执行入口。
- `logs/`为此前已核对的5份完整动作PASS记录副本，全部匹配原plan/执行器；旧记录缺少完整收尾回读，证据LIMITED，不升级REALVERIFIED。
- `inputs/precheck_worlds_report.json` 是此前用户提供的580点只读报告，SHA `d50d95ce31afd1743ba8f1ce4ed2b2e05806496b44f751a203b3476b217daf98`；仅是采样时的SDK预测，不是当前状态或运动证明。
- 所有副本经过 `scrub_log.py`，未发现敏感串。原件不变。测试现使用这些仓库内路径，不依赖个人Downloads或临时目录。
- 当前现场成功组合见 `../20260908_trackA_hybrid_final/`，不要混用两版。
