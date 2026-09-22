# 交接副本工作约定

当前仓库根以本文件所在目录为准，不要使用历史文档里的个人绝对路径。
先读 START_HERE.md、sim/README.md 或 robot/README.md，再查 docs/VERSIONS.md。

- 当前开发源码在 src/；规划在 src/pick_place_coord，执行在 src/frame_calibration/robot_side。
- 标定只认 src/frame_calibration/analysis/calib_common.py 的 CONFIRMED_*；两套T用途不可混。
- src/XF0112048 为只读厂商模型；发布目录只由 src/releases/make_release.py 再生。
- Debian 原字节组合在 handoff/* Git 标签，用 tools/export_version.py 恢复审阅。恢复不等于运行授权。
- ENABLE_REAL_MOTION 默认关闭；先纯离线检查。使能、运动、清故障由现场人员确认。
- 不在Mac导入pypilot验收。j7实限76度；不要为过预检放宽限制。
- 保留robot_lock、保护/速度恢复；新候选需完整场景/持物碰撞、首点、限位、标定与绑定GUI审核。
- 原日志/源码不改，脱敏做副本；新增执行日志记录脚本、轨迹与依赖SHA。
- 不因仿真通过或材料齐全升级真机验收；缺失字段、旧高残差和未知现场状态照实保留。
- 提交用中文，开发修复与历史快照分开；此仓库没有配置远端，推送需另行指定。
