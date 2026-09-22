# 2026-09-01 新桌面左臂瓶子抓放与返回 HOME 后续记录

判定：旧前缀轨迹为 **PASS / LIMITED evidence**；返回 HOME 的当前完整轨迹为
**OPERATOR_REPORTED_5X_PASS / INSUFFICIENT evidence**。后者已有现场问题记录，但尚无一份
从启动到正常退出的完整日志，因此禁止把当前 Mac 发布组合标成 `REAL_VERIFIED`。

| 日期 | 轨迹 SHA | 脚本 SHA | 模式/参数 | 完成标记 | 终点误差 | 保护/速度恢复 | 判定 | 日志 |
|---|---|---|---|---|---:|---|---|---|
| 2026-09-01 | `264808c7` | `bc05bd76…`（日志仅有前缀） | Servo `0.4°/20ms`, speed 8% | 882/882，close/open 完成 | 0.024° | True / 10% | PASS，证据 LIMITED | `logs/exec_20260901_user_pasted_scrubbed.log` |
| 2026-09-01 | `cce2a83f` | `bc05bd76…`（问题记录仅有前缀） | Servo `0.4°/20ms`, speed 8% | 操作者报告连续 5 次完整成功，含 READY→HOME | 未提供 | 未提供 | INCONCLUSIVE，证据 INSUFFICIENT | 未提供；见根目录 `PilotSDK_enable_varget_8080_debug_20260901.md` |

## 关键结果

- 左臂 `arm_id=1`、夹爪2，238 个运动路点、882 个 Servo 帧。
- 首点进入后最大误差 0.468°；抓取前 0.113°；放置前 0.022°；终点 0.024°。
- 实际 18.13s，预计 17.64s；late 76/882（8.6%），最大 87.6ms，保护性重新对齐 9 次。没有积压帧追赶突发。
- 控制器轴限位回读有效；J7 控制器上限 88°，本地物理包络继续保守使用 76°。
- 保护状态恢复为 True，全局速度恢复至 10%，PilotSDK 与 WebSocket 正常停止。
- 终端响应含敏感凭据；归档日志已脱敏并通过 `scrub_log.py --check`。

## 哈希漂移边界

旧日志中的轨迹 SHA 为 `264808c7…`；当前发布已更新为带 READY→HOME 收尾的
`cce2a83f…`，两者不是同一完整轨迹。Debian 记录的执行脚本 SHA 前缀为
`bc05bd76c2584dc6`，当前工作区与发布包的脚本完整 SHA 为
`73be5ab6561ccefbef7cfc9de021bd7e1b45d7f4b278783d245c362555523084`。
因此只能记录操作者报告的五次成功，不能跨轨迹或脚本哈希继承真机验收结论。

## 使能异常根因与处理

现场确认多个被 `Ctrl+Z` 暂停但未退出的旧 PilotSDK/Python 进程仍监听 TCP 8080，
干扰 `varget.log` 回传和状态初始化，导致新进程读到 `enable=False`、`servo=False`。
清理确认不再需要的旧监听进程后，状态连续保持 True，随后操作者报告完整轨迹连续五次成功。
处理手册已由发布生成器收进 `releases/trackC_bottle_left/`，但不参与机器人运行。

若以后要升级为 `FULL / REAL_VERIFIED`，补交 Debian 当次 `execute_servo_grasp.py` 和原始 `exec_*.log`，核对完整 SHA 后再迁移轨迹到 `trajectories/verified/` 并更新发布状态。
