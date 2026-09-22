# 打包离线检查记录

本次只做文件/脚本离线检查；没有运行Robot SDK、真机、路由器、ping或恢复命令。

Collector/summarizer的行为测试使用临时合成fixture，不是把合成JSON当真实机器人数据。实际实验数值来自用户回传；新四份原始JSON尚未收到。

通过 **55 项** 检查：

- PASS：原始ZIP CRC完整性
- PASS：旧ZIP成员hash arm_profiles.py
- PASS：旧ZIP成员hash arm_profiles.v1.json
- PASS：旧ZIP成员hash execute_tabletop_hybrid_trial_reviewfix_field.py
- PASS：旧ZIP成员hash execute_tabletop_pick_place_worlds.py
- PASS：旧ZIP成员hash execute_tabletop_servo.py
- PASS：旧ZIP成员hash precheck_tabletop_hybrid.py
- PASS：旧ZIP成员hash robot_lock.py
- PASS：旧ZIP成员hash scrub_log.py
- PASS：旧ZIP成员hash sdk_session.py
- PASS：旧ZIP成员hash servo_common.py
- PASS：旧ZIP成员hash tabletop_pick_place_SERVO_CANDIDATE.json
- PASS：旧ZIP成员hash tabletop_pick_place_SERVO_gui_review.json
- PASS：旧ZIP成员hash tabletop_servo_contract.py
- PASS：重建candidate仅五个指定settle字段变化
- PASS：重建candidate SHA与现场报告一致
- PASS：未伪造新GUI review
- PASS：最新汇总计数
- PASS：最新加权平均
- PASS：Python AST语法 verify_handoff.py
- PASS：Python AST语法 collect_servo_sources.py
- PASS：Python AST语法 summarize_servo_logs.py
- PASS：Shell语法 debian_readonly_snapshot.sh
- PASS：Shell语法 router_readonly_snapshot.sh
- PASS：Collector默认只计划且不创建输出
- PASS：Collector拒绝输出到源目录内
- PASS：Collector可创建隔离原件快照
- PASS：Collector不跟随软链接/不收SSH密钥/不收默认PCAP
- PASS：Collector保留源字节并不导入项目代码
- PASS：Collector诚实报告缺失文件
- PASS：Collector成员hash field_snapshot/example.py
- PASS：Collector成员hash field_snapshot/nested/a.log
- PASS：Collector显式包含PCAP
- PASS：Summarizer处理四份synthetic fixtures
- PASS：Summarizer分组数据准确
- PASS：Summarizer缺失值保留null
- PASS：Summarizer报告损坏JSON不悄悄跳过
- PASS：收集不修改原测试源码
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-021046).txt
- PASS：日志显式token脱敏 ping_robot_STA_OFF_servo.txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260914-040051).txt
- PASS：日志显式token脱敏 20260914_latest_metrics_user_paste.txt
- PASS：日志显式token脱敏 old_source_servo_common.py.excerpt.txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-025844).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260910-020023).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-063332).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260914-015229).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-073945).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-063932).txt
- PASS：日志显式token脱敏 old_source_execute_tabletop_servo.py.excerpt.txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-040308).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-024116).txt
- PASS：日志显式token脱敏 20260914_router_state_user_excerpts.redacted.txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-073004).txt
- PASS：日志显式token脱敏 粘贴的文本 (1)(20260911-032936).txt

整包最终SHA256清单与ZIP CRC将在封包后独立核对；该校验不等于任何运动资格验证。
