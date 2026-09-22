# 新 candidate 的可核验重建件

`tabletop_pick_place_SERVO_CANDIDATE.json` 不是新收到的现场文件，而是本次打包时用以下步骤离线构造的：

1. 从已上传的 9 月 9 日原始 ZIP 读出原 candidate，核验 SHA 为 `0a18545594212ecf428c36145de7525fcdd16753b89191ecc7e6730f1d1aaec1`。
2. 仅修改对话里用户 diff 确认的 5 个 `settle_s`。总 2487 条 waypoint 中，2484 条关节帧、3 条夹爪事件。未改关节向量。
3. 使用现场相同 `json.dumps(..., indent=2, ensure_ascii=False) + "\n"` 格式。
4. 实算 SHA 为 `71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338`，与现场执行输出完全一致。

变更和哈希见 `candidate_change_audit.json`。这是可复算的数据证据，不代替现场源码、原始报告、当前场景审核或运动授权。

**没有重建新 GUI PASS 文件。**旧 GUI review 仅随原 ZIP 原样保存；现场手动重绑后的 review 原件需另收，其历史 SHA 只是对照信息。
