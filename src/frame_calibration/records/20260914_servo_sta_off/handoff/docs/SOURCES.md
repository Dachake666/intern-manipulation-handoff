# 来源与证据分层

本包依据用户已提供材料；程序化检查仅包含文件/哈希/数据解析，不包含新机器人或网络实验。

| 标记 | 来源 | 本包位置/用途 |
|---|---|---|
| E01 | 9/9 原上传 `trackA_servo.zip`，ID `file_00000000dc088230a719586853e3c487` | `legacy/trackA_servo_20260909_original.zip`；原件、原候选、旧 pacer、旧 review 和资格边界 |
| E02 | 9/11 旧完整归档，ID `file_00000000030081f5918759429bcc2f8a` | `legacy/Servo_真机控制与网络长尾排查完整归档_20260911.md`；历史，不自动当作新结论 |
| E03 | 9/11 字段输出（9/14上传），ID `file_00000000616081fda608ef205c09da85` | `evidence/粘贴的文本 (1)(20260914-015229).txt`；1870行来源，10份旧报告摘要 |
| E04 | 9/14 四轮终端记录，ID `file_000000007838823092474f7772abf6a9` | `evidence/粘贴的文本 (1)(20260914-040051).txt`；507行，命令/阶段/报告文件名 |
| E05 | 用户当前对话直接贴出的四份 JSON 指标 | `evidence/20260914_latest_metrics_user_paste.txt`；转录，不是原JSON |
| E06 | 用户对话路由器 UCI/iw 输出 | `evidence/20260914_router_state_user_excerpts.redacted.txt`；节选，未连续监视 |
| E07 | 历史 ping 原文件，ID `file_000000009d98822fb1e13117fa319241` | `evidence/ping_robot_STA_OFF_servo.txt`；9/11，不是9/14批次 |
| E08 | 9/10–9/11 其他用户终端记录 | `evidence/粘贴的文本*.txt`；含早期长尾、相关性和不同实验批次，不混同条件 |
| E09 | 9/12 历史工作报告，ID `file_000000008a5481fb9021561a9cd3bde4` | `evidence/MAC_baseline_context_excerpt.md`；Mac路径与接收状态线索，非当前仓库核验 |
| D01 | 本次离线重建+哈希审计 | `reconstructed/candidate_change_audit.json` |
| D02 | 本次从E03提取10份摘要 | `state/results_20260911_from_transcript.json` |
| D03 | 本次从E05转录并复算汇总 | `state/results_20260914_reported_summary.json` |

`state/evidence_manifest.json` 记录每份原上传文本的原 SHA、打包后 SHA、脱敏次数。脱敏仅处理显式 token/password 等；未重写实验数值。没有把未经接收的原始日志名制成假原件。

## 外部说明核对（仅恢复语义，不替代用户实验数据）

- W1：OpenWrt 官方 UCI 文档，https://openwrt.org/docs/guide-user/base-system/uci 。用于确认 staged/commit/revert 的区分。
- W2：NetworkManager 官方 802-11-wireless 属性文档，https://www.networkmanager.dev/docs/api/latest/settings-802-11-wireless.html 。用于确认 powersave 0/1/2/3，不能据此推断用户原值。

本次未核验实时 GitHub 状态、Mac 文件系统、设备当前配置、其他人互联网连通性；相应内容均标成历史线索或待确认。
