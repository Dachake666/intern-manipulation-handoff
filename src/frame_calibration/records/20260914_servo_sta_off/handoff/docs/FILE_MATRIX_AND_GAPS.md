# 文件矩阵与补收缺口

## 接收状态

| 对象 | 本包状态 | 接手注意 |
|---|---|---|
| 9 月 9 日 `trackA_servo.zip` | 原始上传，字节保留 | 旧软件候选，不是 9 月 14 日 field 实机版本 |
| 9 月 11 日 MD | 原始生成文档，原样保存 | 历史叙述优先级低于后续用户实测与新主文档 |
| `tabletop_pick_place_SERVO_CANDIDATE.json` 新版 | 可复算重建件，SHA 完全命中现场值 | 不代表独立新 GUI 审核；另收现场原件用于链路追溯 |
| 新 GUI review | 只有现场操作记录和 SHA | 不伪造/重写 PASS；需现场原件与真实审核证据 |
| 三个 field 执行器 | 缺当前完整原件 | 20ms、25ms、vaj20_v3；不能凭命令行重建其全部行为 |
| `vaj3_spline.py` | 缺当前完整原件 | 早期 creation SHA 不能充当最终参数版 SHA |
| `execute_tabletop_servo.py` / `servo_common.py` 等 | 原 ZIP 内有旧版 | 共用依赖可能已在现场改动，必须再收当前字节 |
| 四份 9 月 14 日 `tabletop_servo_*.json` | 有终端执行记录、字段摘要；无原 JSON | 摘要已经单独命名，不冒充原始日志 |
| 9 月 11 日相关终端/旧 ping 文件 | 已收，多数日志脱敏 | 保留原 SHA、打包后 SHA 与脱敏计数 |
| 9 月 14 日同时 ping 文件 | 未收到 | 不能用 9 月 11 日 ping 冒充此批同步证据 |
| 真机视频、独立抓取成功率/误差 | 未收到 | 完成状态不等于无抖动、无滑落或已测绝对精度 |
| Mac 当前工作区/提交 | 历史路径线索 | 本次没有读 Mac 文件系统或核验当前 Git 状态 |

## 版本指纹

| 对象 | SHA-256 | 证据等级 |
|---|---|---|
| 原 candidate | `0a18545594212ecf428c36145de7525fcdd16753b89191ecc7e6730f1d1aaec1` | 本次从原 ZIP 实算 |
| 新 candidate | `71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338` | 现场日志 + 本次重建实算一致 |
| 原 GUI review | `14176342f3ed8fda12bfe82fccd18255b748bad6d17c438576a95a7380f22589` | 本次原 ZIP 实算 |
| 现场重绑 GUI review | `02f94490a01c2332f50a9e5381edd19eea4fb164c600fe809a596b2fa6154b13` | 现场报错/操作记录；原件未收 |
| 原 ZIP | `d651b8f848032c51338a7233f5c16c0ffb9a2f20f078a86496758900bf316a89` | 本次实算 |
| 旧 MD | `c7b6cad008cd85a750805785d060cbc19d1d6b840c34b3a1d768a217b0fe6c8c` | 本次实算 |

旧 `execute_tabletop_servo.py` 为 `c5aad779b9cdf7501baf5f4f88b38c06bb70ba36146352be049a2b58cdbf9c1c`；旧 `servo_common.py` 为 `15fe08fe730881c065369fad9980d4ac02700b6d8390b900d2a42085d2051f83`。其余原 ZIP 成员见 `../state/legacy_zip_inventory.json`。

## 现场必收文件

机器可读完整清单为 `../state/expected_field_artifacts.json`。除了三套 wrapper，必须包含当前 `execute_tabletop_servo.py`、`servo_common.py`、`sdk_session.py`、`robot_lock.py`、`tabletop_servo_contract.py`、`arm_profiles.py`、`arm_profiles.v1.json`、`vaj3_spline.py`、`probe_pulse_latency_stationary.py` 和所有现场动态导入依赖。

四份基准结果文件：

```text
tabletop_servo_20260914_101418_793393004.json
tabletop_servo_20260914_101507_112677088.json
tabletop_servo_20260914_101711_112650029.json
tabletop_servo_20260914_105707_209790145.json
```

保留 `.before_gripper_sha`、`.before_gui_sha`、`*_before_gripper_fast.json` 及原始 `.bak*`。不要批量替换历史 SHA。

## 补收脚本的边界

`collect_servo_sources.py` 默认只打印计划；`--collect` 新建私有快照，保持已选择源文件的原字节。它递归采集源码、JSON、文本记录、常见模型与备份；跳过密钥文件、虚拟环境、Git 内部、软链接与原生 SDK 二进制。PCAP 需 `--include-pcaps`。超过体积限制、外部目录或缺失的文件都会反映在 manifest，不视为已收到。

**这不是脱敏工具。**现场原始日志/PCAP 可能含 token；为保留原始哈希，补收快照只适合私下归档。需要公开材料时另做脱敏副本，不能用脱敏副本哈希冒充原件。
