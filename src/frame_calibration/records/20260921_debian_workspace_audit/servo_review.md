# 完整 Debian 收件：Servo 与旧控制线审查

审查根目录：`/Users/xiaobingxin/Desktop/工作/工作1/.handoff-review-workspace-20260921/full/recovered/workspace`。本记录基于完整 ZIP 的选择性解压，取代早先截断 ZIP 的缺件结论。只读分析收到的源码与 JSON，未执行收到的代码、未连接机器人；仅新建本报告。

## 核心结论

9/14 缺件清单 **18/18 已收到**，四份原始 JSON、20ms/25ms/VAJ3 入口、VAJ3 模块及依赖全部存在。运行 wrapper 与已记录依赖均可按 SHA-256 精确匹配。此前“缺 5 件”仅适用于截断包，现已解除。

Track A Servo 根目录共 74 份运行 JSON：61 份非 MPC、13 份 MPC（后者另审）。61 份均记为 `SERVO_COMPLETED_RETURNED_HOME` 且 cleanup `PASS`；所有已记录的执行器、轨迹、GUI、wrapper、依赖哈希都能在收到文件或备份中找到。

**证据边界：** 61 份全部明确绕过正式 Servo qualification，属于操作员监督现场试验；完成与收尾不能自动升级为通用资格通过。VAJ3 另有 `vaj3_spline.py` 已收到但原运行依赖清单未记录该模块哈希的问题，因此不能声称历史完整依赖身份已全部证明。SDK wheel 存在也不等于历史运行实际安装版本已经证明。

## 9/14 七轮现场记录

原来总结的四轮之外，另有 14:25、15:10、15:11 三轮。全部有三次夹爪事件、最终回 Home、速度/保护恢复和 SDK 停止记录。

| 本地文件时间 | 入口 | 周期 ms | 指令数 | 最大迟到 ms | 平均迟到 ms | realign |
|---|---|---:|---:|---:|---:|---:|
| 10:14:18 | execute_tabletop_servo_field_20ms.py | 20 | 660 | 15.957 | 0.224 | 0 |
| 10:15:07 | execute_tabletop_servo_field_25ms.py | 25 | 660 | 5.253 | 0.166 | 0 |
| 10:17:11 | execute_tabletop_servo_field_vaj20_v3.py | 20 | 800 | 11.393 | 0.166 | 0 |
| 10:57:07 | execute_tabletop_servo_field_20ms.py | 20 | 660 | 35.383 | 0.252 | 0 |
| 14:25:54 | execute_tabletop_servo_field_20ms.py | 20 | 660 | 12.579 | 0.160 | 0 |
| 15:10:26 | execute_tabletop_servo_field_20ms.py | 20 | 660 | 17.468 | 0.168 | 0 |
| 15:11:20 | execute_tabletop_servo_field_20ms.py | 20 | 660 | 83.992 | 0.478 | 2 |

15:11 最后一轮最大迟到 83.992ms、两次 realign，不能将当天整体表述成“始终无长迟到”。更早记录还包含 2794.881ms、1620.286ms、1222.306ms 长停顿，状态写完成不代表实时节拍合格。

## 代码逻辑与 Mac 差异

20ms / 25ms 包装入口锁定轨迹和 GUI 哈希，再替换基础执行器的审核、实时预检、流式发送函数；从 Home 生成首点过渡，对原路点 stride-4 抽样，按 20/25ms 发送，夹爪前停稳检查，最后回 Home 与恢复保护/速度。VAJ3 在此基础上加入 C2 五次 Hermite 样条和速度/加速度/jerk 限制重定时；它不是 MPC。

- Debian `execute_tabletop_servo.py` SHA `a9948adaa48691cdc8c2e0fd6a60b7fbc166eb7a02759601ed6e84f2a002473e`，不同于 Mac `c5aad779…`：提前构造 FloatVector，热循环加入耗时统计，软停读取从每帧改为阶段首帧及每 10 帧（20ms 时约 200ms）。
- Debian `servo_common.py` SHA `444abaa59c2e4a547f0007e314df4fd39465572dc1ee82d3574d08463a13ad89`，不同于 Mac `15fe08fe…`：超过两周期迟到后继续并计数；Mac 当前版会抛错停止。两者均不补发积压帧。这是控制策略差异，应审查后再合并。
- 现场轨迹 `71bc5148…` 与此前重建原件一致，不同于 Mac 当前候选/发布 `0a185455…`；GUI 为 `02f94490…`，Mac 为 `14176342…`。收到 GUI 原件不等于建立新的独立审核。
- sdk_session、robot_lock、tabletop_servo_contract 等与 Mac 对应现行源码匹配。当前包装入口直接 import，本次 inspected closure 无 importlib 加载；VAJ3 额外 import 的 `vaj3_spline.py` 未被旧 runtime_hashes 收录。

## 旧工作线定位

| 目录 | 平台与用途 | 可确认结果与限制 |
|---|---|---|
| `Servo/worlds` | Debian 笛卡尔 Servo SDK 试验 | 存入口、轨迹和 SDK 日志；无完整结构化抓放验收。默认真运动开启、速度 10%；旧节拍器缺少超量追帧保护。 |
| `Servo/pulse` | Debian 左臂关节 Pulse 早期试验；混存 MoveWorlds 检查 | 8/17 JSON 记录十次 10mm、1% MoveWorlds，位置误差最大1.936mm、姿态0.485deg。它不是 Pulse 抓放证据；记录的脚本hash与当前找到同名脚本不符。Pulse入口默认真运动开启、0.4deg/20ms，缺后来的CLI修复。 |
| `Servo/pulse1` | Debian 右臂镜像/平滑候选试验 | 入口默认右臂2、15%、真运动开启；有左右夹爪映射。右j7 -76deg是未实测镜像假设，平滑候选声明real_verified=false；目录没有完整右臂运动验收。 |
| `trackC_servo_stream/trackC_servo_stream` | Debian 历史 Track C 部署包 | README引用8/4左臂验收，不等于此目录所有现场修改重新验收；右臂仍未验收。包含一次使能后只读轮询改进，manifest中diag_enable.py已漂移。 |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck` | Mac离线规划产物部署至Debian，并有后续现场RUN候选与SDK诊断 | 只读预检README已落后于目录内容，混有5份real_motion_authorized=true的RUN候选。9/1说明声称清理暂停进程后5轮成功，但4份原日志是SDK诊断，不能代替5次完整运动日志。 |

瓶子线当前现场执行器 `bc05bd76c258…` 与 Mac 源码存在实质差异：夹爪关闭参数 `[1000,8000]`（Mac `[500,1000]`）；控制器限位直接采用，缺少 Mac 已有的“与物理/运行包络求交集”保护；首点 >20deg 在旧版阻断而新版作为入口确认提醒。这些差异应分别解释，不能笼统以“Debian更新”覆盖 Mac。

瓶子 debug README记录的 Home-start轨迹 SHA `cce2a83fae21468f80f0aad487087b61650eac6f92dea65f9560091d0239ad2b` 与收到文件相符；这只确认文件身份和说明所指对象，未补出五轮运行原日志。三个SDK诊断日志触发敏感值检测，必须只对副本脱敏后再入Git；报告不含秘密。

## 建议更新顺序

1. 将9/14缺件状态更新成已收齐，绑定本次完整ZIP身份，保留历史判断与更正关系；将七轮事实写入已有记录。
2. 归档61份结构化非MPC记录及对应哈希映射；备份中可匹配历史hash的文件是有用证据，不能按名字像备份就删除。
3. 保持Mac现行真源、Debian现场组合、旧Servo实验三个层次；收到现场版本不等于应全部回灌。优先审查热循环优化、软停检查频率、迟到策略；保留Mac实限交集和默认禁止运动约束。
4. 以后日志增加VAJ3模块、SDK/配置来源及全运行依赖hash；旧日志不补写伪造历史信息。需要完整资格验收时，另跑离线/场景/碰撞与人工门，成功记录不跨场景继承。
5. 旧瓶子和右臂以历史/候选状态交接，五连成功与右臂验收缺原日志的部分明确列出。源码整合后通过对应离线测试，再由make_release.py再生发布包。

## 全部61份非MPC结果索引

以下每条仅复述收到JSON状态和节拍；精确完整SHA、原文件路径、wrapper备份匹配、依赖、cleanup与SDK耗时统计见同目录servo_review.json。

| 文件 | 周期 ms | 指令数 | 最大迟到 ms | realign | 结果/收尾 |
|---|---:|---:|---:|---:|---|
| tabletop_servo_20260909_182932_719907228.json | 20 | 1248 | 138.977 | 10 | 返回Home / PASS |
| tabletop_servo_20260910_095643_352311931.json | 20 | 1248 | 921.402 | 371 | 返回Home / PASS |
| tabletop_servo_20260910_103844_659712178.json | 20 | 1248 | 194.444 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_105143_645973957.json | 20 | 1248 | 196.426 | 2 | 返回Home / PASS |
| tabletop_servo_20260910_111452_8747698.json | 20 | 1248 | 2794.881 | 73 | 返回Home / PASS |
| tabletop_servo_20260910_115323_657498295.json | 30 | 1248 | 136.235 | 1 | 返回Home / PASS |
| tabletop_servo_20260910_151044_480345410.json | 30 | 1248 | 227.523 | 3 | 返回Home / PASS |
| tabletop_servo_20260910_152201_759043648.json | 30 | 1309 | 186.405 | 1 | 返回Home / PASS |
| tabletop_servo_20260910_152558_723994735.json | 30 | 876 | 189.163 | 2 | 返回Home / PASS |
| tabletop_servo_20260910_152823_962154881.json | 30 | 660 | 120.558 | 3 | 返回Home / PASS |
| tabletop_servo_20260910_160524_761066947.json | 40 | 443 | 506.889 | 32 | 返回Home / PASS |
| tabletop_servo_20260910_163524_510903507.json | 40 | 443 | 199.528 | 1 | 返回Home / PASS |
| tabletop_servo_20260910_163935_446934665.json | 20 | 660 | 151.576 | 5 | 返回Home / PASS |
| tabletop_servo_20260910_164833_358988350.json | 25 | 660 | 219.554 | 17 | 返回Home / PASS |
| tabletop_servo_20260910_165133_659514199.json | 25 | 660 | 213.555 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_165218_247380274.json | 25 | 660 | 207.736 | 14 | 返回Home / PASS |
| tabletop_servo_20260910_165333_586410181.json | 25 | 660 | 250.737 | 31 | 返回Home / PASS |
| tabletop_servo_20260910_165814_339613079.json | 20 | 660 | 426.548 | 14 | 返回Home / PASS |
| tabletop_servo_20260910_165859_76747268.json | 20 | 660 | 219.672 | 13 | 返回Home / PASS |
| tabletop_servo_20260910_165949_514480706.json | 20 | 660 | 346.033 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_170030_54997245.json | 20 | 660 | 426.293 | 21 | 返回Home / PASS |
| tabletop_servo_20260910_173320_291819593.json | 20 | 562 | 319.870 | 19 | 返回Home / PASS |
| tabletop_servo_20260910_173431_697572496.json | 20 | 556 | 235.531 | 21 | 返回Home / PASS |
| tabletop_servo_20260910_173521_589861042.json | 20 | 556 | 383.134 | 14 | 返回Home / PASS |
| tabletop_servo_20260910_180535_22326500.json | 20 | 800 | 286.175 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_180627_526312116.json | 20 | 800 | 260.706 | 24 | 返回Home / PASS |
| tabletop_servo_20260910_180731_375394954.json | 20 | 800 | 360.776 | 21 | 返回Home / PASS |
| tabletop_servo_20260910_181646_765421386.json | 25 | 660 | 350.262 | 25 | 返回Home / PASS |
| tabletop_servo_20260910_181735_484313421.json | 25 | 660 | 202.870 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_181834_738155680.json | 20 | 800 | 256.603 | 19 | 返回Home / PASS |
| tabletop_servo_20260910_181919_767725741.json | 20 | 800 | 247.446 | 22 | 返回Home / PASS |
| tabletop_servo_20260910_182016_844508656.json | 20 | 660 | 329.705 | 13 | 返回Home / PASS |
| tabletop_servo_20260910_182307_274445019.json | 20 | 660 | 256.653 | 6 | 返回Home / PASS |
| tabletop_servo_20260910_182524_262924111.json | 20 | 660 | 192.903 | 12 | 返回Home / PASS |
| tabletop_servo_20260910_182801_44143728.json | 20 | 660 | 250.700 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_182900_405886792.json | 20 | 800 | 353.893 | 15 | 返回Home / PASS |
| tabletop_servo_20260910_183059_347999147.json | 20 | 800 | 258.414 | 22 | 返回Home / PASS |
| tabletop_servo_20260910_183148_517462003.json | 25 | 660 | 348.827 | 14 | 返回Home / PASS |
| tabletop_servo_20260910_183332_505827440.json | 25 | 660 | 315.023 | 16 | 返回Home / PASS |
| tabletop_servo_20260910_183436_249825816.json | 20 | 800 | 184.995 | 14 | 返回Home / PASS |
| tabletop_servo_20260910_183541_154425693.json | 20 | 800 | 284.945 | 30 | 返回Home / PASS |
| tabletop_servo_20260911_094917_877773855.json | 20 | 800 | 1620.286 | 18 | 返回Home / PASS |
| tabletop_servo_20260911_095055_638425600.json | 20 | 800 | 303.205 | 7 | 返回Home / PASS |
| tabletop_servo_20260911_162954_394142049.json | 20 | 660 | 189.664 | 2 | 返回Home / PASS |
| tabletop_servo_20260911_163212_676609931.json | 20 | 660 | 171.086 | 1 | 返回Home / PASS |
| tabletop_servo_20260911_163918_193229041.json | 20 | 800 | 70.636 | 1 | 返回Home / PASS |
| tabletop_servo_20260911_164012_821682725.json | 20 | 800 | 168.013 | 5 | 返回Home / PASS |
| tabletop_servo_20260911_164104_41510079.json | 25 | 660 | 46.018 | 0 | 返回Home / PASS |
| tabletop_servo_20260911_171849_607487523.json | 20 | 660 | 124.194 | 3 | 返回Home / PASS |
| tabletop_servo_20260911_172054_219248171.json | 20 | 660 | 134.312 | 2 | 返回Home / PASS |
| tabletop_servo_20260911_172409_814593890.json | 20 | 800 | 555.430 | 13 | 返回Home / PASS |
| tabletop_servo_20260911_172450_753647860.json | 20 | 800 | 232.013 | 15 | 返回Home / PASS |
| tabletop_servo_20260911_172614_921383877.json | 25 | 660 | 1222.306 | 14 | 返回Home / PASS |
| tabletop_servo_20260911_172732_408886835.json | 20 | 660 | 228.277 | 6 | 返回Home / PASS |
| tabletop_servo_20260914_101418_793393004.json | 20 | 660 | 15.957 | 0 | 返回Home / PASS |
| tabletop_servo_20260914_101507_112677088.json | 25 | 660 | 5.253 | 0 | 返回Home / PASS |
| tabletop_servo_20260914_101711_112650029.json | 20 | 800 | 11.393 | 0 | 返回Home / PASS |
| tabletop_servo_20260914_105707_209790145.json | 20 | 660 | 35.383 | 0 | 返回Home / PASS |
| tabletop_servo_20260914_142554_223889841.json | 20 | 660 | 12.579 | 0 | 返回Home / PASS |
| tabletop_servo_20260914_151026_745041422.json | 20 | 660 | 17.468 | 0 | 返回Home / PASS |
| tabletop_servo_20260914_151120_671751502.json | 20 | 660 | 83.992 | 2 | 返回Home / PASS |
