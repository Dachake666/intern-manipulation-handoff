# MPC 完整包交接审查（2026-09-21，完整包接续）

**结论：十轮原始真机报告和源文件已收齐，34.45% 的 RMSE 降幅已独立复算一致。真机正式运行资格仍未完成。** 本报告接续此前残缺 ZIP 的部分恢复结论；此前“controller.py 和六份原报告缺失”不再是本次收件缺口。

工作区：`/Users/xiaobingxin/Desktop/工作/工作1`。收到文件根目录：`/Users/xiaobingxin/Desktop/工作/工作1/.handoff-review-workspace-20260921/full/recovered/workspace`。逐文件 SHA、各轮执行器/依赖核对、平台信息和全部统计见 [mpc_review.json](/Users/xiaobingxin/Desktop/工作/工作1/frame_calibration/records/20260921_debian_workspace_audit/mpc_review.json)。本次仅标准库读取、哈希、数值复算及 AST 静态解析；未执行收到代码、导入 SDK、连接设备或重新跑仿真/测试。

## 1. 收件身份与闭合项

- 原 `mpc_handoff_20260921_102256_clean.tar.gz` 大小 44,572,564 字节；SHA-256 `ffbbc8a1e181cbbc2544ebe91cfef580e641f6095f3ff0c8dbfab6a77ef8cb63`，与收到的发送端 `.sha256` 及既有核验资料完全一致。
- 既有260项快照清单中，所选解压的259项功能文件逐字匹配，无差异；仅 `XF0112048/.DS_Store` 未出现在所选解压中，不构成功能缺件。
- Shadow/Active 的4个执行文件、MPC核心、10轮原始 JSON、各报告记录的10项依赖、轨迹/GUI审核均已收到。18项9/14 Servo原缺件清单也已全部找到；这一项仅确认收件，9/14逐次执行资格需按其报告另行核对。
- 十轮原 JSON 本体、执行器、wrapper、轨迹和10项依赖哈希全部吻合，MPC保持启用到结束且无记录错误。

## 2. 必须区分的四类内容

| 内容 | 入口/证据 | 用途与状态 |
| --- | --- | --- |
| Mac离线仿真 | `/Users/xiaobingxin/Desktop/工作/工作1/pick_place_coord/mpc_experiment`，9/15 `results/report.json` | PyBullet六轮对照；没有SDK运动 |
| Debian离线仿真 | `mpc_experiment/mpc_experiment/results/report.json`，9/16 03:08 UTC | Linux上重新执行的六轮PyBullet对照；也不是硬件运行 |
| Debian真机Shadow | `trackA_worlds_line/trackA_servo/execute_tabletop_servo_field_mpc_20ms.py` → `execute_tabletop_servo_mpc_base.py` | 真实反馈供MPC旁路计算，发送原reference；Shadow失效时基础Servo可继续 |
| Debian真机Active | 同目录 `execute_tabletop_servo_field_mpc_active_20ms.py` → `execute_tabletop_servo_mpc_active_base.py` | MPC候选经端点渐收和独立检查后真正发送；错误立即终止 |

真机两种模式的执行链为：已有可行关节参考JSON → field wrapper首段接入/stride=4主体630帧 → 反馈/预测 → JointMPC一步候选 → Active后处理与检查 → SDK Servo → 夹爪事件/端点停稳 → 回Home → 恢复保护/速度并停止SDK。其中Shadow候选仅旁路记录，Active候选才经后处理发送。Mac/Debian离线实验各轮为2609周期，不经过SDK及真夹爪链。MPC不承担XYZ转IK、全局避障或完整场景碰撞检查。

核心模型 `c[k+1]=c[k]+dt*u[k]`、`q[k+1]=(1-alpha)*q[k]+alpha*c[k+1]`；20ms、15步预测、alpha=0.4；OSQP优化关节跟踪、指令速度和速度变化，非动力学NMPC。真机每两帧回读、间隔帧预测；Active末段10帧逐渐收回修正，最后点回到reference。

## 3. 十轮独立复算与收尾

每轮剔除entry段，315个7轴反馈样本。按原记录规则计算每轮reference减measured误差，再汇总各轮指标。

| 组 | 次数 | RMSE均值±样本SD / ° | 每轮P99均值 / ° | 每轮MAX均值 / ° |
| --- | ---: | ---: | ---: | ---: |
| Shadow | 6 | 1.10708254 ± 0.03087718 | 2.54481320 | 2.91808385 |
| Active | 4 | 0.72566155 ± 0.00854412 | 1.78973512 | 2.47591630 |

RMSE组均值下降 **34.4528046065%**。P99/MAX为每次指标的均值，不是合并样本的分位数/全局最大值。前两轮主体采样偏移1，其余偏移0；Shadow在当帧reference发送前读取，未按设备时间戳严格配对，不能称为同一时刻SDK跟随误差认证。

| 报告尾号（前缀tabletop_servo_20260916_） | 模式 | 主体RMSE / ° | 最大迟到 / ms | JSON收尾 |
| --- | --- | ---: | ---: | --- |
| 150937_394611035 | SHADOW | 1.10857534 | 82.20 | PASS |
| 161007_195575844 | ACTIVE | 0.72714968 | 138.95 | PASS |
| 161804_427508716 | SHADOW | 1.12534486 | 107.78 | PASS |
| 161842_290824538 | ACTIVE | 0.72547455 | 54.82 | PASS |
| 161957_559208120 | SHADOW | 1.04912949 | 390.82 | PASS |
| 162057_54388464 | ACTIVE | 0.71462085 | 59.87 | PASS |
| 162340_144861635 | SHADOW | 1.13334096 | 72.75 | PASS |
| 162429_177216570 | SHADOW | 1.10056382 | 79.13 | PASS |
| 162614_575109989 | SHADOW | 1.12554075 | 202.05 | PASS |
| 162706_519367266 | ACTIVE | 0.73540114 | 16.83 | PASS |

十轮均记录左臂1、夹爪2，开/闭/开三次事件、回Home和最终快照；cleanup无错误、保护回读开启、速度回读恢复到10.0、SDK已停止。正式成功标志为 `SERVO_COMPLETED_RETURNED_HOME`。这是这十次记录支持的范围，不能推导总尝试100%成功或新工况可靠性。

## 4. 早期试验也要保留

9/16另有3份报告：`120026_105169918` 在frame 0求解超时，`144907_221796708` 在frame 1报上次指令速度超限，均关闭Shadow后由基础Servo完成抓放；因此“抓放完成”不等于“MPC全过程有效”。`145400_861015006` 的Shadow全程有效，但执行器版本早于固定十轮，本次不混入原统计。三份早期报告的执行器哈希在收到Servo源码/备份树中未找到对应字节。

两个wrapper仍会删除失败运行JSON，Active内部部分诊断只在正常退出循环后写入。完整压缩包解决收件完整性，不能补出历史已被删除的证据；需保留相关终端记录并修复后续记录策略。

## 5. 与Mac的差异及文档陷阱

- `run_experiment.py`、`test_mpc.py`、`requirements.txt`、`README.md`与Mac逐字相同。Debian `controller.py`只扩充QP失败错误信息（迭代数、耗时、原始/对偶残差），控制公式/约束未变。Mac核心SHA `4341f2fd73217591c41205f21c65022647e5dd6ec65eaf05ec7291c3bad4dc09`；收到核心SHA `314f5cbe6c43876027bb1d25dd3a24515e4790d054a818281c78f26bc24c515c`。
- Debian离线report记录的核心哈希对应未增加诊断的版本，与当前收到核心不同；其余8项源码/输入哈希及两份产物 `traces.npz`、`comparison.png` 均匹配。不要拿当前核心哈希覆盖旧报告。
- Linux离线报告记录Python3.10.12、NumPy2.2.6；Mac为Python3.13.5、NumPy2.1.3。两侧OSQP1.1.1/SciPy1.15.3/PyBullet3.2.5一致。六轮物理误差近同，Linux求解p99约0.81–0.88ms、最大3.12ms，Macp99约0.58–0.60ms、最大2.64ms；都不包含真实网络/SDK。
- Debian MPC包内 `frame_calibration/analysis/calib_common.py` 只有21行，是从arm_profiles取左臂映射的仿真兼容层，含Linux绝对URDF路径；**它没有CONFIRMED标定内容，绝不能覆盖Mac主标定真源**。
- 旧README只描述9/15阶段；Linux离线report还遗留“local Mac observations”和`debian_execution_allowed=false`，应按schema/platform解释为离线实验，不据所在目录或旧文案判断真机身份。

## 6. 尚未关闭的运行资格与下一步

1. 十份真机报告没有记录 `controller.py` 哈希；收到核心的当前哈希不能追溯绑定旧运行。今后还需记录Python/NumPy/SciPy/OSQP/厂商SDK版本及实际导入路径。
2. wrapper明确绕过正式Servo资格。1.5°关节范围不证明全臂、工具、持物、桌框/料箱的碰撞净空；新命令不能继承旧reference批准。
3. alpha=0.4缺真机辨识；回读缺设备采样时间戳/序列号，不能由40ms调度推断反馈每次更新。
4. QP之后的端点渐收改变实际发送命令，尚无后处理加速度的独立复验。检查原QP残差不覆盖后处理全部性质。
5. 20ms周期并非硬实时通过。Shadow最大迟到390.82ms、Active138.95ms；求解器10ms预算不含全循环SDK/网络/调度。需明确异常时停机/记录策略，不用放宽阈值代替验证。
6. 后续在开发副本做路径适配、诊断合并、失败日志保留及离线回归；保持原包/报告不变，发布仍通过现有生成器。应单独列出Mac仿真、Debian仿真、Shadow、Active的入口和证据，不合并成一个“最新可上机”标签。
