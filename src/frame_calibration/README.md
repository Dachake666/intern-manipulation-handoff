# 标定、真机执行与证据

本目录由标定分析、执行器、必要标定输入和选定运行证据组成。总入口在仓库根 `START_HERE.md`；真机版本恢复见 `robot/README.md`。

| 目录 | 职责 | 主要入口 |
|---|---|---|
| `analysis/` | 离线标定与坐标核查 | `calib_common.py`、`verify_worlds_record.py`、`fit_eye_to_hand.py` |
| `robot_side/` | 真机 SDK 会话、执行器、Servo、预检、夹爪与安全回归 | `execute_trajectory.py`、`execute_servo_grasp.py`、`execute_tabletop_*`、`sdk_session.py` |
| `data/worlds/` | 会话漂移检查的参考与候选样本 | 7/10参考记录、8/31候选记录；历史时间戳不证明当前有效 |
| `records/` | 选定原日志、固定输入、源码身份与测试夹具 | 由源码/输入 SHA 与运行参数共同识别 |

`robot_side/test_*.py` 同时存在离线回归和硬件能力探针，不能广泛自动执行。选定离线入口为仓库根 `tools/offline_checks.py`。

## 坐标与标定

标定结论只认 `analysis/calib_common.py` 的 `CONFIRMED_*` 及其置信度标记。规划平移 `CONFIRMED_T_SESSIONS_MM` 与 v2 位置解释的 `CONFIRMED_T_SESSIONS_V2_MM` 服务不同模型，不能混用。UVW 说明见 [UVW_TO_QUATERNION.md](UVW_TO_QUATERNION.md)。

`verify_worlds_record.py` 已实现 Model-A 会话漂移门，离线比较参考和候选的平移、残差与新鲜度。明确给入当天采样记录，避免把“最新历史文件”当成当前现场：

```bash
# 从仓库根执行；将 candidate-worlds.json 替换为已采集记录
(cd src && python3 -B frame_calibration/analysis/verify_worlds_record.py ../.runtime/candidate-worlds.json --json-out ../.runtime/frame-gate.json)
```

缺少候选、漂移超限或过期时应保持阻断。该检查不能代替工具、腰/底盘、相机与场景的现场核实。

`fit_eye_to_hand.py` 能计算外参，但当前输出 VALIDATED/locked 的实现缺少独立质量批准门，不能据此直接激活视觉运动。具体问题见根 `docs/KNOWN_ISSUES.md`。旧网格、腕部、触碰和命令侧采样工具及不完整的配套分析流程保存在 PRE_PRUNE，仅作历史参考。

## 真机代码身份

开发目录的共享依赖和冻结历史组合分别维护。B2～B7 恢复历史成果，`tools/restore_run.py` 按一轮报告装配；不把当前 `sdk_session.py` 覆盖到旧运行组合。新执行器默认关闭运动，使用互斥锁，并验证正常/异常路径的保护与速度恢复。
