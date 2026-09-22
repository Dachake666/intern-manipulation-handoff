# 2026-09-03 桌上 MoveWorlds：现场成功工作参考，未封存

用户报告当前整套抓取、放置、归位已成功并拍摄视频。本次记录经验并完善离线生成器，**不提升 REALVERIFIED，不覆盖原问题轨迹，不替换 Debian 已跑通的文件**。

2026-09-04 已收到用户提供的原 Debian ZIP（SHA-256 `96c606335c14ff54…e8340832`）。最终执行器和计划已按原字节保存到 `artifacts/`，哈希分别精确匹配 `807edfce…` 和 `6b8ad92c…`；详情见 `package_audit.json`。原 ZIP 含缓存、控制器变量快照、旧失败预检和敏感字段，不复制进 Git。

## 实测事实与证据边界

| 日期 | 计划 SHA | 执行器 SHA | 模式 | 完成证据 | 末端误差 | 保护/速度 | 结论 | 文件 |
|---|---|---|---|---|---|---|---|---|
| 2026-09-03 | `6b8ad92c…` | `807edfce…` | 5% MoveWorlds，运行时限位过滤修改 | 用户/摘要报告全流程成功；粘贴日志止于 PICK_DESCEND | 未提供数值 | 执行会话最终状态未见 | LIMITED / INCONCLUSIVE，不封存 | `logs/debian_summary.md`、`logs/debian_run_excerpt_redacted.log` |

完整哈希、路点、当次 IP 在 `as_run.json`；IP 不是永久配置。原摘要的“PASS/基线”属于来源自身措辞，不代表本工作区完成正式验收。原文件保持不变；归档副本经 `scrub_log.py` 检查。后补日志含登录凭据，归档中已经删除，未保存明文。首个只读会话的正常退出不能当作后续使能执行会话的收尾证明。

- SAFE 是 SDK endpoint，不是夹爪抓取中心。实测 pick=`[700,250,360]`、pick hover=`[700,250,540]`、place=`[720,100,495]`、place hover=`[720,100,550]`，单位 mm；UVW 固定 `[-72.779,13.241,-6.634]`。放置下降距离 **55 mm**，不是仍沿用 180 mm。
- PLACE Z=370 过低；现场改为520、再降至495。495是当前场景经验点，不是可套用到任意瓶子/料框的常量。
- 最初远端 XYZ 的1008个姿态粗扫 raw IK=0，只能说明这些采样未解出，不能宣称整个连续姿态空间数学上无解。移近后剩下的是默认5°内缩裕量边缘问题，两者不要混同。
- 实测命令把 `LIMIT_MARGIN_DEG` 设为0，并绕过整个 `_controller_limit_violations` 过滤。`sdk_session.check_limits` 实际使用历史控制器回读的静态限位表及物理 j7 上界；摘要称“URDF-only”不准确。仍需实时控制器硬限位交集。本次没有将绕过方式写进源码或默认配置。
- 用户补充：料框基本没动、桌子移动了，场景坐标有偏移。旧相机框位置与新实测放置点不吻合，不能直接组合成现场仿真。此处不要求精确中心；允许保存现场确认的框内可放区域和已试过的放置点。

## 本次源代码改进

源：`robot_mission/tabletop_plan.py`、`placement_clearance.py`，契约仍为 `tabletop_plan.v1`；12个状态、7条 MoveWorlds 和两个 hover 不变，没有新增 Servo 执行分支。

1. `motion_policy.placement_clearance` 以姿态相关的完整手腕/开爪包络计算放置高度，只向上调整请求点。包络最低点须高于框沿最高点加余量；包络是输入，必须来自实测或本次完整 IK 路径的保守联合几何，不可把近端腕关节永远视为 endpoint 的刚体。
2. `hover_z_mm` 是独立世界高度下限，和 `minimum_hover_gap_mm`、持物最低点净空共同决定 hover。满足条件时保留495→550；不足时明确报告抬高量，并重新做 IK/限位检查。抓取抬升仍保留 `minimum_lift_mm` 原门槛。配置此策略时旧 `place_hover_offset_mm` 只保留字段兼容，不参与高度选择。
3. `object.T_object_geometry` 明确从相机 feature 系到真实包围盒中心的变换；`T_object_grasp` 则描述 feature 到抓点。两者用途不同，不能把“偏右上”的检测点当几何中心来计算瓶底。
4. `SUPPORTED_ONLY` 检查物体最低点与内底的间隙，不能将抬高后松爪静默称为落底放置。`ALLOW_BOUNDED_DROP` 需明确给出允许落差，仅生成候选；不证明物体稳定落座。`place_object_after_release` 表示开爪瞬间位姿，不是物体落底后位置，退离/落座碰撞仍须另行校验。
5. 框内检查覆盖物体整个投影，不只是 TCP/抓取点。可以用 `material_box` 几何，或用 `station.place` + `station.place_region` 保存一次确认的内缩可放区域，**不必精确识别中心**。两种模式不能混用来绕过模型一致性检查。
6. 缺少手腕包络、feature→几何中心、支持面或可放区域证据时，新生成计划保持 BLOCKED。旧文件可解析；不会因为 JSON 格式有效而假装碰撞预检完成。

坐标合同：相机 `OBJECT_POSE/GRASP_POSE` 按姿态经工具变换得到 SDK endpoint；`EE_POSE`（示教 safe 或这次已经下发的实测点）不再做第二次 TCP 补偿。任何净空抬高是明确记录的路点调整，不是 TCP 补偿。

## 离线计算与仿真结果

使用当前左夹爪 proxy `100×178×80 mm`、中心 link11 `[0,84,0] mm`，以及真源 TCP `[0,168,-39] mm`。给定当前 UVW，工具包络最低点比 SDK endpoint 低 **47.922273 mm**：

| endpoint Z | 工具 proxy 最低世界 Z |
|---:|---:|
| 370 | 322.077727 |
| 495 | 447.077727 |
| 520 | 472.077727 |
| 550 | 502.077727 |

PyBullet DIRECT 旋转 box AABB 与八角点矩阵计算误差 <1.2e-13 mm。该测试已收进 `robot_mission.test_placement_clearance.ToolProxyBulletTests`，**只证明局部几何公式，不证明真实手腕/完整路径无碰撞**。

旧框“检测点=外形中心”的假设模型最高口沿417.311890 mm，则495的工具净空约29.77 mm。但新点对应抓点在旧框局部 x=-299.57 mm，旧框外半宽仅155 mm；因此这一口沿不能当作当前现场的碰撞验收依据。

整场景复核门仍 BLOCKED：20260831 worlds 记录的 Model-A 相对参考漂移105.68 mm，z漂移约-60.41 mm；本次未修改 `CONFIRMED_*`，未混用V2会话平移。直接 SDK-world 局部工具补偿公式验证不依赖这套全局平移。

## 测试与发布状态

- `python -m unittest robot_mission.test_placement_clearance robot_mission.test_tabletop_plan -q`：47项通过（含PyBullet局部box测试）。
- `test_execute_tabletop_pick_place_worlds.py`：11项；`test_execute_trajectory_options.py`：14项；`test_servo_safety.py`：17项通过，全部使用离线测试替身，不连接机器人。
- 扩展检查 `test_mission` + `test_obstacle_scene_support`：17项中3项报旧字段 `safe_corridor_endpoint_z_mm` 缺失。未改动的原始候选JSON也能复现，是旧避障脚手架与两-hover计划的不兼容，本次未修改暂停研究分支去掩盖问题。
- `releases/make_release.py --verify` 通过。发布目录完全未改、未重建，原计划仍为`0f234276…`，不是本次Debian成功的`6b8ad92c…`。
- 本地执行器/source与release一致为`267eee1b…`，Debian现场执行器为`807edfce…`；sdk_session为`c3e464f4…`一致。Debian最终JSON及执行器原文件现已收到并原样记录，但暂不覆盖真源：执行器放宽了safe起止判断到20mm/8°，包内SHA清单仍引用中间版本；最终JSON是手改路点快照，旧派生字段没有同步重算。

## 以后如何复用

一次保存：机械臂/TCP、已验证safe、物体类别尺寸及feature→抓点/几何中心、夹爪可调参数、框内放置点/可放区域、框沿和内底高度。无需每次提供框中心。

每次只更新：相机输出 `observation_id`、类别、SDK-world XYZ（mm）和四元数xyzw及其参考点语义。同一固定工位直接复用放置配置；换物体型号补一次尺寸与抓取关系，桌子/料框移动后重新确认可放区域及高度即可。改变机器人/底盘/腰部/TCP后重核相应坐标映射。

流程仍是：相机输入 → 生成已补偿SDK endpoint JSON（含高度/净空报告）→ 本次密集IK、真实限位及完整碰撞/退离检查 → Debian只读预检 → 人工确认低速试跑。

执行器及契约对齐后，日常只需发送新的计划JSON，必要时同步校验清单；尺寸/站点模板与仿真报告留在Mac。不改SDK/执行器就不重发程序。不能把“只传JSON”理解为省略每次预检，也不保证任意相机坐标都可达。夹爪开口、闭合速度、力度和等待时间继续由JSON中的 `gripper_policy` 配置。
