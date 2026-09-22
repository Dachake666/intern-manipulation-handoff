# xifeng_vision

ROS 2 只负责把同步的 RGB、registered depth 和 CameraInfo 送入同一个离线可回放
检测器。检测器不生成关节角，也不调用厂商 SDK。输出必须通过
`vision_observation.v1`，再交给任务适配、规划和无运动预检。

- 同步门：RGB/深度时间差 ≤ 50 ms。
- 稳定门：5 帧中至少 3 帧；中心标准差 ≤ 5 mm；yaw 标准差 ≤ 3°；
  尺寸误差 ≤ 15%；有效深度 ≥ 80%；置信度 ≥ 0.8。
- 每帧保存无损 RGB/depth NPY、CameraInfo、检测中间结果及 SHA-256，可脱离相机回放。

Humble 环境通过 `colcon build --packages-select xifeng_vision` 构建。相机话题、
目录和证据目录全部是 ROS 参数，不在节点内写死。

## 已在 SDK world 的桌上抓取接口

若相机上游已经输出机器人 SDK 世界系的 XYZ + 四元数，直接调用
`robot_mission.tabletop_plan.build_tabletop_plan(request)`，或运行：

```bash
python3 -m robot_mission.tabletop_plan request.json --out candidate.json
```

这条入口要求并记录以下不可省略的语义：

- `frame_id: sdk_world`、`length_unit: millimeter`、`quaternion_order: xyzw`；
- `pose_role: OBJECT_POSE` 表示物体坐标，按物体模板的 `T_object_grasp`
  计算抓取中心；`GRASP_POSE` 表示上游已给抓取中心；`EE_POSE` 仅用于示教的
  SDK endpoint 直通点；
- 左臂 SDK endpoint 到抓取中心的补偿由 `arm_profiles.v1.json` 和
  `calib_common.py` 的确认值实时推导，不在相机代码里复制固定世界 Z 偏移；
- 固定料框的 `station.place` 与桌上安全起点 `station.safe_endpoint` 配置一次，
  每帧只替换 `observation.pose`。物体类型变化时只更换该类的尺寸和
  `T_object_grasp`，不需要人工重填这次物体坐标。
- `gripper_policy` 是现场可调接口：`open.position`、`close.speed`、
  `close.force` 使用夹爪协议原生 uint16 值；当前只写入并绑定候选计划，
  `executor_binding` 固定为 `PENDING_SITE_INTEGRATION`，不会直接下发串口命令。

请求中的配置形状如下，数值可按现场夹爪与物体调整：

```json
{
  "gripper_policy": {
    "policy_id": "left-gripper2-site-tunable-v1",
    "gripper_id": 2,
    "tuning_status": "SITE_TUNABLE",
    "protocol_encoding": "EB90_UINT16_LITTLE_ENDIAN",
    "open_command": 17,
    "close_command": 16,
    "command_contract": "OPEN_POSITION_CLOSE_SPEED_FORCE",
    "value_unit": "device_native_uint16",
    "executor_binding": "PENDING_SITE_INTEGRATION",
    "open": {"position": 500},
    "close": {"speed": 500, "force": 1000}
  }
}
```

输出恒为 `tabletop_plan.v1 / OFFLINE_CANDIDATE_PRECHECK_REQUIRED`，动态和持物段
只生成非混合、逐段等到位的 MoveWorlds。它不会连接或使能机器人；live
`armTryWorlds`、控制器限位与完整碰撞预检通过前，`real_motion_authorized` 永远为
`false`。
