# 视觉观测与离线回放

本模块接收同步RGB、registered depth和CameraInfo，生成 `vision_observation.v1`；不输出关节角、不调用厂商SDK。`vision_system/replay.py` 离线回放帧，ROS节点负责采集并调用同一检测逻辑。

## 负责人维护的边界

- RGB/深度时间差≤50ms；5帧中至少3帧稳定支持。
- 中心标准差≤5mm、yaw标准差≤3°、尺寸误差≤15%、有效深度≥80%、置信度≥0.8。
- 每帧保存无损RGB/depth NPY、CameraInfo、中间结果与SHA，便于脱离相机复核。
- 观测契约的单位、frame、标定身份和矩阵方向必须随输出传递；检测成功不能替代有效外参。

从仓库根查看离线回放参数：

```bash
(cd src/vision && python3 -m vision_system.replay --help)
```

真实回放需要按时间排序的5个frame bundle、catalog、相机/内外参身份和输出路径。选定回归入口为根 `tools/offline_checks.py`。

## SDK世界位姿到桌上任务

上游已提供SDK世界位姿时，使用 `robot_mission.tabletop_plan.build_tabletop_plan(request)`；请求示例路径只是待准备输入，不是默认已采集场景。以下从仓库根执行：

```bash
(cd src && python3 -m robot_mission.tabletop_plan ../.runtime/request.json --out ../.runtime/candidate.json)
```

必须明确 `frame_id=sdk_world`、`length_unit=millimeter`、`quaternion_order=xyzw` 和 `pose_role`：OBJECT_POSE通过物体模板求抓取位姿；GRASP_POSE表示抓取中心；EE_POSE仅表示SDK endpoint。共享臂配置负责补偿，已经补偿的endpoint不再补TCP。

固定 `station.place` 与 `station.safe_endpoint` 属场景配置，观测只更新当前目标。夹爪policy使用设备原生uint16参数，`executor_binding=PENDING_SITE_INTEGRATION`，写入候选不等于串口已接通。

输出保持 `tabletop_plan.v1 / OFFLINE_CANDIDATE_PRECHECK_REQUIRED`。需要live IK、控制器限位、完整碰撞与人工授权后才可进入运动；同级 `robot_mission/mission_runner.py` 是真机入口，不是离线回放工具。

## ROS与C++工程

本Python包在兼容Humble环境中构建 `xifeng_vision`。相机话题与输出目录由参数配置。`extensions/debian_vision/` 的C++相机/检测/手眼工程使用另一套ROS服务/动作接口，不能直接视为已接入本观测契约；适配层、SAM3服务和标定质量门仍有缺口，详见根 `docs/KNOWN_ISSUES.md`。
