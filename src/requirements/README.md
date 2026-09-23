# Python依赖清单

按用途建立独立环境，不共享仿真、厂商SDK和ROS的虚拟环境。详细安装步骤与已知边界在根 `docs/ENVIRONMENT_REFERENCE.md`。

| 文件 | 用途 |
|---|---|
| `simulation-py313-macos-arm64.lock.txt` | Python 3.13.5 / macOS arm64 的完整仿真依赖锁；按根环境说明先构建 PyBullet |
| `planning-py310.lock.txt` | 历史规划依赖；自述Linux x86_64 / CPython3.10，不与当前清单混装 |
| `vision-py310.lock.txt` | 历史离线视觉直接依赖子集，不含完整ROS系统库 |
| `../pick_place_coord/mpc_experiment/requirements.txt` | MPC模块依赖范围，不是完全锁定环境 |

在新设备创建隔离环境后，从仓库根执行 `python3 tools/offline_checks.py`，记录解释器、架构和测试结果。独立虚拟环境验证不等同于格式化操作系统、GUI 和现场设备验收。

厂商wheel位于根 `sdk/`，只支持CPython3.10 / Linux x86_64；不作为仿真依赖。ROS 2 Humble的rclpy、sensor_msgs、message_filters、cv_bridge来自兼容ROS安装，不以任意pip包替代。
