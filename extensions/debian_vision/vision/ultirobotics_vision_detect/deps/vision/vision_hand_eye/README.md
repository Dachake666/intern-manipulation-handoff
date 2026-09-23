# vision_hand_eye

独立手眼标定节点，通过相机服务读取数据并提供 `vision_interface/srv/HandEyeCalibration`。它与相机/检测启动入口分开构建和启动，不由 `app.launch.py` 自动启动。

## 构建与配置

从视觉工程根目录执行 `colcon build --packages-up-to vision_hand_eye`。依赖 ROS 2、OpenCV、PCL、Eigen、nlohmann JSON、日志库及 `vision_interface`；环境要求见 [视觉扩展入口](../../../README.md)。`local_data_test.cpp` 被正式节点编译，是构建依赖。

节点的第一个参数是包含 `config.json` 与 `mark.json` 的配置目录。核对标记板、标定模式、相机身份和可写数据目录后，从视觉工程根目录启动：

```bash
source install/setup.bash
ros2 run vision_hand_eye vision_hand_eye "$PWD/deps/vision/vision_hand_eye/config"
```

服务请求字段以 `vision_interface/srv/HandEyeCalibration.srv` 为准；保存结果字段为 `save_json_file`。机器人手位姿必须明确坐标系、长度单位与 `xyzw` 四元数顺序，不使用示例数值代替现场读数。采集配对数据需记录相机、机器人、配置和工具版本；机器人运动由现场人员确认。

## 验收边界

`data/handeye/` 是历史原始证据，配置中的旧外参不代表当前安装有效。应在目标设备重新核对采集条件、评估残差并保留原始配对数据；未完成验收前不覆盖交接包的权威标定，也不据此授权机器人运动。

此文档提供入口与边界，不声明完整 ROS/C++ 构建、相机连接或新手眼标定已通过。
