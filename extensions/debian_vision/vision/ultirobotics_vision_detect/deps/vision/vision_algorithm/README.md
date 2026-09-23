# vision_algorithm

检测算法库，包含 ONNX 分割模型加载、SAM3 客户端、图像处理、点云处理与结果保存，由 `vision_detect_ros` 调用。

在视觉工程根目录执行 `colcon build --packages-up-to vision_algorithm`。构建链为 `CMakeLists.txt` → `build_vision.sh` → `CMakeLists.txt.build`；这些文件共同决定实际源列表和依赖搜索路径。依赖 OpenCV、PCL、Eigen、ONNX Runtime、ZeroMQ、`vision_utils` 与日志库。

模型路径由调用方显式传入，无工作目录默认模型。随包工作站配置共享工程根目录的 `best.onnx`，定位规则与模型身份见 [视觉扩展入口](../../../README.md)。SAM3 服务地址由工作站配置指定，需部署者确认服务可用及协议匹配。

独立路径检查不等于完整编译、模型推理或视觉闭环验收。
