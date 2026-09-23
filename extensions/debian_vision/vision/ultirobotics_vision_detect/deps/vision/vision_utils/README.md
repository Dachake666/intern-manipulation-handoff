# vision_utils

相机和检测工程共用的 JSON 读写、字符串、计时、错误定义与日志支持库。

在视觉工程根目录执行 `colcon build --packages-up-to vision_utils`。`CMakeLists.txt` 从 `src/` 收集实现并安装 `include/` 头文件，依赖 ROS 2、nlohmann JSON 和兼容的日志/格式化库。环境边界见 [视觉扩展入口](../../../README.md)。

此库是当前构建依赖；保留其导出头文件和依赖的许可证信息。
