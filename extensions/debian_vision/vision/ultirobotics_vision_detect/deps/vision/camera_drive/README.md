# camera_drive

RGB-D 相机驱动库，为 `camera_ros` 提供设备打开、配置和图像/内参读取接口，包含 Orbbec 与 TY 两类驱动。设备选择来自工作站相机配置。

在视觉工程根目录执行 `colcon build --packages-up-to camera_drive`。依赖 ROS 2、OpenCV、`vision_utils` 和与目标架构匹配的厂商相机库；完整环境要求见 [视觉扩展入口](../../../README.md)。

`CMakeLists.txt` 明确编译 `src/ty_cam/TYThread.cpp`；厂商与本库的头文件仍有独立包含路径，不应仅因同名或同哈希删除。编译成功不证明相机连接、序列号、深度单位或配准正确；这些须在目标设备检查。
