# ROS 视觉扩展

本目录提供 RGB-D 相机驱动、物体检测服务和独立手眼标定节点。机器人任务适配与无运动预检位于交接包 `src/robot_mission/`；Python 视觉回放位于 `src/vision/`。这些接口尚不构成已验收的自主抓放闭环。

## 环境与构建入口

在兼容 ROS 2 Humble 的 Linux 环境准备 OpenCV、PCL、Eigen、ZeroMQ、ONNX Runtime 和相机厂商库。按机器架构补齐 `deps/vision/vision_third_party/` 的库文件，保留厂商许可证和头文件；现有源码清单不等于完整可安装的二进制依赖包。

从本目录执行构建入口：

```bash
colcon build --packages-up-to vision_bringup vision_hand_eye camera_test --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 2
```

`vision_algorithm` 的 `CMakeLists.txt` 会调用 `build_vision.sh` 和 `CMakeLists.txt.build`；三者均为构建必需文件。新环境需要核对其安装前缀与依赖搜索路径。完整 ROS/C++ 编译、相机连接和推理需在目标机器验证。

## 配置与模型

`config/test1/workstation.json` 是单相机示例；`config/multiple/workstation.json` 是多相机示例。两者显式引用同一份 `best.onnx`，模型 SHA-256：

```text
7264c02ef1d06815aaa367f65c41e4285ad6bf51b9803f1461e34e9e72b3aa71
```

`model_path` 必须是非空字符串。带目录的相对路径以配置文件所在目录解析；旧式纯文件名以该配置目录的 `models/` 子目录解析；绝对路径按其本身解析。随包配置使用 `../../best.onnx`，不依赖启动工作目录。缺少字段或模型文件时初始化失败，不回退读取工作目录的同名模型。

配置目录和根 `best.onnx` 必须保持上述相对布局；colcon 安装节点不会自动替代这套配置和模型目录。

两份配置的相机身份、工作站外参和算法参数均需现场核对；共享模型不使两套历史外参等价。模型文件来自接收的视觉工程；交接材料不包含完整训练数据、训练过程或新增授权声明。

目标机器完成构建并核对配置后，从本目录启动：

```bash
source install/setup.bash
ros2 launch vision_bringup app.launch.py config_file:="$PWD/config/test1/workstation.json"
```

此入口启动 `camera_ros` 与 `vision_detect_ros`，会访问配置中的相机；它不启动手眼节点。输出目录 `archive.save_path` 由部署者设置为可写运行目录。手眼入口见 [vision_hand_eye](deps/vision/vision_hand_eye/README.md)。
