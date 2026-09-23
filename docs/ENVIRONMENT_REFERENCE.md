# 环境与安装参考

使用三个独立环境：仿真/离线验证、机器人 SDK、相机与 ROS/C++。选择依据是解释器、系统架构与依赖 ABI，而不是目录名。仿真安装使用不继承系统包的独立虚拟环境验证；这不是格式化操作系统后的整机验收。SDK 动态链接、完整 ROS/C++ 和现场设备仍需在目标机器检查。

## 仿真与离线验证

参考平台为 **macOS 15.7.4 / arm64、CPython 3.13.5、Apple clang 17**。完整 Python 依赖锁为 `src/requirements/simulation-py313-macos-arm64.lock.txt`；安装记录和验证范围在 `evidence/handoff_checks/clean_environment.json`。其他平台不能直接套用这一验收结论。

先安装匹配的 Python 和 Xcode Command Line Tools。以下命令从仓库根执行；虚拟环境不启用 `--system-site-packages`：

```bash
python3.13 -m venv .venv-sim
source .venv-sim/bin/activate
python -m pip install pip==25.1.1 setuptools==80.9.0 wheel==0.45.1 numpy==2.1.3
CFLAGS="-Dfdopen=fdopen" python -m pip install --no-build-isolation --no-binary=pybullet pybullet==3.2.5
python -m pip install -r src/requirements/simulation-py313-macos-arm64.lock.txt
python -m pip check
python tools/offline_checks.py
```

PyBullet 3.2.5 在上述工具链直接源码编译会触发其内置旧 zlib 的 `fdopen` 宏冲突。该编译选项保留系统 `fdopen`，不修改第三方源码或升级 PyBullet；`--no-build-isolation` 使用提前安装的 NumPy 与构建工具。完整依赖锁随后将运行依赖固定为验证版本。构建需联网、编译器和时间；交接包不携带仿真环境镜像或 PyBullet 二进制轮子。

版本锁不包含操作系统库。若目标平台安装失败，保留失败日志，登记新的解释器/依赖组合，再跑对应回归；不得把改过的环境标为同一验证组合。GUI 显示、设备接入和现场运动不在该离线安装验收范围内。

历史 `planning-py310.lock.txt` 自述 Linux x86_64 / Python 3.10，`vision-py310.lock.txt` 仅含部分视觉依赖。B5 历史仿真报告记录 Python 3.10.12、NumPy 2.2.6、SciPy 1.15.3、OSQP 1.1.1、PyBullet 3.2.5、matplotlib 3.10.9。它们不是当前开发环境的替代锁，不应混装。MPC 自带 `requirements.txt` 中的版本范围也不等于完整锁定快照。

## 机器人 SDK

安装物位于 `sdk/`：

| 文件 | 身份 |
|---|---|
| `pypilot-1.0.0422.1-cp310-cp310-linux_x86_64.whl` | CPython 3.10 / Linux x86_64；SHA `9e99bb01e672cbb3616aac0ab3dd43c9112992907ecad37b4de93ef91f44a74b` |
| `“隙锋”人形机器人SDK使用指南（Python）V260526.1.pdf` | SHA `74d78fc3f71d28d6d3792bf957a36cdc97c173e7f7b91dded25761556c88177f` |

在匹配系统建立独立环境：

```bash
python3.10 -m venv .venv-sdk
source .venv-sdk/bin/activate
python -m pip install sdk/pypilot-1.0.0422.1-cp310-cp310-linux_x86_64.whl
```

wheel 包含 pypilot.so 与部分网络/SSL 库，仍需核对 Boost Python 3.10 和目标系统动态库。手册与 SDK 版本不同，以安装轮子的真实接口为准；接口检查工具为 `src/frame_calibration/robot_side/probe_sdk.py`。安装和接口导入成功不授权连接、使能或运动。

## 相机、检测与手眼

工程位于 `extensions/debian_vision/vision/ultirobotics_vision_detect/`。构建历史指向 ARM64 / ROS 2 Humble，不等于机器人 SDK 的 x86_64 环境。

系统依赖涉及 Humble/ament/colcon、C++ 编译器、OpenCV、PCL、Eigen3、fmt、spdlog、libzmq、cppzmq 和 ROS 消息/动作支持。`package.xml` / CMake 不是完整系统安装锁。Python ROS 模块如 rclpy、sensor_msgs、message_filters、cv_bridge 必须来自兼容的 ROS 安装。

`docs/vision_selection.json` 记录源码、模型、第三方源依赖和可选厂商库。默认包不含各架构的完整 `.so` 集合；构建前按目标架构补齐 Orbbec、TY camera、ONNX Runtime、spdlog 等实际需要的库并保留 SONAME 关系。Orbbec ARM 文件身份为 2.7.6，wrapper CMake 标称 2.4.11，兼容性需检查。

`CMakeLists.txt.build`、`build_vision.sh` 和配置引用的模型/背景图属于构建或运行依赖。重新导入浮动分支不能替代精确源码。相机插件按实际设备选择。构建与启动细节以工程内维护说明为准。

主 `app.launch.py` 启动相机与检测，手眼是独立节点。SAM3 client 不能替代缺失的服务端/模型。相机、实际配置、二进制 SHA 和外参质量均需在新设备登记后再接运动链路。

## 数据与验收记录

`data/handeye/` 保存39组历史样本、199个成员，分11/10/9/9四组；它们用于分析，不是已批准外参。大数据不随单独 Git 克隆自动恢复，迁移时保留完整包。

新设备验收记录：包 SHA → 系统/架构/解释器 → 依赖版本 → 离线测试 → 必要构建/模型加载 → 设备与配置核实 → 单独现场测试。每步保留命令、结果和未覆盖项。不能从前一步 PASS 推定后一步通过。
