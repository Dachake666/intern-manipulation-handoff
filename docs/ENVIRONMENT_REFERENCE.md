# 首版环境交接说明

本说明来自收到源码、构建元数据和既有报告的静态核对。此次没有安装依赖、构建 C++、启动 ROS、导入 pypilot、连接设备或重跑实验；“报告记载的环境”与“本次已复现环境”分开。

## 1. 两端包含不同运行环境

| 环境 | 已有依据 | 首版边界 |
|---|---|---|
| Mac 规划、PyBullet、RGB-D 离线回放 | `requirements/planning-py310.lock.txt`：NumPy 2.1.3、PyBullet 3.2.5、pybullet-planning 0.6.1、jsonschema 4.23.0；vision lock 为 NumPy/jsonschema 同版本 | README 指定 CPython 3.10，但 planning lock 自述 Linux x86_64；这不是 Mac 安装成功证据。交付后需在实际 Mac 解释器验证，并记录解释器路径、架构与测试结果 |
| Mac MPC 既有离线实验 | 9/15 报告：macOS 15.7.4 arm64，Python 3.13.5，OSQP 1.1.1、NumPy 2.1.3、SciPy 1.15.3、PyBullet 3.2.5、matplotlib 3.10.0 | 六种场景/模式组合是离线结果；不要称为项目统一 Python 3.10 环境 |
| Debian MPC 既有离线实验 | 9/16 报告：Linux `6.12.101+deb13-amd64`、x86_64、glibc 2.35，Python 3.10.12；OSQP 1.1.1、NumPy 2.2.6、SciPy 1.15.3、PyBullet 3.2.5、matplotlib 3.10.9 | 这是报告记录的平台组合，不足以证明宿主机与容器发行版一致。PyBullet 对照归仿真，不归真机 |
| Debian 机器人 SDK | 已收到 `pypilot-1.0.0422.1-cp310-cp310-linux_x86_64.whl` 与厂商文档 | wheel 标签只支持 CPython 3.10 / Linux x86_64；不能安装到 Mac/ARM，不能依据 METADATA 的“3.10+”放宽 wheel ABI。未在本次验证安装与动态链接 |
| C++ ROS 相机、检测与手眼 | 收到 CMakeCache 指向 `/opt/ros/humble`、`/usr/lib/aarch64-linux-gnu/cmake/opencv4`、PCL；工具目录包含 CMake 3.22.1；所检 ARM `.so` 的 ELF e_machine=183 | 有 ARM/Humble 历史构建痕迹，不能把它和 x86_64 pypilot/MPC 当同一已复现环境；旧 build/install 不作为可移植交付环境 |

MPC 两端的 `requirements.txt` 含 NumPy/SciPy/matplotlib/pytest 版本范围，属于依赖约束，不是完全锁定的安装快照。两套 `.venv*` 与 site-packages 不直接交付；保留报告所列实际版本后在隔离环境重建。ROS 的 rclpy、sensor_msgs、message_filters、cv_bridge 必须来自兼容的 Humble 安装，不能用任意 pip 包替代。

## 2. 必须保留的 SDK 安装物

以下两项在新原包 `workspace/` 根目录，精确路径和 SHA 也写入 `vision_selection.json.sdk_installation_materials`：

- `pypilot-1.0.0422.1-cp310-cp310-linux_x86_64.whl`，9,243,776 字节，SHA-256 `9e99bb01e672cbb3616aac0ab3dd43c9112992907ecad37b4de93ef91f44a74b`。
- `“隙锋”人形机器人SDK使用指南（Python）V260526.1.pdf`，1,825,331 字节，SHA-256 `74d78fc3f71d28d6d3792bf957a36cdc97c173e7f7b91dded25761556c88177f`。

wheel 包含 pypilot.so、类型提示及 libyouizmq/libxoip/OpenSSL 等随包库；METADATA 声明 pypilot 1.0.0422.1、MIT、Boost Python 3.10 需求。不能据“self-contained”文字断言目标系统依赖闭合。保留完整 wheel 即可，不重复交付其解压目录。接手时先静态核对 ABI/系统依赖，再按现场授权验证 SDK；安装不等于可使能或运动。

## 3. 视觉源码与依赖允许清单

主工程来源固定为新包的 `vision/ultirobotics_vision_detect/`。相机节点、检测节点、ROS 接口、launch、camera_drive、vision_algorithm、vision_utils、vision_hand_eye、vision_third_party 构建文件及 camera_test 源码已逐文件列出。`CMakeLists.txt.build` 与 `build_vision.sh` 是算法库的实际构建依赖，不能漏掉。

手眼开发源只选 `deps/vision/vision_hand_eye/`；根部 `vision_hand_eye/` 和 `vision/vision_hand_eye/` 的 19 项功能文件同 SHA，记录身份后不再复制。8 个归档 Git 身份均记录在 JSON；这是主项目、5 个 deps 子项目、2 个重复手眼项目的路径身份，不是 8 套独立能力，也不证明各工作树干净。`deps.yaml` 中算法分支写 `crop_region`，实际记录 HEAD 是 `polygon`；不要重新按浮动分支拉取来代替收到源码。

清单分为：

- `files`：126 项工程源码、接口、构建、配置与模型，约 35.75 MB。三个 `best.onnx` 字节相同，SHA 为 `7264c02ef1d06815aaa367f65c41e4285ad6bf51b9803f1461e34e9e72b3aa71`；保留路径关系，主代理可在存储层去重，不能让配置失去模型。`test1/algorithm/background/color.jpg` 是配置引用资产，已列入。
- `required_source_dependencies`：211 项第三方头文件/源码依赖，约 3.40 MB；保留内嵌版权声明。
- `optional_vendor_binaries`：按原 ZIP member、SHA、架构、用途精确列出；全部默认不进入主包。ARM 构建链接集 26 项含别名与 CMake 元数据，原字节约 108.85 MB、按 SHA 去重约 38.80 MB；不是 618 MB 的全部架构目录都必须交付。

与历史 ARM 构建对应的最小库族为 Orbbec、TY camera、ONNX Runtime、spdlog。Orbbec ARM 文件名为 2.7.6，但 wrapper CMake 声明 2.4.11，实际版本/头文件/ABI 兼容性未确认。ONNX Runtime 文件名为 1.23.1，TY camera 为 4.1.1；这些是收到文件身份，不是本次动态链接验证。`.so`、SONAME 和版本全名存在同 SHA 实体副本；精简时仍需提供消费者要求的名称，不能只删别名。

ARM 相机 extensions 在可选表单列：是否需要 depthengine/filter/frameprocessor 取决于实际设备，未凭文件存在认定全部必要；固件更新插件不属于普通抓放入口。x64 CUDA/TensorRT provider 等仅保留原包取用索引：当前 YOLO 源码未显式选择 GPU provider，不能据目录里有 CUDA 库宣称 GPU 推理已配置。其他架构也不进入首版主包。

## 4. 视觉尚未闭合的环境依赖

系统构建至少涉及 ROS 2 Humble/ament/colcon、C++ 编译器、OpenCV、PCL、Eigen3、fmt、spdlog、libzmq、cppzmq 头文件和 ROS 消息/动作支持；package.xml 与 CMake 并非完整 apt 锁定清单。`camera_ros`/`camera_drive` 声明的 `vision_lib` 未在收到源码中识别，实际 CMake 没直接 find 此包，需要确认是历史元数据残留还是外部依赖。

检测算法初始化会建立 SAM3 ZeroMQ client，配置示例为 `127.0.0.1:5555`。新包识别到 client 源码，未识别到相匹配的 SAM3 服务端、模型与启动环境；因此不能称完整视觉检测闭环可离线重建。仅有 YOLO ONNX 模型不补齐该服务。旧 build/install 内 py_srvcli 链接也不等于已收到其源码；当前主 launch 不调用它，不将其混入必要接口源码。

`app.launch.py` 只启动 camera_ros 和 vision_detect_ros；手眼是另一个独立可执行节点 `vision_hand_eye`，其入口参数是配置目录。按原 README 的 `--packages-up-to vision_bringup` 不能据此声称手眼节点也已构建。实际加载的 config、相机身份、二进制 SHA 均未与现场运行绑定；源码 test1/multiple 和历史标定矩阵不同，不能自动挑一个作为有效标定。

已有 `build_vision.sh` 使用 GNU/Linux 命令，并根据 CMake 传入 WORKSPACE_DIR 查找 install；路径迁移后必须核对其实际展开位置。收到脚本未在本轮执行。`script/install_deps.sh` 会联网导入浮动版本，只保留为来源资料，不作为冻结源码重建的默认第一步。

## 5. 数据和许可边界

手眼原件 `handeye_calibration_all_20260826.tar.gz` 的 SHA 为 `aa83ad9a312fef8b9b1ad07a1842d6fb4eb7ebe1d91aa6ca4d9533421b7d9bf9`。JSON 的 `nested_members` 列出全部 199 个成员及 SHA，约 120.89 MB：39 组样本分 11/10/9/9 四个编号组，各有彩色、深度、内参、手位姿、结果图；另外保留历史结果和配置。这些是排查资料，不是已批准标定；未重做角点检测/拟合。

收到工程没有独立 LICENSE/COPYING/NOTICE 文件；包声明和第三方头文件版权已保留。vision_interface、vision_bringup、vision_third_party 的 package.xml 仍是 `TODO: License declaration`，模型授权材料也未识别。不能补写未经来源证实的许可证；对外再分发前需要补齐实际授权与第三方声明，内部整理仍保留原件及身份。

接手验收依次记录：包 SHA 与成员完整性 → 指定目标系统/架构 → 重建隔离环境 → 离线源码/路径/接口测试 → 必要的 ROS/C++ 构建与模型加载检查 → 人工确认设备、配置与标定后再进行现场测试。本轮只完成前置材料和静态身份核对。
