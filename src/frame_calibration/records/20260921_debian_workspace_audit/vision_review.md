# 视觉、手眼与 ROS 内容审阅

**结论：源码、接口和原始样本比此前完整；手眼有效性仍未通过，不能直接把收到外参投入抓放。** 本轮读取源码、文件身份和样本结构，没有运行C++程序、重新识别标记点或拟合外参。

## 工程身份和职责

- `vision/ultirobotics_vision_detect` 是C++ ROS视觉工程，包含相机、检测、bringup及 `vision_interface`；收到的HEAD为 `c021ed5fcc220072d10a61c4424bdb3ff1e17889`，分支polygon。
- `vision_hand_eye`、`vision/vision_hand_eye`、主工程deps中的 `vision_hand_eye` 三个目录的19个功能文件逐字一致，HEAD均为 `47729a52eb9e80e936673bd3b96e6ee1a5274260`。这不能证明现场加载了哪份构建产物。
- 8个子仓库HEAD及文件SHA见 `vision_review.json`。原Git对象留在大包；尚未从档案重建并验证各工作树相对HEAD的全部差异，不宣称工作树干净。
- Mac `vision/` 是 `xifeng_vision` Python观测契约，`robot_mission`负责目标适配/规划/预检。它与C++ service/action不是同一代码库，也不是直接替换关系。
- `code/vision_log_tool-main`用于可视化日志；`my_package`当前只是hello-world，`learning_tf2_py`没有注册console入口，均不是已经完成的抓放功能。

## 已从实际代码确认的接口

| 项目 | 实际行为 |
|---|---|
| 请求字段 | `request_type, camera_id, hand_pose, save_json_file, workspace_id` |
| request_type=1 | 读取当前相机帧、检测，再与请求中提供的手位姿配对；图像保存使用异步线程 |
| request_type=2 | 对当前内存样本求解；正常到函数末尾及样本不足分支清空内存，部分失败提前返回不清空 |
| request_type=3 | 只pop最后一个内存位姿/目标配对，不删除磁盘图像，不重置完整会话 |
| workspace_id | setter仅赋字符串；换ID不自动隔离或清空已有内存样本 |
| 手位姿 | XYZ数值直接进入计算，无隐式米/毫米转换；xyzw进入Eigen wxyz构造并归一化 |
| 结果矩阵 | 4×4、16个元素按行展开；README的12元素说法错误 |

实际 `.srv` 字段是 `save_json_file`，旧README命令使用 `json_file_name`，需要修正文档。不能复制旧命令后根据其描述猜接口。

源码锚点均在收到的主视觉deps中：`calibration_server.cpp:58–99`；`camera_calibration.cpp:850–856, 1147–1160, 1167, 1198, 1225, 1236–1240, 1326–1327`。冻结内容已收进 `received_sources.zip`，没有修改。

## 明确需要修复的质量门缺陷

`camera_calibration.cpp:1139–1141` 对旋转均差>10或平移均差>100设置 `error_data_=-105`，却未退出；随后仍把矩阵写入配置（1179/1200），1222再把错误清成0。服务按该错误值返回success。因此**高残差结果可能被写入配置并报告success=true**。本轮只记录，未替换现场程序；后续应在开发版本修复并验证失败分支。

配置加载失败（1167）或输出打开失败（1198）会提前返回，绕过最后的内存清空。会话重试和workspace_id切换需要显式设计，不能把request3当作清空全部样本的接口。

## 39组原始数据究竟证明什么

完整嵌套包 `handeye_calibration_all_20260826.tar.gz` SHA为 `aa83ad9a312fef8b9b1ad07a1842d6fb4eb7ebe1d91aa6ca4d9533421b7d9bf9`，199个文件。

- 有39组彩色、深度、内参、手位姿及标注图；四个编号组0/1/2/3分别11/10/9/9组，配套文件无缺项。
- 四元数范数约0.99999935–1.00000054；这仅证明数值结构，不能证明旋转约定、同步性或TCP定义正确。
- `calibration_result.json`与8/26交接中的旧高残差响应是同一矩阵（双精度与float32差异）。旧记录旋转均差7.14688、平移均差134.44865；本轮没有重新拟合这些指标。
- 四个编号组不自动等于四次已知机构状态的独立会话，更不能未经核实混用。需要逐组确认当时的姿态约定、相机/腰/底盘状态、目标和TCP。
- 源码目录 `config/test1/workstation.json` 使用另一外参，`multiple`又有其他外参；嵌套包也有配置快照。实际launch引用哪份配置尚未确认，不能静默用calibration_result替换。
- `vision/ultirobotics_vision_detect/save_data/calibration_server.log`只有8/11初始化的6行，配套`camera1.log`含相机连接及一次取图错误。另一个目录`code/vision_log_tool-main/save_data/calibration_server.log`及其backup各有822行，覆盖8/11–8/20，包含两条历史高残差标定记录；都不能证明当前标定或视觉抓放验收通过。

下一步先修质量门和会话语义，再核实实际部署配置及数据分组，最后做有留出验证的重算和Mac观测契约适配。当前 `CONFIRMED_*`、发布包和机器人参数均未改动。
