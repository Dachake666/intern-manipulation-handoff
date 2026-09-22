# vision_hand_eye 手眼标定

## 项目简介

本项目实现了手眼标定功能，支持两种标定模式：
- **眼在手外 (Eye-to-Hand)**: 相机固定在环境中，机器人末端执行器移动
- **眼在手上 (Eye-in-Hand)**: 相机固定在机器人末端执行器上，随机器人移动

## 编译

```bash
colcon build --packages-up-to vision_hand_eye
```

## 配置文件

在启动节点前，需要创建配置文件。配置文件路径通过命令行参数传递。

配置文件示例 `config/hand_in_eye/config.json`:

```json
{
    "mark_type": "MY_TEST",
    "save_path": "save_data",
    "hand_eye_type": 1
}
```

**参数说明：**
- `mark_type`: 标记类型，可选值: MY_TEST, TAG16h5, TAG36h11, ARUCO_MIP_16h3
- `save_path`: 数据保存路径
- `hand_eye_type`: 标定模式
  - `1`: 眼在手外 (Eye-to-Hand)
  - `2`: 眼在手上 (Eye-in-Hand)

## 启动节点

```bash

# 启动手眼标定节点，传入配置文件夹路径
ros2 run vision_hand_eye vision_hand_eye /home/dev/workspace/code_test/vision_test/vision/config/hand_in_eye
```

## ROS服务接口

**服务名称:** `/hand_eye_calibration_server`

**接口定义:** `vision_interface/srv/HandEyeCalibration`

### Request

| 参数 | 类型 | 说明 |
|------|------|------|
| request_type | int32 | 请求类型: 1=采集数据, 2=执行标定, 3=删除上次采集, -100=本地测试 |
| camera_id | string | 相机ID |
| hand_pose | geometry_msgs/Pose | 机器人位姿 (位置+四元数) |
| json_file_name | string | 保存标定结果的文件名 |
| workspace_id | string | 工作区ID (用于数据保存路径) |

### Response

| 参数 | 类型 | 说明 |
|------|------|------|
| success | bool | 是否成功 |
| mean_rot_error | float32 | 平均旋转误差 |
| mean_trans_error | float32 | 平均平移误差 |
| result_matrix | float32[] | 标定结果矩阵 (12个元素: 3x4变换矩阵) |
| error_data | int32 | 错误码 |

## ROS命令示例

### 1. 采集数据 (request_type=1)

每次移动机器人到不同位置，采集一次数据。建议采集10-20组数据。

```bash
ros2 service call /hand_eye_calibration_server vision_interface/srv/HandEyeCalibration \
  "{request_type: 1, camera_id: 'camera_1', hand_pose: {position: {x: 0.0, y: 100.0, z: 100.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, json_file_name: 'calibration_result.json', workspace_id: 'box1'}"
```

**参数说明：**
- `hand_pose.position.{x,y,z}`: 机器人末端位置 (mm)
- `hand_pose.orientation.{x,y,z,w}`: 机器人末端姿态 (四元数)

### 2. 执行标定 (request_type=2)

采集足够数据后，执行标定计算。

```bash
ros2 service call /hand_eye_calibration_server vision_interface/srv/HandEyeCalibration \
  "{request_type: 2, camera_id: '', hand_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, json_file_name: 'calibration_result.json', workspace_id: 'box1'}"
```

**返回结果：**
- `success`: 标定是否成功
- `result_matrix`: 4x4变换矩阵的12个元素 (按行优先顺序)
- `mean_rot_error`: 平均旋转误差
- `mean_trans_error`: 平均平移误差 (mm)

### 3. 删除上次采集 (request_type=3)

删除最近一次采集的数据。

```bash
ros2 service call /hand_eye_calibration_server vision_interface/srv/HandEyeCalibration \
  "{request_type: 3, camera_id: '', hand_pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, json_file_name: '', workspace_id: ''}"
```

## 标定方法说明

根据配置文件中的`hand_eye_type`选择不同的标定算法：

### 眼在手外 (Eye-to-Hand, hand_eye_type=1)
- 相机固定，机器人移动
- 使用 OpenCV 的 `CALIB_HAND_EYE_PARK` 方法
- 计算相机坐标系到机器人基座坐标系的变换矩阵

### 眼在手上 (Eye-in-Hand, hand_eye_type=2)
- 相机固定在机器人末端，随机器人移动
- 使用 OpenCV 的 `CALIB_HAND_EYE_TSAI` 方法
- 计算相机坐标系到机器人末端坐标系的变换矩阵

## 注意事项

1. 确保配置文件路径正确
2. 标定前确保相机标定板在相机视野内
3. 采集数据时，机器人应移动到不同位置，覆盖足够大的工作空间
4. 建议采集10-20组数据以获得较好的标定精度
