# SDK UVW 转 ROS 四元数

本文说明如何把机器人 SDK `armGetWorlds()` 返回的：

```text
[X, Y, Z, U, V, W]
```

转换成 `geometry_msgs/Pose` 使用的位置和四元数。

## 2026-08-19 SDK 文档复核结论

复核文件：`workspace/“隙锋”人形机器人SDK使用指南（Python）V260526.1.pdf`。

- 第 17 页示例把 `current_worlds[2] += 20` 说明为 Z 正方向移动 20 mm，支持本项目把机械臂 XYZ 按毫米处理。
- 第 83-84 页只说明 `armMoveWorlds()` 的输入为世界坐标 `[X,Y,Z,U,V,W]`。
- 第 89 页说明 `armGetWorlds(robotId, fb=True)` 读取机械臂实时反馈值。
- 文档没有说明 U/V/W 分别绕哪根轴、角度单位以及欧拉角组合顺序，也没有出现 RPY、ZYX 或对应旋转矩阵公式。
- 该指南修订记录写明配套 SDK `2.0.0.0526`，当前工作区轮子为 `pypilot-1.0.0422.1`；文档与实际运行库存在版本差异，不能用新版指南未明确的行为反推旧轮子。

因此，厂商文档本身**不能证明** UVW 是 ZYX。下面的公式是项目历史实测得到的当前候选；在本次复测通过前，应标记为 `[待复核]`，不要把公式名称当作厂商原文。

## 输入与输出约定

- `X/Y/Z`：SDK 世界坐标，当前项目按毫米保存。
- `U/V/W`：姿态角，单位为度。
- ROS 四元数字段顺序：`x, y, z, w`。
- 标定采样应使用到位后的 `actual_world`，不要用 `target_world` 替代实际反馈。

本项目 `analysis/calib_common.py` 中记录的当前候选为：

```text
R = Rz(W) · Ry(V) · Rx(U)
```

也就是把 `U` 当作绕 X 轴的 roll、`V` 当作绕 Y 轴的 pitch、`W` 当作绕 Z 轴的 yaw。输入按度解释，三角函数计算前先转成弧度。

这里应以完整公式为准：同一个公式可能被称为“ZYX 组合”“标准 RPY”或“外旋 XYZ”，只写一个三字母名称容易产生歧义。

## UVW 约定复测

### 复测必须记录什么

每个姿态必须在机械臂到位并稳定后，从同一次 SDK 会话记录：

1. `armGetJoints(1, True)` 返回的 7 个实际关节角；
2. `armGetWorlds(1, True)` 返回的实际 `[X,Y,Z,U,V,W]`；
3. 样本名称和时间。

只有示教器上的 XYZUVW、没有对应的 7 个实际关节角，无法可靠判断欧拉角顺序。相机图像和手眼标定服务不参与这次测试。

采样期间要求：

- 使用左臂 `ARM_ID=1`；
- 腰部、底盘、相机安装和 TCP 配置全程不变；
- 末端姿态要有明显变化，不能只改变 XYZ；
- 第一条和最后一条回到同一个基准关节姿态，用于检查重复性和漂移。

### Debian 容器内的推荐步骤

把下面三个文件放在同一目录：

```text
collect_wrist_orientation_samples.py
sdk_session.py
robot_lock.py
```

先检查并修改 `collect_wrist_orientation_samples.py` 顶部配置：

```python
ROBOT_IP = "当前机器人 IP"
LOCAL_IP = "当前 Debian 机 IP"
ARM_IP = "当前机器人 IP"
ARM_ID = 1
ENABLE_REAL_MOTION = False
```

运行前确认没有旧进程占用机械臂：

```bash
pgrep -af 'collect_wrist_orientation_samples.py|execute_.*\.py|diag_enable.py'
lslocks | grep xifeng_robot_locks
```

先执行干跑：

```bash
python3 collect_wrist_orientation_samples.py
```

干跑必须满足：打印 12 个目标、没有任何限位错误、没有发送运动。人工确认环境和姿态安全后，再把 `ENABLE_REAL_MOTION=True`，低速逐步回车执行；`q` 可随时中止并保存已完成样本。完成后立刻把开关恢复为 `False`。

脚本采用基准姿态加腕部 `j5/j6/j7` 正负变化，共 12 个姿态；每个目标都是相对最初基准计算，不会累积偏移。

### 需要交回的文件

输出位置相对于运行命令时的当前目录：

```text
data/wrist_orientation_samples_latest.json
data/wrist_orientation_samples_YYYYMMDD_HHMMSS.json
```

请把时间戳版 JSON 和完整终端日志一起拿回来。不要只抄 UVW 数字，也不要用目标关节角代替反馈关节角。

### 数据如何判定

拿到 JSON 后将对轴顺序、U/V/W 映射和正负号候选逐一计算，并用 URDF 正运动学比较多组姿态的相对旋转：

```text
SDK 相对旋转  vs.  关节角正解得到的相对旋转
```

相对旋转比较会消去末端固定工具旋转，不需要先假定 `R_sdk = R_link11·Rz(90°)`。只有当同一个候选在全部样本上残差稳定、明显优于其他候选，而且首尾基准重复一致时，才把它重新标记为 `[已核实]`。

`verify_uvw_command_side.py` 的单轴小角度运动可作为第二重人工验证，但它不是主判据：控制器下发和反馈可能使用同一套内部表示，仅看数值变化容易形成循环证明。

## 计算公式

先转换单位并计算半角：

```text
roll  = U × π / 180
pitch = V × π / 180
yaw   = W × π / 180

cr = cos(roll/2)   sr = sin(roll/2)
cp = cos(pitch/2)  sp = sin(pitch/2)
cy = cos(yaw/2)    sy = sin(yaw/2)
```

对应 `Rz(yaw) · Ry(pitch) · Rx(roll)` 的四元数为：

```text
qx = sr·cp·cy - cr·sp·sy
qy = cr·sp·cy + sr·cp·sy
qz = cr·cp·sy - sr·sp·cy
qw = cr·cp·cy + sr·sp·sy
```

计算后应检查归一化：

```text
sqrt(qx² + qy² + qz² + qw²) ≈ 1
```

`q` 和 `-q` 表示同一个旋转；本文统一保留公式直接算出的结果。

## Python 参考实现

```python
import math


def sdk_world_to_ros_pose(x_mm, y_mm, z_mm, u_deg, v_deg, w_deg):
    roll, pitch, yaw = map(
        math.radians, (u_deg, v_deg, w_deg)
    )

    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy

    return {
        "position_mm": {"x": x_mm, "y": y_mm, "z": z_mm},
        "orientation": {"x": qx, "y": qy, "z": qz, "w": qw},
    }
```

## 本次示例

输入：

```text
X = 554.874 mm
Y = 111.100 mm
Z = 611.751 mm
U = 12.030°
V = -44.143°
W = -104.487°
```

输出：

```text
position = [554.874, 111.100, 611.751] mm
quaternion [x,y,z,w] =
[-0.235989659, -0.305592477, -0.704536190, 0.595441749]
```

四元数模长为 `1.000000000000`。

对应当前约定的 ROS 2 命令（`request_type=1`，位置单位毫米）：

```bash
ros2 service call /hand_eye_calibration_server vision_interface/srv/HandEyeCalibration \
"{request_type: 1, camera_id: 'camera1', hand_pose: {position: {x: 554.874, y: 111.100, z: 611.751}, orientation: {x: -0.235989659, y: -0.305592477, z: -0.704536190, w: 0.595441749}}, save_json_file: 'calibration_result.json', workspace_id: 'box1'}"
```

## 使用边界

这个计算只完成“SDK UVW 数值 → 四元数”，不会自动完成以下工作：

- 判断服务需要 `base→hand` 还是 `hand→base`；如果需要逆变换，位置和姿态都必须一起求逆，不能只对四元数取反。
- 改变或补偿 TCP、法兰、夹爪中心之间的固定变换。
- 验证 PB↔SDK 的会话级平移 `T` 是否仍有效。
- 把历史机器人位姿与当前相机图像重新同步。

做手眼标定时，机器人到位并稳定后，应在标定板可见的同一时刻读取 `actual_world` 并调用服务。只保存历史机器人位姿、没有对应的同步相机观测，不能形成有效标定样本。

当前工作区的 PB↔SDK 标定守门状态为 `BLOCKED`：最新 `worlds_record` 为 2026-07-13，且 `analysis/verify_worlds_record.py` 缺失。这个状态不影响直接转换 SDK 原始 UVW，但在把视觉结果接入 PB↔SDK 规划链之前必须重新采样并恢复验证器。
