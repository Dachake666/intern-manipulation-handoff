# SDK UVW 与四元数约定

输入 `armGetWorlds()` 为 `[X,Y,Z,U,V,W]`。项目按 XYZ 毫米、UVW 度解释；ROS 四元数顺序为 `x,y,z,w`。公式与置信度依据是 `analysis/calib_common.py`，不得仅根据“RPY”缩写推断其他程序的坐标约定。

## 当前候选公式

```text
R = Rz(W) · Ry(V) · Rx(U)
```

即 U→roll、V→pitch、W→yaw。换算前把角度变为弧度，定义半角正弦/余弦 `sr/cr`、`sp/cp`、`sy/cy`：

```text
qx = sr·cp·cy - cr·sp·sy
qy = cr·sp·cy + sr·cp·sy
qz = cr·cp·sy - sr·sp·cy
qw = cr·cp·cy + sr·sp·sy
```

检查四元数归一化；`q` 与 `-q` 表示相同旋转。矩阵记号 `T_A_B` 表示把 B 系坐标变到 A 系。姿态换算不等于 TCP 补偿：先标明 OBJECT_POSE / GRASP_POSE / EE_POSE，已补偿 SDK endpoint 不再重复补 TCP。

## 证据边界

`sdk/“隙锋”人形机器人SDK使用指南（Python）V260526.1.pdf` 解释了 XYZUVW 接口，但没有明确 U/V/W 的旋转轴、顺序和组合公式；手册与 `pypilot-1.0.0422.1` 版本也不同。以上公式是项目历史实测候选，不应表述为厂商文档已证明的规则。

复核必须同步保存到位后的七个真实关节角、真实 XYZUVW、样本时间和设备/工具身份；目标指令不能代替反馈。相对旋转比较应覆盖多种腕姿态，并保持腰、底盘、相机和 TCP 不变。首尾回到同一参考姿态，检查重复性。

旧 `collect_wrist_orientation_samples.py` / `verify_uvw_command_side.py` 可从 PRE_PRUNE 恢复，但其完整分析闭环不在默认入口中。单轴命令数值与反馈一致不能独立证明约定正确，控制器可能在两端使用同一内部表示。新增采样流程必须带同步、单位、轴方向和残差门，结论由负责人确认后登记。
