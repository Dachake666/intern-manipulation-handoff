# Reproducible Python environments

规划、PB↔SDK 校验和 RGB-D 回放固定在 CPython 3.10。先创建独立虚拟环境，再按
对应 lock 文件安装。ROS 2 Humble 的 `rclpy`、`sensor_msgs`、`message_filters`、
`cv_bridge` 必须来自同一 Humble 安装，不通过 pip 混装。

厂商 `pypilot` 只存在于 Debian 真机容器，不属于离线依赖；no-motion 工具不得导入
它。当前 Mac 默认 Python 缺 NumPy/PyBullet 时，应显式使用配置好的规划解释器，
禁止靠“碰巧是哪个 python3”运行。
