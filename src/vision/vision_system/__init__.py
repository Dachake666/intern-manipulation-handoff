"""可回放的 RGB-D 颜色/形状视觉模块；ROS 2 只是输入适配层。"""

from .detector import CameraModel, ColorBoxDetector, load_catalog
from .temporal import TemporalGate

__all__ = ["CameraModel", "ColorBoxDetector", "TemporalGate", "load_catalog"]
