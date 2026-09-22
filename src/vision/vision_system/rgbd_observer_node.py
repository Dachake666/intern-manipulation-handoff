#!/usr/bin/env python3
"""ROS 2 Humble RGB/registered-depth/CameraInfo 同步采集节点。"""
from __future__ import annotations

import datetime as dt
import json
from collections import deque
from pathlib import Path

try:
    import message_filters
    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image
except ImportError:  # 离线测试机允许导入模块，但 main 会给出明确错误
    message_filters = rclpy = CvBridge = Node = CameraInfo = Image = None

from .detector import CameraModel, ColorBoxDetector, load_catalog
from .evidence import save_frame_bundle
from .temporal import TemporalGate


if Node is not None:
    class RgbdObserverNode(Node):
        def __init__(self):
            super().__init__("xifeng_rgbd_observer")
            for name, default in (("rgb_topic", "/camera/color/image_raw"),
                                  ("depth_topic", "/camera/aligned_depth_to_color/image_raw"),
                                  ("camera_info_topic", "/camera/color/camera_info"),
                                  ("camera_id", "chest_rgbd"),
                                  ("intrinsics_id", "UNCONFIGURED"),
                                  ("extrinsics_id", "UNCONFIGURED"),
                                  ("catalog", ""), ("evidence_root", "vision_evidence")):
                self.declare_parameter(name, default)
            self.bridge = CvBridge()
            self.detector = ColorBoxDetector(load_catalog(
                self.get_parameter("catalog").get_parameter_value().string_value))
            self.gate = TemporalGate()
            self.frame_evidence = deque(maxlen=5)
            rgb = message_filters.Subscriber(self, Image, self.get_parameter("rgb_topic").value)
            depth = message_filters.Subscriber(self, Image, self.get_parameter("depth_topic").value)
            info = message_filters.Subscriber(self, CameraInfo, self.get_parameter("camera_info_topic").value)
            self.sync = message_filters.ApproximateTimeSynchronizer(
                [rgb, depth, info], queue_size=10, slop=0.05, allow_headerless=False)
            self.sync.registerCallback(self._callback)

        def _callback(self, rgb_msg, depth_msg, info_msg):
            rgb_ns = rgb_msg.header.stamp.sec * 10**9 + rgb_msg.header.stamp.nanosec
            depth_ns = depth_msg.header.stamp.sec * 10**9 + depth_msg.header.stamp.nanosec
            delta_ms = abs(rgb_ns - depth_ns) / 1e6
            if delta_ms > 50.0:
                self.get_logger().warning(f"拒绝不同步 RGB-D: {delta_ms:.1f}ms")
                return
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, "rgb8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            depth_m = raw_depth.astype("float32") * (0.001 if depth_msg.encoding == "16UC1" else 1.0)
            info = {"width": info_msg.width, "height": info_msg.height,
                    "k": list(info_msg.k), "frame_id": info_msg.header.frame_id}
            camera = CameraModel.from_camera_info(info)
            detections, masks = self.detector.detect_frame(rgb, depth_m, camera)
            accepted = self.gate.add(detections)
            obstacles = self.detector.extract_obstacles(depth_m, camera, masks)
            stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            out = Path(self.get_parameter("evidence_root").value) / stamp
            meta = {"ros_timestamp_ns": rgb_ns, "rgb_depth_delta_ms": delta_ms,
                    "detections": detections}
            evidence = save_frame_bundle(out, rgb, depth_m, info, meta)
            self.frame_evidence.append(evidence)
            if accepted and len(self.frame_evidence) >= 3:
                observation = {
                    "schema_version": "vision_observation.v1",
                    "observation_id": f"live-{stamp}",
                    "camera": {"camera_id": self.get_parameter("camera_id").value,
                               "ros_timestamp_ns": rgb_ns,
                               "frame_id": camera.frame_id,
                               "rgb_topic": self.get_parameter("rgb_topic").value,
                               "depth_topic": self.get_parameter("depth_topic").value,
                               "camera_info_topic": self.get_parameter("camera_info_topic").value,
                               "rgb_depth_delta_ms": delta_ms},
                    "calibration": {
                        "intrinsics_id": self.get_parameter("intrinsics_id").value,
                        "extrinsics_id": self.get_parameter("extrinsics_id").value,
                        "extrinsics_direction": "T_sdk_world_camera",
                        "length_unit": "meter"},
                    "evidence": {**evidence,
                                 "supporting_frames": list(self.frame_evidence)},
                    "objects": accepted,
                    "obstacles": [{"obstacle_id": item["obstacle_id"],
                        "frame_id": item["frame_id"], "geometry": item["geometry"],
                        "pose": {"position_m": item["position_m"],
                                 "quaternion_xyzw": [0, 0, 0, 1]},
                        "dimensions_m": item["dimensions_m"],
                        "confidence": item["confidence"],
                        "uncertainty_1sigma_m": item["uncertainty_1sigma_m"]}
                        for item in obstacles]}
                (out / "observation.json").write_text(
                    json.dumps(observation, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8")
            self.get_logger().info(f"保存 {out}; stable_objects={len(accepted)}")


def main(args=None):
    if rclpy is None:
        raise SystemExit("缺 ROS 2 rclpy/message_filters/cv_bridge；请在 Humble 环境运行")
    rclpy.init(args=args)
    node = RgbdObserverNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
