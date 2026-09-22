#!/usr/bin/env python3
from __future__ import annotations

import math
from collections import defaultdict

import numpy as np


def _yaw_std_deg(values: list[float]) -> float:
    doubled = np.asarray(values) * 2.0  # 盒体长轴 180° 等价
    mean = math.atan2(float(np.sin(doubled).mean()), float(np.cos(doubled).mean()))
    delta = np.angle(np.exp(1j * (doubled - mean))) / 2.0
    return float(np.degrees(np.std(delta)))


class TemporalGate:
    def __init__(self, window_size=5, minimum_support=3, center_std_m=0.005,
                 yaw_std_deg=3.0, size_error_ratio=0.15,
                 depth_valid_ratio=0.80, confidence=0.80):
        self.window_size = int(window_size)
        self.minimum_support = int(minimum_support)
        self.center_std_m = float(center_std_m)
        self.yaw_limit_deg = float(yaw_std_deg)
        self.size_limit = float(size_error_ratio)
        self.depth_limit = float(depth_valid_ratio)
        self.confidence_limit = float(confidence)
        self.frames: list[list[dict]] = []

    def add(self, detections: list[dict]) -> list[dict]:
        self.frames.append(detections)
        self.frames = self.frames[-self.window_size:]
        groups = defaultdict(list)
        # 首版每类目标按图像实例序号关联；多目标粘连/次序变化会降低支持帧并被拒。
        for frame in self.frames:
            for d in frame:
                groups[(d["class_id"], d.get("instance_hint", 1))].append(d)
        accepted = []
        for (class_id, instance), rows in sorted(groups.items()):
            if len(rows) < self.minimum_support:
                continue
            pos = np.asarray([r["position_m"] for r in rows], float)
            center_std = np.std(pos, axis=0)
            yaw_std = _yaw_std_deg([float(r["yaw_rad"]) for r in rows])
            if (float(np.max(center_std)) > self.center_std_m or yaw_std > self.yaw_limit_deg or
                    max(float(r["size_error_ratio"]) for r in rows) > self.size_limit or
                    min(float(r["depth_valid_ratio"]) for r in rows) < self.depth_limit or
                    min(float(r["confidence"]) for r in rows) < self.confidence_limit):
                continue
            yaw = math.atan2(float(np.sin([r["yaw_rad"] * 2 for r in rows]).mean()),
                             float(np.cos([r["yaw_rad"] * 2 for r in rows]).mean())) / 2.0
            accepted.append({
                "object_id": f"{class_id}-{instance}", "class_id": class_id,
                "frame_id": rows[-1]["frame_id"],
                "pose": {"position_m": np.mean(pos, axis=0).tolist(),
                         "quaternion_xyzw": [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]},
                "dimensions_m": rows[-1]["dimensions_m"],
                "confidence": float(min(r["confidence"] for r in rows)),
                "uncertainty_1sigma_m": center_std.tolist(),
                "depth_valid_ratio": float(min(r["depth_valid_ratio"] for r in rows)),
                "support_frames": len(rows), "yaw_std_deg": yaw_std,
            })
        return accepted
