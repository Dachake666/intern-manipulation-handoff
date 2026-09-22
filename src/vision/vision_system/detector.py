#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class CameraModel:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    frame_id: str

    @classmethod
    def from_camera_info(cls, info: dict) -> "CameraModel":
        k = info["k"]
        return cls(int(info["width"]), int(info["height"]), float(k[0]),
                   float(k[4]), float(k[2]), float(k[5]), str(info["frame_id"]))


def load_catalog(path: str | Path) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if data.get("schema_version") != "object_catalog.v1":
        raise ValueError("物体目录 schema_version 必须是 object_catalog.v1")
    for item in data.get("objects", []):
        if len(item.get("dimensions_m", [])) != 3 or not item.get("hsv_ranges"):
            raise ValueError(f"物体目录项不完整: {item.get('class_id')}")
    return data


def rgb_to_hsv(rgb: np.ndarray) -> np.ndarray:
    """RGB uint8 -> HSV，H 为 0..360 度，S/V 为 0..1。"""
    x = np.asarray(rgb, dtype=np.float32) / 255.0
    mx, mn = x.max(axis=2), x.min(axis=2)
    d = mx - mn
    h = np.zeros_like(mx)
    nz = d > 1e-8
    r, g, b = x[..., 0], x[..., 1], x[..., 2]
    m = nz & (mx == r)
    h[m] = (60.0 * ((g[m] - b[m]) / d[m])) % 360.0
    m = nz & (mx == g)
    h[m] = 60.0 * ((b[m] - r[m]) / d[m] + 2.0)
    m = nz & (mx == b)
    h[m] = 60.0 * ((r[m] - g[m]) / d[m] + 4.0)
    s = np.divide(d, mx, out=np.zeros_like(d), where=mx > 1e-8)
    return np.stack((h, s, mx), axis=2)


def _window_sum(mask: np.ndarray, radius: int) -> np.ndarray:
    if radius <= 0:
        return mask.astype(np.int32)
    a = np.pad(mask.astype(np.int32), radius)
    integral = np.pad(a, ((1, 0), (1, 0))).cumsum(0).cumsum(1)
    size = 2 * radius + 1
    return (integral[size:, size:] - integral[:-size, size:] -
            integral[size:, :-size] + integral[:-size, :-size])


def morphology(mask: np.ndarray, radius: int = 1) -> np.ndarray:
    """一次开运算再闭运算，不依赖 OpenCV/SciPy。"""
    if radius <= 0:
        return mask.astype(bool)
    area = (2 * radius + 1) ** 2
    eroded = _window_sum(mask, radius) == area
    opened = _window_sum(eroded, radius) > 0
    dilated = _window_sum(opened, radius) > 0
    return _window_sum(dilated, radius) == area


def components(mask: np.ndarray, min_pixels: int) -> list[np.ndarray]:
    """8 邻域连通域，返回每个域的 [N,2] (v,u)。"""
    mask = np.asarray(mask, bool)
    seen = np.zeros(mask.shape, bool)
    out = []
    h, w = mask.shape
    for v, u in np.argwhere(mask):
        if seen[v, u]:
            continue
        stack, pts = [(int(v), int(u))], []
        seen[v, u] = True
        while stack:
            y, x = stack.pop()
            pts.append((y, x))
            for yy in range(max(0, y - 1), min(h, y + 2)):
                for xx in range(max(0, x - 1), min(w, x + 2)):
                    if mask[yy, xx] and not seen[yy, xx]:
                        seen[yy, xx] = True
                        stack.append((yy, xx))
        if len(pts) >= min_pixels:
            out.append(np.asarray(pts, dtype=np.int32))
    return out


def backproject(camera: CameraModel, uv: np.ndarray, depth_m: np.ndarray) -> np.ndarray:
    u, v, z = uv[:, 0], uv[:, 1], depth_m
    return np.column_stack(((u - camera.cx) * z / camera.fx,
                            (v - camera.cy) * z / camera.fy, z))


class ColorBoxDetector:
    def __init__(self, catalog: dict, min_pixels: int = 80,
                 depth_min_m: float = 0.15, depth_max_m: float = 3.0):
        self.catalog = catalog
        self.min_pixels = int(min_pixels)
        self.depth_min_m = float(depth_min_m)
        self.depth_max_m = float(depth_max_m)

    def _color_mask(self, hsv: np.ndarray, ranges: list[dict]) -> np.ndarray:
        mask = np.zeros(hsv.shape[:2], bool)
        for r in ranges:
            lo, hi = r["lower"], r["upper"]
            if lo[0] <= hi[0]:
                hm = (hsv[..., 0] >= lo[0]) & (hsv[..., 0] <= hi[0])
            else:
                hm = (hsv[..., 0] >= lo[0]) | (hsv[..., 0] <= hi[0])
            mask |= hm & (hsv[..., 1] >= lo[1]) & (hsv[..., 1] <= hi[1]) & \
                (hsv[..., 2] >= lo[2]) & (hsv[..., 2] <= hi[2])
        return morphology(mask, int(self.catalog.get("morphology_radius_px", 1)))

    def detect_frame(self, rgb: np.ndarray, depth_m: np.ndarray,
                     camera: CameraModel) -> tuple[list[dict], list[np.ndarray]]:
        rgb, depth_m = np.asarray(rgb), np.asarray(depth_m, dtype=float)
        if rgb.shape != (camera.height, camera.width, 3):
            raise ValueError(f"RGB shape {rgb.shape} 与 CameraInfo 不一致")
        if depth_m.shape != (camera.height, camera.width):
            raise ValueError(f"depth shape {depth_m.shape} 与 CameraInfo 不一致")
        hsv = rgb_to_hsv(rgb)
        detections, used_masks = [], []
        for spec in self.catalog["objects"]:
            mask = self._color_mask(hsv, spec["hsv_ranges"])
            for index, pts_vu in enumerate(components(mask, self.min_pixels), 1):
                v, u = pts_vu[:, 0], pts_vu[:, 1]
                z = depth_m[v, u]
                valid = np.isfinite(z) & (z >= self.depth_min_m) & (z <= self.depth_max_m)
                ratio = float(valid.mean())
                if not valid.any():
                    continue
                uv = np.column_stack((u[valid], v[valid])).astype(float)
                xyz = backproject(camera, uv, z[valid])
                center = np.median(xyz, axis=0)
                xy0 = xyz[:, :2] - xyz[:, :2].mean(axis=0)
                cov = xy0.T @ xy0 / max(1, len(xy0) - 1)
                eigvals, eigvecs = np.linalg.eigh(cov)
                major = eigvecs[:, int(np.argmax(eigvals))]
                yaw = math.atan2(float(major[1]), float(major[0]))
                extent_xy = np.ptp(xyz[:, :2], axis=0)
                expected = np.asarray(spec["dimensions_m"], float)
                planar_expected = np.sort(expected[:2])
                planar_seen = np.sort(extent_xy)
                size_error = float(np.max(np.abs(planar_seen - planar_expected) /
                                          np.maximum(planar_expected, 1e-6)))
                size_score = max(0.0, 1.0 - size_error)
                confidence = min(1.0, 0.6 + 0.2 * ratio + 0.2 * size_score)
                full_mask = np.zeros(mask.shape, bool)
                full_mask[v, u] = True
                used_masks.append(full_mask)
                detections.append({
                    "class_id": spec["class_id"], "instance_hint": index,
                    "frame_id": camera.frame_id,
                    "position_m": center.tolist(),
                    "yaw_rad": yaw, "dimensions_m": expected.tolist(),
                    "measured_planar_extent_m": extent_xy.tolist(),
                    "size_error_ratio": size_error, "depth_valid_ratio": ratio,
                    "confidence": confidence, "pixel_count": int(len(pts_vu)),
                })
        return detections, used_masks

    def extract_obstacles(self, depth_m: np.ndarray, camera: CameraModel,
                          target_masks: list[np.ndarray], min_pixels: int = 120) -> list[dict]:
        """移除主桌面深度层后，在图像连通域上生成保守 3D 包围盒。"""
        depth = np.asarray(depth_m, float)
        valid = np.isfinite(depth) & (depth >= self.depth_min_m) & (depth <= self.depth_max_m)
        for m in target_masks:
            valid &= ~m
        vals = depth[valid]
        if len(vals) < min_pixels:
            return []
        hist, edges = np.histogram(vals, bins=max(20, min(200, int(np.sqrt(len(vals))))))
        dominant = 0.5 * (edges[int(np.argmax(hist))] + edges[int(np.argmax(hist)) + 1])
        foreground = valid & (np.abs(depth - dominant) > 0.01)
        obstacles = []
        for i, vu in enumerate(components(foreground, min_pixels), 1):
            v, u = vu[:, 0], vu[:, 1]
            z = depth[v, u]
            good = np.isfinite(z)
            xyz = backproject(camera, np.column_stack((u[good], v[good])), z[good])
            low, high = np.quantile(xyz, 0.02, axis=0), np.quantile(xyz, 0.98, axis=0)
            obstacles.append({"obstacle_id": f"depth-obstacle-{i}",
                              "frame_id": camera.frame_id, "geometry": "box",
                              "position_m": ((low + high) / 2).tolist(),
                              "dimensions_m": np.maximum(high - low, 0.005).tolist(),
                              "confidence": min(1.0, len(xyz) / 1000.0),
                              "uncertainty_1sigma_m": [0.005, 0.005, 0.008]})
        return obstacles
