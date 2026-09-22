#!/usr/bin/env python3
from __future__ import annotations

import argparse
import datetime as dt
import json
from pathlib import Path

from .detector import CameraModel, ColorBoxDetector, load_catalog
from .evidence import load_frame_bundle
from .temporal import TemporalGate


def replay(frame_dirs: list[str], catalog_path: str, camera_id: str,
           intrinsics_id: str, extrinsics_id: str) -> dict:
    detector = ColorBoxDetector(load_catalog(catalog_path))
    gate = TemporalGate()
    accepted, last_meta, last_info, last_masks, last_depth = [], None, None, [], None
    hashes, supporting_frames = None, []
    for frame_dir in frame_dirs:
        rgb, depth, info, meta = load_frame_bundle(frame_dir)
        camera = CameraModel.from_camera_info(info)
        detections, masks = detector.detect_frame(rgb, depth, camera)
        accepted = gate.add(detections)
        last_meta, last_info, last_masks, last_depth = meta, info, masks, depth
        hashes = meta.get("evidence")
        if hashes:
            supporting_frames.append(hashes)
    if last_meta is None:
        raise ValueError("至少需要一个回放帧")
    if len(frame_dirs) < 5:
        raise ValueError("稳定门要求提供 5 帧回放")
    camera = CameraModel.from_camera_info(last_info)
    obstacles = detector.extract_obstacles(last_depth, camera, last_masks)
    return {
        "schema_version": "vision_observation.v1",
        "observation_id": last_meta.get("observation_id") or
                          f"replay-{dt.datetime.now().strftime('%Y%m%dT%H%M%S')}",
        "camera": {"camera_id": camera_id,
                   "ros_timestamp_ns": int(last_meta["ros_timestamp_ns"]),
                   "frame_id": camera.frame_id,
                   "rgb_depth_delta_ms": float(last_meta["rgb_depth_delta_ms"])},
        "calibration": {"intrinsics_id": intrinsics_id, "extrinsics_id": extrinsics_id,
                        "extrinsics_direction": "T_sdk_world_camera", "length_unit": "meter"},
        "evidence": {**(hashes or {"rgb_sha256": "0" * 64,
                                    "depth_sha256": "0" * 64,
                                    "camera_info_sha256": "0" * 64,
                                    "bundle_path": str(Path(frame_dirs[-1]).resolve())}),
                     "supporting_frames": supporting_frames[-5:] or [
                         {"rgb_sha256": "0" * 64, "depth_sha256": "0" * 64,
                          "camera_info_sha256": "0" * 64} for _ in range(5)]},
        "objects": accepted,
        "obstacles": [{"obstacle_id": o["obstacle_id"], "frame_id": o["frame_id"],
                       "geometry": o["geometry"],
                       "pose": {"position_m": o["position_m"],
                                "quaternion_xyzw": [0, 0, 0, 1]},
                       "dimensions_m": o["dimensions_m"], "confidence": o["confidence"],
                       "uncertainty_1sigma_m": o["uncertainty_1sigma_m"]}
                      for o in obstacles],
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="离线回放 5 帧 RGB-D 证据")
    ap.add_argument("frames", nargs="+", help="按时间顺序给出 5 个 frame bundle 目录")
    ap.add_argument("--catalog", required=True)
    ap.add_argument("--camera-id", required=True)
    ap.add_argument("--intrinsics-id", required=True)
    ap.add_argument("--extrinsics-id", required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    observation = replay(a.frames, a.catalog, a.camera_id,
                         a.intrinsics_id, a.extrinsics_id)
    Path(a.out).write_text(json.dumps(observation, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"objects={len(observation['objects'])} obstacles={len(observation['obstacles'])} -> {a.out}")
    return 0 if observation["objects"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
