#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
from pathlib import Path

import numpy as np


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 16), b""):
            h.update(block)
    return h.hexdigest()


def save_frame_bundle(output_dir: str | Path, rgb: np.ndarray, depth_m: np.ndarray,
                      camera_info: dict, metadata: dict) -> dict:
    """保存无损 NPY + CameraInfo + 时间元数据，返回 observation 所需哈希。"""
    out = Path(output_dir)
    out.mkdir(parents=True, exist_ok=False)
    rgb_path, depth_path = out / "rgb.npy", out / "depth_m.npy"
    info_path, meta_path = out / "camera_info.json", out / "frame_metadata.json"
    np.save(rgb_path, np.asarray(rgb, dtype=np.uint8), allow_pickle=False)
    np.save(depth_path, np.asarray(depth_m, dtype=np.float32), allow_pickle=False)
    info_path.write_text(json.dumps(camera_info, ensure_ascii=False, sort_keys=True,
                                    separators=(",", ":")) + "\n", encoding="utf-8")
    evidence = {"rgb_sha256": _sha256(rgb_path),
                "depth_sha256": _sha256(depth_path),
                "camera_info_sha256": _sha256(info_path),
                "bundle_path": str(out.resolve())}
    final_metadata = dict(metadata)
    final_metadata["evidence"] = evidence
    meta_path.write_text(json.dumps(final_metadata, ensure_ascii=False, sort_keys=True,
                                    separators=(",", ":")) + "\n", encoding="utf-8")
    return evidence


def load_frame_bundle(path: str | Path) -> tuple[np.ndarray, np.ndarray, dict, dict]:
    root = Path(path)
    return (np.load(root / "rgb.npy", allow_pickle=False),
            np.load(root / "depth_m.npy", allow_pickle=False),
            json.loads((root / "camera_info.json").read_text(encoding="utf-8")),
            json.loads((root / "frame_metadata.json").read_text(encoding="utf-8")))
