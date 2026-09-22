#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from .contracts import load_and_validate


def find_object(observation, object_id):
    for item in observation["objects"]:
        if item["object_id"] == object_id:
            return item
    raise ValueError(f"observation 中找不到 {object_id}")


def verify_grasp_follow(before, after, object_id, expected_delta_m,
                        max_error_m=.02):
    a, b = find_object(before, object_id), find_object(after, object_id)
    if a["class_id"] != b["class_id"]:
        return {"verdict": "FAIL", "reason": "class_changed"}
    observed = [y-x for x, y in zip(a["pose"]["position_m"], b["pose"]["position_m"])]
    error = math.dist(observed, expected_delta_m)
    return {"verdict": "PASS" if error <= max_error_m else "FAIL",
            "observed_delta_m": observed, "expected_delta_m": expected_delta_m,
            "error_m": error, "threshold_m": max_error_m}


def verify_placement(after, object_id, expected_position_camera_m,
                     max_error_m=.02):
    item = find_object(after, object_id)
    error = math.dist(item["pose"]["position_m"], expected_position_camera_m)
    return {"verdict": "PASS" if error <= max_error_m else "FAIL",
            "observed_position_m": item["pose"]["position_m"],
            "expected_position_m": expected_position_camera_m,
            "error_m": error, "threshold_m": max_error_m}


def main(argv=None):
    ap = argparse.ArgumentParser(description="执行器视觉验收钩子的参考实现")
    ap.add_argument("--kind", choices=("grasp_follow", "placement"), required=True)
    ap.add_argument("--object-id", required=True); ap.add_argument("--max-error-m", type=float, default=.02)
    ap.add_argument("--before"); ap.add_argument("--after", required=True)
    ap.add_argument("--expected", type=float, nargs=3, required=True,
                    help="grasp_follow 为相机系位移；placement 为相机系目标位置")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    after = load_and_validate(a.after, "vision_observation.v1")
    if a.kind == "grasp_follow":
        if not a.before:
            ap.error("grasp_follow 需要 --before")
        before = load_and_validate(a.before, "vision_observation.v1")
        result = verify_grasp_follow(before, after, a.object_id, a.expected, a.max_error_m)
    else:
        result = verify_placement(after, a.object_id, a.expected, a.max_error_m)
    payload = {"schema_version": "visual_acceptance.v1", "kind": a.kind,
               "object_id": a.object_id, **result}
    text = json.dumps(payload, ensure_ascii=False, indent=2)
    print(text)
    if a.out:
        Path(a.out).write_text(text + "\n", encoding="utf-8")
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
