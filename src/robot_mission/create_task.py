#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import uuid
from pathlib import Path

from .contracts import load_and_validate, sha256_file, validate_document


def main(argv=None):
    ap = argparse.ArgumentParser(description="把操作员逐项放置位绑定到视觉 observation")
    ap.add_argument("observation"); ap.add_argument("placements",
        help="JSON 数组: object_id/frame_id/position_m/quaternion_xyzw")
    ap.add_argument("--catalog-id", required=True); ap.add_argument("--out", required=True)
    ap.add_argument("--arm-preference", choices=("auto", "left", "right"), default="auto")
    a = ap.parse_args(argv)
    observation = load_and_validate(a.observation, "vision_observation.v1")
    placements = json.loads(Path(a.placements).read_text(encoding="utf-8"))
    known = {x["object_id"] for x in observation["objects"]}
    actions = []
    for i, item in enumerate(placements, 1):
        if item["object_id"] not in known:
            raise ValueError(f"未知 object_id={item['object_id']}")
        actions.append({"action_id": f"pick-place-{i}", "type": "pick_place",
                        "object_id": item["object_id"],
                        "place": {"frame_id": item["frame_id"],
                                  "position_m": item["position_m"],
                                  "quaternion_xyzw": item["quaternion_xyzw"]}})
    task = {"schema_version": "task_request.v1", "task_id": str(uuid.uuid4()),
            "observation": {"path": str(Path(a.observation).resolve()),
                            "sha256": sha256_file(a.observation)},
            "actions": actions, "arm_preference": a.arm_preference,
            "execution_policy": {"dual_arm_mode": "strict_serial", "approval_required": True},
            "object_catalog_id": a.catalog_id,
            "safety_policy": {"obstacle_inflation_min_m": .02,
                              "obstacle_sigma_multiplier": 3,
                              "scene_change_min_m": .01,
                              "minimum_joint_margin_deg": 5}}
    validate_document(task, "task_request.v1")
    Path(a.out).write_text(json.dumps(task, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"task_id={task['task_id']} actions={len(actions)} -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
