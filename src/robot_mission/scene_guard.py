#!/usr/bin/env python3
from __future__ import annotations

import math
import argparse
import json
from pathlib import Path

from .contracts import load_and_validate, sha256_file


def compare_observations(before: dict, after: dict, minimum_change_m=0.01,
                         sigma_multiplier=3.0) -> dict:
    reasons = []
    if before["calibration"]["extrinsics_id"] != after["calibration"]["extrinsics_id"]:
        reasons.append("extrinsics_id_changed")
    old_obj, new_obj = ({x["object_id"]: x for x in d["objects"]} for d in (before, after))
    for oid, old in old_obj.items():
        if oid not in new_obj:
            reasons.append(f"target_missing:{oid}")
            continue
        new = new_obj[oid]
        delta = math.dist(old["pose"]["position_m"], new["pose"]["position_m"])
        sigma = max(old["uncertainty_1sigma_m"] + new["uncertainty_1sigma_m"])
        if delta > max(minimum_change_m, sigma_multiplier * sigma):
            reasons.append(f"target_moved:{oid}:{delta:.6f}m")
    old_obs, new_obs = ({x["obstacle_id"]: x for x in d["obstacles"]} for d in (before, after))
    for oid in set(new_obs) - set(old_obs):
        reasons.append(f"new_occupancy:{oid}")
    for oid in set(old_obs) & set(new_obs):
        a, b = old_obs[oid], new_obs[oid]
        delta = math.dist(a["pose"]["position_m"], b["pose"]["position_m"])
        sigma = max(a["uncertainty_1sigma_m"] + b["uncertainty_1sigma_m"])
        if delta > max(minimum_change_m, sigma_multiplier * sigma):
            reasons.append(f"obstacle_moved:{oid}:{delta:.6f}m")
    return {"verdict": "PASS" if not reasons else "INVALIDATED", "reasons": reasons}


def main(argv=None):
    ap = argparse.ArgumentParser(description="批准后重采场景变化门")
    ap.add_argument("before"); ap.add_argument("after"); ap.add_argument("--out", required=True)
    ap.add_argument("--minimum-change-m", type=float, default=0.01)
    ap.add_argument("--sigma-multiplier", type=float, default=3.0)
    a = ap.parse_args(argv)
    before = load_and_validate(a.before, "vision_observation.v1")
    after = load_and_validate(a.after, "vision_observation.v1")
    result = compare_observations(before, after, a.minimum_change_m, a.sigma_multiplier)
    payload = {"schema_version": "scene_recheck.v1",
               "before_observation_sha256": sha256_file(a.before),
               "after_observation_sha256": sha256_file(a.after), **result}
    Path(a.out).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if result["verdict"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
