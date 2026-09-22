#!/usr/bin/env python3
"""Read the ten frozen JSON reports and reproduce the recorded statistics.

Standard library only. Does not import the MPC executors or vendor SDK, connect
any robot, change source files, or execute commands from the source package.
Usage: python3 recalculate_mpc_reports.py /path/to/extracted/archive/root
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys

REPORTS = (
    ("150937_394611035", "mpc_shadow"),
    ("161007_195575844", "mpc_active"),
    ("161804_427508716", "mpc_shadow"),
    ("161842_290824538", "mpc_active"),
    ("161957_559208120", "mpc_shadow"),
    ("162057_54388464", "mpc_active"),
    ("162340_144861635", "mpc_shadow"),
    ("162429_177216570", "mpc_shadow"),
    ("162614_575109989", "mpc_shadow"),
    ("162706_519367266", "mpc_active"),
)
TRAJECTORY_SHA = "71bc5148f5aba59aa91635e20905918f777c765c105bff6b29c10f670a1db338"


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def vector(value: object, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != 7:
        raise ValueError(f"{label}: expected a 7-element list")
    result = [float(x) for x in value]
    if not all(math.isfinite(x) for x in result):
        raise ValueError(f"{label}: non-finite data")
    return result


def percentile(values: list[float], percent: float) -> float:
    """Linear interpolation, matching the previous NumPy percentile default."""
    ordered = sorted(values)
    if not ordered:
        raise ValueError("Empty percentile input")
    position = (len(ordered) - 1) * percent / 100.0
    left = math.floor(position)
    right = math.ceil(position)
    return ordered[left] + (ordered[right] - ordered[left]) * (position - left)


def rmse(values: list[float]) -> float:
    if not values:
        raise ValueError("Empty RMSE input")
    return math.sqrt(math.fsum(x * x for x in values) / len(values))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path, help="Directory containing trackA_worlds_line, mpc_experiment, XF0112048")
    args = parser.parse_args()
    root = args.root.expanduser().resolve()
    track = root / "trackA_worlds_line" / "trackA_servo"
    trajectory = track / "tabletop_pick_place_SERVO_CANDIDATE.json"
    if not trajectory.is_file() or digest(trajectory) != TRAJECTORY_SHA:
        raise ValueError("Frozen trajectory is missing or its SHA-256 differs")

    runs = []
    for stamp, key in REPORTS:
        path = track / f"tabletop_servo_20260916_{stamp}.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        info = data[key]
        if data.get("status") != "SERVO_COMPLETED_RETURNED_HOME":
            raise ValueError(f"Not a successful final report: {path.name}")
        if not info.get("enabled_until_end") or info.get("errors"):
            raise ValueError(f"MPC did not complete: {path.name}")
        if data.get("trajectory_sha256") != TRAJECTORY_SHA:
            raise ValueError(f"Report trajectory differs: {path.name}")
        executor = track / ("execute_tabletop_servo_mpc_active_base.py" if key == "mpc_active" else "execute_tabletop_servo_mpc_base.py")
        if digest(executor) != data["executor_sha256"]:
            raise ValueError(f"Executor differs from report: {path.name}")
        for name, expected in data["dependencies"].items():
            if Path(name).name != name:
                raise ValueError(f"Unexpected dependency path: {name}")
            if digest(track / name) != expected:
                raise ValueError(f"Dependency differs: {name} in {path.name}")
        wrapper = track / ("execute_tabletop_servo_field_mpc_active_20ms.py" if key == "mpc_active" else "execute_tabletop_servo_field_mpc_20ms.py")
        recorded_wrapper = data.get("reviews", {}).get("field_wrapper_sha256")
        if recorded_wrapper and digest(wrapper) != recorded_wrapper:
            raise ValueError(f"Wrapper differs from report: {path.name}")
        runtime = data["field_runtime_servo"]
        if runtime.get("body_motion_frames") != 630:
            raise ValueError(f"Unexpected body length: {path.name}")
        entry = int(runtime["entry_servo_frames"])
        samples = [row for row in info["feedback_samples"] if int(row["frame"]) >= entry]
        if len(samples) != 315:
            raise ValueError(f"Expected 315 body samples, got {len(samples)}: {path.name}")
        errors = []
        for row in samples:
            ref = vector(row["reference_q_deg"], "reference")
            measured = vector(row["measured_q_deg"], "measured")
            errors.append([a - b for a, b in zip(ref, measured)])
        flat = [x for row in errors for x in row]
        absolute = [abs(x) for x in flat]
        result = {
            "name": path.name,
            "mode": "ACTIVE" if key == "mpc_active" else "SHADOW",
            "rmse": rmse(flat),
            "p99": percentile(absolute, 99.0),
            "max": max(absolute),
            "joint_rmse": [rmse([row[j] for row in errors]) for j in range(7)],
            "offset": int(samples[0]["frame"]) - entry,
        }
        runs.append(result)
        print(f"{result['mode']:6s} RMSE={result['rmse']:.4f} deg P99={result['p99']:.4f} deg MAX={result['max']:.4f} deg | {path.name}")

    grouped = {}
    for mode in ("SHADOW", "ACTIVE"):
        group = [row for row in runs if row["mode"] == mode]
        grouped[mode] = group
        print(f"\n{mode}: n={len(group)}")
        for key in ("rmse", "p99", "max"):
            values = [row[key] for row in group]
            print(f"{key.upper():4s} = {statistics.mean(values):.4f} +/- {statistics.stdev(values):.4f} deg")
        joints = [statistics.mean(row["joint_rmse"][j] for row in group) for j in range(7)]
        print("joint RMSE:", ", ".join(f"{x:.4f}" for x in joints))
    s = statistics.mean(row["rmse"] for row in grouped["SHADOW"])
    a = statistics.mean(row["rmse"] for row in grouped["ACTIVE"])
    print(f"\nMean-RMSE reduction: {100.0 * (1.0 - a / s):.2f}%")
    print("\nMethod: same recorded reference-minus-feedback samples; entry bridge excluded.")
    print("P99/MAX summaries are means of per-run P99/MAX, not pooled statistics.")
    print("This reproduces the existing metric, not timestamp-aligned ground-truth accuracy.")
    print("The first two runs start body sampling at offset 1; the others at offset 0.")
    print("Successful runs only: these ten reports do not establish a 100% success rate.")
    print("No MPC or SDK code executed; no robot connection attempted.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)
