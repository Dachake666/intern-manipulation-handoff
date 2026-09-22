#!/usr/bin/env python3
"""冻结左臂 Track C 的轨迹文件与完整下发点流。只做纯计算。"""
from __future__ import annotations

import hashlib
import json
import os
import sys
import types
from pathlib import Path

WORK = Path(__file__).resolve().parent.parent
ROBOT = WORK / "frame_calibration" / "robot_side"
TRAJ = Path(__file__).resolve().parent / "trajectories" / "verified" / \
       "traj_multi_2grasp_20260804_REALVERIFIED.json"
EXPECTED_FILE = "ac5c6fd7715c81fbfa2f2fc4c9eca1f676b5a3232dff99761a02c7b4812e4968"
EXPECTED_STREAMS = {0.4: (1633, "5b53efbcbe911ed0d451a91c"),
                    0.2: (3232, "3034c58e83f0a65a96e84695")}


def main():
    stub = types.ModuleType("pypilot")
    stub.PilotSDK = stub.AxisParameterIndex = object
    stub.FloatVector = stub.DoubleVector = list
    sys.modules.setdefault("pypilot", stub)
    sys.path.insert(0, str(ROBOT))
    import execute_servo_grasp as executor
    actual = hashlib.sha256(TRAJ.read_bytes()).hexdigest()
    if actual != EXPECTED_FILE:
        print(f"FAIL trajectory SHA: {actual}")
        return 1
    waypoints = json.loads(TRAJ.read_text(encoding="utf-8"))["waypoints"]
    for step, (expected_count, expected_hash) in EXPECTED_STREAMS.items():
        stream, previous = [], None
        for waypoint in waypoints:
            if "gripper" in waypoint:
                stream.append(("grip", waypoint["gripper"]))
                continue
            q = waypoint["q_sdk_deg"]
            if previous is not None:
                stream.extend(tuple(round(v, 9) for v in point)
                              for point in executor.densify(previous, q, step))
            previous = q
        count = sum(1 for x in stream if x[0] != "grip")
        digest = hashlib.sha256(repr(stream).encode()).hexdigest()[:24]
        if (count, digest) != (expected_count, expected_hash):
            print(f"FAIL step={step}: count/hash={(count, digest)}")
            return 1
        print(f"PASS step={step}° frames={count} stream_sha={digest}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
