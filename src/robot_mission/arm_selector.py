#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass
import argparse
import json
from pathlib import Path


@dataclass(frozen=True)
class ArmCandidate:
    arm: str
    reachable: bool
    limits_ok: bool
    ik_continuous: bool
    collision_free: bool
    minimum_joint_margin_deg: float
    minimum_clearance_m: float
    estimated_time_s: float
    path_length_rad: float
    hardware_ready: bool
    trajectory_path: str | None = None

    @property
    def admissible(self):
        return self.reachable and self.limits_ok and self.ik_continuous and self.collision_free


def rank_candidates(candidates: list[ArmCandidate], require_hardware_ready=False) -> ArmCandidate:
    viable = [c for c in candidates if c.admissible and
              (c.hardware_ready or not require_hardware_ready)]
    if not viable:
        raise RuntimeError("左右臂候选均被可达性/限位/连续性/碰撞门拒绝")
    # 先最大化关节余量和障碍净空，再最小化时间和路长；确定性地以 left 打破平局。
    return sorted(viable, key=lambda c: (-c.minimum_joint_margin_deg,
                                         -c.minimum_clearance_m,
                                         c.estimated_time_s, c.path_length_rad,
                                         0 if c.arm == "left" else 1))[0]


def from_qualification(path):
    d = json.loads(Path(path).read_text(encoding="utf-8"))
    gates = d["gates"]
    clearance = d["collision_report"].get("minimum_clearance_m")
    return ArmCandidate(
        arm=d["arm"], reachable=d["verdict"] == "PASS",
        limits_ok=gates["limits"] == "PASS", ik_continuous=gates["limits"] == "PASS",
        collision_free=gates["collision"] == "PASS",
        minimum_joint_margin_deg=float(d["limit_report"].get("minimum_margin_deg", -1)),
        minimum_clearance_m=float(clearance if clearance is not None else 999.0),
        estimated_time_s=float(d["dense_frames"]) * .02,
        path_length_rad=float(d.get("path_length_rad", 999.0)),
        hardware_ready=bool(d.get("hardware_ready")), trajectory_path=d.get("trajectory_path"))


def main(argv=None):
    ap = argparse.ArgumentParser(description="在左右臂离线资格报告间自动选臂")
    ap.add_argument("reports", nargs="+"); ap.add_argument("--out", required=True)
    ap.add_argument("--hardware-ready", action="store_true",
                    help="真机选择：未完成硬件门的候选直接拒绝")
    a = ap.parse_args(argv)
    candidates = [from_qualification(p) for p in a.reports]
    selected = rank_candidates(candidates, require_hardware_ready=a.hardware_ready)
    payload = {"schema_version": "arm_selection.v1", "policy": "strict_serial",
               "selected_arm": selected.arm,
               "candidates": [{**c.__dict__, "admissible": c.admissible} for c in candidates]}
    Path(a.out).write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"selected_arm={selected.arm} strict_serial -> {a.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
