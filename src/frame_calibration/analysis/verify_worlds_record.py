#!/usr/bin/env python3
"""PB↔SDK Model-A 会话有效性门（只读、绝不连接机器人）。

模型与幸存祖先脚本一致：p_sdk_mm = p_pb_link_origin_mm + t_session_mm。
它比较候选 worlds_record 与已知参考记录各自拟合出的 t，而不是把 Model-A 的 t
和 v2/TCP 模型常量混用。退出码：0=PASS，1=BLOCKED，2=输入/环境错误。
"""
from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
WORK = HERE.parent.parent
sys.path.insert(0, str(HERE))
import calib_common as cc

DEFAULT_REFERENCE = (WORK / "frame_calibration" / "data" / "worlds" /
                     "20260710" / "20260710_150059" /
                     "worlds_record_20260710_150059.json")
DATA_ROOT = WORK / "frame_calibration" / "data" / "worlds"


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 16), b""):
            h.update(block)
    return h.hexdigest()


def newest_record(root: Path = DATA_ROOT) -> Path:
    paths = sorted(root.glob("*/*/worlds_record_*.json"))
    if not paths:
        raise FileNotFoundError(f"{root} 下没有 worlds_record_*.json")
    return paths[-1]


def record_time(path: Path, payload: dict) -> dt.datetime:
    candidates = [payload.get("meta", {}).get(k) for k in
                  ("captured_at", "created", "timestamp")]
    for raw in candidates:
        if raw:
            try:
                return dt.datetime.fromisoformat(str(raw).replace("Z", "+00:00")).replace(tzinfo=None)
            except ValueError:
                pass
    m = re.search(r"(20\d{6})_(\d{6})", path.name)
    if not m:
        m = re.search(r"(20\d{6})_(\d{6})", str(path.parent))
    if not m:
        raise ValueError(f"无法从 meta/文件名解析采集时间: {path}")
    return dt.datetime.strptime("_".join(m.groups()), "%Y%m%d_%H%M%S")


def load_records(path: Path) -> tuple[dict, list[dict]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    records = payload.get("records") or payload.get("samples")
    if not isinstance(records, list) or len(records) < 3:
        raise ValueError(f"{path} 至少需要 3 条 records/samples")
    return payload, records


def extract_q(record: dict) -> list[float]:
    q = record.get("q_sdk_deg") or record.get("joints_feedback_true_deg")
    if not isinstance(q, list) or len(q) < 7:
        raise ValueError("记录缺 q_sdk_deg[7]")
    return [float(v) for v in q[:7]]


def extract_xyz(record: dict) -> np.ndarray:
    xyz = record.get("sdk_world_xyz_mm") or record.get("worlds_feedback_true")
    if not isinstance(xyz, list) or len(xyz) < 3:
        raise ValueError("记录缺 sdk_world_xyz_mm[3]")
    return np.asarray(xyz[:3], dtype=float)


def fit_model_a(path: Path, arm: str = "left") -> dict:
    """复现祖先 Model A。输入关节沿用 SDK 记录符号，不套 v2/TCP 修正。"""
    try:
        import pybullet as p
    except ImportError as exc:
        raise RuntimeError("缺 pybullet；请使用锁定的 Python 3.10 规划环境") from exc
    payload, records = load_records(path)
    if int(payload.get("meta", {}).get("arm_id", 1)) != ({"left": 1, "right": 2}[arm]):
        raise ValueError(f"记录 arm_id 与 --arm {arm} 不符")
    client = p.connect(p.DIRECT)
    try:
        robot = p.loadURDF(cc.DEFAULT_URDF, useFixedBase=True)
        deltas = []
        for row in records:
            for jid, q_deg in zip(cc.ARM_JOINT_IDS[arm], extract_q(row)):
                p.resetJointState(robot, jid, math.radians(q_deg))
            state = p.getLinkState(robot, cc.EE_LINK_ID[arm],
                                  computeForwardKinematics=True)
            pb_mm = np.asarray(state[4], dtype=float) * 1000.0
            deltas.append(extract_xyz(row) - pb_mm)
        values = np.asarray(deltas)
        translation = values.mean(axis=0)
        residual = values - translation
        norms = np.linalg.norm(residual, axis=1)
        return {
            "sample_count": len(records),
            "t_session_mm": translation.tolist(),
            "residual_rms_mm": float(np.sqrt(np.mean(norms ** 2))),
            "residual_max_mm": float(np.max(norms)),
            "residual_mean_abs_axis_mm": np.mean(np.abs(residual), axis=0).tolist(),
        }
    finally:
        p.disconnect(client)


def evaluate_gate(reference_t, candidate_t, captured_at, now,
                  axis_limit_mm=5.0, norm_limit_mm=8.0, max_age_days=7.0) -> dict:
    drift = np.asarray(candidate_t, float) - np.asarray(reference_t, float)
    # 现场记录的旧文件名/元数据没有时区，而 --now 常由操作员传入 +08:00。
    # 两者混算会触发 naive/aware TypeError；缺失的一侧按已知一侧的时区解释，
    # 既保留历史本地时间语义，也不悄悄做八小时偏移。
    if now.tzinfo is not None and captured_at.tzinfo is None:
        captured_at = captured_at.replace(tzinfo=now.tzinfo)
    elif now.tzinfo is None and captured_at.tzinfo is not None:
        now = now.replace(tzinfo=captured_at.tzinfo)
    age_days = (now - captured_at).total_seconds() / 86400.0
    checks = {
        "axis_drift": bool(np.all(np.abs(drift) < axis_limit_mm)),
        "euclidean_drift": bool(np.linalg.norm(drift) < norm_limit_mm),
        "freshness": bool(0.0 <= age_days <= max_age_days),
    }
    return {
        "drift_xyz_mm": drift.tolist(),
        "drift_euclidean_mm": float(np.linalg.norm(drift)),
        "age_days": age_days,
        "thresholds": {"axis_abs_lt_mm": axis_limit_mm,
                       "euclidean_lt_mm": norm_limit_mm,
                       "age_lte_days": max_age_days},
        "checks": checks,
        "verdict": "PASS" if all(checks.values()) else "BLOCKED",
    }


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("record", nargs="?", help="候选 worlds_record；默认取最新")
    ap.add_argument("--reference", default=str(DEFAULT_REFERENCE))
    ap.add_argument("--arm", choices=("left", "right"), default="left")
    ap.add_argument("--now", help="测试/复现用 ISO 时间；默认本机当前时间")
    ap.add_argument("--axis-limit-mm", type=float, default=5.0)
    ap.add_argument("--norm-limit-mm", type=float, default=8.0)
    ap.add_argument("--max-age-days", type=float, default=7.0)
    ap.add_argument("--json-out", help="写机器可读报告；不指定则只打印")
    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    try:
        candidate = Path(args.record).expanduser().resolve() if args.record else newest_record()
        reference = Path(args.reference).expanduser().resolve()
        if candidate == reference:
            raise ValueError("候选记录不能与参考记录相同")
        ref_payload, _ = load_records(reference)
        cand_payload, _ = load_records(candidate)
        ref_fit = fit_model_a(reference, args.arm)
        cand_fit = fit_model_a(candidate, args.arm)
        captured = record_time(candidate, cand_payload)
        now = (dt.datetime.fromisoformat(args.now) if args.now else dt.datetime.now())
        gate = evaluate_gate(ref_fit["t_session_mm"], cand_fit["t_session_mm"],
                             captured, now, args.axis_limit_mm,
                             args.norm_limit_mm, args.max_age_days)
        report = {
            "schema_version": "frame_gate_report.v1",
            "model": "A_TRANSLATION_ONLY",
            "transform": "p_sdk_mm = p_pb_ee_link_origin_mm + t_session_mm",
            "arm": args.arm,
            "reference": {"path": str(reference), "sha256": sha256_file(reference), **ref_fit},
            "candidate": {"path": str(candidate), "sha256": sha256_file(candidate),
                          "captured_at": captured.isoformat(), **cand_fit},
            **gate,
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        if args.json_out:
            Path(args.json_out).write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                                           encoding="utf-8")
        return 0 if gate["verdict"] == "PASS" else 1
    except Exception as exc:  # explicit environment/input failure
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
