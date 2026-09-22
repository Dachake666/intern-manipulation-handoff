#!/usr/bin/env python3
"""真机执行前的批准/场景复核绑定校验；只依赖标准库。"""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path


def canonical(value) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def sha256_file(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def report_hash(report: dict) -> str:
    value = copy.deepcopy(report)
    value["approval"]["report_sha256"] = "0" * 64
    value["approval"]["token"] = None
    return hashlib.sha256(canonical(value)).hexdigest()


def verify_authorization(trajectory_path, report_path, token_path,
                         scene_recheck_path, robot_config_identity) -> dict:
    report = json.loads(Path(report_path).read_text(encoding="utf-8"))
    token = json.loads(Path(token_path).read_text(encoding="utf-8"))
    scene = json.loads(Path(scene_recheck_path).read_text(encoding="utf-8"))
    if report.get("schema_version") != "preflight_report.v1" or report.get("verdict") != "PASS":
        raise RuntimeError("preflight 报告不是 PASS")
    bound = report.get("approval", {}).get("report_sha256")
    if bound != report_hash(report):
        raise RuntimeError("preflight 报告内容已变化，批准失效")
    if report["inputs"]["trajectory_sha256"] != sha256_file(trajectory_path):
        raise RuntimeError("轨迹 SHA-256 与 preflight 输入不一致")
    if report["inputs"]["robot_config_identity"] != robot_config_identity:
        raise RuntimeError("机器人配置身份与 preflight 不一致")
    if token.get("schema_version") != "approval_token.v1" or token.get("report_sha256") != bound:
        raise RuntimeError("批准令牌未绑定当前 preflight 报告")
    if not token.get("token") or not token.get("operator"):
        raise RuntimeError("批准令牌缺操作员或 token")
    if scene.get("schema_version") != "scene_recheck.v1" or scene.get("verdict") != "PASS":
        raise RuntimeError("批准后的场景复核未通过")
    if scene.get("before_observation_sha256") != report["inputs"]["observation_sha256"]:
        raise RuntimeError("场景复核基准 observation 与 preflight 不一致")
    return {"report_sha256": bound, "approval_token": token["token"],
            "operator": token["operator"],
            "current_observation_sha256": scene.get("after_observation_sha256")}
