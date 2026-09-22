#!/usr/bin/env python3
"""把已通过密集资格与 GUI 人工门的 legacy Servo 候选生成为 RUN 快照。

不修改候选原件；输出仍只代表 STAGED_TEST_READY，完整匹配真机日志成功后才能标记
REAL_VERIFIED。
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
from pathlib import Path


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} 顶层必须是 object")
    return value


def authorize(candidate_path: Path, report_path: Path, gui_path: Path,
              out_path: Path) -> dict:
    candidate_path = candidate_path.resolve()
    report_path = report_path.resolve()
    gui_path = gui_path.resolve()
    out_path = out_path.resolve()
    candidate, report, gui = (_load(candidate_path), _load(report_path), _load(gui_path))
    candidate_hash = sha256_file(candidate_path)
    report_hash = sha256_file(report_path)
    gui_hash = sha256_file(gui_path)

    if report.get("trajectory_sha256") != candidate_hash:
        raise ValueError("qualification report 未绑定当前 candidate SHA-256")
    if report.get("verdict") != "PASS":
        raise ValueError(f"qualification verdict 不是 PASS: {report.get('verdict')}")
    gates = report.get("gates", {})
    required_gates = ("arm_gripper_contract", "limits", "collision",
                      "pybullet_gui_review")
    failed = [name for name in required_gates if gates.get(name) != "PASS"]
    if failed:
        raise ValueError("qualification gate 未通过: " + ", ".join(failed))
    if gui.get("result") != "PASS" or gui.get("trajectory_sha256") != candidate_hash:
        raise ValueError("GUI review 未 PASS 或未绑定当前 candidate")
    if gui.get("qualification_task_sha256") != report.get("task_sha256"):
        raise ValueError("GUI review 与 qualification report 的任务 SHA 不一致")

    meta = candidate.get("meta", {})
    # 真正未知的参考点仍拒绝。参考点已明确、仅待修正场景 GUI 的状态，
    # 由上面同时绑定轨迹和场景 SHA 的 PASS 记录完成闭环，不再改候选重跑一遍。
    scene_status = meta.get("scene_geometry_status")
    table = meta.get("table", {})
    matched_scene_review = (
        scene_status == "EE_REFERENCE_CONFIRMED_CORRECTED_SCENE_REVIEW_PENDING" and
        table.get("corner_reference") == "EE_LINK_ORIGIN" and
        bool(table.get("corner_reference_evidence")))
    if scene_status is not None and scene_status != "CONFIRMED" and not matched_scene_review:
        raise ValueError(f"场景几何尚未确认，不能签发 RUN: {scene_status}")
    if (meta.get("arm"), meta.get("arm_id"), meta.get("gripper_id")) != (
            "left", 1, 2):
        raise ValueError("仅允许左臂 arm_id=1 / gripper_id=2 的本任务")
    if meta.get("real_motion_authorized") is not False:
        raise ValueError("输入必须是未授权 candidate 原件")
    events = [row.get("gripper") for row in candidate.get("waypoints", [])
              if "gripper" in row]
    if events != ["close", "open"]:
        raise ValueError(f"夹爪事件不是 close/open: {events}")

    run = copy.deepcopy(candidate)
    run_meta = run["meta"]
    if matched_scene_review:
        run_meta["scene_geometry_status"] = "CONFIRMED"
        run_meta["scene_geometry_confirmation"] = {
            "basis": "USER_ENDPOINT_REFERENCE_AND_MATCHED_NEW_SCENE_GUI_REVIEW",
            "candidate_scene_status": scene_status,
            "gui_review_sha256": gui_hash,
            "qualification_task_sha256": gui["qualification_task_sha256"],
        }
    run_meta.update({
        "qualification": "STAGED_REAL_TEST_READY",
        "evidence_status": "STAGED_TEST_READY_NOT_REAL_VERIFIED",
        "real_motion_authorized": True,
        "enforce_live_controller_limits": True,
        "authorization_scope": "GUARDED_SINGLE_REAL_ROBOT_TEST",
        "authorized_at": gui.get("created"),
        "authorized_by": gui.get("reviewer"),
        "authorized_from_candidate": str(candidate_path),
        "authorized_from_candidate_sha256": candidate_hash,
        "qualification_report": str(report_path),
        "qualification_report_sha256": report_hash,
        "gui_review": {
            "required": True,
            "status": "PASS",
            "path": str(gui_path),
            "sha256": gui_hash,
            "evidence_source": gui.get("evidence_source", "GUI_REVIEW_FILE"),
        },
        "runtime_gates": [
            "dynamic robot/local/arm IP required",
            "local limits checked before connection",
            "live controller limits intersected with physical operational limits",
            "operator confirms clear entry path when current-to-first gap exceeds 20deg",
            "point-to-point entry settles before second operator confirmation",
            "protection and global speed restored in finally",
        ],
        "note": (
            "RUN snapshot for one guarded staged real-robot test. This is not "
            "REAL_VERIFIED until a matching complete run log is archived."),
    })
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(run, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8")
    return {
        "run_trajectory": str(out_path),
        "run_trajectory_sha256": sha256_file(out_path),
        "candidate_sha256": candidate_hash,
        "qualification_report_sha256": report_hash,
        "gui_review_sha256": gui_hash,
        "authorization_scope": run_meta["authorization_scope"],
    }


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--candidate", type=Path, required=True)
    ap.add_argument("--qualification-report", type=Path, required=True)
    ap.add_argument("--gui-review", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args(argv)
    print(json.dumps(authorize(args.candidate, args.qualification_report,
                               args.gui_review, args.out),
                     ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
