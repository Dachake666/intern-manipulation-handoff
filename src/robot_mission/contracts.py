#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

WORK = Path(__file__).resolve().parent.parent
SCHEMA_DIR = WORK / "schemas"
SCHEMAS = {
    "vision_observation.v1": SCHEMA_DIR / "vision_observation.v1.schema.json",
    "task_request.v1": SCHEMA_DIR / "task_request.v1.schema.json",
    "trajectory.v2": SCHEMA_DIR / "trajectory.v2.schema.json",
    "preflight_report.v1": SCHEMA_DIR / "preflight_report.v1.schema.json",
    "eye_to_hand_calibration.v1": SCHEMA_DIR / "eye_to_hand_calibration.v1.schema.json",
    "tabletop_request.v1": SCHEMA_DIR / "tabletop_request.v1.schema.json",
    "tabletop_plan.v1": SCHEMA_DIR / "tabletop_plan.v1.schema.json",
    "tabletop_scene_snapshot.v1": SCHEMA_DIR / "tabletop_scene_snapshot.v1.schema.json",
}

# 名称、动作和夹持状态共同构成桌上抓放状态机；只校验名称不足以阻止开闭反转。
TABLETOP_STAGE_CONTRACT = (
    ("START_ASSERT_SAFE", "ASSERT_ENDPOINT", None, "carrying", False),
    ("OPEN_BEFORE_PICK", "GRIPPER", "open", "carrying_after", False),
    ("PICK_HOVER", "MOVE_WORLDS", None, "carrying", False),
    ("PICK_DESCEND", "MOVE_WORLDS", None, "carrying", False),
    ("CLOSE_AT_PICK", "GRIPPER", "close", "carrying_after", True),
    ("PICK_ASCEND", "MOVE_WORLDS", None, "carrying", True),
    ("PLACE_HOVER", "MOVE_WORLDS", None, "carrying", True),
    ("PLACE_DESCEND", "MOVE_WORLDS", None, "carrying", True),
    ("OPEN_AT_PLACE", "GRIPPER", "open", "carrying_after", False),
    ("PLACE_ASCEND", "MOVE_WORLDS", None, "carrying", False),
    ("RETURN_SAFE", "MOVE_WORLDS", None, "carrying", False),
    ("END_ASSERT_SAFE", "ASSERT_ENDPOINT", None, "carrying", False),
)


def canonical_bytes(value) -> bytes:
    return json.dumps(value, ensure_ascii=False, sort_keys=True,
                      separators=(",", ":"), allow_nan=False).encode("utf-8")


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256_file(path: str | Path) -> str:
    h = hashlib.sha256()
    with Path(path).open("rb") as f:
        for block in iter(lambda: f.read(1 << 16), b""):
            h.update(block)
    return h.hexdigest()


def validate_document(document: dict, expected: str | None = None) -> None:
    version = document.get("schema_version")
    if expected and version != expected:
        raise ValueError(f"schema_version={version!r}, 预期 {expected!r}")
    if version not in SCHEMAS:
        raise ValueError(f"未知 schema_version={version!r}")
    try:
        import jsonschema
    except ImportError as exc:
        raise RuntimeError("缺 jsonschema；无法执行版本化契约校验") from exc
    schema = json.loads(SCHEMAS[version].read_text(encoding="utf-8"))
    try:
        jsonschema.Draft202012Validator(schema).validate(document)
    except jsonschema.ValidationError as exc:
        path = ".".join(str(x) for x in exc.absolute_path) or "<root>"
        raise ValueError(f"{version} 校验失败 @{path}: {exc.message}") from exc
    _semantic_checks(document)


def _semantic_checks(document: dict) -> None:
    version = document["schema_version"]
    if version in ("tabletop_request.v1", "tabletop_plan.v1"):
        from frame_calibration.robot_side.execute_tabletop_pick_place_worlds import validate_startup_policy
        validate_startup_policy(document.get("startup_policy"))
    if version in ("vision_observation.v1", "task_request.v1", "tabletop_request.v1"):
        def check_quat(q, label):
            norm = math.sqrt(sum(float(v) ** 2 for v in q))
            if abs(norm - 1.0) > 1e-3:
                raise ValueError(f"{label} 四元数模长={norm:.6f}, 必须归一化")
        if version == "vision_observation.v1":
            for x in document["objects"] + document["obstacles"]:
                check_quat(x["pose"]["quaternion_xyzw"], x.get("object_id", x.get("obstacle_id")))
        elif version == "task_request.v1":
            for x in document["actions"]:
                check_quat(x["place"]["quaternion_xyzw"], x["action_id"])
        else:
            check_quat(document["observation"]["pose"]["quaternion_xyzw"],
                       "observation")
            station = document["station"]
            if "place" in station:
                check_quat(station["place"]["pose"]["quaternion_xyzw"],
                           "station.place")
            else:
                check_quat(station["material_box"]["pose"]["quaternion_xyzw"],
                           "station.material_box")
            check_quat(document["station"]["safe_endpoint"]["pose"]["quaternion_xyzw"],
                       "station.safe_endpoint")
            fixed = document["object"].get("fixed_endpoint_quaternion_xyzw")
            if fixed is not None:
                check_quat(fixed, "object.fixed_endpoint")
            from .grasp_pose import validate_rigid_transform
            validate_rigid_transform(document["object"]["T_object_grasp"],
                                     "object.T_object_grasp")
            if "material_box" in station:
                validate_rigid_transform(
                    station["material_box"]["T_box_place_grasp"],
                    "station.material_box.T_box_place_grasp")
            if "T_endpoint_grasp" in document["transforms"]:
                validate_rigid_transform(document["transforms"]["T_endpoint_grasp"],
                                         "transforms.T_endpoint_grasp")
    if version == "trajectory.v2":
        c = document["contract"]
        expected = {"left": (1, 2), "right": (2, 1)}[c["arm"]]
        if (c["arm_id"], c["gripper_id"]) != expected:
            raise ValueError(f"{c['arm']} 的 arm/gripper 映射必须为 {expected}")
    if version == "tabletop_plan.v1":
        if document.get("real_motion_authorized") is not False:
            raise ValueError("tabletop_plan.v1 只能是离线候选，禁止直接授权真机")
        blockers = document["safety"]["input_blockers"]
        expected_status = ("OFFLINE_CANDIDATE_BLOCKED" if blockers else
                           "OFFLINE_CANDIDATE_PRECHECK_REQUIRED")
        if document["status"] != expected_status:
            raise ValueError("tabletop plan 状态与 input_blockers 不一致")
        expected_gate = "BLOCKED" if blockers else "PASS"
        if document["safety"]["input_geometry_gate"] != expected_gate:
            raise ValueError("tabletop plan input_geometry_gate 与 blockers 不一致")
        for label in ("T_endpoint_grasp", "T_object_grasp",
                      "T_endpoint_object_at_pick"):
            from .grasp_pose import validate_rigid_transform
            validate_rigid_transform(document["transforms"][label], label)
        for stage, expected in zip(document["stages"], TABLETOP_STAGE_CONTRACT):
            name, kind, action, state_key, state_value = expected
            if (stage["name"] != name or stage["kind"] != kind or
                    (action is not None and stage.get("action") != action) or
                    stage.get(state_key) is not state_value):
                raise ValueError("tabletop_plan.v1 状态机动作或夹持状态不匹配")
        from .grasp_pose import quaternion_xyzw_to_matrix, sdk_uvw_deg_to_matrix
        import numpy as np
        poses = [item for item in document["resolved"].values()
                 if isinstance(item, dict) and "quaternion_xyzw" in item]
        poses.extend(stage["pose"] for stage in document["stages"]
                     if "pose" in stage)
        for item in poses:
            norm = math.sqrt(sum(float(v) ** 2 for v in item["quaternion_xyzw"]))
            if abs(norm - 1.0) > 1e-6:
                raise ValueError("tabletop pose 四元数未归一化")
            if not np.allclose(
                    quaternion_xyzw_to_matrix(item["quaternion_xyzw"]),
                    sdk_uvw_deg_to_matrix(item["sdk_world_uvw_deg"]),
                    atol=1e-8, rtol=0):
                raise ValueError("tabletop pose 的 quaternion 与 SDK UVW 不一致")
        for label in ("endpoint", "object"):
            matrix = document["transforms"][f"T_{label}_grasp"]
            actual_hash = sha256_bytes(canonical_bytes(matrix))
            if actual_hash != document["transforms"][f"T_{label}_grasp_sha256"]:
                raise ValueError(f"T_{label}_grasp SHA-256 不匹配")
        policy = dict(document["gripper_policy"])
        policy_hash = policy.pop("source_sha256")
        if sha256_bytes(canonical_bytes(policy)) != policy_hash:
            raise ValueError("gripper_policy SHA-256 不匹配")
        policy_id = policy["policy_id"]
        if policy["gripper_id"] != document["contract"]["gripper_id"]:
            raise ValueError("gripper_policy 的 gripper_id 与计划硬件映射不一致")
        if any(stage.get("policy_id") != policy_id
               for stage in document["stages"]
               if stage["kind"] == "GRIPPER"):
            raise ValueError("GRIPPER stage 的 policy_id 与 gripper_policy 不一致")
        from frame_calibration.robot_side.execute_tabletop_pick_place_worlds import validate_plan
        validate_plan(document)
    if version == "tabletop_scene_snapshot.v1":
        # 该类型只保存现场输入和完整性门禁，不是轨迹或真机授权。
        from .tabletop_scene_snapshot import validate_snapshot_semantics
        validate_snapshot_semantics(document)
    if version == "eye_to_hand_calibration.v1":
        import numpy as np
        a, b = np.asarray(document["T_sdk_world_camera"]), np.asarray(document["T_camera_sdk_world"])
        if float(np.max(np.abs(a @ b - np.eye(4)))) > 1e-6:
            raise ValueError("T_sdk_world_camera 与逆变换不互逆")


def load_and_validate(path: str | Path, expected: str | None = None) -> dict:
    value = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_document(value, expected)
    return value


def validate_task_links(task: dict, observation: dict, observation_path: str | Path) -> None:
    if task["observation"]["sha256"] != sha256_file(observation_path):
        raise ValueError("task_request 绑定的 observation SHA-256 不匹配")
    ids = {o["object_id"] for o in observation["objects"]}
    missing = [a["object_id"] for a in task["actions"] if a["object_id"] not in ids]
    if missing:
        raise ValueError(f"任务引用了 observation 中不存在的 object_id: {missing}")
