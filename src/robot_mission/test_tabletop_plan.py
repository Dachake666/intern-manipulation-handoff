#!/usr/bin/env python3
from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import unittest

import numpy as np

from robot_mission.contracts import validate_document
from robot_mission.grasp_pose import (
    POSE_ENDPOINT,
    POSE_GRASP,
    POSE_OBJECT,
    matrix_to_quaternion_xyzw,
    matrix_to_sdk_uvw_deg,
    pose_matrix,
    resolve_sdk_endpoint_pose,
    sdk_endpoint_to_grasp_transform,
    sdk_uvw_deg_to_matrix,
)
from robot_mission.tabletop_plan import build_tabletop_plan, build_working_reference_variant, verify_reported_baseline
from robot_mission.task_adapter import (
    _entity_pose_in_sdk_world, _oriented_aabb_dimensions)


def _quat_from_uvw(uvw):
    return [float(v) for v in matrix_to_quaternion_xyzw(
        sdk_uvw_deg_to_matrix(uvw))]


def _transform(position=(0, 0, 0), quaternion=(0, 0, 0, 1)):
    return pose_matrix(position, quaternion).tolist()


def request():
    fixed = _quat_from_uvw([-99.833, 26.249, -16.126])
    return {
        "schema_version": "tabletop_request.v1",
        "request_id": "tabletop-test-1",
        "arm": "left",
        "input_status": {"geometry_status": "CONFIRMED",
                         "blocking_assumptions": []},
        "observation": {
            "observation_id": "camera-obs-1",
            "frame_id": "sdk_world",
            "length_unit": "millimeter",
            "quaternion_order": "xyzw",
            "pose_role": "OBJECT_POSE",
            "pose": {"position_mm": [500, 300, 450],
                     "quaternion_xyzw": [0, 0, 0, 1]},
        },
        "object": {
            "object_id": "bottle-1",
            "class_id": "bottle-candidate",
            "dimensions_mm": [70, 70, 220],
            "dimensions_status": "CONFIRMED",
            "T_object_grasp": _transform((0, 0, 50)),
            "object_grasp_transform_status": "CONFIRMED",
            "orientation_policy": "FIXED_ENDPOINT_ORIENTATION",
            "fixed_endpoint_quaternion_xyzw": fixed,
        },
        "transforms": {},
        "station": {
            "station_id": "fixed-bin-1",
            "table_surface_z_mm": 210,
            "table_surface_status": "CONFIRMED",
            "bin_rim_z_mm": 300,
            "bin_rim_status": "CONFIRMED",
            "place": {
                "frame_id": "sdk_world", "length_unit": "millimeter",
                "quaternion_order": "xyzw", "pose_role": "GRASP_POSE",
                "pose": {"position_mm": [780, 100, 520],
                         "quaternion_xyzw": [0, 0, 0, 1]},
            },
            "safe_endpoint": {
                "frame_id": "sdk_world", "length_unit": "millimeter",
                "quaternion_order": "xyzw", "pose_role": "EE_POSE",
                "pose": {"position_mm": [700, 200, 750],
                         "quaternion_xyzw": fixed},
            },
        },
        "motion_policy": {"minimum_lift_mm": 120,
                          "pick_hover_offset_mm": 300,
                          "place_hover_offset_mm": 300,
                          "held_clearance_mm": 20},
        "gripper_policy": {
            "policy_id": "left-gripper2-site-tunable-v1",
            "gripper_id": 2,
            "tuning_status": "SITE_TUNABLE",
            "protocol_encoding": "EB90_UINT16_LITTLE_ENDIAN",
            "open_command": 17,
            "close_command": 16,
            "command_contract": "OPEN_POSITION_CLOSE_SPEED_FORCE",
            "value_unit": "device_native_uint16",
            "executor_binding": "execute_tabletop_pick_place_worlds.py/v1",
            "open": {"position": 500, "settle_s": 2.5},
            "close": {"speed": 500, "force": 1000, "settle_s": 5.0},
        },
    }


class TransformTests(unittest.TestCase):
    def test_current_left_endpoint_to_grasp_translation_is_in_endpoint_frame(self):
        T, evidence = sdk_endpoint_to_grasp_transform("left")
        self.assertTrue(np.allclose(T[:3, 3], [-18, 0, 39], atol=1e-9))
        self.assertIn("MEASURED", evidence["grip_center_status"])

    def test_object_local_offset_rotates_with_object(self):
        yaw90 = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        out = resolve_sdk_endpoint_pose(
            {"position_mm": [10, 20, 30], "quaternion_xyzw": yaw90},
            POSE_OBJECT,
            T_object_grasp=_transform((100, 0, 0)),
            T_endpoint_grasp=np.eye(4))
        self.assertTrue(np.allclose(out["pose"]["position_mm"], [10, 120, 30],
                                    atol=1e-9))

    def test_grasp_and_endpoint_roles_do_not_double_compensate(self):
        T_eg = np.asarray(_transform((10, 0, 0)))
        grasp = {"position_mm": [100, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
        got = resolve_sdk_endpoint_pose(grasp, POSE_GRASP,
                                        T_endpoint_grasp=T_eg)
        self.assertTrue(np.allclose(got["pose"]["position_mm"], [90, 0, 0]))
        with self.assertRaisesRegex(ValueError, "禁止再次"):
            resolve_sdk_endpoint_pose(grasp, POSE_GRASP,
                                      T_endpoint_grasp=T_eg,
                                      T_object_grasp=np.eye(4))
        direct = resolve_sdk_endpoint_pose(grasp, POSE_ENDPOINT)
        self.assertEqual(direct["pose"]["position_mm"], [100.0, 0.0, 0.0])

    def test_random_full_pose_closure(self):
        rng = np.random.default_rng(20260902)
        for _ in range(100):
            def q():
                value = rng.normal(size=4); value /= np.linalg.norm(value)
                return value
            T_og = pose_matrix(rng.uniform(-100, 100, 3), q())
            T_eg = pose_matrix(rng.uniform(-100, 100, 3), q())
            source = {"position_mm": rng.uniform(-1000, 1000, 3).tolist(),
                      "quaternion_xyzw": q().tolist()}
            out = resolve_sdk_endpoint_pose(
                source, POSE_OBJECT, T_object_grasp=T_og,
                T_endpoint_grasp=T_eg)
            expected_grasp = pose_matrix(source["position_mm"],
                                         source["quaternion_xyzw"]) @ T_og
            self.assertLess(float(np.max(np.abs(
                out["T_world_endpoint"] @ T_eg - expected_grasp))), 1e-8)

    def test_quaternion_sign_and_bad_norm(self):
        a = pose_matrix([0, 0, 0], [0.1, 0.2, 0.3, 0.9273618495])
        b = pose_matrix([0, 0, 0], [-0.1, -0.2, -0.3, -0.9273618495])
        self.assertTrue(np.allclose(a, b))
        with self.assertRaisesRegex(ValueError, "模长"):
            pose_matrix([0, 0, 0], [0, 0, 0, 0])

    def test_sdk_uvw_roundtrip_including_gimbal_lock(self):
        for uvw in ([0, 0, 0], [-99.833, 26.249, -16.126],
                    [0, 90, 30], [0, -90, -45], [30, 89.999, 179.9]):
            R = sdk_uvw_deg_to_matrix(uvw)
            got = matrix_to_sdk_uvw_deg(R)
            self.assertTrue(np.allclose(sdk_uvw_deg_to_matrix(got), R,
                                        atol=1e-8))


class AdapterTests(unittest.TestCase):
    def test_sdk_world_bypasses_extrinsics_and_preserves_quaternion(self):
        entity = {"frame_id": "sdk_world", "pose": {
            "position_m": [1, 2, 3], "quaternion_xyzw": [0, 0, 0, 1]}}
        absurd = np.eye(4); absurd[:3, 3] = [1000, 2000, 3000]
        p, q, route = _entity_pose_in_sdk_world(entity, absurd, "camera")
        self.assertTrue(np.allclose(p, [1, 2, 3]))
        self.assertEqual(q, [0.0, 0.0, 0.0, 1.0])
        self.assertEqual(route, "SDK_WORLD_BYPASS_EXTRINSICS")

    def test_camera_frame_applies_extrinsics_exactly_once(self):
        entity = {"frame_id": "camera", "pose": {
            "position_m": [1, 2, 3], "quaternion_xyzw": [0, 0, 0, 1]}}
        T = np.eye(4); T[:3, 3] = [10, 20, 30]
        p, _q, route = _entity_pose_in_sdk_world(entity, T, "camera")
        self.assertTrue(np.allclose(p, [11, 22, 33]))
        self.assertEqual(route, "CAMERA_TO_SDK_WORLD_ONCE")

    def test_obstacle_aabb_uses_entity_world_orientation(self):
        yaw90 = [0, 0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        got = _oriented_aabb_dimensions([1, 2, 3], yaw90)
        self.assertTrue(np.allclose(got, [2, 1, 3], atol=1e-9))


class PlanTests(unittest.TestCase):
    def test_plan_has_closed_state_machine_and_no_move_j(self):
        plan = build_tabletop_plan(request())
        validate_document(plan, "tabletop_plan.v1")
        self.assertFalse(plan["real_motion_authorized"])
        names = [x["name"] for x in plan["stages"]]
        self.assertEqual(names[4], "CLOSE_AT_PICK")
        self.assertEqual(names[8], "OPEN_AT_PLACE")
        self.assertNotIn("MOVE_J", [x["kind"] for x in plan["stages"]])
        self.assertTrue(all(not x["interpolation_en"] and x["wait_until_worlds"]
                            for x in plan["stages"] if x["kind"] == "MOVE_WORLDS"))

    def test_fixed_tool_freezes_actual_endpoint_object_attachment(self):
        value = request()
        value["observation"]["pose"]["quaternion_xyzw"] = _quat_from_uvw([25, 0, 0])
        plan = build_tabletop_plan(value)
        T_eo = np.asarray(plan["transforms"]["T_endpoint_object_at_pick"])
        place = plan["resolved"]["place_endpoint"]
        T_we = pose_matrix(place["position_mm"], place["quaternion_xyzw"])
        actual = plan["resolved"]["place_object_after_release"]
        T_wo = pose_matrix(actual["position_mm"], actual["quaternion_xyzw"])
        self.assertTrue(np.allclose(T_we @ T_eo, T_wo, atol=1e-8))

    def test_gripper_policy_is_bound_without_changing_motion_geometry(self):
        source = request()
        first = build_tabletop_plan(source)
        source["gripper_policy"]["close"]["force"] = 700
        self.assertEqual(first["gripper_policy"]["close"]["force"], 1000)
        changed = request()
        changed["gripper_policy"]["close"]["force"] = 800
        second = build_tabletop_plan(changed)
        self.assertNotEqual(first["plan_id"], second["plan_id"])
        self.assertNotEqual(first["gripper_policy"]["source_sha256"],
                            second["gripper_policy"]["source_sha256"])
        self.assertEqual(first["resolved"], second["resolved"])
        first_motion = [x for x in first["stages"] if "pose" in x]
        second_motion = [x for x in second["stages"] if "pose" in x]
        self.assertEqual(first_motion, second_motion)
        self.assertTrue(all(
            x["policy_id"] == first["gripper_policy"]["policy_id"]
            for x in first["stages"] if x["kind"] == "GRIPPER"))

    def test_gripper_policy_rejects_out_of_range_or_tampered_binding(self):
        bad = request()
        bad["gripper_policy"]["close"]["force"] = 65536
        with self.assertRaisesRegex(ValueError, "65535|maximum"):
            build_tabletop_plan(bad)
        plan = build_tabletop_plan(request())
        plan["stages"][1]["policy_id"] = "different-policy"
        with self.assertRaisesRegex(ValueError, "policy_id"):
            validate_document(plan, "tabletop_plan.v1")
        plan = build_tabletop_plan(request())
        plan["gripper_policy"]["close"]["force"] = 900
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            validate_document(plan, "tabletop_plan.v1")

    def test_gripper_action_semantics_and_hardware_binding_are_locked(self):
        bad = request()
        bad["gripper_policy"]["gripper_id"] = 1
        with self.assertRaisesRegex(ValueError, "2 was expected|2"):
            build_tabletop_plan(bad)
        plan = build_tabletop_plan(request())
        plan["stages"][1]["action"] = "close"
        with self.assertRaisesRegex(ValueError, "状态机动作"):
            validate_document(plan, "tabletop_plan.v1")
        plan = build_tabletop_plan(request())
        plan["stages"][4]["carrying_after"] = False
        with self.assertRaisesRegex(ValueError, "夹持状态"):
            validate_document(plan, "tabletop_plan.v1")

    def test_incompatible_fixed_tool_object_place_pose_is_rejected(self):
        value = request()
        value["observation"]["pose"]["quaternion_xyzw"] = _quat_from_uvw([25, 0, 0])
        value["station"]["place"]["pose_role"] = "OBJECT_POSE"
        with self.assertRaisesRegex(ValueError, "无法实现"):
            build_tabletop_plan(value)

    def test_new_camera_pose_only_changes_pick_side(self):
        first = build_tabletop_plan(request())
        changed_request = copy.deepcopy(request())
        changed_request["observation"]["pose"]["position_mm"][0] += 25
        second = build_tabletop_plan(changed_request)
        self.assertNotEqual(first["resolved"]["pick_endpoint"],
                            second["resolved"]["pick_endpoint"])
        self.assertEqual(first["resolved"]["place_endpoint"],
                         second["resolved"]["place_endpoint"])
        by_name_a = {x["name"]: x for x in first["stages"]}
        by_name_b = {x["name"]: x for x in second["stages"]}
        for name in ("PLACE_HOVER", "PLACE_DESCEND", "PLACE_ASCEND",
                     "RETURN_SAFE"):
            self.assertEqual(by_name_a[name], by_name_b[name])

    def test_hover_clearance_too_low_is_rejected(self):
        bad = request()
        bad["station"]["bin_rim_z_mm"] = 450
        bad["motion_policy"]["pick_hover_offset_mm"] = 120
        with self.assertRaisesRegex(ValueError, "过低|余量"):
            build_tabletop_plan(bad)

    def test_safe_is_only_start_end_and_hover_heights_are_independent(self):
        value = request()
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["resolved"]["safe_endpoint"]["position_mm"][2], 750)
        self.assertNotEqual(plan["resolved"]["pick_hover_endpoint"]["position_mm"][2],
                            750)
        by_name = {stage["name"]: stage for stage in plan["stages"]}
        self.assertEqual(by_name["PICK_HOVER"]["pose"],
                         by_name["PICK_ASCEND"]["pose"])
        self.assertEqual(by_name["PLACE_HOVER"]["pose"],
                         by_name["PLACE_ASCEND"]["pose"])

    def test_material_box_pose_and_local_offset_resolve_place(self):
        value = request()
        value["station"].pop("place")
        value["station"]["material_box"] = {
            "frame_id": "sdk_world", "length_unit": "millimeter",
            "quaternion_order": "xyzw",
            "pose": {"position_mm": [800, 100, 500],
                     "quaternion_xyzw": [0, 0, 0, 1]},
            "dimensions_mm": [310, 390, 130],
            "inner_dimensions_mm": [290, 370, 120],
            "geometry_status": "CONFIRMED",
            "datum_role": "BOX_GEOMETRY_CENTER",
            "T_box_place_grasp": _transform((-25, 50, 20)),
            "placement_wall_margin_mm": 20,
        }
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["input"]["place_source_kind"],
                         "MATERIAL_BOX_LOCAL_OFFSET")
        grasp = plan["resolved"]["place_object_after_release"]["position_mm"]
        self.assertTrue(np.allclose(grasp, [775, 150, 470], atol=1e-8))

    def test_blocking_assumptions_make_plan_non_runnable_candidate(self):
        value = request()
        value["input_status"] = {
            "geometry_status": "SIMULATION_ASSUMPTION",
            "blocking_assumptions": ["物体 feature 到抓点尚未测量"],
        }
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["status"], "OFFLINE_CANDIDATE_BLOCKED")
        self.assertEqual(plan["safety"]["input_geometry_gate"], "BLOCKED")

    def test_unknown_frame_and_implicit_pose_role_are_rejected(self):
        bad = request(); bad["observation"]["frame_id"] = "camera"
        with self.assertRaisesRegex(ValueError, "sdk_world"):
            build_tabletop_plan(bad)
        bad = request(); bad["observation"].pop("pose_role")
        with self.assertRaisesRegex(ValueError, "pose_role"):
            build_tabletop_plan(bad)

    def test_quaternion_uvw_disagreement_is_rejected_on_load(self):
        plan = build_tabletop_plan(request())
        plan["stages"][2]["pose"]["sdk_world_uvw_deg"][0] += 5
        with self.assertRaisesRegex(ValueError, "不一致"):
            validate_document(plan, "tabletop_plan.v1")

    def _clearance_request(self):
        # 合成几何单元测试，不是当前现场/瓶子的测量数据。
        value = request()
        value["object"].update({
            "dimensions_mm": [20, 20, 100],
            "T_object_grasp": _transform(),
            "T_object_geometry": _transform(),
            "object_geometry_transform_status": "CONFIRMED",
            "fixed_endpoint_quaternion_xyzw": [0, 0, 0, 1]})
        value["transforms"] = {"T_endpoint_grasp": _transform(),
                                "endpoint_transform_source": {"status": "CONFIRMED"}}
        value["observation"]["pose"]["position_mm"] = [700, 250, 360]
        value["station"]["safe_endpoint"]["pose"]["quaternion_xyzw"] = [0, 0, 0, 1]
        value["station"].pop("place")
        value["station"].update({"table_surface_z_mm": 300, "bin_rim_z_mm": 450})
        value["station"]["material_box"] = {
            "frame_id": "sdk_world", "length_unit": "millimeter",
            "quaternion_order": "xyzw",
            "pose": {"position_mm": [720, 100, 385], "quaternion_xyzw": [0, 0, 0, 1]},
            "dimensions_mm": [310, 390, 130], "inner_dimensions_mm": [290, 370, 120],
            "geometry_status": "CONFIRMED", "datum_role": "BOX_GEOMETRY_CENTER",
            "T_box_place_grasp": _transform((0, 0, -15)), "placement_wall_margin_mm": 20}
        value["motion_policy"].update({"pick_hover_offset_mm": 180,
            "placement_clearance": {
                "strategy": "KEEP_ENVELOPE_ABOVE_RIM",
                "envelope_points_endpoint_mm": [[-10, -10, -35], [10, 10, 20]],
                "envelope_status": "CONFIRMED", "envelope_source": "SYNTHETIC_TEST_ONLY",
                "covers_wrist_and_open_gripper": True,
                "rim_clearance_mm": 10, "hover_z_mm": 550, "minimum_hover_gap_mm": 30,
                "support_z_mm": 330, "support_status": "CONFIRMED",
                "release_mode": "ALLOW_BOUNDED_DROP", "max_drop_mm": 120,
                "support_tolerance_mm": 1}})
        return value

    def test_missing_wrist_clearance_is_blocked_not_silent_pass(self):
        plan = build_tabletop_plan(request())
        self.assertEqual(plan["status"], "OFFLINE_CANDIDATE_BLOCKED")
        self.assertEqual(plan["safety"]["placement_clearance"]["status"], "MISSING")

    def test_release_lifts_without_adding_180_to_place_hover(self):
        plan = build_tabletop_plan(self._clearance_request())
        self.assertEqual(plan["resolved"]["place_endpoint"]["position_mm"], [720, 100, 495])
        self.assertEqual(plan["resolved"]["place_hover_endpoint"]["position_mm"], [720, 100, 550])
        self.assertEqual(plan["safety"]["placement_clearance"]["release_gap_mm"], 115)
        self.assertEqual(plan["status"], "OFFLINE_CANDIDATE_PRECHECK_REQUIRED")
        self.assertFalse(plan["real_motion_authorized"])

    def test_raising_release_cannot_silently_turn_place_into_drop(self):
        value = self._clearance_request()
        value["motion_policy"]["placement_clearance"]["release_mode"] = "SUPPORTED_ONLY"
        value["motion_policy"]["placement_clearance"]["max_drop_mm"] = 0
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["status"], "OFFLINE_CANDIDATE_BLOCKED")

    def test_feature_to_geometry_offset_changes_release_gap(self):
        value = self._clearance_request()
        value["object"]["T_object_geometry"] = _transform((0, 0, -10))
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["safety"]["placement_clearance"]["release_gap_mm"], 105)

    def test_grasp_inside_bin_but_bottle_outside_is_blocked(self):
        value = self._clearance_request()
        value["station"]["material_box"]["T_box_place_grasp"] = _transform((120, 0, -15))
        plan = build_tabletop_plan(value)
        self.assertFalse(plan["safety"]["placement_clearance"]["object_xy_inside_bin"])
        self.assertEqual(plan["status"], "OFFLINE_CANDIDATE_BLOCKED")

    def test_unknown_wrist_or_geometry_is_blocked(self):
        for field in ("envelope_status", "covers_wrist_and_open_gripper"):
            value = self._clearance_request()
            value["motion_policy"]["placement_clearance"][field] = (
                "SIMULATION_ASSUMPTION" if field == "envelope_status" else False)
            self.assertEqual(build_tabletop_plan(value)["status"], "OFFLINE_CANDIDATE_BLOCKED")
        value = self._clearance_request()
        value["object"].pop("T_object_geometry")
        self.assertEqual(build_tabletop_plan(value)["status"], "OFFLINE_CANDIDATE_BLOCKED")

    def test_mismatched_bin_floor_is_blocked(self):
        value = self._clearance_request()
        value["motion_policy"]["placement_clearance"]["support_z_mm"] += 5
        self.assertEqual(build_tabletop_plan(value)["status"], "OFFLINE_CANDIDATE_BLOCKED")

    def test_saved_interior_region_does_not_require_precise_bin_center(self):
        value = self._clearance_request()
        value["station"].pop("material_box")
        value["station"]["place"] = {
            "frame_id": "sdk_world", "length_unit": "millimeter",
            "quaternion_order": "xyzw", "pose_role": "EE_POSE",
            "pose": {"position_mm": [720, 100, 495], "quaternion_xyzw": [0, 0, 0, 1]}}
        value["station"]["place_region"] = {
            "x_bounds_mm": [690, 750], "y_bounds_mm": [70, 130],
            "geometry_status": "CONFIRMED", "source": "SYNTHETIC_TEST_ONLY"}
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["status"], "OFFLINE_CANDIDATE_PRECHECK_REQUIRED")
        self.assertEqual(plan["resolved"]["place_endpoint"]["position_mm"], [720, 100, 495])
        value["station"]["place_region"]["x_bounds_mm"] = [710, 711]
        self.assertEqual(build_tabletop_plan(value)["status"], "OFFLINE_CANDIDATE_BLOCKED")

    def test_region_cannot_override_a_known_box_model(self):
        value = self._clearance_request()
        value["station"]["place_region"] = {
            "x_bounds_mm": [0, 2000], "y_bounds_mm": [-1000, 1000],
            "geometry_status": "CONFIRMED", "source": "SYNTHETIC_TEST_ONLY"}
        with self.assertRaises(ValueError):
            build_tabletop_plan(value)

    def test_unconfirmed_custom_tcp_cannot_clear_input_gate(self):
        value = self._clearance_request()
        value["transforms"]["endpoint_transform_source"]["status"] = "SIMULATION_ASSUMPTION"
        self.assertEqual(build_tabletop_plan(value)["status"], "OFFLINE_CANDIDATE_BLOCKED")

    def test_horizontal_generator_locks_y_z_and_keeps_independent_vertical_lift(self):
        value = self._clearance_request()
        value["motion_policy"].update(pick_mode="HORIZONTAL_X_POSITIVE",
            approach_distance_mm=150, pick_ascend_offset_mm=200)
        plan = build_tabletop_plan(value)
        self.assertEqual(plan["resolved"]["pick_hover_endpoint"]["position_mm"], [550,250,360])
        self.assertEqual(plan["resolved"]["pick_ascend_endpoint"]["position_mm"], [700,250,560])
        self.assertEqual(plan["stages"][5]["pose"], plan["resolved"]["pick_ascend_endpoint"])
        validate_document(plan, "tabletop_plan.v1")

    def test_horizontal_generator_does_not_assume_70mm_clears_rim(self):
        value = self._clearance_request()
        value["motion_policy"].update(pick_mode="HORIZONTAL_X_POSITIVE",
            approach_distance_mm=150, pick_ascend_offset_mm=70)
        with self.assertRaisesRegex(ValueError, "过低"):
            build_tabletop_plan(value)

    def test_new_startup_policy_survives_generator_and_is_not_mutable_alias(self):
        from frame_calibration.robot_side.test_execute_tabletop_pick_place_worlds import recovery_plan
        value = self._clearance_request()
        value["startup_policy"] = recovery_plan()["startup_policy"]
        plan = build_tabletop_plan(value)
        value["startup_policy"]["clearance_z_mm"] += 1
        self.assertNotEqual(plan["startup_policy"], value["startup_policy"])

    def test_reported_baseline_reconstruction_and_candidate_do_not_fake_geometry(self):
        root = Path(__file__).resolve().parents[1]
        source = root / "frame_calibration/records/20260903_tabletop_worlds_working/artifacts/tabletop_pick_place_worlds_PASS_placeZ495_20260903.json"
        recipe = json.loads((root / "pick_place_coord/tasks/tabletop_horizontal_recovery_20260905.json").read_text())
        reference = json.loads(source.read_text())
        before = copy.deepcopy(reference)
        self.assertEqual(verify_reported_baseline(source, recipe)["plan_sha256"],
                         "8897c932aea102532c0b77c8492ad5c6dd35e6875ec28eafff3558a81ee6f4b3")
        result = build_working_reference_variant(reference, recipe,
            reference_sha256=hashlib.sha256(source.read_bytes()).hexdigest())
        self.assertEqual(reference, before)
        self.assertEqual(result["resolved"]["pick_hover_endpoint"]["position_mm"], [550,250,410])
        self.assertEqual(result["resolved"]["pick_endpoint"]["position_mm"], [700,250,410])
        self.assertEqual(result["resolved"]["pick_ascend_endpoint"]["position_mm"], [700,250,480])
        self.assertEqual(result["resolved"]["place_endpoint"]["position_mm"], [720,100,445])
        self.assertEqual(result["resolved"]["safe_endpoint"], before["resolved"]["safe_endpoint"])
        self.assertEqual(result["gripper_policy"], before["gripper_policy"])
        self.assertIsNone(result["resolved"]["place_object_after_release"])
        self.assertEqual(result["safety"]["held_object_hover_endpoint_check"], "PENDING")
        self.assertEqual(result["status"], "OFFLINE_CANDIDATE_BLOCKED")
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            build_working_reference_variant(reference, recipe, reference_sha256="0"*64)


if __name__ == "__main__":
    unittest.main(verbosity=2)
