#!/usr/bin/env python3
from __future__ import annotations

import copy
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

WORK = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(WORK))

import arm_profiles
from robot_mission.arm_selector import ArmCandidate, rank_candidates
from robot_mission.contracts import validate_document
from robot_mission.preflight import expanded_frames, load_trajectory
from robot_mission.scene_guard import compare_observations
from robot_mission.grasp_pose import local_grasp_axes, top_grasp_rotations


def observation():
    return {"schema_version": "vision_observation.v1", "observation_id": "obs-1",
            "camera": {"camera_id": "cam", "ros_timestamp_ns": 1,
                       "frame_id": "cam_frame", "rgb_depth_delta_ms": 10},
            "calibration": {"intrinsics_id": "i1", "extrinsics_id": "e1",
                            "extrinsics_direction": "T_sdk_world_camera", "length_unit": "meter"},
            "evidence": {"rgb_sha256": "a" * 64, "depth_sha256": "b" * 64,
                         "camera_info_sha256": "c" * 64,
                         "supporting_frames": [
                             {"rgb_sha256": "a" * 64, "depth_sha256": "b" * 64,
                              "camera_info_sha256": "c" * 64} for _ in range(3)]},
            "objects": [{"object_id": "red-1", "class_id": "red", "frame_id": "cam_frame",
                         "pose": {"position_m": [.1, .2, 1], "quaternion_xyzw": [0, 0, 0, 1]},
                         "dimensions_m": [.05, .05, .05], "confidence": .9,
                         "uncertainty_1sigma_m": [.001, .001, .002],
                         "depth_valid_ratio": .9, "support_frames": 5, "yaw_std_deg": 1}],
            "obstacles": []}


class ContractTests(unittest.TestCase):
    def test_observation_schema_and_quaternion(self):
        value = observation(); validate_document(value, "vision_observation.v1")
        value["objects"][0]["pose"]["quaternion_xyzw"] = [0, 0, 0, 2]
        with self.assertRaisesRegex(ValueError, "四元数"):
            validate_document(value, "vision_observation.v1")

    def test_right_hardware_gate_is_explicit(self):
        with self.assertRaisesRegex(RuntimeError, "BLOCKED"):
            arm_profiles.require_hardware_ready(arm_profiles.arm_profile("right"))


class SelectionTests(unittest.TestCase):
    def test_safety_ranking_precedes_speed(self):
        left = ArmCandidate("left", True, True, True, True, 10, .03, 12, 4, True)
        right = ArmCandidate("right", True, True, True, True, 8, .04, 8, 2, False)
        self.assertEqual(rank_candidates([right, left]).arm, "left")
        self.assertEqual(rank_candidates([right, left], require_hardware_ready=True).arm, "left")

    def test_rejected_candidate_never_wins(self):
        bad = ArmCandidate("left", True, True, True, False, 30, .2, 1, 1, True)
        good = ArmCandidate("right", True, True, True, True, 6, .01, 20, 8, False)
        self.assertEqual(rank_candidates([bad, good]).arm, "right")

    def test_two_full_pose_top_grasps_map_tool_down(self):
        profile = arm_profiles.arm_profile("left")
        jaw, _, tool = local_grasp_axes(profile)
        rotations = top_grasp_rotations(profile, .3)
        self.assertEqual(len(rotations), 2)
        for R in rotations:
            self.assertLess(max(abs(R @ tool - [0, 0, -1])), 1e-9)
        self.assertLess(max(abs(rotations[0] @ jaw + rotations[1] @ jaw)), 1e-9)


class SceneTests(unittest.TestCase):
    def test_new_obstacle_invalidates(self):
        before, after = observation(), observation()
        after["obstacles"].append({"obstacle_id": "new", "frame_id": "cam_frame",
            "geometry": "box", "pose": {"position_m": [0, 0, 1], "quaternion_xyzw": [0,0,0,1]},
            "dimensions_m": [.1,.1,.1], "confidence": .9,
            "uncertainty_1sigma_m": [.001,.001,.001]})
        got = compare_observations(before, after)
        self.assertEqual(got["verdict"], "INVALIDATED")

    def test_sub_threshold_jitter_passes(self):
        before, after = observation(), observation()
        after["objects"][0]["pose"]["position_m"][0] += .005
        self.assertEqual(compare_observations(before, after)["verdict"], "PASS")


class TrackTests(unittest.TestCase):
    def test_frozen_track_expands_to_1633_frames(self):
        path = WORK / "pick_place_coord/trajectories/verified/traj_multi_2grasp_20260804_REALVERIFIED.json"
        _, waypoints, _ = load_trajectory(path)
        first, dense = expanded_frames(waypoints, [10,10,0,-15,0,0,0], .4)
        self.assertEqual(len(first), 1)
        self.assertEqual(len(dense), 1633)
        self.assertTrue(any(x["carrying"] for x in dense))


if __name__ == "__main__":
    unittest.main(verbosity=2)
