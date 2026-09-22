#!/usr/bin/env python3
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from vision_system.detector import CameraModel, ColorBoxDetector, rgb_to_hsv
from vision_system.temporal import TemporalGate
from vision_system.evidence import load_frame_bundle, save_frame_bundle


class VisionTests(unittest.TestCase):
    def setUp(self):
        self.camera = CameraModel(120, 100, 400, 400, 60, 50, "camera_color_optical_frame")
        self.catalog = {"schema_version": "object_catalog.v1", "morphology_radius_px": 1,
                        "objects": [{"class_id": "red_box", "dimensions_m": [.05, .05, .05],
                                     "hsv_ranges": [{"lower": [350, .5, .3],
                                                     "upper": [15, 1, 1]}]}]}
        self.detector = ColorBoxDetector(self.catalog, min_pixels=80)
        self.rgb = np.zeros((100, 120, 3), np.uint8)
        self.rgb[40:61, 50:71] = [255, 0, 0]
        self.depth = np.ones((100, 120), np.float32)

    def test_rgb_hsv_red_wrap(self):
        hsv = rgb_to_hsv(np.array([[[255, 0, 0]]], np.uint8))[0, 0]
        self.assertAlmostEqual(hsv[0], 0.0)
        self.assertAlmostEqual(hsv[1], 1.0)

    def test_detect_and_five_frame_gate(self):
        gate = TemporalGate()
        accepted = []
        for _ in range(5):
            rows, _ = self.detector.detect_frame(self.rgb, self.depth, self.camera)
            self.assertEqual(len(rows), 1)
            accepted = gate.add(rows)
        self.assertEqual(len(accepted), 1)
        self.assertGreaterEqual(accepted[0]["confidence"], .8)
        self.assertLess(max(accepted[0]["uncertainty_1sigma_m"]), .005)

    def test_invalid_depth_is_rejected_by_temporal_gate(self):
        depth = self.depth.copy()
        depth[40:51, 50:71] = np.nan
        gate = TemporalGate()
        accepted = []
        for _ in range(5):
            rows, _ = self.detector.detect_frame(self.rgb, depth, self.camera)
            accepted = gate.add(rows)
        self.assertEqual(accepted, [])

    def test_no_color_no_detection(self):
        rows, _ = self.detector.detect_frame(np.zeros_like(self.rgb), self.depth, self.camera)
        self.assertEqual(rows, [])

    def test_evidence_metadata_contains_final_hashes(self):
        with tempfile.TemporaryDirectory() as root:
            bundle = Path(root) / "frame"
            hashes = save_frame_bundle(bundle, self.rgb, self.depth,
                                       {"width": 120, "height": 100, "k": [1] * 9},
                                       {"ros_timestamp_ns": 123, "rgb_depth_delta_ms": 1})
            _rgb, _depth, _info, metadata = load_frame_bundle(bundle)
            self.assertEqual(metadata["evidence"], hashes)
            self.assertNotIn("metadata_sha256", hashes)


if __name__ == "__main__":
    unittest.main(verbosity=2)
