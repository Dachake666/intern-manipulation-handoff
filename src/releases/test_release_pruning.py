"""交接精简后的发布闭包：退役先拒绝，冻结输入不被 latest 替换。"""
from __future__ import annotations

import hashlib
import importlib.util
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

WORK = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("pruned_release", WORK / "releases/make_release.py")
release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release)


class ReleasePruningTests(unittest.TestCase):
    def test_retired_modes_refuse_before_touching_existing_output(self):
        with tempfile.TemporaryDirectory(prefix=".pruned-release-test-", dir=WORK) as folder:
            root = Path(folder)
            marker = root / "keep.txt"
            marker.write_bytes(b"existing release\n")
            with patch.object(release, "_HERE", str(root)):
                for key in release.RETIRED_TRACKS:
                    with self.subTest(key=key), self.assertRaisesRegex(SystemExit, "PRE_PRUNE"):
                        release.build(key)
            self.assertEqual(list(root.iterdir()), [marker])
            self.assertEqual(marker.read_bytes(), b"existing release\n")

    def test_mixed_request_rejects_before_building_any_current_mode(self):
        with patch.object(release, "build") as build:
            with self.assertRaises(SystemExit):
                release.main(["C", "AH"])
            build.assert_not_called()

    def test_track_c_packages_the_immutable_input_and_not_the_latest_alias(self):
        with tempfile.TemporaryDirectory(prefix=".pruned-release-test-", dir=WORK) as folder:
            with patch.object(release, "_HERE", folder):
                out = Path(release.build("C"))
            frozen = out / release.TRACK_C_FROZEN
            self.assertEqual(hashlib.sha256(frozen.read_bytes()).hexdigest(),
                             "ac5c6fd7715c81fbfa2f2fc4c9eca1f676b5a3232dff99761a02c7b4812e4968")
            self.assertFalse((out / "traj_multi_latest.json").exists())
            self.assertFalse((out / "diag_chassis.py").exists())
            text = (out / "README.md").read_text()
            self.assertIn("LIMITED", text)
            self.assertIn("不代表当前组合获准运动", text)

    def test_track_b_keeps_only_the_eight_point_input_with_limited_evidence(self):
        with tempfile.TemporaryDirectory(prefix=".pruned-release-test-", dir=WORK) as folder:
            with patch.object(release, "_HERE", folder):
                out = Path(release.build("B"))
            self.assertTrue((out / "traj_minimal_joint.json").is_file())
            self.assertFalse((out / "traj_4pt.json").exists())
            self.assertFalse((out / "traj_5pt.json").exists())
            self.assertIn("LIMITED", (out / "README.md").read_text())


if __name__ == "__main__":
    unittest.main()
