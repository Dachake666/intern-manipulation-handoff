"""新规划只写候选；显式路径或软链接都不能覆盖冻结输入。"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

WORK = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("planner_export_test", WORK / "pick_place_coord/pick_place_coord.py")
planner = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(planner)


class ExportPathTests(unittest.TestCase):
    def test_frozen_directory_alias_and_symlink_cannot_be_overwritten(self):
        with tempfile.TemporaryDirectory(prefix=".planner-export-test-", dir=WORK) as folder:
            root = Path(folder)
            frozen_dir = root / "trajectories/verified"
            frozen_dir.mkdir(parents=True)
            frozen = frozen_dir / "baseline.json"
            frozen.write_bytes(b"frozen baseline\n")
            legacy = root / "trajectories/traj_multi_latest.json"
            legacy.write_bytes(b"historical alias\n")
            link = root / "candidate-link.json"
            link.symlink_to(frozen)
            with patch.object(planner, "_HERE", str(root)):
                for path in (frozen, legacy, link):
                    with self.subTest(path=path), self.assertRaisesRegex(ValueError, "拒绝覆盖"):
                        planner.Traj().export(path, {})
            self.assertEqual(frozen.read_bytes(), b"frozen baseline\n")
            self.assertEqual(legacy.read_bytes(), b"historical alias\n")

    def test_candidate_output_stays_separate_from_frozen_input(self):
        with tempfile.TemporaryDirectory(prefix=".planner-export-test-", dir=WORK) as folder:
            root = Path(folder)
            candidate = root / "trajectories/candidates/new-plan.json"
            with patch.object(planner, "_HERE", str(root)):
                planner.Traj().export(candidate, {"real_motion_authorized": False})
            data = json.loads(candidate.read_text())
            self.assertFalse(data["meta"]["real_motion_authorized"])
            self.assertFalse((root / "trajectories/verified").exists())


if __name__ == "__main__":
    unittest.main()
