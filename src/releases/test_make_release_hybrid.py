"""[20260908] 混合发布包回归：不接SDK，不触碰现有发布目录。"""
from __future__ import annotations

import hashlib
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import subprocess
import sys
import unittest
from unittest.mock import patch
import zipfile


WORK = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("hybrid_release_builder", WORK / "releases/make_release.py")
release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release)


class HybridReleaseTests(unittest.TestCase):
    def test_servo_does_not_replace_field_package_or_mix_executors(self):
        spec = release.TRACKS["AS"]
        names = [name for _, name in spec["files"] + release.COMMON + spec["tools"]]
        self.assertEqual(len(names), 12)
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("execute_tabletop_servo.py", names)
        self.assertNotIn("execute_servo_grasp.py", names)
        self.assertNotIn("tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json", names)
        self.assertNotEqual(spec["dir"], release.TRACKS["AT"]["dir"])
        self.assertNotIn("AS", release.SUPERSEDED_RELEASES)

    def test_servo_release_dry_run_without_repository_or_site_packages(self):
        with tempfile.TemporaryDirectory(prefix=".servo-release-test-", dir=WORK) as folder:
            root = Path(folder) / "standalone" / "deep"
            root.mkdir(parents=True)
            with patch.object(release, "_HERE", str(root)):
                out = Path(release.build("AS"))
            env = dict(os.environ)
            env.pop("PYTHONPATH", None)
            env.pop("XIFENG_ARM_PROFILES", None)
            result = subprocess.run([sys.executable, "-B", "-S", "execute_tabletop_servo.py",
                                     "tabletop_pick_place_SERVO_CANDIDATE.json", "--dry-run"],
                                    cwd=out, env=env, capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stdout+result.stderr)
            self.assertIn("FILE_CHECKS_PASS", result.stdout)
            self.assertIn("2484", result.stdout)
            self.assertIn("MOTION_BLOCKED", (out / "README.md").read_text())

    def test_canonical_package_has_only_received_field_runtime(self):
        spec = release.TRACKS["AT"]
        names = [name for _, name in spec["files"] + release.COMMON + spec["tools"]]
        self.assertEqual(len(names), len(set(names)))
        self.assertIn("execute_tabletop_hybrid_trial_reviewfix_field.py", names)
        self.assertIn("precheck_tabletop_hybrid.py", names)
        self.assertIn("tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json", names)
        self.assertNotIn("tabletop_pick_place_hybrid_CANDIDATE.json", names)
        self.assertNotIn("tabletop_pick_place_hybrid_OPTIMIZED.json", names)
        self.assertNotIn("gui_review_pair", spec)
        self.assertEqual(len(names), 7)
        self.assertIn("scrub_log.py", names)
        self.assertTrue(spec["zip_archive"])
        self.assertNotIn("AT", release.SUPERSEDED_RELEASES)
        self.assertNotEqual(spec["dir"], release.TRACKS["A"]["dir"])
        self.assertNotEqual(spec["dir"], release.TRACKS["AP"]["dir"])

    def test_readme_records_field_success_without_generalizing(self):
        spec = release.TRACKS["AT"]
        text = release.render_readme("AT", spec, [n for _, n in spec["files"]], [])
        self.assertIn("--precheck-only", text)
        self.assertIn("--dry-run", text)
        self.assertIn("同一机器人/工具/场景", text)
        self.assertIn("tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json --run", text)
        self.assertIn("没有20%运行证据", text)
        self.assertIn("失败运行不保存run JSON", text)
        self.assertNotIn("tabletop_pick_place_hybrid_OPTIMIZED.json --run", text)
        self.assertIn("python3 releases/make_release.py AT", text)
        self.assertIn("不重建、删除或覆盖A/AP", text)
        self.assertIn("CURRENT_DEPENDENCIES_UNVERIFIED", text)
        self.assertIn("不继承B2整套实跑身份", text)
        self.assertIn("本独立发布目录不附带完整历史日志", text)
        self.assertNotIn("10%也有本组合的实跑证据", text)
        self.assertNotIn("frame_calibration/records/20260908_trackA_hybrid_final/", text)
        self.assertNotIn("as_run清单和审查说明", text)

    def test_readme_gui_pass_is_bound_to_exact_current_plan(self):
        with tempfile.TemporaryDirectory(prefix=".hybrid-release-test-", dir=WORK) as folder:
            root = Path(folder)
            plan = root / "tabletop_pick_place_hybrid_OPTIMIZED.json"
            plan.write_bytes(b"{}\n")
            review = {"schema_version": "pybullet_gui_review.v1", "result": "PASS",
                      "trajectory_sha256": hashlib.sha256(plan.read_bytes()).hexdigest(),
                      "full_replay_completed": True}
            (root / "tabletop_pick_place_hybrid_OPTIMIZED_gui_review.json").write_text(
                json.dumps(review), encoding="utf-8")
            with patch.object(release, "CANDIDATES", str(root)):
                text = release.render_hybrid_trial_readme(release.TRACKS["AT"], [], [])
                self.assertIn("绑定轨迹SHA的完整GUI人工PASS记录", text)
                self.assertIn("不解除物理场景/碰撞资格限制", text)
                plan.write_bytes(b'{"changed": true}\n')
                text = release.render_hybrid_trial_readme(release.TRACKS["AT"], [], [])
                self.assertIn("GUI人工审核尚未匹配", text)
                self.assertNotIn("绑定轨迹SHA的完整GUI人工PASS记录", text)

    def test_bad_reference_keeps_existing_package(self):
        with tempfile.TemporaryDirectory(prefix=".hybrid-release-test-", dir=WORK) as folder:
            root = Path(folder)
            source = root / "source"
            source.mkdir()
            (source / "reference.json").write_text("changed", encoding="utf-8")
            out = root / "trial"
            out.mkdir()
            marker = out / "keep.txt"
            marker.write_text("original", encoding="utf-8")
            spec = {"dir": "trial", "files": [], "minimal_release": True,
                    "immutable_sources": [(str(source), "reference.json", "0" * 64)]}
            with patch.object(release, "_HERE", str(root)), patch.object(release, "COMMON", []), \
                    patch.object(release, "TRACKS", {"TEST": spec}):
                with self.assertRaisesRegex(SystemExit, "实跑参考哈希不符"):
                    release.build("TEST")
            self.assertEqual(marker.read_text(), "original")

    def test_only_matching_gui_review_is_packaged(self):
        with tempfile.TemporaryDirectory(prefix=".hybrid-release-test-", dir=WORK) as folder:
            root = Path(folder)
            source = root / "source"
            source.mkdir()
            plan = source / "plan.json"
            plan.write_bytes(b"{}\n")
            review = {"schema_version": "pybullet_gui_review.v1", "result": "PASS",
                      "trajectory_sha256": hashlib.sha256(plan.read_bytes()).hexdigest(),
                      "full_replay_completed": True}
            review_path = source / "review.json"
            review_path.write_text(json.dumps(review), encoding="utf-8")
            spec = {"dir": "trial", "files": [(str(source), "plan.json")],
                    "gui_review_pair": ((str(source), "plan.json"), (str(source), "review.json")),
                    "minimal_release": True, "zip_archive": True}
            with patch.object(release, "_HERE", str(root)), patch.object(release, "COMMON", []), \
                    patch.object(release, "TRACKS", {"TEST": spec}), \
                    patch.object(release, "render_readme", return_value="test package\n"):
                out = Path(release.build("TEST"))
                self.assertTrue((out / "review.json").is_file())
                plan.write_bytes(b'{"changed": true}\n')
                release.build("TEST")
                self.assertFalse((out / "review.json").exists())
                with zipfile.ZipFile(root / "trial.zip") as archive:
                    self.assertNotIn("trial/review.json", archive.namelist())
                self.assertTrue(review_path.is_file())  # 原审核证据不删除。

    def test_zip_and_checksums_cover_exact_runtime_files(self):
        with tempfile.TemporaryDirectory(prefix=".hybrid-release-test-", dir=WORK) as folder:
            root = Path(folder)
            source = root / "source"
            source.mkdir()
            contents = {"executor.py": b"# fixture\n", "reference.json": b"{}\n"}
            for name, value in contents.items():
                (source / name).write_bytes(value)
            expected = hashlib.sha256(contents["reference.json"]).hexdigest()
            spec = {"dir": "trial", "files": [(str(source), n) for n in contents],
                    "minimal_release": True, "zip_archive": True,
                    "immutable_sources": [(str(source), "reference.json", expected)]}
            with patch.object(release, "_HERE", str(root)), patch.object(release, "COMMON", []), \
                    patch.object(release, "TRACKS", {"TEST": spec}), \
                    patch.object(release, "render_readme", return_value="test package\n"):
                out = Path(release.build("TEST"))
            for line in (out / "SHA256SUMS").read_text().splitlines():
                digest, name = line.split("  ", 1)
                self.assertEqual(hashlib.sha256((out / name).read_bytes()).hexdigest(), digest)
            with zipfile.ZipFile(root / "trial.zip") as archive:
                self.assertEqual(set(archive.namelist()),
                                 {"trial/" + n for n in (*contents, "README.md", "SHA256SUMS")})
            self.assertEqual((source / "reference.json").read_bytes(), contents["reference.json"])


if __name__ == "__main__":
    unittest.main()
