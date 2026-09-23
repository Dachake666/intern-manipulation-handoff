"""[20260918] 双臂只读包：独立目录、精确文件集合与候选报告绑定。"""
from __future__ import annotations

from contextlib import ExitStack
import hashlib
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import zipfile


WORK = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "dual_release_builder", WORK / "releases/make_release.py")
release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release)


BUSINESS_FILES = {
    "precheck_dual_arm.py", "sdk_session.py", "robot_lock.py", "scrub_log.py",
    "arm_profiles.py", "arm_profiles.v1.json", "dual_arm_simulation.json",
    "validation_report.json",
}


class DualPrecheckReleaseTests(unittest.TestCase):
    def fixture(self, folder):
        """模拟八个源文件；不导入 pypilot，不动现有任何发布目录。"""
        root = Path(folder)
        source = root / "source"
        source.mkdir()
        for name in BUSINESS_FILES:
            (source / name).write_text("# fixture\n", encoding="utf-8")
        plan = {
            "schema_version": "dual_arm_simulation.v1",
            "real_motion_authorized": False,
            "debian_execution_allowed": False,
            "coordinate_frame": "URDF_WORLD_SIM_ONLY",
            "frames": [{"q_sdk_deg": {"left": [0] * 7, "right": [0] * 7}}] * 12,
        }
        plan_path = source / "dual_arm_simulation.json"
        plan_path.write_text(json.dumps(plan), encoding="utf-8")
        report = {
            "schema_version": "dual_arm_simulation_validation.v1",
            "plan_sha256": hashlib.sha256(plan_path.read_bytes()).hexdigest(),
            "dense_frames_checked": 12,
        }
        (source / "validation_report.json").write_text(json.dumps(report), encoding="utf-8")
        spec = dict(release.TRACKS["DP"])
        spec["files"] = [(str(source), name) for _, name in spec["files"]]
        spec["tools"] = [(str(source), name) for _, name in spec["tools"]]
        return root, source, plan, report, spec

    def patched_builder(self, stack, root, source, spec):
        stack.enter_context(patch.object(release, "_HERE", str(root)))
        stack.enter_context(patch.object(release, "TRACKS", {"DP": spec}))
        stack.enter_context(patch.object(release, "COMMON", [
            (str(source), "sdk_session.py"), (str(source), "robot_lock.py")]))

    def test_exact_minimum_files_and_isolated_directory(self):
        spec = release.TRACKS["DP"]
        names = [name for _, name in spec["files"] + release.COMMON + spec["tools"]]
        self.assertEqual(set(names), BUSINESS_FILES)
        self.assertEqual(len(names), len(set(names)))
        self.assertTrue(spec["minimal_release"])
        self.assertTrue(spec["zip_archive"])
        self.assertNotIn("DP", release.SUPERSEDED_RELEASES)
        self.assertNotIn("gui_review_pair", spec)
        self.assertIn("--precheck-only", spec["entry"])
        for key in ("A", "AT", "AS", "C", "CB"):
            self.assertNotEqual(spec["dir"], release.TRACKS[key]["dir"])

    def test_readme_has_direct_read_only_command_and_explicit_blockers(self):
        with tempfile.TemporaryDirectory(prefix=".handoff-review-dual-release-", dir=WORK) as folder:
            _, _, _, _, spec = self.fixture(folder)
            text = release.render_readme("DP", spec, sorted(BUSINESS_FILES), [])
        self.assertIn("当前候选 12 个双臂密集帧", text)
        self.assertIn("先校验文件", text)
        self.assertIn("CPython 3.10", text)
        self.assertIn("--precheck-only", text)
        self.assertIn('"$robot_ip"', text)
        self.assertNotIn("192.168.", text)
        self.assertIn("不调用 armTryWorlds", text)
        self.assertIn("没有 `--run`", text)
        self.assertNotIn("XIFENG_ALLOW_REAL_MOTION", text)
        self.assertIn("只读 PASS", text)
        self.assertIn("1.885", text)
        self.assertIn("TCP/映射", text)
        self.assertIn("并发", text)
        self.assertIn("dual_arm_precheck_时间_sdk.log", text)
        self.assertIn("python3 releases/make_release.py DP", text)

    def test_build_zip_and_hashes_are_exact_and_do_not_replace_existing_tracks(self):
        with tempfile.TemporaryDirectory(prefix=".handoff-review-dual-release-", dir=WORK) as folder:
            root, source, _, _, spec = self.fixture(folder)
            frozen = root / "trackA_hybrid_trial"
            frozen.mkdir()
            marker = frozen / "keep.json"
            marker.write_bytes(b"old success\n")
            with ExitStack() as stack:
                self.patched_builder(stack, root, source, spec)
                out = Path(release.build("DP"))
            expected = BUSINESS_FILES | {"README.md", "SHA256SUMS"}
            self.assertEqual({path.name for path in out.iterdir()}, expected)
            lines = (out / "SHA256SUMS").read_text().splitlines()
            self.assertEqual(len(lines), 8)
            for line in lines:
                digest, name = line.split("  ", 1)
                self.assertEqual(digest, hashlib.sha256((out / name).read_bytes()).hexdigest())
                self.assertEqual((out / name).read_bytes(), (source / name).read_bytes())
            with zipfile.ZipFile(root / "dual_arm_precheck.zip") as archive:
                self.assertEqual(set(archive.namelist()),
                                 {"dual_arm_precheck/" + name for name in expected})
            self.assertEqual(marker.read_bytes(), b"old success\n")

    def test_stale_report_rejected_without_overwriting_package(self):
        with tempfile.TemporaryDirectory(prefix=".handoff-review-dual-release-", dir=WORK) as folder:
            root, source, _, _, spec = self.fixture(folder)
            out = root / "dual_arm_precheck"
            out.mkdir()
            marker = out / "keep.txt"
            marker.write_bytes(b"existing\n")
            plan_path = source / "dual_arm_simulation.json"
            plan_path.write_bytes(plan_path.read_bytes() + b"\n")
            with ExitStack() as stack:
                self.patched_builder(stack, root, source, spec)
                with self.assertRaisesRegex(SystemExit, "候选/报告绑定"):
                    release.build("DP")
            self.assertEqual(marker.read_bytes(), b"existing\n")

    def test_motion_flags_and_wrong_frame_are_rejected_even_with_rebound_report(self):
        mutations = (
            ("real_motion_authorized", True),
            ("debian_execution_allowed", True),
            ("coordinate_frame", "SDK_WORLD"),
        )
        for field, value in mutations:
            with self.subTest(field=field), tempfile.TemporaryDirectory(
                    prefix=".handoff-review-dual-release-", dir=WORK) as folder:
                _, source, plan, report, spec = self.fixture(folder)
                plan[field] = value
                plan_path = source / "dual_arm_simulation.json"
                plan_path.write_text(json.dumps(plan), encoding="utf-8")
                report["plan_sha256"] = hashlib.sha256(plan_path.read_bytes()).hexdigest()
                (source / "validation_report.json").write_text(json.dumps(report), encoding="utf-8")
                with self.assertRaisesRegex(SystemExit, "禁止运动标记"):
                    release.validate_dual_precheck_sources(spec)

    def test_incomplete_dense_report_rejected(self):
        with tempfile.TemporaryDirectory(prefix=".handoff-review-dual-release-", dir=WORK) as folder:
            _, source, _, report, spec = self.fixture(folder)
            report["dense_frames_checked"] -= 1
            (source / "validation_report.json").write_text(json.dumps(report), encoding="utf-8")
            with self.assertRaises(SystemExit):
                release.validate_dual_precheck_sources(spec)


if __name__ == "__main__":
    unittest.main()
