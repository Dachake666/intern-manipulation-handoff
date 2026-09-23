#!/usr/bin/env python3
"""Compile and exercise the packaged production model-path parsing block.

Run from any directory after copying this file to the package's tools/ directory.
This checks path handling only; it does not compile ROS nodes or run inference.
"""

import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


VISION = Path("extensions/debian_vision/vision/ultirobotics_vision_detect")
CPP = VISION / "vision_detect_ros/src/visionDetectServer.cpp"
DATA_TYPE = VISION / "deps/vision/vision_algorithm/include/data_type.h"
CONFIGS = [VISION / "config" / name / "workstation.json" for name in ("test1", "multiple")]

HARNESS_PREFIX = r'''#include <filesystem>
#include <string>
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <nlohmann/json.hpp>
#define RCLCPP_ERROR(...) do {} while (0)
#define RCLCPP_INFO(...) do {} while (0)
struct algorithmParam { std::string model_path; };
bool resolve(const nlohmann::json &workspace, const std::string &config_path, std::string &actual) {
 const auto config_dir = std::filesystem::path(config_path).parent_path().string();
 const std::string workspace_id = "fixture";
 algorithmParam param;
'''

HARNESS_SUFFIX = r'''
 actual = param.model_path;
 return true;
}
int count = 0;
void check(bool ok, const std::string &name) {
 if (!ok) { std::cerr << "FAIL " << name << "\n"; std::exit(1); }
 ++count; std::cout << "PASS " << name << "\n";
}
int main(int argc, char **argv) {
 namespace fs = std::filesystem;
 if (argc != 2) return 2;
 const fs::path root = fs::absolute(fs::path(argv[1]));
 const auto expected = (root / "best.onnx").string();
 std::string actual;
 for (int round=0; round<2; ++round) {
  if (round==1) fs::current_path(root / "other cwd");
  for (const auto &config : {"test1", "multiple"}) {
   const auto filename = root / "config" / config / "workstation.json";
   std::ifstream input(filename); nlohmann::json data; input >> data;
   for (const auto &workspace : data["workspaces"]) {
    check(resolve(workspace, filename.string(), actual) && actual == expected,
      std::string(config) + "/" + workspace["workspace_id"].get<std::string>() + (round==0 ? " explicit config" : " changed cwd"));
   }
  }
 }
 const auto config=(root/"config"/"test1"/"workstation.json").string();
 check(!resolve(nlohmann::json::object(), config, actual), "missing field rejected");
 check(!resolve({{"model_path", ""}}, config, actual), "empty string rejected");
 check(!resolve({{"model_path", 42}}, config, actual), "number rejected");
 check(!resolve({{"model_path", nlohmann::json::object()}}, config, actual), "object rejected");
 check(!resolve({{"model_path", nullptr}}, config, actual), "null rejected");
 check(!resolve({{"model_path", "../../missing.onnx"}}, config, actual), "missing file rejected");
 check(!resolve({{"model_path", "../test1"}}, config, actual), "directory rejected");
 check(resolve({{"model_path", expected}}, config, actual) && actual==expected, "absolute path preserved");
 const auto legacy_config=(root/"legacy config"/"workstation.json").string();
 check(resolve({{"model_path", "legacy.onnx"}},legacy_config,actual) && actual==(root/"legacy config"/"models"/"legacy.onnx").string(), "legacy basename resolved under models");
 fs::current_path(root);
 check(resolve({{"model_path", "../../best.onnx"}},"config/test1/workstation.json",actual) && actual==expected, "relative config file resolved");
 check(!fs::exists(root/"config"/"test1"/"models") && !fs::exists(root/"config"/"multiple"/"models"), "no per-config models directories required");
 std::cout << "TOTAL " << count << " PASS\n";
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--package-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--overlay-root", type=Path, help="Development-only replacement files, using package-relative paths")
    parser.add_argument("--runtime-root", type=Path, help="Temporary build parent; defaults to package .runtime")
    parser.add_argument("--output", type=Path, help="Result JSON; defaults to package .runtime/vision_model_checks.json")
    args = parser.parse_args()
    root = args.package_root.resolve()
    overlay = args.overlay_root.resolve() if args.overlay_root else None
    runtime = (args.runtime_root or root / ".runtime").resolve()
    output = (args.output or runtime / "vision_model_checks.json").resolve()
    report = {
        "schema": "vision_model_path_checks.v1",
        "status": "FAIL",
        "checked_at_utc": datetime.now(timezone.utc).isoformat(),
        "overlay_used": overlay is not None,
        "inputs": [],
        "checks": [],
        "limitations": [
            "Only the production model-path parsing block is compiled; ROS logging is stubbed.",
            "Fixtures contain dummy model bytes. No ROS node, camera, ONNX inference, or robot SDK is run.",
        ],
    }
    private_paths = [str(root), str(runtime), str(Path.cwd()), str(Path.home())]
    if overlay:
        private_paths.append(str(overlay))

    def scrub(text):
        for value in sorted(set(private_paths), key=len, reverse=True):
            if value != "/":
                text = text.replace(value, "<local-path>")
        return text

    def selected(relative):
        candidate = overlay / relative if overlay else None
        return candidate if candidate and candidate.is_file() else root / relative

    def read(relative):
        path = selected(relative)
        data = path.read_bytes()
        report["inputs"].append({
            "path": relative.as_posix(),
            "sha256": hashlib.sha256(data).hexdigest(),
            "from_overlay": bool(overlay and path == overlay / relative),
        })
        return data

    try:
        source = read(CPP).decode("utf-8-sig")
        header = read(DATA_TYPE).decode("utf-8-sig")
        if not re.search(r"std::string\s+model_path\s*;", header):
            raise ValueError("algorithmParam.model_path must have no default initializer")
        start = source.index('            if (!workspace.contains("model_path")')
        end = source.index("            param.result_data_path = save_path;", start)
        block = source[start:end]
        report["production_block_sha256"] = hashlib.sha256(block.encode()).hexdigest()
        configs = [json.loads(read(path)) for path in CONFIGS]
        if [len(config.get("workspaces", [])) for config in configs] != [1, 2]:
            raise ValueError("Expected test1 and multiple to contain 1 and 2 workspaces; update this check for a new configuration contract")
        for config in configs:
            if any(w.get("model_path") != "../../best.onnx" for w in config["workspaces"]):
                raise ValueError("All packaged workspaces must explicitly reference ../../best.onnx")
        read(VISION / "best.onnx")
        compiler = shutil.which("clang++") or shutil.which("g++")
        if not compiler:
            raise RuntimeError("A C++17 compiler is required: install clang++ or g++; no checks were skipped as passing")
        report["compiler"] = Path(compiler).name
        runtime.mkdir(parents=True, exist_ok=True)
        with tempfile.TemporaryDirectory(prefix="vision-model-", dir=runtime) as temp:
            scratch = Path(temp)
            private_paths.append(str(scratch))
            fixture = scratch / "中文 含空格视觉工程"
            fixture.mkdir()
            for name, config in zip(("test1", "multiple"), configs):
                target = fixture / "config" / name / "workstation.json"
                target.parent.mkdir(parents=True)
                target.write_text(json.dumps(config, ensure_ascii=False), encoding="utf-8")
            (fixture / "best.onnx").write_bytes(b"path-fixture-not-an-onnx-model")
            legacy = fixture / "legacy config" / "models"
            legacy.mkdir(parents=True)
            (legacy / "legacy.onnx").write_bytes(b"legacy-path-fixture")
            (fixture / "other cwd").mkdir()
            cpp = scratch / "model_path_test.cpp"
            cpp.write_text(HARNESS_PREFIX + block + HARNESS_SUFFIX, encoding="utf-8")
            binary = scratch / "model_path_test"
            include = root / VISION / "deps/vision/vision_third_party/third_party/nlohmann"
            # The complete delivered vendor include tree is compiled, not a JSON mock.
            compile_result = subprocess.run(
                [compiler, "-std=c++17", "-Wall", "-Wextra", "-pedantic", "-I", str(include), str(cpp), "-o", str(binary)],
                cwd=scratch, capture_output=True, text=True, timeout=120,
            )
            report["compile"] = {"returncode": compile_result.returncode, "stdout": scrub(compile_result.stdout), "stderr": scrub(compile_result.stderr)}
            if compile_result.returncode:
                raise RuntimeError("Independent production path-block compilation failed")
            run = subprocess.run([str(binary), str(fixture)], cwd=scratch, capture_output=True, text=True, timeout=30)
            report["run"] = {"returncode": run.returncode, "stdout": scrub(run.stdout), "stderr": scrub(run.stderr)}
            report["checks"] = [{"name": line[5:], "status": "PASS"} for line in run.stdout.splitlines() if line.startswith("PASS ")]
            if run.returncode or len(report["checks"]) != 17 or "TOTAL 17 PASS" not in run.stdout:
                raise RuntimeError("Expected 17 passing production path-block cases")
            report["static_checks"] = [
                {"name": "algorithmParam has no cwd model default", "status": "PASS"},
                {"name": "test1/multiple explicitly share root model", "status": "PASS"},
            ]
            report["status"] = "PASS"
    except Exception as exc:
        report["error"] = scrub(str(exc))
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    if "run" in report:
        print(report["run"]["stdout"], end="")
    if report["status"] != "PASS":
        print("FAIL:", report.get("error", "unknown error"))
    print("vision_model_path_checks:", report["status"])
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
