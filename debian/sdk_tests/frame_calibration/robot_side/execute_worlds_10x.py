#!/usr/bin/env python3
"""左臂世界坐标十步运动与逐步反馈记录。

运动配方以启动时 armGetWorlds() 读到的当前位姿为锚点，姿态 UVW 全程不变：

  1 上抬 Z；2/3 +X/回中心；4/5 +Y/回中心；
  6/7 -X/回中心；8/9 -Y/回中心；10 回到原始位姿。

每一段均使用 armMoveWorlds 点到点运动，wait_until_worlds 的最终反馈作为该段
唯一归档的世界坐标记录。因为要求每步都读取到位坐标，本程序不会把十点连续混合，
所以它是坐标与反馈验收工具，不是连续平滑轨迹演示。

使用方式与 collect_wrist_orientation_samples.py 一致：
  * 只修改下方“配置”区域，运行命令始终是 python3 execute_worlds_10x.py。
  * ROBOT_IP 改动时 ARM_IP 会自动跟随；一般只需改 ROBOT_IP/LOCAL_IP。
  * ENABLE_REAL_MOTION=False 时只连接并预检，不清报警、不使能、不设速度。
  * ENABLE_REAL_MOTION=True 才允许运动；默认每一步都需回车确认，q 中止。

安全边界：
  * 运动前密集检查每条世界直线的 IK 连续性、控制器实时限位和本地保守限位。
  * 每一步到位并稳定后重新读取一次世界坐标与关节角，再写入运行记录。
  * 本文件不依赖 robot_lock.py；运行前必须确认没有其他控制进程占用同一机械臂。

Debian 容器内运行：python3 execute_worlds_10x.py
"""
from __future__ import annotations

import datetime as dt
import hashlib
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Sequence


# ---------------------------------------------------------------- 配置
# 本次成功记录使用 .148；机器人 IP 变化时只改 ROBOT_IP 即可，ARM_IP 自动跟随。
ROBOT_IP = "192.168.8.147"
LOCAL_IP = "192.168.8.185"
ARM_IP = ROBOT_IP
ARM_PORT = 8080

ARM_ID = 1                       # 1=左臂；本脚本不允许右臂
GLOBAL_SPEED = 1.0               # 全局速度百分比，保持低速
STEP_MM = 80.0                   # 新候选步长；已成功真机基线是 10mm
ENABLE_REAL_MOTION = True       # False=只读预检；True=真实运动
CONFIRM_EACH_STEP = True         # 每一步前回车确认，q 中止并保存已完成记录
SETTLE_SECONDS = 1.0             # 到位后稳定，再读取实际世界坐标
OUT_DIR = "data"

# 预检/到位参数。通常无需修改。
SAMPLE_MM = 2.0
LIMIT_MARGIN_DEG = 8.0
MAX_JOINT_STEP_DEG = 4.0
TOLERANCE_MM = 2.0
TOLERANCE_DEG = 1.0
TIMEOUT_S = 20.0
ANGLE_DEG = 5.0

# 2026-08-17 真机 10mm 成功记录，仅用于说明新候选的来源，不作为输入轨迹。
SOURCE_RECORD_SHA256 = "07c9e96e4363283c4d17ee6f38d70209a5cd5a80324abf61deb31535d62b4c0e"
SOURCE_RECORD_STEP_MM = 10.0

MOVE_COUNT = 10
DEFAULT_STEP_MM = STEP_MM
MAX_STEP_MM = 500.0
DEFAULT_SAMPLE_MM = SAMPLE_MM
DEFAULT_LIMIT_MARGIN_DEG = LIMIT_MARGIN_DEG
DEFAULT_MAX_JOINT_STEP_DEG = MAX_JOINT_STEP_DEG
LEFT_OPERATIONAL_LIMITS_DEG = (
    (-180.0, 60.0), (0.0, 150.0), (-140.0, 140.0), (-120.0, 0.0),
    (-170.0, 170.0), (-45.0, 45.0), (-90.0, 76.0),
)


class PrecheckError(RuntimeError):
    """候选路径未通过真机运动前的安全门。"""


def _pose6(values: Sequence[float], label: str) -> list[float]:
    if len(values) != 6:
        raise ValueError(f"{label} 必须有 6 个值 [X,Y,Z,U,V,W]")
    out = [float(v) for v in values]
    if not all(math.isfinite(v) for v in out):
        raise ValueError(f"{label} 含非有限数值")
    return out


def build_offsets(step_mm: float = DEFAULT_STEP_MM) -> list[list[float]]:
    """返回恰好十个相对锚点的 [dx,dy,dz,du,dv,dw]。"""
    step = float(step_mm)
    if not math.isfinite(step) or not (1.0 <= step <= MAX_STEP_MM):
        raise ValueError(f"step-mm 必须在 [1,{MAX_STEP_MM:g}] mm 内")
    z = step
    angle = ANGLE_DEG
    return [
        [0, 0, z, angle, 0, 0],
        [step, 0, z, 0, angle, 0],
        [0, 0, z, -angle, 0, 0],
        [0, step, z, 0, -angle, 0],
        [0, 0, z, 0, 0, angle],
        [-step, 0, z, angle, angle, 0],
        [0, 0, z, 0, 0, -angle],
        [0, -step, z, -angle, 0, angle],
        [0, 0, z, 0, angle, -angle],
        [0, 0, 0, 0, 0, 0],
    ]


def build_targets(anchor: Sequence[float], step_mm: float) -> list[list[float]]:
    base = _pose6(anchor, "anchor")
    return [[a + d for a, d in zip(base, off)] for off in build_offsets(step_mm)]


def densify_segment(start: Sequence[float], end: Sequence[float],
                    sample_mm: float = DEFAULT_SAMPLE_MM,
                    sample_deg: float = 1.0) -> list[list[float]]:
    """按 armMoveWorlds 的笛卡尔直线语义加密；不含起点、包含终点。"""
    a, b = _pose6(start, "segment start"), _pose6(end, "segment end")
    if sample_mm <= 0 or sample_deg <= 0:
        raise ValueError("sample-mm 和 sample-deg 必须大于 0")
    distance = math.sqrt(sum((x - y) ** 2 for x, y in zip(a[:3], b[:3])))
    angle = max(abs(x - y) for x, y in zip(a[3:], b[3:]))
    count = max(1, math.ceil(max(distance / sample_mm, angle / sample_deg)))
    return [[x + (y - x) * k / count for x, y in zip(a, b)]
            for k in range(1, count + 1)]


def limit_violations(joints: Sequence[float], limits: Sequence[Sequence[float]],
                     margin_deg: float, source: str = "控制器") -> list[str]:
    if len(joints) != 7 or len(limits) != 7:
        return [f"关节/限位维度异常: joints={len(joints)} limits={len(limits)}"]
    bad = []
    for index, (q, pair) in enumerate(zip(joints, limits), start=1):
        if len(pair) != 2:
            bad.append(f"j{index} 限位不是 [lo,hi]")
            continue
        lo, hi = float(pair[0]), float(pair[1])
        if not (lo + margin_deg <= float(q) <= hi - margin_deg):
            bad.append(
                f"j{index}={float(q):+.2f}° 不在{source}安全区间 "
                f"[{lo + margin_deg:+.1f},{hi - margin_deg:+.1f}]°")
    return bad


def precheck_plan(ss: Any, sdk: Any, arm_id: int, anchor: Sequence[float],
                  targets: Sequence[Sequence[float]], current_joints: Sequence[float],
                  sample_mm: float = DEFAULT_SAMPLE_MM,
                  limit_margin_deg: float = DEFAULT_LIMIT_MARGIN_DEG,
                  max_joint_step_deg: float = DEFAULT_MAX_JOINT_STEP_DEG) -> dict[str, Any]:
    """密集 IK + 双限位 + 分支连续性预检；失败即抛 PrecheckError。"""
    if len(targets) != MOVE_COUNT:
        raise PrecheckError(f"运动数量必须为 {MOVE_COUNT}，实际 {len(targets)}")
    if arm_id != 1:
        raise PrecheckError("本候选程序只允许 arm-id=1 左臂；右臂真实限位尚未验收")
    if limit_margin_deg < 5.0:
        raise PrecheckError("限位余量不得小于 5°")
    if max_joint_step_deg <= 0 or max_joint_step_deg > 5.0:
        raise PrecheckError("相邻 IK 跳变门限必须在 (0,5]°")

    try:
        controller_limits = ss.read_axis_limits(sdk, arm_id)
    except Exception as exc:  # noqa: BLE001 - 无真实限位时必须阻断
        raise PrecheckError(f"无法读取控制器真实轴限位，拒绝运动: {exc}") from exc
    valid, details = ss.axis_limits_look_valid(controller_limits)
    if not valid:
        raise PrecheckError(f"控制器轴限位读数无效，拒绝运动: {details}")

    previous_q = [float(v) for v in current_joints]
    if len(previous_q) != 7:
        raise PrecheckError(f"当前关节反馈应为 7 维，实际 {len(previous_q)}")
    bad = limit_violations(previous_q, controller_limits, limit_margin_deg)
    bad += limit_violations(previous_q, LEFT_OPERATIONAL_LIMITS_DEG,
                            limit_margin_deg, source="本地实测")
    if bad:
        raise PrecheckError("当前姿态已经过于接近限位:\n  " + "\n  ".join(bad))

    segment_reports: list[dict[str, Any]] = []
    min_margins = [math.inf] * 7
    previous_pose = _pose6(anchor, "anchor")
    total_dense = 0
    for move_index, target in enumerate(targets, start=1):
        dense = densify_segment(previous_pose, target, sample_mm=sample_mm)
        worst_jump = 0.0
        for sample_index, pose in enumerate(dense, start=1):
            q = ss.try_worlds(sdk, arm_id, pose)
            if q is None:
                raise PrecheckError(
                    f"第{move_index}次运动的密集点 {sample_index}/{len(dense)} "
                    f"不可达或奇异: {[round(v, 3) for v in pose]}")
            q = [float(v) for v in q]
            bad = limit_violations(q, controller_limits, limit_margin_deg)
            bad += limit_violations(q, LEFT_OPERATIONAL_LIMITS_DEG,
                                    limit_margin_deg, source="本地实测")
            if bad:
                raise PrecheckError(
                    f"第{move_index}次运动的密集点 {sample_index}/{len(dense)} "
                    "触及限位安全区:\n  " + "\n  ".join(bad))
            jump = max(abs(x - y) for x, y in zip(q, previous_q))
            worst_jump = max(worst_jump, jump)
            if jump > max_joint_step_deg:
                raise PrecheckError(
                    f"第{move_index}次运动的密集点 {sample_index}/{len(dense)} "
                    f"IK 跳变 {jump:.2f}° > {max_joint_step_deg:.2f}°")
            for axis, (value, pair) in enumerate(zip(q, controller_limits)):
                lo, hi = float(pair[0]), float(pair[1])
                min_margins[axis] = min(min_margins[axis], value - lo, hi - value)
            previous_q = q
        total_dense += len(dense)
        segment_reports.append({
            "move": move_index,
            "dense_points": len(dense),
            "max_adjacent_joint_step_deg": worst_jump,
            "predicted_joints_deg": previous_q,
        })
        previous_pose = list(target)

    return {
        "controller_limits_deg": [[float(x), float(y)] for x, y in controller_limits],
        "limit_margin_required_deg": float(limit_margin_deg),
        "minimum_predicted_hard_limit_margin_deg": min_margins,
        "dense_points": total_dense,
        "segments": segment_reports,
    }


def execute_plan(ss: Any, sdk: Any, arm_id: int,
                 targets: Sequence[Sequence[float]],
                 tolerance_mm: float = 2.0,
                 tolerance_deg: float = 1.0,
                 timeout_s: float = 20.0,
                 records: list[dict[str, Any]] | None = None,
                 confirm_each_step: bool = False,
                 settle_seconds: float = 0.0,
                 input_fn: Any = input) -> list[dict[str, Any]]:
    """最多执行十次；每次到位稳定后重新读取 worlds 并归档。"""
    if len(targets) != MOVE_COUNT:
        raise ValueError(f"运动数量必须为 {MOVE_COUNT}")
    if settle_seconds < 0:
        raise ValueError("settle_seconds 不能小于 0")
    output = records if records is not None else []
    for move_index, raw_target in enumerate(targets, start=1):
        target = _pose6(raw_target, f"target {move_index}")
        if confirm_each_step:
            answer = input_fn(
                f"\n[{move_index}/{MOVE_COUNT}] 目标 XYZ="
                f"{[round(v, 3) for v in target[:3]]}，"
                "回车执行 / q 中止: ").strip().lower()
            if answer == "q":
                print("用户中止，保存已完成的运动记录。")
                break
        ss.check_soft_stop(sdk, f"第{move_index}次运动前")
        ss.move_worlds(sdk, arm_id, target, interpolation_en=False)
        ss.wait_until_worlds(
            sdk, arm_id, target,
            tol_mm=tolerance_mm,
            tol_deg=tolerance_deg,
            timeout_s=timeout_s,
            label=f"第{move_index}次")
        if settle_seconds:
            time.sleep(settle_seconds)
        # 与 wrist 采样脚本一致：到位并稳定后，再主动读取一次实际反馈。
        actual = ss.read_worlds(sdk, arm_id)
        joints = ss.read_joints(sdk, arm_id)
        pos_error = max(abs(a - b) for a, b in zip(actual[:3], target[:3]))
        ori_error = max(abs(a - b) for a, b in zip(actual[3:], target[3:]))
        record = {
            "move": move_index,
            "timestamp": dt.datetime.now().isoformat(timespec="milliseconds"),
            "target_world": target,
            "actual_world": [float(v) for v in actual],
            "actual_joints_deg": [float(v) for v in joints],
            "max_position_error_mm": pos_error,
            "max_orientation_error_deg": ori_error,
        }
        output.append(record)
        print(
            f"[{move_index:02d}/{MOVE_COUNT}] actual XYZ="
            f"{[round(v, 3) for v in actual[:3]]} UVW="
            f"{[round(v, 3) for v in actual[3:]]} "
            f"err={pos_error:.2f}mm/{ori_error:.2f}°")
    return output


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def save_record(output_dir: Path, payload: dict[str, Any]) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    path = output_dir / f"worlds_10x_record_{stamp}.json"
    with path.open("x", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write("\n")
    return path


def render_preview(path: Path, step_mm: float) -> Path:
    """用纯标准库生成等轴投影 SVG；不导入 SDK，也不依赖 matplotlib。"""
    if path.suffix.lower() != ".svg":
        raise ValueError("预览文件必须使用 .svg 后缀")
    pts = [[0.0, 0.0, 0.0]] + [off[:3] for off in build_offsets(step_mm)]
    width, height = 1100, 650
    cx, cy = 380.0, 440.0
    scale = 12.0 * (10.0 / step_mm)

    def project(point):
        x, y, z = point
        return cx + (x - y) * scale, cy + (x + y) * scale * 0.45 - z * scale

    screen = [project(point) for point in pts]
    lines = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1100" height="650" '
        'viewBox="0 0 1100 650">',
        '<rect width="1100" height="650" fill="#f7f9fc"/>',
        '<text x="550" y="42" text-anchor="middle" font-size="24" '
        'font-family="sans-serif" fill="#17233c">World-coordinate 10-move candidate</text>',
        '<text x="550" y="70" text-anchor="middle" font-size="14" '
        'font-family="sans-serif" fill="#52627a">relative anchor frame · UVW fixed · units mm</text>',
    ]
    origin = project([0, 0, 0])
    axis_defs = [
        ([step_mm * 1.5, 0, 0], "X", "#d32f2f"),
        ([0, step_mm * 1.5, 0], "Y", "#2e7d32"),
        ([0, 0, step_mm * 1.5], "Z", "#1565c0"),
    ]
    for endpoint, name, color in axis_defs:
        ex, ey = project(endpoint)
        lines.append(
            f'<line x1="{origin[0]:.1f}" y1="{origin[1]:.1f}" '
            f'x2="{ex:.1f}" y2="{ey:.1f}" stroke="{color}" stroke-width="2"/>')
        lines.append(
            f'<text x="{ex + 8:.1f}" y="{ey - 5:.1f}" font-size="16" '
            f'font-family="sans-serif" fill="{color}">{name}</text>')
    polyline = " ".join(f"{x:.1f},{y:.1f}" for x, y in screen)
    lines.append(
        f'<polyline points="{polyline}" fill="none" stroke="#1769aa" '
        'stroke-width="5" stroke-linejoin="round" stroke-linecap="round"/>')
    point_groups: dict[tuple[float, float, float], list[int]] = {}
    for index, point in enumerate(pts):
        point_groups.setdefault(tuple(point), []).append(index)
    for point, indexes in point_groups.items():
        x, y = project(point)
        if 0 in indexes:
            color, radius = "#2e7d32", 9
            label = "S / " + ",".join(str(i) for i in indexes if i)
        else:
            color, radius = "#ff8f00", 7
            label = ",".join(str(i) for i in indexes)
        lines.append(
            f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{radius}" '
            f'fill="{color}" stroke="white" stroke-width="2"/>')
        lines.append(
            f'<text x="{x + 11:.1f}" y="{y - 9:.1f}" font-size="14" '
            f'font-family="sans-serif" font-weight="bold" fill="#17233c">{label}</text>')
    lines += [
        '<rect x="650" y="120" width="410" height="420" rx="12" '
        'fill="white" stroke="#d7dfeb"/>',
        '<text x="675" y="150" font-size="17" font-family="sans-serif" '
        'font-weight="bold" fill="#17233c">10 moves [dx, dy, dz]</text>',
    ]
    for index, point in enumerate(pts[1:], start=1):
        y = 150 + index * 34
        lines.append(
            f'<text x="675" y="{y}" font-size="14" font-family="monospace" '
            f'fill="#33435c">{index:02d}  [{point[0]:+g}, {point[1]:+g}, '
            f'{point[2]:+g}]</text>')
    lines += [
        '<text x="550" y="610" text-anchor="middle" font-size="14" '
        'font-family="sans-serif" fill="#8a3b12">Candidate preview only — clear the real workspace before motion.</text>',
        '</svg>',
    ]
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def _print_plan(step_mm: float) -> None:
    print("十步相对路径 [dx,dy,dz] mm（UVW 固定）:")
    for index, off in enumerate(build_offsets(step_mm), start=1):
        print(f"  {index:02d}: {[float(v) for v in off[:3]]}")
    print("注意: 只验证小范围运动和逐步反馈；没有现场障碍物模型，运行前必须清空路径。")


def _validate_config() -> None:
    if not ROBOT_IP or not LOCAL_IP or not ARM_IP:
        raise SystemExit("请先在文件顶部填写 ROBOT_IP / LOCAL_IP / ARM_IP")
    if ARM_ID != 1:
        raise SystemExit("本候选程序只允许 ARM_ID=1 左臂")
    if not (0.1 <= GLOBAL_SPEED <= 5.0):
        raise SystemExit("GLOBAL_SPEED 必须在 [0.1,5.0]%")
    if not (1.0 <= STEP_MM <= MAX_STEP_MM):
        raise SystemExit(f"STEP_MM 必须在 [1,{MAX_STEP_MM:g}] mm")
    if SETTLE_SECONDS < 0:
        raise SystemExit("SETTLE_SECONDS 不能小于 0")


def run_robot() -> int:
    """按文件顶部配置运行；仅在这里延迟导入真机 SDK。"""
    import sdk_session as ss

    _validate_config()
    sdk = None
    anchor: list[float] | None = None
    precheck: dict[str, Any] | None = None
    records: list[dict[str, Any]] = []
    failure: str | None = None
    script_path = Path(__file__).resolve()
    sdk_path = Path(ss.__file__).resolve()

    print("=" * 64)
    print("左臂世界坐标十步运动")
    print("=" * 64)
    print(f"robot={ROBOT_IP}  local={LOCAL_IP}  arm={ARM_IP}:{ARM_PORT}")
    print(f"ARM_ID={ARM_ID}  SPEED={GLOBAL_SPEED}%  STEP={STEP_MM}mm")
    print(f"ENABLE_REAL_MOTION={ENABLE_REAL_MOTION}")
    print("提醒：本文件无进程锁，运行前确认其他机械臂控制脚本均已退出。")
    _print_plan(STEP_MM)

    try:
        sdk = ss.open_session(
            ROBOT_IP, LOCAL_IP, ARM_IP, ARM_PORT,
            GLOBAL_SPEED, (ARM_ID,), enable=ENABLE_REAL_MOTION)
        anchor = ss.read_worlds(sdk, ARM_ID)
        current_joints = ss.read_joints(sdk, ARM_ID)
        targets = build_targets(anchor, STEP_MM)
        print(f"当前 anchor: {[round(v, 3) for v in anchor]}")
        precheck = precheck_plan(
            ss, sdk, ARM_ID, anchor, targets, current_joints,
            sample_mm=SAMPLE_MM,
            limit_margin_deg=LIMIT_MARGIN_DEG,
            max_joint_step_deg=MAX_JOINT_STEP_DEG)
        print(
            f"预检通过: {precheck['dense_points']} 个密集 IK 点，"
            f"控制器硬限位最小预测余量 "
            f"{min(precheck['minimum_predicted_hard_limit_margin_deg']):.2f}°")

        if not ENABLE_REAL_MOTION:
            print("DRY RUN：未清报警、未使能、未设置速度、未发送运动。")
            print("确认计划无误后，将文件顶部 ENABLE_REAL_MOTION 改为 True 再运行。")
            return 0

        execute_plan(
            ss, sdk, ARM_ID, targets,
            tolerance_mm=TOLERANCE_MM,
            tolerance_deg=TOLERANCE_DEG,
            timeout_s=TIMEOUT_S,
            records=records,
            confirm_each_step=CONFIRM_EACH_STEP,
            settle_seconds=SETTLE_SECONDS)
        if len(records) == MOVE_COUNT:
            print(f"十次世界坐标运动完成，取得 {len(records)} 条到位反馈。")
            return 0
        failure = f"UserAbort: completed {len(records)}/{MOVE_COUNT} moves"
        print(f"本次只完成 {len(records)}/{MOVE_COUNT} 次，记录标记为 INCOMPLETE。")
        return 2
    except BaseException as exc:
        failure = f"{type(exc).__name__}: {exc}"
        if sdk is not None and ENABLE_REAL_MOTION:
            try:
                ss.clear_robot_route(sdk, ARM_ID, emergency_stop=True)
                print("异常收尾：已请求清除运动路径并立即停止。")
            except Exception as stop_exc:  # noqa: BLE001
                print(f"警告：异常停止请求失败，请人工确认机器人: {stop_exc}")
        raise
    finally:
        if ENABLE_REAL_MOTION and sdk is not None:
            payload = {
                "meta": {
                    "status": "PASS" if len(records) == MOVE_COUNT and failure is None else "INCOMPLETE",
                    "created": dt.datetime.now().isoformat(timespec="seconds"),
                    "script": str(script_path),
                    "script_sha256": _sha256(script_path),
                    "sdk_session": str(sdk_path),
                    "sdk_session_sha256": _sha256(sdk_path),
                    "robot_ip": ROBOT_IP,
                    "arm_ip": ARM_IP,
                    "local_ip": LOCAL_IP,
                    "arm_id": ARM_ID,
                    "speed_percent": GLOBAL_SPEED,
                    "step_mm": STEP_MM,
                    "settle_seconds": SETTLE_SECONDS,
                    "source_record_sha256": SOURCE_RECORD_SHA256,
                    "source_record_step_mm": SOURCE_RECORD_STEP_MM,
                    "failure": failure,
                },
                "anchor_world": anchor,
                "precheck": precheck,
                "moves": records,
            }
            try:
                out = save_record(Path(OUT_DIR), payload)
                print(f"运行记录: {out}")
            except Exception as save_exc:  # noqa: BLE001
                print(f"警告：运行记录保存失败: {save_exc}")
        if sdk is not None:
            ss.close_session(sdk)


def main() -> int:
    return run_robot()


if __name__ == "__main__":
    sys.exit(main())
