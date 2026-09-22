#!/usr/bin/env python3
"""[20260915] 真正 stepSimulation 反馈的 MPC 对照实验；没有真机执行入口。"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
import math
import os
from pathlib import Path
import platform
import sys
import time
import xml.etree.ElementTree as ET

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path[:0] = [str(HERE), str(ROOT), str(ROOT / "frame_calibration/robot_side")]

import numpy as np
import pybullet as pb
import arm_profiles
from frame_calibration.analysis import calib_common as cc
import tabletop_servo_contract as contract  # 纯离线校验；不创建 SDK 会话
from controller import JointMPC, MPCError, Settings, finite

DEFAULT_REFERENCE = (ROOT / "frame_calibration/records/20260914_servo_sta_off/handoff/"
                     "reconstructed/tabletop_pick_place_SERVO_CANDIDATE.json")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def audit_urdf_inertias(path):
    issues = []
    for link in ET.parse(path).getroot().findall("link"):
        inertial = link.find("inertial")
        if inertial is None or float(inertial.find("mass").get("value")) <= 0:
            continue
        a = {key: float(value) for key, value in inertial.find("inertia").attrib.items()}
        matrix = [[a["ixx"], a["ixy"], a["ixz"]], [a["ixy"], a["iyy"], a["iyz"]],
                  [a["ixz"], a["iyz"], a["izz"]]]
        eig = np.linalg.eigvalsh(matrix)
        if eig[0] <= 0 or eig[-1] > sum(eig[:-1])+1e-10:
            issues.append({"link": link.get("name"), "principal_inertia_kg_m2": eig.tolist()})
    return issues


def load_reference(path, dt, final_settle_s=0.5):
    """保留三次事件及等待；预测不能跨夹爪事件预先开始下一段运动。"""
    plan = json.loads(Path(path).read_text())
    source_metrics = contract.validate(plan)
    if not math.isclose(plan["stream_policy"]["period_s"], dt, abs_tol=1e-12):
        raise ValueError("本实验不隐式改参考周期或加速重采样")
    blocks, rows, stages = [], [], []
    last = None
    for w in plan["waypoints"]:
        if "q_sdk_deg" in w:
            last = finite(w["q_sdk_deg"], (7,), "SDK 参考")
            rows.append(last.copy())
            stages.append(w["stage"])
        else:
            if last is None:
                raise ValueError("夹爪事件前没有位置")
            waits = max(1, math.ceil(float(w["settle_s"]) / dt))
            rows.extend([last.copy() for _ in range(waits)])
            stages.extend([w["stage"]] * waits)
            blocks.append({"q": np.asarray(rows), "stages": stages,
                           "event": {"action": w["action"], "stage": w["stage"],
                                     "minimum_hold_s": w["settle_s"]}})
            rows, stages = [], []
    if last is None:
        raise ValueError("空参考")
    rows.extend([last.copy() for _ in range(math.ceil(final_settle_s/dt))])
    stages.extend(["FINAL_SETTLE"] * math.ceil(final_settle_s/dt))
    blocks.append({"q": np.asarray(rows), "stages": stages, "event": None})
    return plan, blocks, source_metrics


class BulletPlant:
    """SDK 度 ↔ URDF 弧度只变换一次；每步用电机，不用 resetJointState 播放。

    零重力代表理想重力补偿工况，不代表辨识过的真机动力学。碰撞和负载未建模，
    保留原 URDF 质量/effort；源惯量存在非法值，使用 PB 形状估算惯量并明确报告。
    没有桌子或夹爪抓取接触，不能据此认证抓放安全。
    """

    def __init__(self, q0, dt, gui=False, position_gain=0.06, substeps=8):
        self.client = pb.connect(pb.GUI if gui else pb.DIRECT)
        try:
            self.profile = arm_profiles.arm_profile("left")
            self.joints = self.profile["joint_ids"]
            if self.joints != cc.ARM_JOINT_IDS["left"]:
                raise ValueError("共享关节映射与标定模型漂移")
            if self.profile["ee_link_id"] != cc.EE_LINK_ID["left"]:
                raise ValueError("末端 link 映射漂移")
            check = np.arange(1.0, 8.0)
            if not np.array_equal(arm_profiles.sdk_to_urdf_deg(self.profile, check),
                                  cc.sdk_q_to_urdf_q(check)):
                raise ValueError("J6 符号映射漂移")
            self.dt, self.substeps, self.position_gain = dt, substeps, position_gain
            pb.setAdditionalSearchPath(str(ROOT), physicsClientId=self.client)
            pb.setGravity(0, 0, 0, physicsClientId=self.client)
            pb.setTimeStep(dt/substeps, physicsClientId=self.client)
            pb.setPhysicsEngineParameter(numSolverIterations=80, deterministicOverlappingPairs=1,
                                         physicsClientId=self.client)
            self.robot = pb.loadURDF(cc.DEFAULT_URDF, useFixedBase=True,
                                     flags=0,  # 不使用非法源惯量，不修改厂商 URDF。
                                     physicsClientId=self.client)
            self.moving = [j for j in range(pb.getNumJoints(self.robot, self.client))
                           if pb.getJointInfo(self.robot, j, self.client)[2] != pb.JOINT_FIXED]
            self.forces = [pb.getJointInfo(self.robot, j, self.client)[10] for j in self.moving]
            if any(not math.isfinite(f) or f <= 0 for f in self.forces):
                raise ValueError("URDF effort 无效，不猜测电机力矩")
            self.inertia_issues = audit_urdf_inertias(cc.DEFAULT_URDF)
            for j in self.moving:
                inertia = pb.getDynamicsInfo(self.robot, j, self.client)[2]
                if min(inertia) <= 0 or not np.isfinite(inertia).all():
                    raise ValueError("PB 几何估算惯量仍无效，不能作动态实验")
            self.hold = np.zeros(len(self.moving))
            initial = np.radians(arm_profiles.sdk_to_urdf_deg(self.profile, q0))
            for j, q in zip(self.joints, initial):
                self.hold[self.moving.index(j)] = q
                # 仅仿真初态初始化；step() 内禁止重置任何关节。
                pb.resetJointState(self.robot, j, q, physicsClientId=self.client)
            self.urdf_limits = []
            for jid, sign in zip(self.joints, self.profile["sdk_from_urdf_sign"]):
                info = pb.getJointInfo(self.robot, jid, self.client)
                self.urdf_limits.append(sorted(np.degrees([info[8]*sign, info[9]*sign])))
            self.urdf_limits = np.asarray(self.urdf_limits)
            ctrl = np.asarray(self.profile["controller_limits_deg"], float)
            margin = contract.policy()["margin_deg"]
            self.limits = np.column_stack((np.maximum(ctrl[:, 0]+margin, self.urdf_limits[:, 0]),
                                          np.minimum(ctrl[:, 1]-margin, self.urdf_limits[:, 1])))
            if gui:
                pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 0, physicsClientId=self.client)
                pb.resetDebugVisualizerCamera(1.8, 135, -20, [0, 0, 0.7],
                                              physicsClientId=self.client)
                pb.addUserDebugText("MPC FEEDBACK EXPERIMENT / NO GRASP-COLLISION VALIDATION",
                                    [0, 0, 1.5], [0.9, 0.15, 0.1], textSize=1.1,
                                    physicsClientId=self.client)
        except BaseException:
            self.close()
            raise

    def read(self):
        states = pb.getJointStates(self.robot, self.joints, physicsClientId=self.client)
        q = arm_profiles.urdf_to_sdk_deg(self.profile, np.degrees([x[0] for x in states]))
        v = arm_profiles.urdf_to_sdk_deg(self.profile, np.degrees([x[1] for x in states]))
        return np.asarray(q), np.asarray(v)

    def step(self, command, force_world=None):
        angles = np.radians(arm_profiles.sdk_to_urdf_deg(self.profile, command))
        target = self.hold.copy()
        for j, q in zip(self.joints, angles):
            target[self.moving.index(j)] = q
        pb.setJointMotorControlArray(self.robot, self.moving, pb.POSITION_CONTROL,
                                    targetPositions=target, targetVelocities=[0]*len(target),
                                    forces=self.forces, positionGains=[self.position_gain]*len(target),
                                    velocityGains=[1.0]*len(target), physicsClientId=self.client)
        samples = []
        for _ in range(self.substeps):
            if force_world is not None:
                link = self.profile["ee_link_id"]
                pos = pb.getLinkState(self.robot, link, physicsClientId=self.client)[0]
                pb.applyExternalForce(self.robot, link, force_world, pos, pb.WORLD_FRAME,
                                      physicsClientId=self.client)
            pb.stepSimulation(physicsClientId=self.client)
            samples.append(self.read())
        return samples

    def grip_pose(self):
        st = pb.getLinkState(self.robot, self.profile["ee_link_id"],
                            computeForwardKinematics=True, physicsClientId=self.client)
        R = np.asarray(pb.getMatrixFromQuaternion(st[5])).reshape(3, 3)
        return np.asarray(st[4])*1000 + R @ np.asarray(self.profile["grip_center_link_mm"])

    def close(self):
        if pb.isConnected(self.client):
            pb.disconnect(self.client)


def margin(q, limits):
    return np.minimum(np.min(q, axis=0)-limits[:, 0], limits[:, 1]-np.max(q, axis=0))


def metrics(q, v, commands, reference, fine_q, fine_v, initial, dt, substeps,
            timings, limits, residuals):
    command_v = np.diff(np.vstack((initial, commands)), axis=0)/dt
    command_a = np.diff(np.vstack((np.zeros(7), command_v)), axis=0)/dt
    command_j = np.diff(np.vstack((np.zeros(7), command_a)), axis=0)/dt
    feedback_a = np.diff(np.vstack((np.zeros(7), fine_v)), axis=0)/(dt/substeps)
    error = q-reference
    return {
        "rmse_joint_deg": float(np.sqrt(np.mean(error**2))),
        "rmse_per_joint_deg": np.sqrt(np.mean(error**2, axis=0)).tolist(),
        "max_abs_tracking_error_deg": float(np.max(np.abs(error))),
        "max_command_deviation_from_reference_deg": float(np.max(np.abs(commands-reference))),
        "final_max_error_deg": float(np.max(np.abs(error[-1]))),
        "max_command_step_deg": float(np.max(np.abs(command_v))*dt),
        "max_command_velocity_deg_s": float(np.max(np.abs(command_v))),
        "max_command_acceleration_deg_s2": float(np.max(np.abs(command_a))),
        "rms_command_acceleration_deg_s2": float(np.sqrt(np.mean(command_a**2))),
        "max_command_jerk_deg_s3": float(np.max(np.abs(command_j))),
        "max_feedback_velocity_deg_s": float(np.max(np.abs(fine_v))),
        "max_feedback_acceleration_deg_s2": float(np.max(np.abs(feedback_a))),
        "rms_feedback_acceleration_deg_s2": float(np.sqrt(np.mean(feedback_a**2))),
        "command_margin_to_experiment_limits_deg": margin(commands, limits).tolist(),
        "feedback_substep_margin_to_experiment_limits_deg": margin(fine_q, limits).tolist(),
        "max_qp_constraint_residual": max(residuals, default=0.0),
        "solver_wall_ms": ({"p50": float(np.percentile(timings, 50)),
                            "p95": float(np.percentile(timings, 95)),
                            "p99": float(np.percentile(timings, 99)),
                            "max": float(np.max(timings)),
                            "over_period_count": int(np.sum(np.asarray(timings)>dt*1000))}
                           if timings else None),
    }


def simulate(blocks, mode, scenario, settings, gui=False):
    dt = settings.dt
    first = blocks[0]["q"][0]
    # mismatch 故意让仿真位置响应变慢；MPC 仍使用同一个 alpha，不重新调参。
    plant = BulletPlant(first, dt, gui=gui, position_gain=0.03 if scenario == "lag_mismatch" else 0.06)
    try:
        policy = contract.policy()
        # 实验可用命令上限只从现有 Servo policy 引入；不复制 j7/HOME/SEED/T。
        vmax = min(policy["max_velocity_deg_s"], policy["max_step_deg"]/dt)
        mpc = JointMPC(plant.limits, vmax, policy["max_acceleration_deg_s2"], settings)
        all_ref = np.vstack([b["q"] for b in blocks])
        if np.min(margin(all_ref, plant.limits)) < 0:
            raise ValueError("原轨迹超出共享软限位与 URDF 硬限位交集")
        command, previous_v = first.copy(), np.zeros(7)
        qs, vs, refs, commands, fine_qs, fine_vs = [], [], [], [], [], []
        timings, residuals, event_checks, grip_positions = [], [], [], []
        k = 0
        run_start = time.perf_counter()
        for block in blocks:
            reference = block["q"]
            for index, ref in enumerate(reference):
                started = time.perf_counter()
                measured, _ = plant.read()
                if mode == "mpc":
                    idx = np.minimum(np.arange(index, index+settings.horizon), len(reference)-1)
                    result = mpc.step(measured, command, previous_v, reference[idx])
                    command, previous_v = result["command_deg"], result["velocity_deg_s"]
                    timings.append(result["solve_wall_ms"])
                    residuals.append(result["constraint_residual"])
                else:
                    command = ref.copy()
                force = ([0.0, 12.0, 0.0] if scenario == "external_force"
                         and 25.0 <= k*dt < 25.3 else None)
                sample = plant.step(command, force)
                q, v = sample[-1]
                if not np.isfinite(q).all() or not np.isfinite(v).all():
                    raise MPCError("PyBullet 反馈非有限，停止实验")
                for fq, fv in sample:
                    if np.min(margin(fq[None, :], plant.limits)) < -settings.constraint_tolerance:
                        raise MPCError("仿真子步反馈越过限位，停止实验")
                    fine_qs.append(fq)
                    fine_vs.append(fv)
                qs.append(q); vs.append(v); refs.append(ref); commands.append(command.copy())
                grip_positions.append(plant.grip_pose())
                k += 1
                if k % 700 == 0:
                    print(f"{scenario}/{mode}: {k}/{len(all_ref)} cycles", flush=True)
                if gui:
                    time.sleep(max(0.0, dt-(time.perf_counter()-started)))
            if block["event"]:
                # 本实验只核验逻辑事件，无 COM/夹爪驱动，无接触/抓取成功判定。
                error = float(np.max(np.abs(q-reference[-1])))
                speed = float(np.max(np.abs(v)))
                passed = error <= 0.25 and speed <= 0.5
                event_checks.append({**block["event"], "at_cycle": k,
                                     "max_joint_error_deg": error,
                                     "max_joint_speed_deg_s": speed,
                                     "status": "SIM_ENDPOINT_SETTLED" if passed else "FAIL",
                                     "gripper_actuated": False})
                if not passed:
                    raise MPCError("夹爪事件边界未到位/未停稳；不跳到下一阶段")
        arrays = {"q": np.asarray(qs), "v": np.asarray(vs), "reference": np.asarray(refs),
                  "command": np.asarray(commands), "grip_pb_mm": np.asarray(grip_positions),
                  "solve_wall_ms": np.asarray(timings),
                  "fine_q": np.asarray(fine_qs), "fine_v": np.asarray(fine_vs)}
        summary = metrics(arrays["q"], arrays["v"], arrays["command"], arrays["reference"],
                          arrays["fine_q"], arrays["fine_v"], first, dt, plant.substeps,
                          timings, plant.limits, residuals)
        summary.update({"mode": mode, "scenario": scenario, "cycles": k,
                        "simulation_duration_s": k*dt, "wall_duration_s": time.perf_counter()-run_start,
                        "event_checks": event_checks, "limits_deg": plant.limits.tolist(),
                        "urdf_limits_sdk_deg": plant.urdf_limits.tolist(),
                        "plant": {"gravity_m_s2": [0, 0, 0], "position_gain": plant.position_gain,
                                  "velocity_gain": 1.0, "substeps": plant.substeps,
                                  "effort_source": "unchanged URDF joint effort",
                                  "inertia_source": "PyBullet collision-shape estimate, NOT calibrated inertia",
                                  "invalid_original_inertias": plant.inertia_issues,
                                  "self_environment_payload_collision": "NOT_MODELED",
                                  "external_force": ({"world_N": [0, 12, 0], "start_s": 25.0,
                                                      "end_s": 25.3, "point": "EE link COM"}
                                                     if scenario == "external_force" else None)}})
        return summary, arrays
    finally:
        plant.close()


def single_joint_demo(settings):
    """教学数值模型，不是机械臂限位；超界参考必须被约束而不是跟过去。"""
    # 采用与局部跟踪同量级的1度教学动作，不将大角度跳变当成可执行参考。
    s = Settings(dt=settings.dt, horizon=settings.horizon, response_alpha=1.0)
    results = []
    for speed in (1.0, 0.3):
        mpc = JointMPC([[0, 1]], speed, 6.0, s)
        q, c, v = np.zeros(1), np.zeros(1), np.zeros(1)
        states, controls = [], []
        for _ in range(round(5/s.dt)):
            r = mpc.step(q, c, v, np.full((s.horizon, 1), 1.2))
            c, v = r["command_deg"], r["velocity_deg_s"]
            q = c.copy()  # 此教学案例明确采用理想积分器，不冒称 PB 物理反馈。
            states.append(float(q[0])); controls.append(float(v[0]))
        results.append({"velocity_limit_deg_s": speed, "requested_target_deg": 1.2,
                        "teaching_position_upper_deg": 1.0, "final_deg": states[-1],
                        "maximum_deg": max(states), "max_speed_deg_s": max(map(abs, controls)),
                        "time_to_0_99deg_s": next(((i+1)*s.dt for i, q in enumerate(states) if q >= 0.99), None)})
    return results


def write_plot(path, traces, dt):
    os.environ.setdefault("MPLCONFIGDIR", str(ROOT/".venv-mpc/matplotlib"))
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), constrained_layout=True)
    for key, data in traces.items():
        if not key.startswith("nominal/"):
            continue
        label = key.split("/")[-1]
        t = np.arange(1, len(data["q"])+1)*dt
        axes[0].plot(t, np.linalg.norm(data["q"]-data["reference"], axis=1), label=label)
        c = data["command"]
        velocity = np.diff(c, axis=0)/dt
        axes[1].plot(t[2:], np.max(np.abs(np.diff(velocity, axis=0)/dt), axis=1), label=label)
        if label == "mpc":
            axes[2].plot(t, data["solve_wall_ms"], label="MPC update + solve + verify")
    axes[0].set_ylabel("Joint error L2 (deg)")
    axes[1].set_ylabel("Max |command accel| (deg/s^2)")
    axes[2].set_ylabel("Solver wall time (ms)")
    axes[2].set_xlabel("Simulated time (s)")
    axes[2].axhline(dt*1000, color="red", linestyle="--", label="20 ms control period")
    for ax in axes:
        ax.grid(alpha=0.25); ax.legend(loc="upper right")
    fig.suptitle("7-DOF PyBullet position-servo tracking (NOT real robot / NOT collision qualification)")
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--reference", type=Path, default=DEFAULT_REFERENCE)
    parser.add_argument("--gui", action="store_true", help="仅 MPC nominal 动态仿真；不覆盖批量结果")
    args = parser.parse_args(argv)
    settings = Settings(dt=contract.policy()["period_s"])
    plan, blocks, source_metrics = load_reference(args.reference, settings.dt)
    if args.gui:
        result, _ = simulate(blocks, "mpc", "nominal", settings, gui=True)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0
    scenarios, traces = {}, {}
    teaching = single_joint_demo(settings)
    for scenario in ("nominal", "lag_mismatch", "external_force"):
        for mode in ("interpolation", "mpc"):
            key = f"{scenario}/{mode}"
            summary, data = simulate(blocks, mode, scenario, settings)
            scenarios[key], traces[key] = summary, data
            print(f"{key}: RMSE={summary['rmse_joint_deg']:.6f} deg, "
                  f"command accel={summary['max_command_acceleration_deg_s2']:.3f} deg/s²", flush=True)
    out = HERE / "results"
    out.mkdir(exist_ok=True)
    compact = {f"{key.replace('/', '__')}__{field}": values
               for key, data in traces.items() for field, values in data.items()}
    np.savez_compressed(out/"traces.npz", **compact)
    write_plot(out/"comparison.png", traces, settings.dt)
    source_paths = [Path(args.reference).resolve(), Path(arm_profiles._config_path()),
                    Path(cc.DEFAULT_URDF), Path(cc.__file__), Path(arm_profiles.__file__),
                    Path(contract.__file__), Path(contract.ss.__file__),
                    HERE/"controller.py", HERE/"run_experiment.py"]
    result = {"schema_version": "offline_joint_mpc_experiment.v1",
              "generated_at_utc": datetime.now(timezone.utc).isoformat(),
              "status": "OFFLINE_EXPERIMENT_COMPLETED_NOT_HARDWARE_QUALIFIED",
              "real_motion_authorized": False, "debian_execution_allowed": False,
              "source_reference": str(Path(args.reference).resolve().relative_to(ROOT)),
              "source_metrics": source_metrics, "arm_id": plan["meta"]["arm_id"],
              "gripper_id": plan["meta"]["gripper_id"],
              "settings": asdict(settings), "one_dof_teaching": teaching, "scenarios": scenarios,
              "source_sha256": {str(p.relative_to(ROOT)): sha(p) for p in source_paths},
              "artifacts_sha256": {name: sha(out/name) for name in ("traces.npz", "comparison.png")},
              "versions": {n: importlib.metadata.version(n) for n in
                           ("osqp", "numpy", "scipy", "pybullet", "matplotlib")},
              "python": platform.python_version(), "platform": platform.platform(),
              "limitations": ["No physical robot, SDK session, torque control or network connected",
                              "URDF elbow inertias are invalid; PB uses shape-estimated inertia, not identified dynamics",
                              "No registered table/bin, self-collision, physical gripper or held object",
                              "No latest field VAJ3 source: comparison uses archived dense reference at 20ms",
                              "Gripper events are simulated endpoint/settling assertions, not grasp success",
                              "Joint constraints do not ensure TCP height, attitude or collision clearance",
                              "Solver timings are local Mac observations, not Debian hard real-time guarantees",
                              "New commands alter the path; old GUI/hardware approval is not inherited"]}
    (out/"report.json").write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    print(f"Report: {out/'report.json'}", flush=True)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValueError, MPCError) as exc:
        print(f"OFFLINE EXPERIMENT ABORTED: {exc}", file=sys.stderr)
        raise SystemExit(2)
