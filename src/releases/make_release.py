#!/usr/bin/env python3
"""把某条线的"能跑的全套"打成一个自包含目录, 丢到真机上就能直接跑。

为什么要这个: 代码平时散在 frame_calibration/robot_side/ 和 pick_place_coord/
trajectories/ 两处, 上真机要手工挑文件, 挑漏一个(比如 sdk_session.py)就白跑一趟。
封装后每条线一个目录, 里面是执行器 + 依赖 + 轨迹 + 说明 + 校验和, 拷过去即可。

用法:
    python3 releases/make_release.py B          # 只打 Track B
    python3 releases/make_release.py A B C      # 全打
    python3 releases/make_release.py --verify   # 只校验已有封装有没有被改动

校验: 每个封装目录带 SHA256SUMS, --verify 会逐个比对, 发现漂移就报出来。
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import sys
import zipfile

_HERE = os.path.dirname(os.path.abspath(__file__))
_WORK = os.path.dirname(_HERE)
ROBOT = os.path.join(_WORK, "frame_calibration", "robot_side")
TRAJ = os.path.join(_WORK, "pick_place_coord", "trajectories")
SCHEMAS = os.path.join(_WORK, "schemas")
CANDIDATES = os.path.join(TRAJ, "candidates")

# 运行必需的共同依赖: 三条线的执行器都 import sdk_session, 漏了就 ImportError。
# (sdk_session 自己只依赖 pypilot —— 那是装在真机容器里的 SDK 轮子, 不打包。)
COMMON = [(ROBOT, "sdk_session.py"), (ROBOT, "robot_lock.py")]
# 附带工具: 没有任何执行器 import 它, 不装也能跑。放进来只是因为日志是在真机
# 侧产生的, 外发/归档前就地脱敏最省事。
TOOLS = [(ROBOT, "scrub_log.py"), (ROBOT, "test_servo_safety.py")]
NOTES = {
    "sdk_session.py": "SDK 会话/运动封装 —— 执行器 import 它, 必需",
    "servo_common.py": "透传公共件: 节拍器 + 跟随误差监控 —— 两条透传线都 import, 必需",
    "scrub_log.py": "日志脱敏工具 —— 没人 import, 可不装; 日志外发前过一遍",
    "diag_joint_transfer.py": "空爪诊断 armMoveJoints 大行程是慢还是停; 排查用, 不参与抓取",
    "probe_sdk.py": "探测本机 SDK 实际有哪些接口/什么签名; 不连机器人, 随时可跑",
    "diag_chassis.py": "底盘(AGV)状态诊断; 全程只读不发运动指令, 不碰机械臂, 可与手臂脚本同时跑",
    "test_worlds_servo_minimal.py": "armWorldsToServo 最小验证(局部精修能力); 参数走命令行, 逐点预检 + 硬验收门限",
    "robot_lock.py": "机器人互斥锁 —— 四个执行器都 import, 必需; 同一台臂同时只允许一个进程",
    "test_servo_safety.py": "离线回归测试(不连机器人): 遥测关闭/节拍/向量类型/互斥锁/只读会话/速度还原",
    "arm_profiles.py": "双臂配置加载器；右臂未测项会强制阻断真机",
    "arm_profiles.v1.json": "左右臂关节/TCP/HOME/限位/夹爪契约唯一权威",
    "execute_tabletop_pick_place_worlds.py": "两 hover 桌上抓放执行器；默认只做 live armTryWorlds 预检",
    "tabletop_pick_place_worlds.json": "Mac 端已完成 TCP 补偿的绝对 SDK endpoint 计划",
    "execution_authorization.py": "校验 preflight、人工批准和场景复核的绑定关系",
    "soft_stop_watchdog.py": "视觉下发热循环之外的独立只读软急停监控",
    "mission_runner.py": "在一个机器人级锁内严格串行执行跨左右臂任务",
    "traj_bottle_taught_tcp_20260831_RUN.json": "新桌面示教姿态左臂瓶子完整 RUN 轨迹；当前完整运行日志待归档",
    "PilotSDK_enable_varget_8080_debug_20260901.md": "8080 旧监听进程导致 PilotSDK 使能状态异常的现场排障手册；不参与运行",
}

# 同一任务只保留一个现行发布目录。这里列出的都是 make_release 生成的旧快照；
# 候选、任务、资格报告和 GUI 记录仍保留在各自真源目录，不在这里清理。
SUPERSEDED_RELEASES = {
    "CB": (
        "trackC_bottle_left_precheck",
        "trackC_bottle_left_orientation_precheck",
        "trackC_bottle_left_vertical_run",
    ),
}

TRACKS = {
    "DP": {
        "dir": "dual_arm_precheck",
        "title": "双臂候选 · Debian 只读限位诊断（不运动）",
        "files": [(ROBOT, "precheck_dual_arm.py"),
                  (_WORK, "arm_profiles.py"),
                  (_WORK, "arm_profiles.v1.json"),
                  (os.path.join(_WORK, "pick_place_coord", "dual_arm_demo"),
                   "dual_arm_simulation.json"),
                  (os.path.join(_WORK, "pick_place_coord", "dual_arm_demo"),
                   "validation_report.json")],
        "tools": [(ROBOT, "scrub_log.py")],
        "minimal_release": True,
        "zip_archive": True,
        "entry": "python3 precheck_dual_arm.py --precheck-only",
        "status": "READONLY_DIAGNOSTIC / MOTION_BLOCKED。读取左右臂实时状态和控制器限位，"
                  "与仿真候选逐帧比较；本包没有双臂运动入口，不继承单臂真机 PASS。",
    },
    "AS": {
        "dir": "trackA_servo",
        "title": "Track A Servo · 最终成功 A 路线派生候选",
        "files": [(ROBOT, "execute_tabletop_servo.py"),
                  (ROBOT, "tabletop_servo_contract.py"),
                  (ROBOT, "servo_common.py"),
                  (ROBOT, "execute_tabletop_hybrid_trial_reviewfix_field.py"),
                  (ROBOT, "execute_tabletop_pick_place_worlds.py"),
                  (ROBOT, "precheck_tabletop_hybrid.py"),
                  (_WORK, "arm_profiles.py"),
                  (_WORK, "arm_profiles.v1.json"),
                  (CANDIDATES, "tabletop_pick_place_SERVO_CANDIDATE.json")],
        "tools": [(ROBOT, "scrub_log.py")],
        "minimal_release": True,
        "zip_archive": True,
        "gui_review_pair": ((CANDIDATES, "tabletop_pick_place_SERVO_CANDIDATE.json"),
                            (CANDIDATES, "tabletop_pick_place_SERVO_gui_review.json")),
        "entry": "python3 execute_tabletop_servo.py tabletop_pick_place_SERVO_CANDIDATE.json --dry-run",
        "status": "SOFTWARE_CANDIDATE / MOTION_BLOCKED：关节流与执行器已生成；可离线或只读预检，"
                  "新 Servo 的完整场景/持物碰撞与动态时序资格尚未通过，不继承 A 的真机 PASS。",
        "immutable_sources": [
            (CANDIDATES, "tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json",
             "e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59"),
            (ROBOT, "execute_tabletop_hybrid_trial_reviewfix_field.py",
             "36a0f659122eb492f63616f5c34a7e3b1f342c67f28e9906e2b9d6ad5b04e7ee"),
        ],
    },
    "AT": {
        "dir": "trackA_hybrid_trial",
        "title": "Track A 混合抓放 · Debian 20260908 最终现场版",
        "files": [(ROBOT, "execute_tabletop_hybrid_trial_reviewfix_field.py"),
                  (ROBOT, "precheck_tabletop_hybrid.py"),
                  (ROBOT, "execute_tabletop_pick_place_worlds.py"),
                  (CANDIDATES, "tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json")],
        "tools": [(ROBOT, "scrub_log.py")],
        "minimal_release": True,
        "zip_archive": True,
        "entry": "python3 execute_tabletop_hybrid_trial_reviewfix_field.py tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json --precheck-only",
        "status": "FIELD_TESTED：原样收到的计划/执行器在5%和10%均有完整动作及收尾PASS日志。"
                  "仅限此次现场与文件组合，不推定任意新坐标或20%已验证。",
        "immutable_sources": [
            (CANDIDATES, "tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json",
             "e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59"),
            (ROBOT, "execute_tabletop_hybrid_trial_reviewfix_field.py",
             "36a0f659122eb492f63616f5c34a7e3b1f342c67f28e9906e2b9d6ad5b04e7ee"),
        ],
    },
    "AP": {
        "dir": "trackA_hybrid_precheck",
        "title": "Track A 混合轨迹 · Debian 只读测试包（不运动）",
        "files": [(ROBOT, "precheck_tabletop_hybrid.py"),
                  (ROBOT, "execute_tabletop_pick_place_worlds.py"),
                  (CANDIDATES, "tabletop_pick_place_hybrid_CANDIDATE.json"),
                  (CANDIDATES, "tabletop_pick_place_hybrid_gui_playback.json")],
        "tools": [(ROBOT, "scrub_log.py")],
        "minimal_release": True,
        "zip_archive": True,
        "entry": "python3 precheck_tabletop_hybrid.py tabletop_pick_place_hybrid_CANDIDATE.json --precheck-only",
        "status": "READONLY_TEST_PACKAGE / MOTION_BLOCKED；当前没有混合运动执行器。",
    },
    "A": {
        "dir": "trackA_worlds_line",
        "title": "Track A · 水平抓取 + 受限启动回 safe（MoveWorlds）",
        "files": [(ROBOT, "execute_tabletop_pick_place_worlds.py"),
                  (CANDIDATES, "tabletop_pick_place_worlds.json")],
        "minimal_release": True,
        "entry": ("XIFENG_ROBOT_IP=<机器人IP> XIFENG_LOCAL_IP=<本机IP> "
                  "XIFENG_ARM_IP=<机械臂IP> \\\n"
                  "  python3 execute_tabletop_pick_place_worlds.py "
                  "tabletop_pick_place_worlds.json --precheck-only"),
        "status": "SOFTWARE_CANDIDATE / OFFLINE_CANDIDATE_BLOCKED。水平抓取与启动恢复已实现；"
                  "Z445 放置 EE 点按 20260905 摘要重建，父 executor/JSON 精确哈希均匹配。"
                  "当前仅交付 live 无运动预检；新水平段、持瓶转运和恢复区域净空尚待确认，非 REALVERIFIED。",
    },
    "AH": {
        "dir": "trackA_hybrid_joints",
        "title": "Track A(hybrid 留档) · worlds 直线 + 一条 joints 转运",
        "files": [(ROBOT, "execute_world_grasp.py"),
                  (TRAJ, "world_grasp_joint_transfer.json")],
        "tools": [(ROBOT, "diag_joint_transfer.py")],
        "entry": "python3 execute_world_grasp.py world_grasp_joint_transfer.json "
                 "--mode hybrid",
        "status": "⚠ 留档研究用, 20260729 真机三次未跑通 —— 详见下方「已知问题」",
    },
    "B": {
        "dir": "trackB_move_joints",
        "title": "Track B · 关节路点(armMoveJoints 逐点停稳)",
        "files": [(ROBOT, "execute_trajectory.py"),
                  (TRAJ, "traj_minimal_joint.json"),
                  (TRAJ, "traj_5pt.json"),
                  (TRAJ, "traj_4pt.json")],
        "entry": "python3 execute_trajectory.py traj_minimal_joint.json",
        "status": "真机已验收(traj_minimal_joint.json 三次成功)",
    },
    "C": {
        "dir": "trackC_servo_stream",
        "title": "Track C · 关节伺服透传(armPluseToServo 密集流式)",
        "files": [(ROBOT, "execute_servo_grasp.py"),
                  (ROBOT, "servo_common.py"),
                  (_WORK, "arm_profiles.py"),
                  (_WORK, "arm_profiles.v1.json"),
                  (_WORK, "execution_authorization.py"),
                  (TRAJ, "traj_minimal_joint.json"),
                  (TRAJ, "traj_multi_latest.json")],
        "tools": [(ROBOT, "probe_sdk.py"), (ROBOT, "diag_chassis.py")],
        "entry": "python3 execute_servo_grasp.py traj_multi_latest.json "
                 "--step-deg 0.4 --speed 15",
        "status": "真机已验收 —— 20260804 双抓放(traj_multi_latest.json, 133 路点/"
                  "4 次夹爪)连跑 5 轮全过, 推荐基线 0.4°/20ms: 32.7s, 终点误差 "
                  "0.048°, 迟到帧 1.4%。轨迹优化空间见 records/"
                  "20260804_pulse_multi_grasp_verified/",
    },
    "CB": {
        "dir": "trackC_bottle_left",
        "title": "Track C-bottle · 新桌面左臂示教姿态瓶子抓放",
        "files": [(ROBOT, "execute_servo_grasp.py"),
                  (ROBOT, "servo_common.py"),
                  (_WORK, "arm_profiles.py"),
                  (_WORK, "arm_profiles.v1.json"),
                  (_WORK, "execution_authorization.py"),
                  (CANDIDATES,
                   "traj_bottle_taught_tcp_20260831_RUN.json")],
        "tools": [(_WORK, "PilotSDK_enable_varget_8080_debug_20260901.md")],
        "minimal_release": True,
        "entry": ("XIFENG_ALLOW_REAL_MOTION=1 python3 execute_servo_grasp.py \\\n"
                  "  traj_bottle_taught_tcp_20260831_RUN.json \\\n"
                  "  --run --step-deg 0.4 --period-ms 20 --speed 8"),
        "status": "OPERATOR_REPORTED_5X_PASS / INSUFFICIENT_EVIDENCE / NOT_REAL_VERIFIED（2026-09-01）。"
                  "当前 RUN SHA-256 为 `cce2a83fae21468f80f0aad487087b61650eac6f92dea65f9560091d0239ad2b`，"
                  "共 275 个运动路点及 close/open 事件。前 238 个运动路点和两次夹爪事件与"
                  "已完成真机抓放的旧 RUN（264808c7…）逐项完全一致；仅在旧终点 READY 后追加"
                  "父 Track C 已验证的 37 点 READY->HOME 收尾，最终关节与起始 HOME 完全相同。"
                  "1356 个 Servo 密集帧（GUI 含首帧 1357）已通过限位、自碰撞、"
                  "机器人/新桌面/夹爪/刚性携带瓶体及放置后回撤检查；GUI 全程播放后用户确认 PASS。"
                  "桌角使用用户确认的 EE link 原点，不把夹爪或配置 TCP 当取坐标点；抓放 EE Z"
                  "相同、双 hover EE Z 相同，并锁定 pick 朝向避免瓶底倾斜穿桌。"
                  "旧前缀已在现场以 882 帧完成抓放，终点误差 0.024°，保护与速度恢复。"
                  "操作者记录当前完整 SHA 在清理占用 TCP 8080 的旧 PilotSDK/Python 进程后连续运行五次成功，"
                  "并已附带使能异常排障手册；但五次运行的完整日志尚未进入工作区。"
                  "同时，现场脚本 SHA 前缀 bc05bd76… 与本发布脚本 73be5ab6… 不同，且原始日志与"
                  "现场脚本文件未提供，因此当前完整组合证据 INSUFFICIENT、不得称 REAL_VERIFIED。"
                  "现行发布包仅保留此 RUN；候选、任务、资格证据与脱敏真机记录留在工作区真源目录。",
    },
    "CW": {
        "dir": "trackC_worlds_servo",
        "title": "Track C-worlds · 末端伺服透传(armWorldsToServo)",
        "files": [(ROBOT, "execute_worlds_servo_grasp.py"),
                  (ROBOT, "servo_common.py"),
                  (TRAJ, "world_grasp_latest.json")],
        "tools": [(ROBOT, "probe_sdk.py"),
                  (ROBOT, "test_worlds_servo_minimal.py")],
        "entry": "python3 test_worlds_servo_minimal.py --run  "
                 "# 先跑这个; 完整抓放执行器目前只允许 --precheck-only",
        "status": "实验线【局部笛卡尔精修】。20260804 armWorldsToServo 最小验证 5 轮"
                  "全过(30~50mm 平移 / 10~20° W 旋转 / 25~50mm/s, 位置≤0.48mm "
                  "姿态≤0.17° 预测差≤0.96°)。⚠ 完整抓放【未验收】: "
                  "execute_worlds_servo_grasp.py 走的是 armTryWorlds+armPluseToServo "
                  "而非 ToServo, 且有起手阶跃缺陷, 目前只应 --precheck-only",
    },
    "V": {
        "dir": "trackV_vision_dualarm",
        "title": "Track V · 固定工位双臂视觉抓取（软件候选，硬件门阻断）",
        "files": [(ROBOT, "execute_servo_grasp.py"),
                  (ROBOT, "servo_common.py"),
                  (ROBOT, "soft_stop_watchdog.py"),
                  (os.path.join(_WORK, "robot_mission"), "mission_runner.py"),
                  (_WORK, "arm_profiles.py"),
                  (_WORK, "arm_profiles.v1.json"),
                  (_WORK, "execution_authorization.py"),
                  (os.path.join(TRAJ, "verified"),
                   "traj_multi_2grasp_20260804_REALVERIFIED.json"),
                  (CANDIDATES, "traj_multi_right_20260806_CANDIDATE.json"),
                  (SCHEMAS, "vision_observation.v1.schema.json"),
                  (SCHEMAS, "task_request.v1.schema.json"),
                  (SCHEMAS, "trajectory.v2.schema.json"),
                  (SCHEMAS, "preflight_report.v1.schema.json"),
                  (SCHEMAS, "eye_to_hand_calibration.v1.schema.json")],
        "tools": [(ROBOT, "test_gripper_right.py")],
        "entry": "python3 execute_servo_grasp.py "
                 "traj_multi_2grasp_20260804_REALVERIFIED.json "
                 "--step-deg 0.4 --period-ms 20 --speed 15",
        "status": "SOFTWARE_CANDIDATE / HARDWARE_BLOCKED。左臂轨迹仍是 5/5 "
                  "REAL_VERIFIED 父基线；右臂轨迹仅 CANDIDATE，真实限位/TCP/"
                  "SDK 映射/worlds/手眼标定完成前禁止 --run。视觉 trajectory.v2 "
                  "还必须带 PASS preflight、人工批准和批准后场景复核。",
    },
}


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 16), b""):
            h.update(chunk)
    return h.hexdigest()


def matching_gui_review(plan_path, review_path):
    """发布时不携带旧轨迹的PASS；执行器仍负责完整资格校验。"""
    try:
        with open(review_path, encoding="utf-8") as handle:
            review = json.load(handle)
        return (isinstance(review, dict)
                and review.get("schema_version") == "pybullet_gui_review.v1"
                and review.get("trajectory_sha256") == sha256(plan_path)
                and review.get("result") == "PASS"
                and review.get("full_replay_completed") is True)
    except (OSError, ValueError):
        return False


def build(key):
    spec = TRACKS[key]
    out = os.path.join(_HERE, spec["dir"])
    # [20260908] 先确认实跑参考未变，再动本线路的生成目录；不覆盖成功参考。
    for src_dir, name, expected in spec.get("immutable_sources", []):
        source = os.path.join(src_dir, name)
        if not os.path.isfile(source) or sha256(source) != expected:
            raise SystemExit(f"✗ 实跑参考哈希不符: {source}；保留现有发布目录")
    default_tools = [] if spec.get("minimal_release") else TOOLS
    required_files = list(spec["files"])
    if spec.get("gui_review_pair"):
        plan_source, review_source = spec["gui_review_pair"]
        if matching_gui_review(os.path.join(*plan_source), os.path.join(*review_source)):
            required_files.append(review_source)
        else:
            print("提示: 当前轨迹尚无匹配GUI人工PASS，旧或未确认的GUI审核不打入新包。")
    for src_dir, name in required_files + COMMON + default_tools + spec.get("tools", []):
        if not os.path.isfile(os.path.join(src_dir, name)):
            raise SystemExit(f"✗ 缺文件: {os.path.join(src_dir, name)}；保留现有发布目录")
    if key == "DP":
        # [20260918] 只读包也须绑定本次候选，不能把旧碰撞报告附到新轨迹。
        validate_dual_precheck_sources(spec)
    if os.path.isdir(out):
        shutil.rmtree(out)
    os.makedirs(out)
    need, extra = [], []
    for bucket, items in ((need, required_files + COMMON),
                          (extra, default_tools + spec.get("tools", []))):
        for src_dir, name in items:
            src = os.path.join(src_dir, name)
            if not os.path.exists(src):
                raise SystemExit(f"✗ 缺文件: {src}")
            shutil.copy2(src, os.path.join(out, name))
            bucket.append(name)
    names = need + extra

    lines = [f"{sha256(os.path.join(out, n))}  {n}" for n in sorted(names)]
    with open(os.path.join(out, "SHA256SUMS"), "w") as f:
        f.write("\n".join(lines) + "\n")

    with open(os.path.join(out, "README.md"), "w", encoding="utf-8") as f:
        f.write(render_readme(key, spec, need, extra))
    print(f"✓ {spec['dir']}: {len(names)} 个文件")
    for ln in lines:
        print(f"    {ln[:16]}  {ln[66:]}")
    remove_superseded(key, keep=out)
    if key == "A" or spec.get("zip_archive"):
        # 由唯一发布器重建现行压缩包，显式文件列表排除缓存和 __MACOSX。
        archive = out + ".zip"
        with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as handle:
            for name in sorted(names + ["README.md", "SHA256SUMS"]):
                handle.write(os.path.join(out, name), arcname=spec["dir"] + "/" + name)
        print(f"✓ {archive} SHA-256: {sha256(archive)}")
    return out


def remove_superseded(key, keep):
    """只删除与当前线路明确绑定的旧生成快照，不触碰真源和证据目录。"""
    release_root = os.path.realpath(_HERE)
    keep = os.path.realpath(keep)
    for name in SUPERSEDED_RELEASES.get(key, ()):
        target = os.path.join(_HERE, name)
        real_target = os.path.realpath(target)
        if real_target == keep or os.path.dirname(real_target) != release_root:
            raise RuntimeError(f"拒绝清理非发布目录: {target}")
        if os.path.isdir(real_target):
            shutil.rmtree(real_target)
            print(f"  - 已移除被替代的生成包: {name}/")
        elif os.path.isfile(real_target):
            os.remove(real_target)
            print(f"  - 已移除被替代的生成文件: {name}")


def render_readme(key, spec, need, extra):
    if key == "DP":
        return render_dual_precheck_readme(spec, need, extra)
    if key == "AS":
        return render_tabletop_servo_readme(spec, need, extra)
    if key == "AT":
        return render_hybrid_field_readme(spec, need, extra)
    if key == "AP":
        return render_hybrid_precheck_readme(spec, need, extra)
    checksum_command = ("sha256sum -c SHA256SUMS" if key in ("A", "CB") else
                        "shasum -a 256 -c SHA256SUMS")
    if key == "V":
        run_gate = ["视觉线真机执行必须同时设置 `XIFENG_ALLOW_REAL_MOTION=1`、给 `--run`，",
                    "并提供运行时网络参数、PASS preflight、批准令牌、场景复核、视觉验收",
                    "命令和软急停/场景两个独立看门狗命令。",
                    "缺任一项都会拒绝下发。", ""]
    elif key == "CB":
        run_gate = ["运行前设置本次现场的 `XIFENG_ROBOT_IP`、`XIFENG_LOCAL_IP`、",
                    "`XIFENG_ARM_IP`；不要把历史 `.147`/`.148` 写死。确认机器人、",
                    "左臂、夹爪/TCP、腰部、底盘、桌子和瓶子与此次已确认的固定场景相同。", "",
                    "执行器会先检查本地限位，再把控制器回读限位与 J7<=76deg 物理",
                    "包络取交集复检。当前位置离首点超过 20deg 时只会要求人工确认",
                    "进入路径；点到点到首点并停稳后，还会再次等你回车才开始 Servo。",
                    "人守急停；异常时不会自动松开持物夹爪。", ""]
    elif key == "A":
        run_gate = [
            "本包先在 Debian 做只读预检。它不会清报警、使能、开串口或运动。",
            "不要沿用旧 python -c 命令中清空 controller_limit_violations 的 monkeypatch。",
            "如果需按零内缩余量诊断，用 `--limit-margin-deg 0`，实时控制器限位和 j7 物理限位仍执行。", "",
            "## 本次动作", "",
            "`SAFE → [550,250,410] → [700,250,410] → 闭爪 → [700,250,480]`",
            "`→ [720,100,550] → [720,100,445] → 松爪 → [720,100,550] → SAFE`", "",
            "全部 XYZ 是 SDK EE mm；无需再做 TCP 补偿。接近段命令 Y/Z 固定；",
            "实际反馈仍按到位容差检查，不承诺物理轨迹零误差。任务为 7 条 MoveWorlds、3 次夹爪动作。", "",
            "## 启动回位", "",
            "读取实时 Worlds/Joints；在 safe 附近则跳过；否则生成原位抬高、必要时高处转姿态、",
            "高处横移、下降到 safe，共 0~4 条 MoveWorlds。home 指任务 safe，不是配置关节 HOME。",
            "新计划到位容差 3 mm/1.5°，同时检查 safe 关节分支。旧 JSON 缺少 startup_policy 时不自动回位，",
            "并兼容 9/5 摘要的 50 mm/15° 判定容差。CLI 速度范围已统一为 0.1~20%，默认 1%。", "",
            "候选恢复范围：X=[550,720]、Y=[100,366.085]、Z=[443.775,550] mm，",
            "走廊高度 550 mm，UVW 各轴偏差不超过 15°；这些是只读测试参数，不是已验证安全区域。",
            "当前 `startup_policy.qualification.status=PENDING`，因此不在 safe 时只能诊断，不能自动使能回位。",
            "实际恢复前须补当前场景禁入盒（按腕部/开爪包络扩张），并确认整个受限区域的整臂净空；",
            "填写资格来源与场景版本。不能仅把 PENDING 改成 CONFIRMED 来绕过检查。",
            "已确认后仍须现场输入 EMPTY 确认空夹爪，再按回车使能。恢复时不操作夹爪，恢复完重新预检任务。", "",
            "## 两次只读测试", "",
            "先在当前非 safe 位置测试恢复路径：", "", "```bash",
            "python3 execute_tabletop_pick_place_worlds.py tabletop_pick_place_worlds.json --precheck-only --recovery-only",
            "```", "", "再测试恢复连接完整抓放路径：", "", "```bash",
            "python3 execute_tabletop_pick_place_worlds.py tabletop_pick_place_worlds.json --precheck-only",
            "```", "",
            "两条命令都要先设置现场 XIFENG_ROBOT_IP / XIFENG_LOCAL_IP / XIFENG_ARM_IP。",
            "保留完整输出，含 current_world/current_joints、恢复路点、各文件哈希和失败位置。",
            "若首轮因 5° 内缩余量失败，可加 --limit-margin-deg 0 做只读比较，不能取消真实限位检查。", "",
            "## 真机测试门", "",
            "当前新任务 BLOCKED：70 mm 抬升及向 PLACE_HOVER 的斜线仍需持瓶/腕部/桌框净空核对。",
            "armTryWorlds PASS 只证明所采样 IK/限位/连续性，不是碰撞 PASS；本轮未重复 PyBullet GUI。",
            "净空确认并重新生成后，使用以下命令，默认速度 1%；观察后再用 --speed 5：", "", "```bash",
            "XIFENG_ALLOW_REAL_MOTION=1 python3 execute_tabletop_pick_place_worlds.py tabletop_pick_place_worlds.json --run --recovery-only",
            "XIFENG_ALLOW_REAL_MOTION=1 python3 execute_tabletop_pick_place_worlds.py tabletop_pick_place_worlds.json --run",
            "```", "",
            "仅回位测试独立检查恢复资格；不执行抓放。全任务必须解除真实输入阻断后才能运行。", "",
            "## 版本来源与回退", "",
            "从归档 9/3 原件精确重建摘要所报 9/5 文件：executor e0118805…、Z445 JSON 8897c932…。",
            "新的组合不是原成功组合：新增抓取几何与恢复流程，限位过滤仍按当前真值检查。",
            "历史成功包/JSON 原件均保留。更新 Debian 前备份现行 executor；保留已有 *_placeZ445_*.json，",
            "运行本包时显式选择 tabletop_pick_place_worlds.json，避免选错旧文件。", "",
            "## Mac 再生成", "", "```bash",
            "/opt/anaconda3/bin/python -m robot_mission.tabletop_plan pick_place_coord/tasks/tabletop_horizontal_recovery_20260905.json --working-reference frame_calibration/records/20260903_tabletop_worlds_working/artifacts/tabletop_pick_place_worlds_PASS_placeZ495_20260903.json --out pick_place_coord/trajectories/candidates/tabletop_pick_place_worlds.json",
            "python3 releases/make_release.py A", "```", "",
            "抓取距离/抬升/放置参数集中在上述 Mac recipe；未来有明确几何时可用标准 tabletop_request.v1 生成器。", ""]
    else:
        run_gate = ["跑之前确认执行器顶部的开关：`ENABLE_REAL_MOTION` 默认 `False`",
                    "(只打印不下发)，确认打印无误后再改 `True`。真跑人守急停。", ""]
    body = [f"# {spec['title']}", "",
            f"**状态**: {spec['status']}", "",
            "**全新部署**用这个目录：整个拷过去就能跑，不用再从别处补文件。",
            "**已经跑过的机器**只需要补送变化的那几个文件，见下面「增量更新」。",
            "", "## 运行", "", "```bash",
            spec["entry"], "```", ""] + run_gate + [
            "## 文件", "", "运行必需（少一个就跑不起来）：", ""]
    for n in sorted(need):
        note = NOTES.get(n, "")
        body.append(f"- `{n}`{('  — ' + note) if note else ''}")
    body += ["", "另外还依赖 `pypilot`（厂商 SDK 轮子），那是装在真机容器里的，",
             "不在封装内。", ""]
    if extra:
        body += ["附带工具（不装也能跑）：", ""]
        for n in sorted(extra):
            note = NOTES.get(n, "")
            body.append(f"- `{n}`{('  — ' + note) if note else ''}")
        body.append("")
    if key == "AH":
        body += ["## 已知问题（先读这段再跑）", "",
                 "20260729 真机连续三次没跑通：", "",
                 "| 运行 | 现象 |", "|---|---|",
                 "| 1 | 预检说 `TRANSFER_END` j4 余量 3.3° 通过，执行前 "
                 "`check_limits` 又拒（`j4=-116.67` 超出收 5° 后的 `[-115,-5]`）|",
                 "| 2 | `armMoveJoints` 25s 未到位，残差 23.720° |",
                 "| 3 | `armMoveJoints` 25s 未到位，残差 18.533° |", "",
                 "第 1 条是代码缺陷，**已修**：`preflight()` 现在按最终生效的那套",
                 "标准判，会在**夹爪闭合之前**中止。", "",
                 "第 2/3 条**原因未定**。run3 的 j4 行程约 53°，25s 走了约 34°",
                 "（1.37°/s），按此速率还差约 14s——看起来像「走得慢没走完」，但",
                 "当时没有 `move_state`／报警／误差曲线，**排除不了中途真停住**。",
                 "这两种情况处置完全相反（前者加 `timeout_s` 即可，后者加多久都没用）。",
                 "", "`wait_until_joints` 现在会把这些一次打全（逐关节误差／已走多少／",
                 "平均速率／预计还需多久／`move_state`／`alarms`／判读提示），",
                 "并且真停住 4s 就报错，不干等到超时。", "",
                 "想直接定性就跑 `diag_joint_transfer.py`（90s 超时 + 5Hz 全程采样，",
                 "直接给「慢／停／报警」的判读）。", "",
                 "**两次超时都发生在夹爪已闭合、物体在手之后** —— 异常退出时夹爪",
                 "仍是闭合的，恢复现场前先确认物体和夹爪状态。", "",
                 "日常抓取请用 `../trackA_worlds_line/`，这份只作研究留档。", ""]
    body += ["## 增量更新", "",
             "真机上已经有这一套的话，按 `SHA256SUMS` 只补送哈希变化的文件。",
             "`sdk_session.py` 只在它自己被改过时才需要重送；不要再复制一套新目录。", "",
             "## 校验", "",
             "```bash", checksum_command, "```", "",
             "或在工作区根目录跑 `python3 releases/make_release.py --verify`。", "",
             "## 重新生成", "",
             "```bash", f"python3 releases/make_release.py {key}", "```", "",
             "封装是从工作区当前代码复制出来的快照，**不要直接改封装里的文件** —— ",
             "改源文件再重新生成，否则下次生成会把你的改动覆盖掉。", ""]
    return "\n".join(body)


def validate_dual_precheck_sources(spec):
    """生成前拒绝陈旧报告和误开的仿真运动标记，不删除现有包。"""
    sources = {name: os.path.join(folder, name) for folder, name in spec["files"]}
    try:
        with open(sources["dual_arm_simulation.json"], encoding="utf-8") as handle:
            plan = json.load(handle)
        with open(sources["validation_report.json"], encoding="utf-8") as handle:
            report = json.load(handle)
        valid = (isinstance(plan, dict) and isinstance(report, dict)
                 and plan.get("schema_version") == "dual_arm_simulation.v1"
                 and plan.get("real_motion_authorized") is False
                 and plan.get("debian_execution_allowed") is False
                 and plan.get("coordinate_frame") == "URDF_WORLD_SIM_ONLY"
                 and report.get("schema_version") == "dual_arm_simulation_validation.v1"
                 and report.get("plan_sha256") == sha256(sources["dual_arm_simulation.json"])
                 and isinstance(plan.get("frames"), list) and len(plan["frames"]) > 0
                 and report.get("dense_frames_checked") == len(plan["frames"]))
    except (OSError, ValueError, KeyError, TypeError):
        valid = False
    if not valid:
        raise SystemExit("✗ 双臂只读候选/报告绑定或禁止运动标记不符；保留现有发布目录")
    return plan, report


def render_dual_precheck_readme(spec, need, extra):
    """这不是双臂执行包；只提供现场只读数据采集和逐帧限位比较。"""
    _, report = validate_dual_precheck_sources(spec)
    return "\n".join([
        f"# {spec['title']}", "", f"状态：{spec['status']}", "",
        "## 直接在 Debian 连接只读检查", "",
        "整个 ZIP 单独解压，不覆盖原 A/C 成功包。使用已经安装厂商 pypilot 的 Debian Python；",
        "不需要 NumPy 或 PyBullet，不必先运行离线 dry-run。以下 IP 都要填本次现场值，",
        "不把任何历史地址当作当前机器人身份。机械臂应保持静止，关闭其他控制进程。", "",
        "```bash", "cd dual_arm_precheck", "sha256sum -c SHA256SUMS",
        "read -r -p '机器人 IP: ' robot_ip",
        "read -r -p 'Debian 本机 IP: ' local_ip",
        "read -r -p '机械臂控制器 IP: ' arm_ip",
        "python3 precheck_dual_arm.py --precheck-only \\",
        '  --robot-ip "$robot_ip" --local-ip "$local_ip" --arm-ip "$arm_ip"',
        "```", "",
        "控制器端口不是默认值时加 `--arm-port <实际端口>`。也可显式提供计划文件，",
        "默认即本包 `dual_arm_simulation.json`。`--output <报告路径>` 可选择输出位置。", "",
        "不连接机器人、仅验证文件结构时才使用：", "", "```bash",
        "python3 precheck_dual_arm.py --dry-run", "```", "",
        "## 它检查什么", "",
        f"- 当前候选 {report['dense_frames_checked']} 个双臂密集帧，读取双方实时 joints/worlds、控制器限位。",
        "- 逐帧比较真实限位、配置限位和预留余量，报告最靠近边界的关节与首点差距。",
        "- SDK worlds 只作现场记录；候选采用 URDF_WORLD_SIM_ONLY，不能冒充 SDK 世界坐标。",
        "- 不调用 armTryWorlds，不下发 MoveJ、MoveWorlds 或 Servo；不清报警、使能、设速度或操作夹爪。",
        "- 只读 PASS 只表示相应数据读取/限位比较通过，绝不授权运动。", "",
        "## 为什么暂时没有运动命令", "",
        "右臂真实控制器限位尚待现场回读；当前左 J2 距 URDF 下限仅约 1.885°，",
        "不能将相对已存控制器下限的 11.885° 余量误当作所有模型都留足 5°。",
        "这版 GUI 人工确认仍待绑定当前 SHA；手部自相交/抓取区接触的模型碰撞资格未完成。",
        "场景是合成桌面，不是完成配准的现场；右臂 TCP/映射以及双臂 SDK 并发与异常收尾也未验证。",
        "本包没有 `--run` 或跳过保护的开关，不能直接把密集仿真关节点当真机执行轨迹。", "",
        "## 测后发回", "",
        "请回传 `dual_arm_precheck_时间.json` 和自动生成的同名 `dual_arm_precheck_时间_sdk.log`。",
        "报告包含本次网络配置、当前关节/位姿、限位比较、失败原因和源码/计划/依赖 SHA-256。",
        "自动日志经 scrub_log 脱敏；不要另传含明文 token 的原始终端输出。",
        "文件检查失败或 SDK 初始化失败也不代表轨迹失败；以报告中各检查项和阻断原因分别判断。", "",
        "## 文件", "",
    ] + [f"- `{name}`" for name in sorted(need + extra)] + [
        "", "其中 sdk_session 是共用依赖；本入口只走其只读会话，不提供运动调度。", "",
        "## Mac 重新生成", "", "```bash", "python3 releases/make_release.py DP", "```", "",
        "只重建 dual_arm_precheck 目录和同名 ZIP，不重建、删除或覆盖现有 A/AT/AS/C 包。",
        "不要手改发布副本；改源码后由此命令再生。", "",
    ])


def render_tabletop_servo_readme(spec, need, extra):
    with open(os.path.join(CANDIDATES, "tabletop_pick_place_SERVO_CANDIDATE.json"), encoding="utf-8") as handle:
        plan = json.load(handle)
    metrics = plan["kinematics"]
    return "\n".join([
        f"# {spec['title']}", "", f"状态：{spec['status']}", "",
        "## 与原 A 的关系", "",
        "只更换执行方式：原规划关节折线进一步密化，保留所有旧关节锚点、抓放目标和固定 Home。",
        "全程运动用 armPluseToServo；无 MoveJ、MoveWorlds、自动寻找首点或第二次 TCP/关节符号转换。",
        "来源是 A JSON 内 SDK 预测/插值回放，不是真机逐帧录制；原控制器连续插值与新 Servo 不等价。",
        "原 A 的现场执行器作为初始化/夹爪编码/收尾依赖保留，不调用它的运动 main。", "",
        f"- {metrics['motion_frame_count']} 个关节点，最大步长 {metrics['max_step_deg']:.6f}°，20ms 一帧。",
        f"- 名义关节流 {metrics['stream_duration_s']:.2f}s；开爪1s、闭爪2s、放置开爪1s，停稳/网络耗时另计。",
        f"- 离散指令最大速度 {metrics['max_velocity_deg_s']:.3f}°/s，差分加速度 {metrics['max_acceleration_deg_s2']:.3f}°/s²；不是机器人反馈或厂商动力学额定值。",
        "- max_step/period 由 JSON 和共享配置固定；--speed 仅控制器全局速度，不能据此把 Servo 周期任意加速。",
        "- 抓放事件要先停稳并核对实际 EE；转运没有人为插入中途停顿，但折线细分不保证 jerk 连续。",
        "- 单调时钟，不追帧；发送迟到超过两个周期停止。SDK调用阻塞仍受SDK自身超时约束，不是硬实时系统。",
        "- 真机只读限位交集保持5°余量；失败停止，不自动开爪丢物；收尾回读保护和原速度。", "",
        "## 起点特别注意", "",
        "首点是成功包保存的历史采集构型，不是结束 Home，二者不能混同。没有从未知位置自动返回这一段。",
        "只有现场已停稳且每轴距首点≤0.4°才继续；只读预检不替你移动，人工审核也不绕过首点突跳限制。",
        f"首点 SDK joints deg：`{plan['first_point_q_sdk_deg']}`。",
        f"最终 Home SDK joints deg：`{plan['home_joints_sdk_deg']}`。",
        "若当前为 Home，先做只读预检并回传结果；不要单纯放宽起点阈值。需单独规划/审阅 Home 接入段，或者在现场确认净空后用示教器到首点。", "",
        "## Debian 目前可做的检查（不会运动）", "",
        "保留原 trackA_hybrid_trial 目录；将本 ZIP 单独解压。使用原厂商 SDK 的 Debian Python 环境，不安装 PyBullet/NumPy。",
        "```bash", "cd trackA_servo", "sha256sum -c SHA256SUMS",
        "python3 execute_tabletop_servo.py tabletop_pick_place_SERVO_CANDIDATE.json --dry-run", "```", "",
        "连接只读预检时，请使用现场实际IP（以下是上次地址示例，不是实时确认）：", "",
        "```bash",
        "python3 execute_tabletop_servo.py tabletop_pick_place_SERVO_CANDIDATE.json \\",
        "  --precheck-only --robot-ip 192.168.8.147 --local-ip 192.168.8.185 --arm-ip 192.168.8.147",
        "```", "",
        "默认生成不覆盖旧文件的 tabletop_servo_时间.json；成功/失败均含候选、执行器、全部依赖哈希和只读结果。",
        "请回传这个 JSON。只读 PASS 只表示文件、实时限位、静止首点检查通过，不代表碰撞或真实 Servo 时序通过。", "",
        "## 为什么此包还不能直接真机 --run", "",
        "新 Servo 必须有当前 JSON 完整 GUI 人工 PASS，以及独立的 tabletop_servo_qualification.v1 PASS 报告。",
        "当前自碰撞模型出现非相邻腕部 link9/11 重叠，尚未核实；桌框空间配准、真实夹爪/持瓶/释放后扫掠未完成。",
        "不能拿历史 A 现场 PASS、旧GUI或仅限位 PASS 替代这些证据；包内不会提供伪造PASS或跳过检查的开关。",
        "未来资格报告须绑定轨迹/消费者/依赖 SHA、场景、步长/周期/帧数，逐项提供FK、自碰撞、环境、持物/释放、接入区域、Servo动态时序的哈希证据。",
        "合格后执行器支持 --run --gui-review <审核文件> --qualification-report <资格文件>，还需 XIFENG_ALLOW_REAL_MOTION=1 和现场回车；此处不是当前放行命令。", "",
        "## Mac 再生与 GUI", "", "```bash",
        "/opt/anaconda3/bin/python -B pick_place_coord/gen_tabletop_hybrid_candidate.py --servo-parent pick_place_coord/trajectories/candidates/tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json",
        "/opt/anaconda3/bin/python -B pick_place_coord/gen_tabletop_hybrid_candidate.py --replay pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_CANDIDATE.json --gui --gui-report pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_gui_playback.json --gui-review pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_gui_review.json --reviewer operator",
        "python3 releases/make_release.py AS", "```", "",
        "以上从工作1根目录运行。AS 仅重建 trackA_servo 和 ZIP，不改 A/AT/C 既有发布包。",
        "夹爪参数仍在 gripper_policy 接口中；改参数或路径后应重新生成、校验并绑定新 SHA，不使用旧审核。", "",
        "## 文件", "",
    ] + [f"- `{name}`" for name in sorted(need + extra)] + [""])


def render_hybrid_field_readme(spec, need, extra):
    return "\n".join([
        f"# {spec['title']}", "", f"状态：{spec['status']}", "",
        "本包7个运行文件与用户提供的最终Debian文件逐字节一致；不再混入旧CANDIDATE/OPTIMIZED或旧GUI审核。",
        "当前真源与证据：`frame_calibration/records/20260908_trackA_hybrid_final/`，含两份脱敏JSON日志、as_run清单和审查说明。",
        "原交接ZIP保留；工作区旧候选/旧执行器仍在源码目录，不覆盖或删除历史证据。", "",
        "## 文件和现场行为", "",
        "- 计划SHA：`e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59`。",
        "- 执行器SHA：`36a0f659122eb492f63616f5c34a7e3b1f342c67f28e9906e2b9d6ad5b04e7ee`。",
        "- 5条MoveWorlds、单次转运MoveJ、固定关节MoveJ回Home；开始前一次Enter，中途不再回车。",
        "- 使用现场SDK对齐的PLACE_HOVER构型；XYZUVW保持原目标，已经是补偿后的SDK EE_POSE，不要二次TCP补偿。",
        "- open等待1s、close等待2s；全程预检在启动前，运行时Worlds检查前3点接续，MoveJ检查实际关节到目标的密集限位。",
        "- 保留硬限位/5°余量、保护、到位/停止检查和异常停止；当前失败运行不保存run JSON，排错要保留终端日志。", "",
        "## 先做无运动检查", "",
        "使用原来成功的Debian厂商SDK环境。本机无需PyBullet/NumPy。保留旧目录，按SHA更新这一现行目录。", "",
        "```bash", "cd trackA_hybrid_trial", "sha256sum -c SHA256SUMS",
        "python3 execute_tabletop_hybrid_trial_reviewfix_field.py tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json --dry-run",
        "```", "",
        "下列IP仅是9/8实跑值，若现场不同就替换；本次整理未连接机器人确认当前身份。", "",
        "```bash",
        "python3 execute_tabletop_hybrid_trial_reviewfix_field.py tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json \\",
        "  --precheck-only --robot-ip 192.168.8.147 --local-ip 192.168.8.185 --arm-ip 192.168.8.147 --arm-port 8080",
        "```", "",
        "## 现场复测", "",
        "只有确认同一机器人/工具/场景、空爪、整只手臂和实际起点接出路径净空、预检通过且有人值守急停后，才运行。",
        "这是收到的现场命令，不是授权从任意未知位置或任意新目标自动运动：", "",
        "```bash",
        "XIFENG_ALLOW_REAL_MOTION=1 python3 execute_tabletop_hybrid_trial_reviewfix_field.py \\",
        "  tabletop_pick_place_hybrid_OPTIMIZED_SDKALIGNED_REVIEW.json --run --start-policy current-reviewed \\",
        "  --robot-ip 192.168.8.147 --local-ip 192.168.8.185 --arm-ip 192.168.8.147 --arm-port 8080 \\",
        "  --speed 5.0 --limit-margin-deg 5", "```", "",
        "10%也有本组合的实跑证据，可在现场确认后改成--speed 10.0；虽然接口接受20%，本包没有20%运行证据。",
        "此现场文件不再调用旧qualification/GUI起点门，文件头部分描述和JSON旧诊断未同步；以实际main和本README为准。",
        "不因此认定完整场景碰撞已通过或新相机坐标可直接运行。以后改目标必须重新评估路径/构型与现场净空。", "",
        "## 文件清单", "",
    ] + [f"- `{name}`" for name in sorted(need + extra)] + ["",
        "## Mac重建", "", "```bash", "python3 releases/make_release.py AT", "```", "",
        "只重建trackA_hybrid_trial及ZIP，不重建、删除或覆盖A/AP。计划或执行器哈希改变会拒绝覆盖已收到的现场组合。", ""])


def render_hybrid_trial_readme(spec, need, extra):
    # GUI人工审核只适用于它绑定的轨迹，重生成新轨迹不能继承旧PASS。
    gui_status = "GUI人工审核尚未匹配当前OPTIMIZED；不能沿用旧审核。"
    plan_path = os.path.join(CANDIDATES, "tabletop_pick_place_hybrid_OPTIMIZED.json")
    review_path = os.path.join(CANDIDATES, "tabletop_pick_place_hybrid_OPTIMIZED_gui_review.json")
    if matching_gui_review(plan_path, review_path):
        gui_status = ("当前OPTIMIZED已由用户完整观看并确认GUI人工PASS；绑定当前轨迹SHA的"
                      "`tabletop_pick_place_hybrid_OPTIMIZED_gui_review.json`已随包附带。"
                      "这只证明画面/运动观察符合预期，不解除物理场景/碰撞资格限制。")
    pose_summary = "当前姿态及高度诊断以OPTIMIZED内pose_selection/qualification为准。"
    try:
        with open(plan_path, encoding="utf-8") as f:
            plan = json.load(f)
        uvw = plan["pose_selection"]["selected_endpoint_uvw_deg"]
        metrics = plan["qualification"]["kinematics"]
        drop = max(0., -metrics["transfer_grip_delta_xyz_min_mm"][2])
        delta = metrics["replayed_place_grip_delta_from_recorded_arrival_mm"][2]
        pose_summary = (f"当前抓取UVW（度）=[{uvw[0]:.3f}, {uvw[1]:.3f}, {uvw[2]:.3f}]。"
                        f"转运模型最大下垂{drop:.3f}mm；新放置夹持点相对原记录的PB模型Z差{delta:+.3f}mm，"
                        "这是模型解释差异，不是实测真机误差，也不是碰撞PASS。"
                        "保持原SDK夹持目标（包括高度），改姿态后反算EE，因此EE的XYZ变化不等于把物品目标改高/改低。")
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        pass
    return "\n".join([
        f"# {spec['title']}", "", f"状态：{spec['status']}", "",
        gui_status, "",
        "## 先分清两份 JSON", "",
        "- `tabletop_pick_place_hybrid_CANDIDATE.json`：20260907 五份现场完整动作 PASS 对应的原始轨迹，"
        "SHA 为 `3f3e36f370521e4ab815e377ddb491a3cd2adb24d95dcf843451d03d7587c06e`。"
        "仍有两段转运 MoveJ，中间停顿尚在；这份用于单独测试执行器升级。",
        "- `tabletop_pick_place_hybrid_OPTIMIZED.json`：新生成的单段 MoveJ 转运候选。"
        "原轨迹的五次 PASS 不适用于它。缺少新 GUI 审核或运动资格报告时，执行器拒绝 `--run`。",
        pose_summary,
        "- 新执行器不是旧 `de5ccc12…` 的相同文件，需重新测试；原成功 ZIP、脚本和日志不得覆盖。", "",
        "## 放到哪里", "",
        "使用原来能跑成功的 Debian 厂商 SDK 容器/Python 环境。"
        "整个 ZIP 在原成功目录旁解压，进入 `trackA_hybrid_trial`；不要混装到旧目录。",
        "本包不需要 PyBullet/NumPy/SciPy，也不包含厂商 Linux SDK wheel。", "",
        "## 1. 离线验包（不连接机器人）", "", "```bash",
        "cd trackA_hybrid_trial",
        "sha256sum -c SHA256SUMS",
        "python3 execute_tabletop_hybrid_trial.py tabletop_pick_place_hybrid_CANDIDATE.json --dry-run",
        "python3 execute_tabletop_hybrid_trial.py tabletop_pick_place_hybrid_OPTIMIZED.json --dry-run",
        "```", "",
        "## 2. 连接真机做只读预检", "",
        "先停止其他机器人程序和示教移动。下面地址仅来自20260907最后实跑，现场不同就替换；"
        "不把 `.147` 或 `.148` 写死为机器人身份。", "", "```bash",
        "export XIFENG_ROBOT_IP=192.168.8.147",
        "export XIFENG_ARM_IP=192.168.8.147",
        "export XIFENG_LOCAL_IP=192.168.8.185",
        "python3 execute_tabletop_hybrid_trial.py tabletop_pick_place_hybrid_CANDIDATE.json --precheck-only",
        "python3 execute_tabletop_hybrid_trial.py tabletop_pick_place_hybrid_OPTIMIZED.json --precheck-only",
        "```", "",
        "预检默认不要求当前位置贴近 nominal Safe，不自动回Home；"
        "但必须静止、实际关节在有效限位内，实际起点到抓取预备点的Worlds解连续。",
        "退出码0只表示当前只读检查通过，不是碰撞/运动PASS。失败时发回生成的报告，不要删限位过滤。", "",
        "## 3. 先用原轨迹测试升级执行器（人工值守）", "",
        "只在上述原轨迹预检通过、现场空爪、整只手臂及首段路径人工检查无干涉后使用。"
        "不要求先绕回Home，但不允许仅凭末端Z在桌上方就认定手腕/肘部/夹爪不会碰撞。",
        "原v1候选仍保留历史碰撞/模型资格限制，`--accept-blockers` 是显式接受有人值守试验的证据边界，"
        "不是取消硬限位，也不把人工检查记成自动碰撞PASS。", "", "```bash",
        "XIFENG_ALLOW_REAL_MOTION=1 python3 execute_tabletop_hybrid_trial.py \\",
        "  tabletop_pick_place_hybrid_CANDIDATE.json --run --accept-blockers --speed 3.0",
        "```", "",
        "预检后会显示并要求人工确认本次实际起点、空爪与路径；未确认不得使能。"
        "抬升后和放置Hover处默认仍有人工确认。首次测试不加 `--auto-continue`。",
        "结束仍固定关节MoveJ回现场Home。`--start-policy nominal-safe` 可恢复旧的Safe附近启动要求。", "",
        "## 4. 单段转运候选的运动边界", "",
        "OPTIMIZED只读预检可以现在执行。其真机运动还需绑定该文件SHA的 "
        "`pybullet_gui_review.v1` 人工PASS（`--gui-review`），以及有证据支撑的 "
        "`tabletop_hybrid_qualification.v1` 资格报告（`--qualification-report`）。",
        "不能用旧GUI记录、只写几个PASS字段、删除中间点后沿用旧回放、或 `--accept-blockers` 绕过v2资格。",
        "新抓姿态接口保持SDK夹持点不变并随姿态重新换算EE；"
        "Mac生成器不带 --horizontal-long-axis 或 --pose-template 时，仍保留实跑抓姿态；"
        "可用 --pose-template U V W 提供现场认可的姿态模板，"
        "--placement-follow-grasp-tilt 使放置采用同一U/V并保留原朝框W。"
        "这不保证MoveJ严格等高，也不保证整段保持完全相同UVW。没有实现多物品批次功能。", "",
        "## 发回什么", "",
        "发回本次生成的只读预检JSON/执行JSON。它们记录计划、执行器及依赖哈希、参数、起点、"
        "阶段时刻和收尾结果。SDK原始终端内容可能含token，另附终端日志前先用scrub_log.py脱敏副本。",
        "不要把只有动作结束但收尾失败的报告当成完整PASS。", "",
        "## 文件清单", "",
    ] + [f"- `{name}`" for name in sorted(need + extra)] + ["",
        "## Mac 重建", "", "```bash", "python3 releases/make_release.py AT", "```", "",
        "AT只重建trackA_hybrid_trial及其ZIP，不重建、删除或覆盖A/AP及原始成功交接ZIP。", ""])


def render_hybrid_precheck_readme(spec, need, extra):
    return "\n".join([
        f"# {spec['title']}", "", f"状态：{spec['status']}", "",
        "这是连接真实控制器、但不使能/不清故障/不改变速度保护/不运动/不操作夹爪的测试包。",
        "不是一键抓放包，也没有 `--run` 或限位绕过参数。急停和现场观察不能把未验证路径变成通过。", "",
        "## 解压位置", "",
        "在原成功目录外解压整个ZIP；不要把这些文件合并到原 trackA_worlds_line 目录。",
        "原成功 JSON、执行器和日志全部保留。新目录名是 trackA_hybrid_precheck。",
        "后续仅更新这个测试目录里哈希变化的文件。", "",
        "## 依赖", "",
        "使用原来能运行成功抓放的 Debian SDK 容器/Python 环境（已有 pypilot 和厂商动态库）。",
        "不需要 PyBullet、NumPy、SciPy，也不需要把 Mac 规划器或 URDF 发过去。", "",
        "## 1. 不连接机器人，先验包", "", "```bash",
        "cd trackA_hybrid_precheck", "sha256sum -c SHA256SUMS",
        "python3 precheck_tabletop_hybrid.py tabletop_pick_place_hybrid_CANDIDATE.json --dry-run", "```", "",
        "## 2. 连接控制器，只读测试", "",
        "先停止其他机器人程序和示教移动，保持机器人静止；本程序不会自动回Home或使能。",
        "下列地址来自你上次报告，仅在现场仍一致时使用；若变更请替换。", "", "```bash",
        "python3 precheck_tabletop_hybrid.py tabletop_pick_place_hybrid_CANDIDATE.json \\",
        "  --precheck-only --robot-ip 192.168.8.147 --local-ip 192.168.8.185 \\",
        "  --arm-ip 192.168.8.147 --arm-port 8080", "```", "",
        "也可沿用当前 XIFENG_ROBOT_IP / XIFENG_LOCAL_IP / XIFENG_ARM_IP 环境变量。",
        "无需设置 XIFENG_ALLOW_REAL_MOTION=1；即使终端残留1，本工具也会置0。",
        "报告自动保存为 hybrid_precheck_日期_时间.json，不覆盖旧文件；发生错误/中断会尽量保留已获取诊断。", "",
        "## 会测什么", "",
        "- 实际 Worlds/Joints、静止状态、控制器限位与共享物理限位的交集（保留5°余量）。",
        "- 6个Worlds段的2mm/1°密集armTryWorlds、限位、相邻逆解跳变。",
        "- 两段MoveJ的候选密集关节限位；另查询预期EE位姿，但该查询不验证指定q的FK。",
        "- 每段SDK逆解和离线关节构型差异，包括PICK_ASCEND及转运后的下降入口。",
        "- 查询期间机器人若移动，报告失效并停止查询；始终不下发运动或松爪。", "",
        "## 结果怎么理解", "",
        "FILE_CHECKS_PASS 只是本地文件检查；READONLY_DIAGNOSTICS_COMPLETE 只是数据采集结束。",
        "WITH_ISSUES / 退出码2表示发现不可达、限位、构型差异等问题；保留报告发回来即可，不要清空检查。",
        "退出码0也不是混合路径/碰撞/真机运动PASS；MoveJ未执行，SDK没有seed接口，后段逆解仍来自当前静止构型。",
        "当前位置不等于Safe时只报告差值，不擅自补回位。不要把预测PICK_ASCEND关节当成实测到位。", "",
        "## 仍未解除的运动阻断", "",
        "候选完整GUI播放已完成，但桌框物理配准/真实夹具持瓶碰撞尚未验证；URDF腕部link9/11有网格重叠。",
        "新的后段Worlds使用离线软构型引导，实际控制器能否复现要核对。",
        "这次只补齐只读测试，不附带可执行混合运动入口；不能把状态改成PASS、改限位或套旧执行器强行运行。", "",
        "## 发回什么", "",
        "把新生成的 hybrid_precheck_*.json 发回即可，含候选/脚本/依赖 SHA-256。",
        "如果连接失败没生成报告，提供报错；厂商终端输出可能含登录token，外发前用 scrub_log.py 对日志副本脱敏。", "",
        "## 文件清单", ""] + [f"- `{name}`" for name in sorted(need + extra)] + ["",
        "其中 execute_tabletop_pick_place_worlds.py 仅作为纯计算辅助依赖，不是这份新JSON的执行入口。",
        "不得用它覆盖Debian原成功的a97dd208…执行器。", "",
        "## Mac 重建", "", "```bash", "python3 releases/make_release.py AP", "```", "",
        "只有AP会重建此只读目录和ZIP，不会重建/删除原A成功目录。", ""])


def verify():
    bad = 0
    for key, spec in TRACKS.items():
        out = os.path.join(_HERE, spec["dir"])
        sums = os.path.join(out, "SHA256SUMS")
        if not os.path.exists(sums):
            print(f"  {spec['dir']}: 未封装, 跳过")
            continue
        drift = []
        for line in open(sums, encoding="utf-8"):
            want, name = line.strip().split("  ", 1)
            path = os.path.join(out, name)
            if not os.path.exists(path):
                drift.append(f"{name} 缺失")
            elif sha256(path) != want:
                drift.append(f"{name} 内容被改过")
        if drift:
            bad += 1
            print(f"  ✗ {spec['dir']}: " + "; ".join(drift))
        else:
            print(f"  ✓ {spec['dir']}: 与 SHA256SUMS 一致")
    return 1 if bad else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description="按线路封装真机可运行的自包含目录")
    ap.add_argument("tracks", nargs="*", choices=list(TRACKS))
    ap.add_argument("--verify", action="store_true", help="只校验, 不重新生成")
    a = ap.parse_args(argv)
    if a.verify:
        return verify()
    if not a.tracks:
        ap.error("给出要封装的线路(A/B/C), 或用 --verify")
    for k in a.tracks:
        build(k)
    return 0


if __name__ == "__main__":
    sys.exit(main())
