#!/usr/bin/env python3
"""Run the explicitly selected offline tests; never discover hardware demo scripts."""
import os
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
TESTS=[
 'pick_place_coord/mpc_experiment/test_mpc.py',
 'frame_calibration/robot_side/test_execute_trajectory_options.py',
 'frame_calibration/robot_side/test_servo_safety.py',
 'frame_calibration/robot_side/test_execute_tabletop_pick_place_worlds.py',
 'frame_calibration/robot_side/test_precheck_tabletop_hybrid.py',
 'frame_calibration/robot_side/test_execute_tabletop_hybrid_trial.py',
 'frame_calibration/robot_side/test_tracka_final_handoff.py',
 'frame_calibration/robot_side/test_tabletop_servo.py',
 'pick_place_coord/test_gen_tabletop_hybrid_candidate.py',
 'releases/test_make_release_hybrid.py',
 'releases/test_make_release_dual.py',
 'frame_calibration/robot_side/test_precheck_dual_arm.py',
 'pick_place_coord/dual_arm_demo/test_demo.py',
 'vision/test_vision_system.py',
 'frame_calibration/analysis/test_fit_eye_to_hand.py',
]

if __name__=='__main__':
    env=dict(os.environ, XIFENG_ALLOW_REAL_MOTION='0', PYTHONDONTWRITEBYTECODE='1')
    env.pop('PYTHONPATH',None)
    tmp=ROOT/'.runtime/tmp';tmp.mkdir(parents=True,exist_ok=True)
    env['TMPDIR']=str(tmp)
    raise SystemExit(subprocess.call([sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',*TESTS],
                                     cwd=ROOT/'src',env=env))
