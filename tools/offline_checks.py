#!/usr/bin/env python3
"""Run the explicitly selected offline tests; never discover hardware demo scripts."""
import os
import argparse
import hashlib
import json
import importlib.metadata
from pathlib import Path
import platform
import subprocess
import sys
import xml.etree.ElementTree as ET

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
 'pick_place_coord/test_export_paths.py',
 'releases/test_make_release_hybrid.py',
 'releases/test_make_release_dual.py',
 'releases/test_release_pruning.py',
 'frame_calibration/robot_side/test_precheck_dual_arm.py',
 'pick_place_coord/dual_arm_demo/test_demo.py',
 'vision/test_vision_system.py',
 'frame_calibration/analysis/test_fit_eye_to_hand.py',
]


def source_snapshot():
    selection=json.loads((ROOT/'docs/DELIVERY_SELECTION.json').read_text())
    return {x['path']:hashlib.sha256((ROOT/x['path']).read_bytes()).hexdigest()
            for x in selection['active'] if x['path'].startswith('src/') and Path(x['path']).suffix!='.md'}

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record',action='store_true',help='Record successful selected checks and source identities.')
    args=parser.parse_args()
    env=dict(os.environ, XIFENG_ALLOW_REAL_MOTION='0', PYTHONDONTWRITEBYTECODE='1')
    env.pop('PYTHONPATH',None)
    tmp=ROOT/'.runtime/tmp';tmp.mkdir(parents=True,exist_ok=True)
    env['TMPDIR']=str(tmp)
    junit=tmp/'offline-tests.xml'
    command=[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',f'--junitxml={junit}',*TESTS]
    if not args.record:
        raise SystemExit(subprocess.call(command,cwd=ROOT/'src',env=env))
    # Missing a Servo fixture can silently skip meaningful regression tests.
    fixture=ROOT/'src/pick_place_coord/trajectories/traj_minimal_joint.json'
    if not fixture.is_file():
        raise SystemExit('Missing required Servo safety fixture')
    before=source_snapshot()
    report_path=ROOT/'verification.json'
    report=json.loads(report_path.read_text()) if report_path.exists() else {}
    report['latest_attempt_status']='RUNNING'
    report['status']='RUNNING'
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    result=subprocess.run(command,cwd=ROOT/'src',env=env,text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    print(result.stdout,end='')
    report['latest_attempt_status']='FAILED'
    report['status']='FAILED'
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    if result.returncode:
        raise SystemExit(result.returncode)
    tree=ET.parse(junit)
    suites=list(tree.getroot().iter('testsuite'))
    counts={key:sum(int(s.attrib.get(key,0)) for s in suites) for key in ['tests','failures','errors','skipped']}
    counts['passed']=counts['tests']-counts['failures']-counts['errors']-counts['skipped']
    if counts['failures'] or counts['errors'] or counts['skipped'] or counts['passed']<279:
        raise SystemExit(f'Selected suite incomplete: {counts}')
    hashes=source_snapshot()
    if hashes!=before:
        raise SystemExit('Selected sources changed during checks; no successful result recorded')
    report.update(schema='robot_handoff_verification.v2',
                  status='PASS',
                  latest_attempt_status='PASS',
                  scope='Explicit offline regression suite only; no hardware connection or acceptance.',
                  selected_offline_tests=dict(counts,modules=TESTS),checked_source_sha256=hashes,
                  test_environment={'python':platform.python_version(),'platform':platform.system(),
                                    'architecture':platform.machine(),
                                    'packages':{name:importlib.metadata.version(name) for name in
                                                ['numpy','scipy','pybullet','pybullet-planning','osqp','matplotlib','pytest','jsonschema']}},
                  excluded_checks=['real robot motion','SDK installation','GUI human acceptance','full ROS/C++ build'])
    report_path.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    log=ROOT/'evidence/handoff_checks/offline_tests.log'
    log.parent.mkdir(parents=True,exist_ok=True)
    # Logs must be portable: absolute temporary/device paths carry no acceptance meaning.
    log.write_text(result.stdout.replace(str(ROOT),'$HANDOFF_ROOT')
                   .replace(sys.prefix,'$SIM_ENV')
                   .replace(os.path.relpath(sys.prefix,ROOT/'src'),'$SIM_ENV'))
