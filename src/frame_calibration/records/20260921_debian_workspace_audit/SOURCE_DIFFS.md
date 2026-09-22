# 与 Mac 的逐文件差异标记

本表列出95条字节不同及38条同名歧义。路径映射是比对线索，不代表两者应互相覆盖；关键逻辑解释见[总表](README.md)。119条相同和1030条无对应条目在[source_comparison.json](source_comparison.json)中。Mac身份基于收件审阅时的实际文件（含未提交内容），不是仅以Git HEAD推断。

## 字节不同（95条）

Debian路径相对收到的`workspace/`，Mac路径相对`工作1/`。完整哈希在JSON中。

| Debian 文件 | Mac 对应候选 | Debian / Mac SHA前12位 |
|---|---|---|
| `traj_multi_latest.json` | `frame_calibration/robot_side/traj_multi_latest.json` | `08b0d5fbfb75` / `ac5c6fd7715c` |
| `AGENTS.md` | `AGENTS.md` | `2c33b1bd05e7` / `7b03b66e5325` |
| `Servo/worlds/world_grasp_latest.json` | `pick_place_coord/trajectories/world_grasp_latest.json` | `89ddca86b89c` / `ccc1ca9d8adb` |
| `Servo/worlds/robot_lock.py` | `frame_calibration/robot_side/robot_lock.py` | `064528b196f3` / `3dc07df2343a` |
| `Servo/worlds/test_worlds_servo_minimal.py` | `frame_calibration/robot_side/test_worlds_servo_minimal.py` | `2e9f3aad98d9` / `98e9bcddc72f` |
| `Servo/worlds/test_servo_safety.py` | `frame_calibration/robot_side/test_servo_safety.py` | `869986092586` / `67833b628e42` |
| `Servo/worlds/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `0fcefe95d842` / `15fe08fe7308` |
| `Servo/worlds/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `744313cca7d1` / `c3e464f49242` |
| `Servo/worlds/execute_worlds_servo_grasp.py` | `frame_calibration/robot_side/execute_worlds_servo_grasp.py` | `76670d757747` / `7af40e37a66f` |
| `Servo/pulse/robot_lock.py` | `frame_calibration/robot_side/robot_lock.py` | `064528b196f3` / `3dc07df2343a` |
| `Servo/pulse/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `744313cca7d1` / `c3e464f49242` |
| `Servo/pulse/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `0fcefe95d842` / `15fe08fe7308` |
| `Servo/pulse/test_servo_safety.py` | `frame_calibration/robot_side/test_servo_safety.py` | `869986092586` / `67833b628e42` |
| `Servo/pulse/execute_servo_grasp.py` | `frame_calibration/robot_side/execute_servo_grasp.py` | `9c08f459e57c` / `73be5ab6561c` |
| `Servo/pulse/test_gripper_right.py` | `frame_calibration/robot_side/test_gripper_right.py` | `45464e962199` / `fe06c0ea25db` |
| `Servo/pulse1/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `f22bc4c36484` / `c3e464f49242` |
| `Servo/pulse1/robot_lock.py` | `frame_calibration/robot_side/robot_lock.py` | `064528b196f3` / `3dc07df2343a` |
| `Servo/pulse1/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `d84d6e76e6b3` / `15fe08fe7308` |
| `Servo/pulse1/test_gripper_right.py` | `frame_calibration/robot_side/test_gripper_right.py` | `b4e47acc984d` / `fe06c0ea25db` |
| `Servo/pulse1/execute_servo_grasp.py` | `frame_calibration/robot_side/execute_servo_grasp.py` | `6d84406a76ef` / `73be5ab6561c` |
| `trackA_worlds_line/trackA_worlds_line/execute_tabletop_pick_place_worlds.py` | `frame_calibration/robot_side/execute_tabletop_pick_place_worlds.py` | `e011880560b0` / `ad6c4c90fd5b` |
| `trackA_worlds_line/trackA_worlds_line/tabletop_pick_place_worlds.json` | `pick_place_coord/trajectories/candidates/tabletop_pick_place_worlds.json` | `0f2342768e8a` / `453b76c05bb4` |
| `trackA_worlds_line/trackA_worlds_line_1/execute_tabletop_pick_place_worlds.py` | `frame_calibration/robot_side/execute_tabletop_pick_place_worlds.py` | `a97dd2080170` / `ad6c4c90fd5b` |
| `trackA_worlds_line/trackA_hybrid_precheck/execute_tabletop_hybrid_trial.py` | `frame_calibration/robot_side/execute_tabletop_hybrid_trial.py` | `de5ccc1252a0` / `29b790de58ab` |
| `trackA_worlds_line/trackA_hybrid_precheck/trackA_hybrid_final_handoff_20260907/execute_tabletop_hybrid_trial.py` | `frame_calibration/robot_side/execute_tabletop_hybrid_trial.py` | `de5ccc1252a0` / `29b790de58ab` |
| `trackA_worlds_line/trackA_hybrid_precheck/trackA_hybrid_final_handoff_20260907/precheck_tabletop_hybrid.py` | `frame_calibration/robot_side/precheck_tabletop_hybrid.py` | `e6b904225dd3` / `266ee78fcae3` |
| `trackA_worlds_line/trackA_hybrid_precheck/precheck_tabletop_hybrid.py` | `frame_calibration/robot_side/precheck_tabletop_hybrid.py` | `e6b904225dd3` / `266ee78fcae3` |
| `trackA_worlds_line/trackA_servo/trackA_servo_baseline_30ms_stride2_20260910_142220/execute_tabletop_servo.py` | `frame_calibration/robot_side/execute_tabletop_servo.py` | `a9948adaa486` / `c5aad779b9cd` |
| `trackA_worlds_line/trackA_servo/trackA_servo_baseline_30ms_stride2_20260910_142220/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `444abaa59c2e` / `15fe08fe7308` |
| `trackA_worlds_line/trackA_servo/execute_tabletop_servo.py` | `frame_calibration/robot_side/execute_tabletop_servo.py` | `a9948adaa486` / `c5aad779b9cd` |
| `trackA_worlds_line/trackA_servo/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `444abaa59c2e` / `15fe08fe7308` |
| `trackA_worlds_line/trackA_servo/tabletop_pick_place_SERVO_CANDIDATE.json` | `pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_CANDIDATE.json` | `71bc5148f5ab` / `0a1854559421` |
| `trackA_worlds_line/trackA_servo/tabletop_pick_place_SERVO_gui_review.json` | `pick_place_coord/trajectories/candidates/tabletop_pick_place_SERVO_gui_review.json` | `02f94490a01c` / `14176342f3ed` |
| `mpc_experiment/mpc_experiment/frame_calibration/analysis/calib_common.py` | `frame_calibration/analysis/calib_common.py` | `1c9787d57a98` / `4d1d73c0984d` |
| `mpc_experiment/mpc_experiment/controller.py` | `pick_place_coord/mpc_experiment/controller.py` | `314f5cbe6c43` / `4341f2fd7321` |
| `mpc_experiment/mpc_experiment/results/report.json` | `pick_place_coord/mpc_experiment/results/report.json` | `b427f0125a25` / `ad81c3e38544` |
| `my_package/package.xml` | `vision/package.xml` | `ded06c03a8ce` / `0e923d95bbe9` |
| `learning_tf2_py/package.xml` | `vision/package.xml` | `2fe8fc0f3ba7` / `0e923d95bbe9` |
| `learning_tf2_py/setup.py` | `vision/setup.py` | `c7e25bc8d12c` / `19641a7828bd` |
| `trackC_servo_stream/trackC_servo_stream/execute_servo_grasp.py` | `frame_calibration/robot_side/execute_servo_grasp.py` | `808f12e1baa8` / `73be5ab6561c` |
| `trackC_servo_stream/trackC_servo_stream/robot_lock.py` | `frame_calibration/robot_side/robot_lock.py` | `064528b196f3` / `3dc07df2343a` |
| `trackC_servo_stream/trackC_servo_stream/test_servo_safety.py` | `frame_calibration/robot_side/test_servo_safety.py` | `9d0c13790b0b` / `67833b628e42` |
| `trackC_servo_stream/trackC_servo_stream/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `2fe4426a2e7f` / `c3e464f49242` |
| `trackC_servo_stream/trackC_servo_stream/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `d84d6e76e6b3` / `15fe08fe7308` |
| `trackC_servo_stream/trackC_servo_stream/test_gripper_right.py` | `frame_calibration/robot_side/test_gripper_right.py` | `b4e47acc984d` / `fe06c0ea25db` |
| `vision/vision_hand_eye/package.xml` | `vision/package.xml` | `7ef79ba2f37b` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_hand_eye/package.xml` | `vision/package.xml` | `7ef79ba2f37b` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/deps/vision/camera_drive/package.xml` | `vision/package.xml` | `1f3a17e1072d` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_algorithm/package.xml` | `vision/package.xml` | `a1704721e086` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_third_party/package.xml` | `vision/package.xml` | `91d01fc26905` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_utils/package.xml` | `vision/package.xml` | `f6af1614bf88` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/camera_ros/package.xml` | `vision/package.xml` | `57b43796c51e` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/camera_test/package.xml` | `vision/package.xml` | `5ab7518caa57` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/vision_interface/package.xml` | `vision/package.xml` | `5a4573b2c908` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_hand_eye/share/vision_hand_eye/package.xml` | `vision/package.xml` | `7ef79ba2f37b` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/camera_drive/share/camera_drive/package.xml` | `vision/package.xml` | `1f3a17e1072d` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_algorithm/share/vision_algorithm/package.xml` | `vision/package.xml` | `a1704721e086` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/camera_ros/share/camera_ros/package.xml` | `vision/package.xml` | `57b43796c51e` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/camera_test/share/camera_test/package.xml` | `vision/package.xml` | `5ab7518caa57` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_third_party/share/vision_third_party/package.xml` | `vision/package.xml` | `91d01fc26905` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_utils/share/vision_utils/package.xml` | `vision/package.xml` | `f6af1614bf88` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_interface/share/vision_interface/package.xml` | `vision/package.xml` | `5a4573b2c908` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_detect_ros/share/vision_detect_ros/package.xml` | `vision/package.xml` | `2fb51b5cef89` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/install/vision_bringup/share/vision_bringup/package.xml` | `vision/package.xml` | `3e40bf598c18` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/vision_detect_ros/package.xml` | `vision/package.xml` | `2fb51b5cef89` / `0e923d95bbe9` |
| `vision/ultirobotics_vision_detect/build/vision_interface/ament_cmake_python/vision_interface/setup.py` | `vision/setup.py` | `c6e326732873` / `19641a7828bd` |
| `vision/ultirobotics_vision_detect/vision_bringup/package.xml` | `vision/package.xml` | `3e40bf598c18` / `0e923d95bbe9` |
| `vision_hand_eye/package.xml` | `vision/package.xml` | `7ef79ba2f37b` / `0e923d95bbe9` |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/traj_bottle_taught_tcp_20260831_RUN.json` | `pick_place_coord/trajectories/candidates/traj_bottle_taught_tcp_20260831_RUN.json` | `264808c7da7f` / `cce2a83fae21` |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/execute_servo_grasp.py` | `frame_calibration/robot_side/execute_servo_grasp.py` | `bc05bd76c258` / `73be5ab6561c` |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/test_servo_safety.py` | `frame_calibration/robot_side/test_servo_safety.py` | `f3564be84704` / `67833b628e42` |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/servo_common.py` | `frame_calibration/robot_side/servo_common.py` | `d84d6e76e6b3` / `15fe08fe7308` |
| `sdk_tests/frame_calibration/robot_side/world_grasp_latest.json` | `pick_place_coord/trajectories/world_grasp_latest.json` | `89ddca86b89c` / `ccc1ca9d8adb` |
| `sdk_tests/frame_calibration/robot_side/collect_wrist_orientation_samples.py` | `frame_calibration/robot_side/collect_wrist_orientation_samples.py` | `f1795b88e1b7` / `92bd054824a2` |
| `sdk_tests/frame_calibration/robot_side/test_move_worlds.py` | `frame_calibration/robot_side/test_move_worlds.py` | `de8dbc53f891` / `5baa58dd5690` |
| `sdk_tests/frame_calibration/robot_side/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `744313cca7d1` / `c3e464f49242` |
| `sdk_tests/frame_calibration/robot_side/execute_worlds_10x.py` | `frame_calibration/robot_side/execute_worlds_10x.py` | `491962a134b9` / `46f86bb9ee79` |
| `sdk_tests/frame_calibration/robot_side/robot_lock.py` | `frame_calibration/robot_side/robot_lock.py` | `064528b196f3` / `3dc07df2343a` |
| `sdk_tests/frame_calibration/robot_side/execute_servo_grasp.py` | `frame_calibration/robot_side/execute_servo_grasp.py` | `298aa5ba0739` / `73be5ab6561c` |
| `sdk_tests/frame_calibration/robot_side/test_servo_stream.py` | `frame_calibration/robot_side/test_servo_stream.py` | `15daf6787c42` / `d761876c908c` |
| `sdk_tests/frame_calibration/robot_side/capture_tool_down_pose.py` | `frame_calibration/robot_side/capture_tool_down_pose.py` | `6752be6d2af9` / `1cd917641a9a` |
| `sdk_tests/frame_calibration/robot_side/execute_world_grasp.py` | `frame_calibration/robot_side/execute_world_grasp.py` | `dad67eed6fe3` / `a69a0722282a` |
| `sdk_tests/frame_calibration/robot_side/test_gripper_right.py` | `frame_calibration/robot_side/test_gripper_right.py` | `9cb7d6b89ad3` / `fe06c0ea25db` |
| `sdk_tests/frame_calibration/robot_side/execute_trajectory.py` | `frame_calibration/robot_side/execute_trajectory.py` | `66e4e5f1dc2f` / `2261df8312a0` |
| `sdk_tests/frame_calibration/records/20260715_multi_pick_place_153pts/trajectory/traj_multi_latest.json` | `frame_calibration/robot_side/traj_multi_latest.json` | `656a835dc148` / `ac5c6fd7715c` |
| `sdk_tests/frame_calibration/records/20260716_multi_pick_place_153pts/trajectory/traj_multi_latest.json` | `frame_calibration/robot_side/traj_multi_latest.json` | `656a835dc148` / `ac5c6fd7715c` |
| `sdk_tests/frame_calibration/records/20260716_multi_pick_place_153pts/post_run_safe_baseline/execute_trajectory.py` | `frame_calibration/robot_side/execute_trajectory.py` | `f6e9d93c8549` / `2261df8312a0` |
| `sdk_tests/frame_calibration/records/20260716_multi_pick_place_153pts/post_run_safe_baseline/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `f8c50c45af89` / `c3e464f49242` |
| `sdk_tests/frame_calibration/records/20260720_pre_gripper_modification_baseline/code/execute_trajectory.py` | `frame_calibration/robot_side/execute_trajectory.py` | `f6e9d93c8549` / `2261df8312a0` |
| `sdk_tests/frame_calibration/records/20260720_pre_gripper_modification_baseline/code/sdk_session.py` | `frame_calibration/robot_side/sdk_session.py` | `f8c50c45af89` / `c3e464f49242` |
| `sdk_tests/frame_calibration/records/20260720_pre_gripper_modification_baseline/trajectory/traj_multi_latest.json` | `frame_calibration/robot_side/traj_multi_latest.json` | `656a835dc148` / `ac5c6fd7715c` |
| `sdk_tests/frame_calibration/archive/deploy_backups/20260720_171928/execute_trajectory.py` | `frame_calibration/robot_side/execute_trajectory.py` | `f6e9d93c8549` / `2261df8312a0` |
| `sdk_tests/frame_calibration/archive/deploy_backups/20260720_171928/traj_multi_latest.json` | `frame_calibration/robot_side/traj_multi_latest.json` | `656a835dc148` / `ac5c6fd7715c` |
| `sdk_tests/motion/test_move_worlds.py` | `frame_calibration/robot_side/test_move_worlds.py` | `de8dbc53f891` / `5baa58dd5690` |
| `sdk_tests/motion/test_move_joints.py` | `frame_calibration/robot_side/test_move_joints.py` | `b101f640d212` / `74162393acf4` |

## 同名但对应关系不唯一（38条）

不能仅按文件名选版本；须结合调用方、导入路径和运行哈希。

| Debian 文件 | Mac 同名候选 |
|---|---|
| `trackA_worlds_line/trackA_worlds_line/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `trackA_worlds_line/trackA_worlds_line_1/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `trackA_worlds_line/trackA_hybrid_precheck/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `trackA_worlds_line/trackA_servo/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `trackA_worlds_line/trackA_servo/network_records/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `mpc_experiment/mpc_experiment/frame_calibration/analysis/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `mpc_experiment/mpc_experiment/frame_calibration/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `learning_tf2_py/learning_tf2_py/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `trackC_servo_stream/trackC_servo_stream/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/vision_hand_eye/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/ultirobotics_vision_detect/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_hand_eye/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/ultirobotics_vision_detect/deps/vision/camera_drive/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_algorithm/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/ultirobotics_vision_detect/deps/vision/vision_utils/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `vision/ultirobotics_vision_detect/build/vision_interface/rosidl_generator_py/vision_interface/srv/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/rosidl_generator_py/vision_interface/msg/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/rosidl_generator_py/vision_interface/action/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/rosidl_generator_py/vision_interface/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/ament_cmake_python/vision_interface/vision_interface/srv/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/ament_cmake_python/vision_interface/vision_interface/msg/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/ament_cmake_python/vision_interface/vision_interface/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision/ultirobotics_vision_detect/build/vision_interface/ament_cmake_python/vision_interface/vision_interface/action/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `vision_hand_eye/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `code/vision_log_tool-main/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `trackC_bottle_left_orientation_precheck/trackC_bottle_left_orientation_precheck/debug_archive/arm_enable_varget_8080_20260901/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `pypilot-1.0.0422.1-cp310-cp310-linux_x86_64/pypilot/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `xf_humanoid_simple_sdk/README.md` | `pick_place_coord/dual_arm_demo/README.md`<br>`pick_place_coord/mpc_experiment/README.md`<br>`requirements/README.md`<br>`vision/README.md`<br>`frame_calibration/README.md`<br>`pick_place_coord/experimental/README.md`<br>`pick_place_coord/trajectories/candidates/README.md`<br>`pick_place_coord/trajectories/verified/README.md` |
| `xf_humanoid_simple_sdk/xf_humanoid_sdk/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/motion/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/battery/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/diagnostics/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/experimental/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/sdk/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/tests/unit/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/tests/integration/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
| `sdk_tests/tests/__init__.py` | `vision/vision_system/__init__.py`<br>`robot_mission/__init__.py` |
