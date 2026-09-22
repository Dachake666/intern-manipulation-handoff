# Arm Enable / varget / TCP 8080 issue — 2026-09-01

## Symptom
PilotSDK initialization intermittently showed:

- armInitData: 0
- link: (0, True)
- enable status: (0, False)
- servo: (0, False)

while robot login information already reported:

- svo_on: true

Failed runs did not show:

- 文件上传成功: uploads/varget.log

## Root cause
Multiple old/stopped Python + PilotSDK processes were still alive and simultaneously
listening on TCP 8080.

Observed stale processes included:

- test_move_worlds.py
- execute_worlds_10x.py
- execute_trajectory.py

They were in stopped (`T`) state but still retained their TCP 8080 listeners.

This interfered with the current PilotSDK varget.log upload/state initialization path.

## Verification
Before cleanup:
- varget.log missing
- enable=False
- servo=False
- waiting 10 seconds did not recover

After stale processes were terminated:
- 文件上传成功: uploads/varget.log
- enable=True immediately after armInitData
- servo=True
- diagnostic polling remained True for 10 seconds
- real trajectory subsequently completed successfully 5 consecutive times

## Conclusion
This was not primarily a physical robot enable problem and was not solved by adding sleep.

When this symptom happens again, first check:

    netstat -ltnp 2>/dev/null | grep ':8080'

Also inspect robot-related Python processes:

    ps -ef | grep -E 'python3.*(test_move_worlds|execute_worlds|execute_trajectory|execute_servo)' | grep -v grep

Avoid leaving PilotSDK robot programs suspended with Ctrl+Z for long periods.
A suspended process may still retain its TCP listener.

## Validated real trajectory
traj_bottle_taught_tcp_home_start_20260831_RUN.json

SHA256:
cce2a83fae21468f80f0aad487087b61650eac6f92dea65f9560091d0239ad2b

Executor:
execute_servo_grasp.py

Executor SHA prefix:
bc05bd76c2584dc6

Gripper force parameter:
8000

Real test:
5 consecutive successful executions after stale TCP 8080 listeners were removed.
