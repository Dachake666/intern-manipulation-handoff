# Track A Hybrid Pick/Place — Final Debian Handoff

## Final status

Hybrid pick/place trajectory has been executed successfully on the real robot
multiple times.

The original hybrid candidate JSON was NOT modified.

### Candidate plan

File:
`tabletop_pick_place_hybrid_CANDIDATE.json`

SHA-256:
`3f3e36f370521e4ab815e377ddb491a3cd2adb24d95dcf843451d03d7587c06e`

### Final real-motion executor

File:
`execute_tabletop_hybrid_trial.py`

Final executor SHA at the time of final modification:
`de5ccc1252a03248cc9a5315a2b2fc180953bd70217176116bf294ca653684c5`

---

## Final hybrid motion structure

1. PICK_HOVER      - MoveWorlds
2. PICK_DESCEND    - MoveWorlds
3. CLOSE_AT_PICK   - gripper close
4. PICK_ASCEND     - MoveWorlds
5. TRANSFER_MID    - MoveJoints
6. PLACE_HOVER     - MoveJoints
7. PLACE_DESCEND   - MoveWorlds
8. OPEN_AT_PLACE   - gripper open
9. PLACE_ASCEND    - MoveWorlds
10. RETURN_SAFE    - MoveJoints to fixed canonical Home joints

---

## Changes made on Debian

### 1. Added real hybrid executor

The original precheck package was read-only.

A new executor was added:

`execute_tabletop_hybrid_trial.py`

It supports:

MoveWorlds -> MoveJoints -> MoveJoints -> MoveWorlds

---

### 2. Fixed move-state false abort

An early trial reached the requested Worlds pose but the controller move-state
remained true briefly while motion was settling.

The executor was changed to wait for controller idle after arrival instead of
immediately treating the state as another robot program moving the arm.

---

### 3. Software tolerances were relaxed

The relaxed checks apply to software arrival / redundant-branch diagnostics.

They do NOT change the physical joint limits.

Representative final software tolerances include:

- start position tolerance: 80 mm
- start orientation tolerance: 20 deg
- Worlds arrival tolerance: 5 mm
- Worlds orientation tolerance: 3 deg
- MoveJ arrival tolerance: 1 deg
- PICK redundant-branch diagnostic threshold: 8 deg
- handoff position tolerance: 60 mm
- handoff orientation tolerance: 20 deg
- dense Worlds IK step threshold: 6 deg

---

### 4. PICK_ASCEND branch delta changed from hard abort to warning

For the 7-DOF arm, the same XYZUVW can correspond to different redundant
joint configurations.

The observed PICK_ASCEND branch difference varied between runs even when
the Cartesian endpoint was correct.

Therefore the branch difference is now diagnostic information only.

A large branch delta prints a WARNING but is no longer itself a motion abort.

---

### 5. Added live MoveJ dense joint-limit precheck

Before each real MoveJ command the executor now reads the ACTUAL live
seven-joint configuration.

It then densely interpolates:

actual live joints -> MoveJ target joints

and checks every interpolated joint point against the effective joint limits.

This makes the safety check depend on the real current redundant configuration,
rather than only the offline planned branch.

---

### 6. Hard joint-limit protection remains enabled

The following protections were NOT removed:

- controller J1-J7 hard limits
- armJointOutLimit checks
- soft-stop checks
- emergency route clearing on abnormal exit
- 5 deg joint-limit safety margin during the tested runs

Typical runtime argument:

`--limit-margin-deg 5`

---

### 7. RETURN_SAFE changed from MoveWorlds to MoveJoints

Returning to the same Cartesian Safe pose with MoveWorlds did not guarantee
the same seven-joint redundant configuration.

RETURN_SAFE was therefore overridden in the executor to use MoveJoints.

Canonical original Home joint target:

J1 = -17.711 deg
J2 =  29.247 deg
J3 =   6.936 deg
J4 = -75.605 deg
J5 = -13.196 deg
J6 = -11.823 deg
J7 =  12.235 deg

The candidate JSON itself remains unchanged. The RETURN_SAFE behavior is
overridden only by the real-motion executor.

---

## Real-robot results

The hybrid trajectory has completed the full sequence on the real robot:

pick
-> lift
-> TRANSFER_MID MoveJ
-> PLACE_HOVER MoveJ
-> descend
-> release
-> ascend
-> return Home

and completed with:

`PASS_RETURNED_SAFE`

Representative measured MoveJ endpoint results from an early full successful run:

TRANSFER_MID:
- Cartesian endpoint error approximately 2.15 mm / 0.25 deg

PLACE_HOVER:
- Cartesian endpoint error approximately 2.20 mm / 0.25 deg

Multiple subsequent full PASS runs were completed.

The five most recent PASS logs are included under:

`pass_logs/`

---

## Network used during testing

Local IP:
`192.168.8.185`

Robot/arm IP changed between:
`192.168.8.147`
and
`192.168.8.148`

The runtime CLI arguments should always match the robot's current IP.

Arm port:
`8080`

Successful trials were performed at low global speeds including 3% and 5%.

---

## Files

- `tabletop_pick_place_hybrid_CANDIDATE.json`
  Original unmodified hybrid candidate.

- `execute_tabletop_hybrid_trial.py`
  Final Debian real hybrid-motion executor.

- `precheck_tabletop_hybrid.py`
  Original read-only hybrid controller precheck.

- `execute_tabletop_pick_place_worlds.py`
  Original dependency/reference executor.

- `sdk_session.py`
  SDK/session helpers.

- `robot_lock.py`
  Robot mutual exclusion lock.

- `scrub_log.py`
  Log utility.

- `hybrid_precheck_20260907_151817_557889.json`
  Real-controller read-only precheck report.

- `pass_logs/hybrid_trial_run_*.json`
  Five most recent successful real-motion logs.

- `SHA256SUMS`
  SHA-256 manifest for this handoff.
