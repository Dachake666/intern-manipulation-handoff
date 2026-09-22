from pathlib import Path
import numpy as np
import arm_profiles

_PROFILE = arm_profiles.arm_profile("left")

ARM_JOINT_IDS = {
    "left": list(_PROFILE["joint_ids"])
}

EE_LINK_ID = {
    "left": int(_PROFILE["ee_link_id"])
}

DEFAULT_URDF = "/home/dev/workspace/XF0112048/robot.urdf"

def sdk_q_to_urdf_q(q):
    return np.asarray(
        arm_profiles.sdk_to_urdf_deg(_PROFILE, q),
        dtype=float,
    )
