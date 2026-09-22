from __future__ import annotations

import tempfile
import unittest
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import gen_tabletop_vision_replay_candidate as replay


SNAPSHOT = HERE / "tasks" / "task_tabletop_vision_ab_20260902.json"


class TabletopVisionReplayCandidateTests(unittest.TestCase):
    def test_current_single_anchor_fixed_pose_fails_closed(self):
        """当前 AB 数据不得为了展示动画而写出不可达的伪关节轨迹。"""
        with tempfile.TemporaryDirectory() as root:
            trajectory = Path(root) / "candidate.json"
            qualification = Path(root) / "qualification.json"
            with self.assertRaises(replay.ReplayCandidateBlocked) as caught:
                replay.build(SNAPSHOT, trajectory, qualification)
            self.assertEqual(caught.exception.stage, "TRANSFER_ABOVE_BIN")
            self.assertIn("需要有效 PB-SDK 标定", str(caught.exception))
            self.assertFalse(trajectory.exists())
            self.assertFalse(qualification.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
