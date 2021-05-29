import unittest
import stretch_body.robot

import time


class TestRobot(unittest.TestCase):

    def test_home_and_stow(self):
        """Test robot homes and stows correctly.

        Note: this test assumes the stretch_gripper end-effector is attached.
        """
        r = stretch_body.robot.Robot()
        r.startup()

        r.home()
        r.pull_status()
        time.sleep(1) # wrist_yaw yields 3.4 (stowed position) unless sleep here
        self.assertAlmostEqual(r.status['lift']['pos'], 0.58, places=1)
        self.assertAlmostEqual(r.status['arm']['pos'], 0.1, places=4)
        self.assertAlmostEqual(r.status['head']['head_tilt']['pos'], 0.0, places=2)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], 0.0, places=2)
        self.assertAlmostEqual(r.status['end_of_arm']['wrist_yaw']['pos'], 0.0, places=1)
        self.assertAlmostEqual(r.status['end_of_arm']['stretch_gripper']['pos'], 0.0, places=1)

        r.stow()
        r.pull_status()
        self.assertAlmostEqual(r.status['lift']['pos'], 0.2, places=1)
        self.assertAlmostEqual(r.status['arm']['pos'], 0.0, places=4)
        self.assertAlmostEqual(r.status['head']['head_tilt']['pos'], 0.0, places=2)
        self.assertAlmostEqual(r.status['head']['head_pan']['pos'], 0.0, places=2)
        self.assertAlmostEqual(r.status['end_of_arm']['wrist_yaw']['pos'], 3.4, places=1)
        self.assertAlmostEqual(r.status['end_of_arm']['stretch_gripper']['pos'], 0.0, places=1)

        r.stop()
