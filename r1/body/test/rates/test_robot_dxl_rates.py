# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.robot
import time



r=stretch_body.robot.Robot()
r.startup()

class TestRobotDxlRates(unittest.TestCase):
    def test_robot_head(self):
        ts=time.time()
        sign=1.0
        for i in range(100):
            r.head.move_by(joint='head_pan',x_r=sign*.000025)
            r.head.move_by(joint='head_tilt', x_r=sign*.000025)
            sign=sign*-1
        dt=time.time()-ts
        rate=100/dt
        print('Robot Head DXL push rate of %f Hz' % rate)
        self.assertTrue(rate>20.0)


    def test_robot_wrist(self):
        ts=time.time()
        sign=1.0
        for i in range(100):
            r.end_of_arm.move_by(joint='wrist_yaw',x_r=sign*.000025)
            r.end_of_arm.move_by(joint='stretch_gripper', x_r=sign*.000025)
            sign=sign*-1
        dt=time.time()-ts
        rate=100/dt
        print('Robot Head DXL push rate of %f Hz' % rate)
        self.assertTrue(rate>20.0)