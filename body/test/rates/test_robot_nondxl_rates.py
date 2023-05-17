# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.robot
import time



r=stretch_body.robot.Robot()
r.startup()

class TestRobotNonDxlRates(unittest.TestCase):

    def test_robot_v0(self):
        r.arm.motor.transport.set_version(0)
        r.lift.motor.transport.set_version(0)
        r.base.left_wheel.transport.set_version(0)
        r.base.right_wheel.transport.set_version(0)
        r.pimu.transport.set_version(0)
        r.wacc.transport.set_version(0)
        ts=time.time()
        for i in range(100):
            r.arm.move_by(0)
            r.lift.move_by(0)
            r.base.translate_by(0)
            r.pimu.set_fan_on()
            r.wacc.set_D2(0)
            r.push_command()
        dt=time.time()-ts
        rate=100/dt
        print('Robot NonDxl V0 push rate of %f Hz' % rate)
        self.assertTrue(rate>25.0)

    def test_robot_v1(self):
        r.arm.motor.transport.set_version(1)
        r.lift.motor.transport.set_version(1)
        r.base.left_wheel.transport.set_version(1)
        r.base.right_wheel.transport.set_version(1)
        r.pimu.transport.set_version(1)
        r.wacc.transport.set_version(1)
        ts=time.time()
        for i in range(100):
            r.arm.move_by(0)
            r.lift.move_by(0)
            r.base.translate_by(0)
            r.pimu.set_fan_on()
            r.wacc.set_D2(0)
            r.push_command()
        dt=time.time()-ts
        rate = 100 / dt
        print('Robot NonDxl V1 push rate of %f Hz' % rate)
        self.assertTrue(rate > 45.0)
