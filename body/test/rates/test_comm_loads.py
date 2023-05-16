# # Logging level must be set before importing any stretch_body class
# import stretch_body.robot_params
# stretch_body.robot_params.RobotParams.set_logging_level("DEBUG") # TODO: Fails under PY3!
# import logging
# logging.getLogger('matplotlib').setLevel(logging.WARNING)

import unittest
import time
import random
import threading
import numpy as np
import stretch_body.hello_utils as hello_utils
import stretch_body.robot as robot
import stretch_body.stepper as stepper
from stretch_body.transport import TransportError

class TestCommLoads(unittest.TestCase):

    def test_comm_load_v0(self):
        print('Starting test_comm_load_v0')
        r = robot.Robot()
        self.assertTrue(r.startup())
        r.arm.motor.transport.set_version(0)
        r.base.left_wheel.transport.set_version(0)
        r.base.right_wheel.transport.set_version(0)
        r.lift.motor.transport.set_version(0)
        try:
            ts = time.time()
            for i in range(100):
                print(i)
                r.pimu.set_load_test()
                r.wacc.set_load_test()
                r.arm.motor.set_load_test()
                r.lift.motor.set_load_test()
                r.base.right_wheel.set_load_test()
                r.base.left_wheel.set_load_test()
                r.push_command()
                time.sleep(0.1)
            dt = time.time() - ts
            print('RATE for Transport V0', 100 / dt)
        except TransportError:
            self.assertTrue(0)
        r.stop()

