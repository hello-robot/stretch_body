# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.stepper
import stretch_body.pimu
import stretch_body.robot
import stretch_body.wacc
import time
import threading
import random



s = stretch_body.stepper.Stepper('/dev/hello-motor-arm')
s.startup(threaded=False)
w = stretch_body.wacc.Wacc()
w.startup(threaded=False)
p = stretch_body.pimu.Pimu()
p.startup(threaded=False)

class TestSingleNonDxlRates(unittest.TestCase):
    """
    This will test the locking mechanism for each arduino device.
    It ensures that the transport layer locks are correctly implemented,
    preventing two threads from trouncing eachother
    """
    def test_stepper_v1(self):
        s.transport.set_version(1)
        ts=time.time()
        for i in range(100):
            s.set_command(mode=0)
            s.push_command()
            s.pull_status()
        dt=time.time()-ts
        print('Stepper V1 push-pull rate of %f Hz' % (100 / dt))

    def test_stepper_v0(self):
        s.transport.set_version(0)
        ts=time.time()
        for i in range(100):
            s.set_command(mode=0)
            s.push_command()
            s.pull_status()
        dt=time.time()-ts
        print('Stepper V0 push-pull rate of %f Hz' % (100 / dt))

    def test_wacc_v0(self):
        w.transport.set_version(0)
        ts=time.time()
        for i in range(100):
            w.set_D3(0)
            w.push_command()
            w.pull_status()
        dt=time.time()-ts
        print('Wacc V0 push-pull rate of %f Hz' % (100 / dt))