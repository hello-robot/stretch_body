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

class Waccthread(threading.Thread):
    def __init__(self, name,wacc):
        threading.Thread.__init__(self)
        self.wacc=wacc
        self.name=name
        self.done=False

    def run(self):
        for i in range(100):
            self.wacc.pull_status()
            time.sleep(random.random()/100.0)
        self.done=True

class Pimuthread(threading.Thread):
    def __init__(self, name,pimu):
        threading.Thread.__init__(self)
        self.pimu=pimu
        self.name=name
        self.done=False

    def run(self):
        for i in range(100):
            self.pimu.pull_status()
            time.sleep(random.random()/100.0)
        self.done=True

class Stepperthread(threading.Thread):
    def __init__(self, name,stepper):
        threading.Thread.__init__(self)
        self.stepper=stepper
        self.name=name
        self.done=False

    def run(self):
        for i in range(1000):
            self.stepper.pull_status()
            time.sleep(random.random()/100.0)
        self.done=True

class TestThreadLocks(unittest.TestCase):
    """
    This will test the locking mechanism for each arduino device.
    It ensures that the transport layer locks are correctly implemented,
    preventing two threads from trouncing eachother
    """
    def test_stepper_thread_locks_v1(self):
        w = stretch_body.stepper.Stepper('/dev/hello-motor-arm')
        w.startup()
        w.transport.set_version(1)

        t0 = Stepperthread('T0', w)
        t0.setDaemon(True)
        t0.start()

        t1 = Stepperthread('T1', w)
        t1.setDaemon(True)
        t1.start()

        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        print('Two Stepper thread rate of %f Hz' % (1000 / dt))

    def test_stepper_thread_locks_v0(self):
        w = stretch_body.stepper.Stepper('/dev/hello-motor-arm')
        w.startup()
        w.transport.set_version(0)

        t0 = Stepperthread('T0', w)
        t0.setDaemon(True)
        t0.start()

        t1 = Stepperthread('T1', w)
        t1.setDaemon(True)
        t1.start()

        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        print('Two Stepper thread rate of %f Hz' % (1000 / dt))

    def test_wacc_thread_locks_v0(self):
        """
        Check that two wacc threads can't trounce each other'
        Should test with both V0 and V1 transport
        """
        w = stretch_body.wacc.Wacc()
        w.startup()
        w.transport.set_version(0)

        t0 = Waccthread('T0', w)
        t0.setDaemon(True)
        t0.start()

        t1 = Waccthread('T1', w)
        t1.setDaemon(True)
        t1.start()

        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        print('Two Wacc thread rate of %f Hz' % (100 / dt))

    def test_wacc_thread_locks_v1(self):
        """
        Check that two wacc threads can't trounce each other'
        Should test with both V0 and V1 transport
        """
        w = stretch_body.wacc.Wacc()
        w.startup()
        w.transport.set_version(1)

        t0 = Waccthread('T0', w)
        t0.setDaemon(True)
        t0.start()

        t1 = Waccthread('T1', w)
        t1.setDaemon(True)
        t1.start()

        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        print('Two Wacc thread rate of %f Hz' % (100 / dt))

    def test_pimu_thread_locks_v0(self):
        """
        Check that two pimu threads can't trounce each other'
        Should test with both V0 and V1 transport
        """
        w = stretch_body.pimu.Pimu()
        w.startup()
        w.transport.set_version(0)

        t0 = Pimuthread('T0', w)
        t0.setDaemon(True)
        t0.start()

        t1 = Pimuthread('T1', w)
        t1.setDaemon(True)
        t1.start()

        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        print('Two Pimu thread rate of %f Hz' % (100 / dt))

    def test_pimu_thread_locks_v1(self):
        """
        Check that two pimu threads can't trounce each other'
        Should test with both V0 and V1 transport
        """
        w = stretch_body.pimu.Pimu()
        w.startup()
        w.transport.set_version(1)

        t0 = Pimuthread('T0', w)
        t0.setDaemon(True)
        t0.start()

        t1 = Pimuthread('T1', w)
        t1.setDaemon(True)
        t1.start()

        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        print('Two Pimu thread rate of %f Hz' % (100 / dt))