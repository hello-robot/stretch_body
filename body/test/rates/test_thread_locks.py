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
        for i in range(100):
            self.stepper.pull_status()
            time.sleep(random.random()/100.0)
        self.done=True

s = stretch_body.stepper.Stepper('/dev/hello-motor-arm')
s.startup(threaded=False)
w = stretch_body.wacc.Wacc()
w.startup(threaded=False)
p = stretch_body.pimu.Pimu()
p.startup(threaded=False)

class TestThreadLocks(unittest.TestCase):
    """
    This will test the locking mechanism for each arduino device.
    It ensures that the transport layer locks are correctly implemented,
    preventing two threads from trouncing eachother
    """
    def test_stepper_thread_locks_v1(self):
        s.transport.set_version(1)
        t0 = Stepperthread('T0', s)
        t0.setDaemon(True)
        t0.start()
        t1 = Stepperthread('T1', s)
        t1.setDaemon(True)
        t1.start()
        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        rate = 100 / dt
        print('Two Stepper V1 thread rate of %f Hz' % (100 / dt))
        self.assertTrue(rate > 100.0)

    def test_stepper_thread_locks_v0(self):
        s.transport.set_version(0)
        t0 = Stepperthread('T0', s)
        t0.setDaemon(True)
        t0.start()
        t1 = Stepperthread('T1', s)
        t1.setDaemon(True)
        t1.start()
        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        rate = 100 / dt
        print('Two Stepper V0 thread rate of %f Hz' % (100 / dt))
        self.assertTrue(rate > 85.0)


    def test_wacc_thread_locks_v0(self):
        """
        Check that two wacc threads can't trounce each other'
        """
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
        rate = 100 / dt
        print('Two Wacc V0 thread rate of %f Hz' % (100 / dt))
        self.assertTrue(rate > 75.0)


    def test_wacc_thread_locks_v1(self):
        """
        Check that two wacc threads can't trounce each other'
        """
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
        rate = 100 / dt
        print('Two Wacc V1 thread rate of %f Hz' % (100 / dt))
        self.assertTrue(rate > 100.0)


    def test_pimu_thread_locks_v0(self):
        """
        Check that two pimu threads can't trounce each other'
        """
        p.transport.set_version(0)
        t0 = Pimuthread('T0', p)
        t0.setDaemon(True)
        t0.start()
        t1 = Pimuthread('T1', p)
        t1.setDaemon(True)
        t1.start()
        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        rate = 100 / dt
        print('Two Pimu V0 thread rate of %f Hz' % (100 / dt))
        self.assertTrue(rate > 45.0)


    def test_pimu_thread_locks_v1(self):
        """
        Check that two pimu threads can't trounce each other'
        """
        p.transport.set_version(1)
        t0 = Pimuthread('T0', p)
        t0.setDaemon(True)
        t0.start()
        t1 = Pimuthread('T1', p)
        t1.setDaemon(True)
        t1.start()
        ts = time.time()
        while not t0.done and not t1.done:
            time.sleep(.001)
        dt = time.time() - ts
        rate = 100 / dt
        print('Two Pimu V1 thread rate of %f Hz' % (100 / dt))
        self.assertTrue(rate > 75.0)
