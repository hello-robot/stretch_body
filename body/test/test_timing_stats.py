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


class TestTimingStats(unittest.TestCase):

    @unittest.skip(reason='Currently hitting ~11hz for dxl')
    def test_robot_loops(self, rate_missed_threshold=0.1):
        print('Starting test_robot_loops')
        r = robot.Robot()
        self.assertTrue(r.startup())
        time.sleep(3.0)
        r.non_dxl_thread.stats.pretty_print()
        r.dxl_thread.stats.pretty_print()
        r.stop()
        self.assertLess(r.dxl_thread.stats.status['loop_warns'] / r.dxl_thread.stats.loop_cycles, rate_missed_threshold)
        self.assertLess(r.non_dxl_thread.stats.status['loop_warns'] / r.non_dxl_thread.stats.loop_cycles, rate_missed_threshold)

    @unittest.skip(reason='Currently hitting ~6hz and ~4hz for non_dxl and dxl respectively')
    def test_robot_loops_heavy(self, rate_missed_threshold=0.1):
        print('Starting test_robot_loops')
        r = robot.Robot()
        self.assertTrue(r.startup())
        start = time.time()
        while time.time() - start < 20.0:
            x = np.random.rand(3, 1000, 1000)
            x.tolist()
        r.non_dxl_thread.stats.pretty_print()
        r.dxl_thread.stats.pretty_print()
        r.stop()
        self.assertLess(r.dxl_thread.stats.status['loop_warns'] / r.dxl_thread.stats.loop_cycles, rate_missed_threshold)
        self.assertLess(r.non_dxl_thread.stats.status['loop_warns'] / r.non_dxl_thread.stats.loop_cycles, rate_missed_threshold)

    @unittest.skip(reason='Used to measure isolated perf, doesnt exit')
    def test_dxl_on_main_thread(self):
        r = robot.Robot()
        for k in r.devices.keys():
            if r.devices[k] is not None:
                if not r.devices[k].startup():
                    r.logger.warning('device %s failed to start' % k)
        target = 11.0
        dxl_thread = robot.DXLStatusThread(r, target_rate_hz=target)
        dxl_thread.run()
        self.assertAlmostEqual(dxl_thread.stats.status['loop_rate_av=g_hz'], target)

    @unittest.skip(reason='Used to measure isolated perf, doesnt exit')
    def test_non_dxl_on_main_thread(self):
        r = robot.Robot()
        for k in r.devices.keys():
            if r.devices[k] is not None:
                if not r.devices[k].startup():
                    r.logger.warning('device %s failed to start' % k)
        target = 25.0
        non_dxl_thread = robot.NonDXLStatusThread(r, target_rate_hz=target)
        non_dxl_thread.run()
        self.assertAlmostEqual(non_dxl_thread.stats.status['loop_rate_av=g_hz'], target)

    @unittest.skip(reason='Used to measure perf, doesnt test anything')
    def test_stepper_on_status_thread(self):
        class StatusThread(threading.Thread):
            def __init__(self, device, target_rate_hz=15.0):
                threading.Thread.__init__(self)
                self.device = device
                self.device_update_rate_hz = target_rate_hz
                self.stats = hello_utils.LoopStats(loop_name='StatusThread', target_loop_rate=self.device_update_rate_hz)
                self.shutdown_flag = threading.Event()
                self.start_time = time.time()

            def run(self):
                while not self.shutdown_flag.is_set() and time.time() - self.start_time < 10.0:
                    self.stats.mark_loop_start()
                    s.pull_status()
                    # s.pretty_print()
                    self.stats.mark_loop_end()
                    if not self.shutdown_flag.is_set():
                        time.sleep(self.stats.get_loop_sleep_time())
                s.logger.debug('Shutting down StatusThread')

        s = stepper.Stepper("/dev/hello-motor-lift")
        s.startup()

        target = 15.5
        status_thread = StatusThread(s, target_rate_hz=target)
        status_thread.setDaemon(True)
        status_thread.start()

        time.sleep(11.0)
        print('stopping')
        status_thread.stats.pretty_print()
        s.stop()
