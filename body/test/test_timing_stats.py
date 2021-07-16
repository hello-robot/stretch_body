# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG") # TODO: Fails under PY3!
import logging
logging.getLogger('matplotlib').setLevel(logging.WARNING)

import unittest
import time
import random
import numpy as np
import stretch_body.hello_utils as hello_utils
import stretch_body.robot as robot


class TestTimingStats(unittest.TestCase):

    def test_faux_loop(self):
        print('Starting test_faux_loop ')
        target_loop_rate = 25.0
        s = hello_utils.LoopStats(loop_name='TestLoop', target_loop_rate=target_loop_rate)
        for i in range(200):
            s.mark_loop_start()
            time.sleep(0.5/ target_loop_rate) #Executing some computation
            s.mark_loop_end()
            noise_pct = random.random()*0.1
            time.sleep(s.get_loop_sleep_time()*(1+noise_pct)) #Add jitter due to 'threading', etc
        s.pretty_print()
        # s.display_rate_histogram()
        self.assertTrue(s.status['loop_warns'] == 0)

    @unittest.skip(reason='Currently hitting ~11hz for dxl')
    def test_robot_loops(self, rate_missed_threshold=0.1):
        print('Starting test_robot_loops')
        r = robot.Robot()
        r.startup()
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
        r.startup()
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
