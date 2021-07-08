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

    @unittest.skip(reason='Currently hittin ~11hz for dxl')
    def test_robot_loops(self):
        print('Starting test_robot_loops')
        r = robot.Robot()
        r.startup()
        time.sleep(3.0)
        r.non_dxl_thread.stats.pretty_print()
        r.dxl_thread.stats.pretty_print()
        r.stop()
        self.assertTrue(r.dxl_thread.stats.status['loop_warns'] == 0)
        self.assertTrue(r.non_dxl_thread.stats.status['loop_warns'] == 0)

    @unittest.skip(reason='Currently hitting ~6hz and ~4hz for non_dxl and dxl respectively')
    def test_robot_loops_heavy(self):
        print('Starting test_robot_loops')
        r = robot.Robot()
        r.startup()
        start = time.time()
        while time.time() - start < 3.0:
            x = np.random.rand(3, 1000, 1000)
            x.tolist()
        r.non_dxl_thread.stats.pretty_print()
        r.dxl_thread.stats.pretty_print()
        r.stop()
        self.assertTrue(r.dxl_thread.stats.status['loop_warns'] == 0)
        self.assertTrue(r.non_dxl_thread.stats.status['loop_warns'] == 0)
