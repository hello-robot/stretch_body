# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
# TODO: Fails under PY3!
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import time
from stretch_body.hello_utils import LoopStats
import random
import stretch_body.device
import stretch_body.robot as robot
d=stretch_body.device.Device() #Create logger

class TestTimingStats(unittest.TestCase):
    def test_faux_loop(self):
        print('Starting test_faux_loop ')
        target_loop_rate = 25.0
        s = LoopStats(loop_name='TestLoop', target_loop_rate=target_loop_rate)
        for i in range(200):
            s.mark_loop_start()
            time.sleep(0.5/ target_loop_rate) #Executing some computation
            s.mark_loop_end()
            noise_pct = random.random()*0.1
            time.sleep(s.get_loop_sleep_time()*(1+noise_pct)) #Add jitter due to 'threading', etc
        s.pretty_print()
        #s.display_rate_histogram()
        self.assertTrue(s.status['loop_warns'] == 0)

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



    def test_mark_loop_start(self):
        """verify that loop start time works properly
        """ 
        test_loop_name = "test_loop_name"
        test_loop_rate = 100
        test_stats = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)

        test_stats.mark_loop_start()
        time_ = time.time()
        self.assertAlmostEqual(time_, test_stats.ts_loop_start)

    def test_mark_loop_end(self):
        """Verify that mark loop end updates LoopStats correctly
        """
        test_loop_name = "test_loop_name"
        test_loop_rate = 100
        test_stats = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)

        stall_time = 5 #Stall for 5 seconds
        self.assertEqual(test_stats.mark_loop_end(), None)

        test_stats.mark_loop_start()
        time.sleep(stall_time)
        test_stats.mark_loop_end()
        self.assertAlmostEqual(test_stats.ts_loop_end, time.time())

        time.sleep(stall_time)

        self.assertNotEqual(test_stats.status['execution_time_ms'], 0)
        self.assertNotEqual(test_stats.status['loop_rate_hz'], 0)
        self.assertNotEqual(test_stats.status['loop_rate_min_hz'], 10000000)
        self.assertNotEqual(test_stats.status['loop_rate_max_hz'], 0)
        self.assertNotEqual(type(self.rate_log), type(None))
        self.assertNotEqual(test_stats.status['loop_rate_avg_hz'], 0)
    
    def test_pretty_print_loopStats(self): #Should this test asert something or should I change the function name
        test_loop_name = "test_loop_1"
        test_loop_rate = 100
        test_stats_1 = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)
        stall_time = 5

        test_stats_1.pretty_print()

        test_loop_name = "test_loop_2"
        test_loop_rate = 100
        test_stats_2 = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)
        
        test_stats_2.mark_loop_start()
        test_stats_2.mark_loop_end()
        
        time.sleep(stall_time)
        
        test_stats_2.mark_loop_end()
        test_stats_1.pretty_print()



    def display_rate_histogram(self):
        test_loop_name = "test_loop_name"
        test_loop_rate = 100
        test_stats = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)
        stall_time = 5
        
        test_stats.mark_loop_start()
        test_stats.mark_loop_end()
        
        time.sleep(stall_time)
        test_stats.mark_loop_end()
        
        time.sleep(stall_time)
        test_stats.mark_loop_end()

        time.sleep(stall_time)
        test_stats.mark_loop_end()

        time.sleep(stall_time)
        test_stats.mark_loop_end()

        test_stats.display_rate_histogram()

    def test_loop_rate_avg(self):
        print("Starting test for loop rate average")
        test_loop_name = "TestLoopAvg"
        test_loop_rate = 25.0
        test_stats = LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        stall_time = 5.0 #Trying to create a frequency of 5Hz

        #Average caclulated over the last 100 runs, so a few dry runs to ensure fair checking
        for i in range(test_stats.nlog):
            test_stats.mark_loop_start()
            time.sleep(stall_time)
            test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['loop_rate_avg_hz'], 5)


        for i in range(100):
            test_stats.mark_loop_start()
            time.sleep(stall_time)
            test_stats.mark_loop_end()
            self.assertAlmostEqual(test_stats.status['loop_rate_avg_hz'], 5)
 
        
    def test_loop_rate_min(self):
        print("Starting test for min loop rate ")

        test_loop_name = "TestLoopRateMin"
        test_loop_rate = 25.0
        test_stats = LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        loop_rate_target = {3.0, 3.125, 3.25, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0}

        test_stats.mark_loop_start()
        for stall_time in loop_rate_target:
            time.sleep(stall_time)
            test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['loop_rate_min_hz'], 3.0)

    def test_loop_rate_max(self):
        print("Starting test for max loop rate ")

        test_loop_name = "TestLoopRateMax"
        test_loop_rate = 25.0
        test_stats = LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        loop_rate_target = {3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 25.0, 25.1}

        test_stats.mark_loop_start()

        for stall_time in loop_rate_target:
            time.sleep(stall_time)
            test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['loop_rate_max_hz'], 25.1)

    def test_execution_time_ms(self):
        print("Starting test for execution time ms")

        test_loop_name = "TestLoopExecution"
        test_loop_rate = 25.0
        test_stats = LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        loop_rate_target = {3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0,}
        test_stats.mark_loop_start()


        test_stats.mark_loop_end()
        self.assertEqual(test_stats.status['execution_time_ms'], 0)

        total_time = 0
        for stall_time in loop_rate_target:
            time.sleep(stall_time)
            total_time += stall_time
            self.assertAlmostEqual(test_stats.status['execution_time_ms'], total_time*1000)

    def test_loop_warns(self):
        print("Starting test for loop warns")

        test_loop_name = "TestLoopAvg"
        test_loop_rate = 25.0
        test_stats = LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        test_stats.mark_loop_start()
        time.sleep(0.1)
        test_stats.mark_loop_end()
        self.assertEqual(test_stats.status['loop_warns'], 0)

        test_stats.mark_loop_start()
        time.sleep(0.5)
        test_stats.mark_loop_end()
        self.assertEqual(test_stats['loop_warns'], 1)

        
