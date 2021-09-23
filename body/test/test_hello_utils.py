import unittest
import stretch_body.hello_utils

import time
import random
import warnings


class TestHelloUtils(unittest.TestCase):

    def test_yaml_file_released(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            # read yaml, generating a ResourceWarning if the file is not released
            yaml = stretch_body.hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
            self.assertTrue(len(w) == 0)

    def test_reading_invalid_yaml(self):
        """Verify that read_fleet_yaml returns empty dict on invalid file.
        """
        read_params = stretch_body.hello_utils.read_fleet_yaml('invalid_file123.yaml')
        self.assertEqual(read_params, {})
        read_params1 = stretch_body.hello_utils.read_fleet_yaml('')
        self.assertEqual(read_params1, {})

    def test_overwriting_params(self):
        """Test the behavior of the overwrite_dict method.
        """
        dee1 = {'param1': 1}
        der1 = {'param2': 2}
        stretch_body.hello_utils.overwrite_dict(dee1, der1)
        self.assertEqual(dee1, {'param1': 1, 'param2': 2})

        dee2 = {'param1': 'to_override'}
        der2 = {'param1': 'over'}
        stretch_body.hello_utils.overwrite_dict(dee2, der2)
        self.assertEqual(dee2, {'param1': 'over'})

        dee3 = {'param1': {'motion': 'to_override', 'no_change': 1}}
        der3 = {'param1': {'motion': 'over'}}
        stretch_body.hello_utils.overwrite_dict(dee3, der3)
        self.assertEqual(dee3, {'param1': {'motion': 'over', 'no_change': 1}})

        dee4 = {'param1': {'motion': 'same', 'no_change': 1}}
        der4 = {'param1': {'motion': {}}}
        stretch_body.hello_utils.overwrite_dict(dee4, der4)
        self.assertEqual(dee4, {'param1': {'motion': 'same', 'no_change': 1}})

        dee5 = {'param1': {'motion': {}, 'no_change': 1}}
        der5 = {'param1': {'motion': 2}}
        stretch_body.hello_utils.overwrite_dict(dee5, der5)
        self.assertEqual(dee5, {'param1': {'motion': {}, 'no_change': 1}})

    def test_overwriting_vs_updating_params(self):
        """Verify the difference between overwrite_dict and updating a dict.
        """
        overider1 = {"robot": {"motion": {"max": 100}}}
        overidee1 = {"robot": {"motion": {"min": -100}}}
        stretch_body.hello_utils.overwrite_dict(overidee1, overider1)
        self.assertEqual(overidee1, {"robot": {"motion": {"max": 100, "min": -100}}})

        overider2 = {"robot": {"motion": {"max": 100}}}
        overidee2 = {"robot": {"motion": {"min": -100}}}
        overidee2.update(overider2)
        self.assertNotEqual(overidee1, overidee2)

    def test_pretty_print_dict(self):
        dict1 = {"param1": 1, "param2": 2}
        stretch_body.hello_utils.pretty_print_dict("params", dict1)

        dict2 = {"robot": {"motion": {"max": 100, "min": -100}, "retry": True}}
        stretch_body.hello_utils.pretty_print_dict("Stretch", dict2)

    def test_create_time_string(self):
        """Verify time strings match
        """
        t = time.localtime()
        expected_time_string = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(2) + str(t.tm_min).zfill(2) + str(t.tm_sec).zfill(2)
        actual_time_string = stretch_body.hello_utils.create_time_string()
        self.assertEqual(expected_time_string, actual_time_string)

    def test_get_stretch_directory(self):
        """
        """
        import os
        if os.environ.get('HELLO_FLEET_PATH', None) is not None:
            self.assertNotEqual(stretch_body.hello_utils.get_stretch_directory(), "/tmp/")
            original_fleet_path = os.environ['HELLO_FLEET_PATH']
            del os.environ['HELLO_FLEET_PATH']
            self.assertEqual(stretch_body.hello_utils.get_stretch_directory(), "/tmp/")
            os.environ['HELLO_FLEET_PATH'] = original_fleet_path
        else:
            self.assertEqual(stretch_body.hello_utils.get_stretch_directory(), "/tmp/")

    def test_mark_loop_start(self):
        """Verify that loop start time works properly
        """ 
        test_loop_name = "test_loop_name"
        test_loop_rate = 100
        test_stats = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)

        test_stats.mark_loop_start()
        time_ = time.time()
        ##Just to determine places value
        print("loop start ", test_stats.ts_loop_start, " time ", time_)
        self.assertAlmostEqual(time_, test_stats.ts_loop_start, places = 1)

    def test_mark_loop_end(self):
        """Verify that mark loop end updates LoopStats correctly
        """
        test_loop_name = "test_loop_name"
        test_loop_rate = 100
        test_stats = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)

        target_freq = 5 #In Hz
        self.assertEqual(test_stats.mark_loop_end(), None)

        test_stats.mark_loop_start()
        time.sleep(1/target_freq)
        test_stats.mark_loop_end()
        self.assertAlmostEqual(test_stats.ts_loop_end, time.time(), places = 1)

        time.sleep(1/target_freq)

        #Maybe not needed anymore
        #self.assertNotEqual(test_stats.status['execution_time_s'], 0)
        #self.assertNotEqual(test_stats.status['curr_rate_hz'], 0)
        #self.assertNotEqual(test_stats.status['min_rate_hz'], 10000000)
        #self.assertNotEqual(test_stats.status['max_rate_hz'], 0)
        #self.assertNotEqual(type(self.rate_log), type(None))
        #self.assertNotEqual(test_stats.status['avg_rate_hz'], 0)
    
    @unittest.skip(reason="Doesn't assert anything, helpful to check different prints")
    def test_pretty_print_loopStats(self):
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
        
        time.sleep(1/stall_time)
        
        test_stats_2.mark_loop_end()
        test_stats_1.pretty_print()

    @unittest.skip(reason="Doesn't assert anything, helpful to visualize loopstats histogram")
    def test_generate_rate_histogram(self):
        test_loop_name = "test_loop_name"
        test_loop_rate = 100
        test_stats = stretch_body.hello_utils.LoopStats(test_loop_name, test_loop_rate)
        stall_time = 5
        
        test_stats.mark_loop_start()
        test_stats.mark_loop_end()
        
        time.sleep(1/stall_time)
        test_stats.mark_loop_end()
        
        time.sleep(1/stall_time)
        test_stats.mark_loop_end()

        time.sleep(1/stall_time)
        test_stats.mark_loop_end()

        time.sleep(1/stall_time)
        test_stats.mark_loop_end()

        test_stats.generate_rate_histogram()

    def test_loop_rate_avg(self):
        """Verify that loop rate averages out correctly after few iterations
        """
        test_loop_name = "TestLoopAvg"
        test_loop_rate = 5.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)
        
        target_freq = 5.0 #Trying to create a frequency in Hz
        iterations = 50 #Number of iterations to check average loop rate

        for i in range(iterations):
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()

        #Average caclulated over the last 100 runs, so a few dry runs to ensure fair checking
        for i in range(iterations):
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()
            self.assertAlmostEqual(test_stats.status['avg_rate_hz'], target_freq, places = 1)

        #TODO: Add places according to actual numbers generated
        print(" Loop rate average: ", test_stats.status['avg_rate_hz'], " target frequency: ", target_freq)
        
    def test_loop_rate_min(self):
        print("Starting test for min loop rate ")

        test_loop_name = "TestLoopRateMin"
        test_loop_rate = 5.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)
        
        loop_rate_target = [3.0, 3.125, 3.25, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]

        for target_freq in loop_rate_target:
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()
        
        self.assertAlmostEqual(test_stats.status['min_rate_hz'], min(loop_rate_target), places = 0)

    def test_loop_rate_max(self):
        print("Starting test for max loop rate ")

        test_loop_name = "TestLoopRateMax"
        test_loop_rate = 25.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        loop_rate_target = [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0, 25.0, 25.1]

        for target_freq in loop_rate_target:
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['max_rate_hz'], max(loop_rate_target), places=-1)

    def test_execution_time_ms(self):
        print("Starting test for execution time ms")

        test_loop_name = "TestLoopExecution"
        test_loop_rate = 5.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        loop_rate_target = [3.0, 3.5, 4.0, 4.5, 5.0, 5.5, 6.0]
        
        test_stats.mark_loop_start()
        test_stats.mark_loop_end()
        self.assertAlmostEqual(test_stats.status['execution_time_s'], 0, places = 1)

        for target_freq in loop_rate_target:
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()

            self.assertAlmostEqual(test_stats.status['execution_time_s'], 1/target_freq, places = 1)

    @unittest.skip(reason="Doesn't work yet")
    def test_loop_warns(self):
        print("Starting test for loop warns")

        test_loop_name = "TestLoopAvg"
        test_loop_rate = 5.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name = test_loop_name, target_loop_rate = test_loop_rate)

        #For a target_loop_rate of 5, execution time > 0.2s should generate a warning
        loop_rate_target_warning = [1.0, 2.0, 2.5, 3.0, 4.0] #Since the target rate is 5.0, values less than that will generate warning
        loop_rate_target_no_warning =  [5, 6, 8, 10]
        
        test_stats.mark_loop_start()
        test_stats.mark_loop_end()
        for target_freq in loop_rate_target_warning:
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()
            #print(" execution time was ", test_stats.status['execution_time_s'], " and loop warns ", test_stats.status['loop_warns'])

        self.assertEqual(test_stats.status['missed_loops'], len(loop_rate_target_warning))

        for target_freq in loop_rate_target_no_warning:
            test_stats.mark_loop_start()
            time.sleep(1/target_freq)
            test_stats.mark_loop_end()

        #No new warnings
        self.assertEqual(test_stats.status['missed_loops'], len(loop_rate_target_warning))

    def test_faux_loop(self):
        print('Starting test_faux_loop ')
        target_loop_rate = 25.0
        s = stretch_body.hello_utils.LoopStats(loop_name='TestLoop', target_loop_rate=target_loop_rate)
        for i in range(200):
            s.mark_loop_start()
            time.sleep(0.5/ target_loop_rate) #Executing some computation
            s.mark_loop_end()
            noise_pct = random.random()*0.1
            time.sleep(s.get_loop_sleep_time()*(1+noise_pct)) #Add jitter due to 'threading', etc
        s.pretty_print()
        # s.generate_rate_histogram()
        self.assertTrue(s.status['missed_loops'] == 0)

    #TODO: Std deviation - use array of values 
