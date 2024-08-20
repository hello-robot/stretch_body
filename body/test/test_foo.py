import unittest
import stretch_body.hello_utils
import time

class TestSample(unittest.TestCase):

    def test_case_1(self):
        print("Running test_case_1")
        test_loop_name = "TestLoop1"
        test_loop_rate = 10.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name=test_loop_name, target_loop_rate=test_loop_rate)

        test_stats.mark_loop_start()
        time.sleep(0.1)
        test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['execution_time_s'], 0.1, places=1)

    def test_case_2(self):
        print("Running test_case_2")
        test_loop_name = "TestLoop2"
        test_loop_rate = 20.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name=test_loop_name, target_loop_rate=test_loop_rate)

        test_stats.mark_loop_start()
        time.sleep(0.05)
        test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['execution_time_s'], 0.05, places=1)

    def test_case_3(self):
        print("Running test_case_3")
        test_loop_name = "TestLoop3"
        test_loop_rate = 5.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name=test_loop_name, target_loop_rate=test_loop_rate)

        test_stats.mark_loop_start()
        time.sleep(0.2)
        test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['execution_time_s'], 0.2, places=1)

    def test_case_4(self):
        print("Running test_case_4")
        test_loop_name = "TestLoop4"
        test_loop_rate = 50.0
        test_stats = stretch_body.hello_utils.LoopStats(loop_name=test_loop_name, target_loop_rate=test_loop_rate)

        test_stats.mark_loop_start()
        time.sleep(0.02)
        test_stats.mark_loop_end()

        self.assertAlmostEqual(test_stats.status['execution_time_s'], 0.02, places=1)

if __name__ == '__main__':
    unittest.main()