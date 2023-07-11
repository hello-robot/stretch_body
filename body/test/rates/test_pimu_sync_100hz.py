# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu
import stretch_body.robot
import time


class TestPimuSync100hz(unittest.TestCase):

    def test_pimu_sync_100Hz(self):
        """
        Check that can trigger motor sync at 50hz and no overruns
        """
        p=stretch_body.pimu.Pimu()
        self.assertTrue(p.startup(threaded=False))
        p.pull_status()
        p.trigger_motor_sync()
        msc0 = p.status['motor_sync_cnt']

        n_itr = 1000
        target_rate = 100.0  # Hz

        init = time.time()
        prev = init
        itr = 0
        lcnt = 0

        while itr < n_itr:
            curr = time.time()
            diff = curr - prev
            lcnt = lcnt + 1
            if diff > (1 / target_rate):
                prev = curr

                rate = None if diff == 0.0 else 1 / diff
                itr = itr + 1
                p.trigger_motor_sync()
                dt=(time.time()-curr)*1000.0
                pp = p.status['motor_sync_cnt'] - msc0
                print(f'LOOP={itr}! rate={rate:.2f} Pimu={pp}  Error={itr-pp} Lcnt={lcnt} Dt_ms={dt}')
                lcnt = 0
                # if(itr-pp!=0):
                #     print('Error')
                #     print(p.status)
                #     exit(0)

        print('Motor sync queues', p.status['motor_sync_queues'])
        print('Motor sync counts', p.status['motor_sync_cnt'], msc0)
        #self.assertTrue(p.status['motor_sync_queues'] == 0)
        self.assertTrue(p.status['motor_sync_cnt'] - msc0 == n_itr)
        p.stop()

