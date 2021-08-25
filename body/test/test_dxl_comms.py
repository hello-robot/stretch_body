# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.device
import stretch_body.robot as robot
import numpy as np

class TestTimingStats(unittest.TestCase):
    def test_thread_starvation_group_sync_read(self):
        robot = stretch_body.robot.Robot()
        robot.end_of_arm.params['use_group_sync_read']=1
        print(robot.end_of_arm.joints)
        print('Starting test_thread_starvation')
        print('Latency timer of %f'%robot.end_of_arm.params['dxl_latency_timer'])
        print('Testing on tool %s'%robot.params['tool'])
        robot.startup()
        try:
            for itr in range(100): #Make large CPU load
                x = np.random.rand(3, 1000, 1000)
                x.tolist()
        except (IndexError, IOError) as e:
            self.fail("IndexError or IOError failure in comms")
        self.assertTrue(robot.end_of_arm.comm_errors.status['n_rx']<2)
        robot.end_of_arm.comm_errors.pretty_print()
        robot.stop()
