# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu
import stretch_body.robot
import time


class TestRobotCommandAtMaxRate(unittest.TestCase):


    def is_incremented_uint16_t(self,old_val,new_val):
        """
        Check that new value is old_val+1 (modulo 16 bit rollover)
        """
        if old_val == 65535:
            return new_val==0
        return old_val+1==new_val

    def test_robot_command_at_max_rate(self):
        """
        Check that can command all steppers at fastest rate comms will allow and no commands are dropped
        Do it w/o threads
        """
        r = stretch_body.robot.Robot()
        self.assertTrue(r.startup(start_non_dxl_thread=False,start_dxl_thread=False))

        stat_last=None
        n_itr=1000
        init = time.time()
        prev = init
        itr=0

        for i in range(n_itr):
            curr = time.time()
            diff = curr - prev
            prev = curr
            rate = None if diff == 0.0 else 1 / diff
            itr = itr + 1

            print('')
            print(f'---LOOP={itr}!  rate={rate:.2f}-----')

            r.base.translate_by(0)
            r.lift.move_by(0)
            r.arm.move_by(0)
            r.push_command()

            time.sleep(.01) #Give time for stepper to respond to sync pulse

            r.pull_status()
            r.arm.motor.pull_status_aux()
            r.lift.motor.pull_status_aux()
            r.base.left_wheel.pull_status_aux()
            r.base.right_wheel.pull_status_aux()

            # print('pimu',r.pimu.status,r.pimu.status_aux)
            # print('arm', r.arm.motor.status,r.arm.motor.status_aux)
            # print('lift', r.lift.motor.status,r.lift.motor.status_aux)
            # print('right_wheel', r.base.right_wheel.status,r.base.right_wheel.status_aux)
            # print('left_wheel', r.base.left_wheel.status,r.base.left_wheel.status_aux)


            if stat_last is not None: #Check that sync made it through

                self.assertTrue(self.is_incremented_uint16_t(stat_last['arm']['cmd_cnt_rpc'],r.arm.motor.status_aux['cmd_cnt_rpc']))
                self.assertTrue(self.is_incremented_uint16_t(stat_last['arm']['cmd_cnt_exec'],r.arm.motor.status_aux['cmd_cnt_exec']))
                self.assertTrue(r.arm.motor.status_aux['cmd_rpc_overflow'] - stat_last['arm']['cmd_rpc_overflow'] == 0)
                self.assertFalse(r.arm.motor.status['waiting_on_sync'])

                self.assertTrue(self.is_incremented_uint16_t(stat_last['lift']['cmd_cnt_rpc'],r.lift.motor.status_aux['cmd_cnt_rpc']))
                self.assertTrue(self.is_incremented_uint16_t(stat_last['lift']['cmd_cnt_exec'],r.lift.motor.status_aux['cmd_cnt_exec']))
                self.assertTrue(r.lift.motor.status_aux['cmd_rpc_overflow'] - stat_last['lift']['cmd_rpc_overflow'] == 0)
                self.assertFalse(r.lift.motor.status['waiting_on_sync'])

                self.assertTrue(self.is_incremented_uint16_t(stat_last['right_wheel']['cmd_cnt_rpc'],r.base.right_wheel.status_aux['cmd_cnt_rpc']))
                self.assertTrue(self.is_incremented_uint16_t(stat_last['right_wheel']['cmd_cnt_exec'],r.base.right_wheel.status_aux['cmd_cnt_exec']))
                self.assertTrue(r.base.right_wheel.status_aux['cmd_rpc_overflow'] - stat_last['arm']['cmd_rpc_overflow'] == 0)
                self.assertFalse(r.base.right_wheel.status['waiting_on_sync'])

                self.assertTrue(self.is_incremented_uint16_t(stat_last['left_wheel']['cmd_cnt_rpc'],r.base.left_wheel.status_aux['cmd_cnt_rpc']))
                self.assertTrue(self.is_incremented_uint16_t(stat_last['left_wheel']['cmd_cnt_exec'],r.base.left_wheel.status_aux['cmd_cnt_exec']))
                self.assertTrue(r.base.left_wheel.status_aux['cmd_rpc_overflow'] - stat_last['arm']['cmd_rpc_overflow'] == 0)
                self.assertFalse(r.base.left_wheel.status['waiting_on_sync'])

                self.assertTrue(r.pimu.status['motor_sync_cnt'] - stat_last['motor_sync_cnt'] == 1)
            stat_last = {'arm':r.arm.motor.status_aux.copy(),'right_wheel':r.base.right_wheel.status_aux.copy(),
                         'left_wheel':r.base.left_wheel.status_aux.copy(),'lift':r.lift.motor.status_aux.copy()}
            stat_last['motor_sync_cnt'] = r.pimu.status['motor_sync_cnt']

        r.stop()