# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu
import stretch_body.robot
import time


class TestArmCommandAtMaxRate(unittest.TestCase):


    def is_incremented_uint16_t(self,old_val,new_val):
        """
        Check that new value is old_val+1 (modulo 16 bit rollover)
        """
        if old_val == 65535:
            return new_val==0
        return old_val+1==new_val
    def test_arm_command_at_max_rate(self):
        """
        Check that can command arm stepper at fastest rate comms will allow and no commands are dropped
        Do it w/o threads
        """
        a=stretch_body.arm.Arm()
        p=stretch_body.pimu.Pimu()
        self.assertTrue(a.startup(threaded=False))#r.startup(start_non_dxl_thread=False,start_dxl_thread=False))
        self.assertTrue(p.startup(threaded=False))
        a.pull_status()
        a.motor.pull_status_aux()
        #a.pretty_print()
        #print(a.motor.status_aux)
        stat_last=None
        n_itr=1000 #100000
        init = time.time()
        prev = init
        itr=0

        for i in range(n_itr):
            curr = time.time()
            diff = curr - prev
            prev = curr
            rate = None if diff == 0.0 else 1 / diff
            itr = itr + 1

            print(f'---LOOP={itr}!  rate={rate:.2f}-----')

            a.pull_status()
            p.pull_status()
            a.motor.pull_status_aux()

            if stat_last is not None: #Compare to previous cycle
                #print('1',a.motor.status_aux, 'motor_sync_cnt',p.status['motor_sync_cnt'],'waiting_on_sync',a.motor.status['waiting_on_sync'],'queues', p.status['motor_sync_queues'])
                self.assertTrue(a.motor.status_aux['cmd_cnt_rpc']-stat_last['cmd_cnt_rpc']==0)
                self.assertTrue(a.motor.status_aux['cmd_cnt_exec'] - stat_last['cmd_cnt_exec'] == 0)
                self.assertTrue(a.motor.status_aux['cmd_rpc_overflow'] - stat_last['cmd_rpc_overflow'] == 0)
                #On RE1 noise on sync line can cause inadvertant IRQ (very infrequently). Net result is an unexpected sync signal
                #Worst case the motion trigger early
                #self.assertTrue(a.motor.status_aux['sync_irq_overflow'] - stat_last['sync_irq_overflow'] == 0)
                #self.assertTrue(a.motor.status_aux['sync_irq_cnt'] - stat_last['sync_irq_cnt'] == 0)
                self.assertFalse(a.motor.status['waiting_on_sync'])
                self.assertTrue(p.status['motor_sync_cnt']- stat_last['motor_sync_cnt'] == 0)

            a.move_by(0)
            #print('Ctrl Cycle Count before Push',a.motor.status['ctrl_cycle_cnt'])
            a.push_command()
            #print('Ctrl Cycle Count after Push', a.motor.status['ctrl_cycle_cnt'])
            #time.sleep(0.01) #Give time for a ctrl cycle to run
            a.pull_status()
            #print('Ctrl Cycle Count after pull_status', a.motor.status['debug'])
            p.pull_status()
            a.motor.pull_status_aux()
            # print('%%%%%%%%%%%%%%%%')
            # print('pimu', p.status, p.status_aux)
            # print('arm', a.motor.status, a.motor.status_aux)

            if stat_last is not None:  # Check that RPC made it through
                #print('2',a.motor.status_aux, 'motor_sync_cnt',p.status['motor_sync_cnt'],'waiting_on_sync',a.motor.status['waiting_on_sync'],'queues', p.status['motor_sync_queues'])
                self.assertTrue(self.is_incremented_uint16_t(stat_last['cmd_cnt_rpc'],a.motor.status_aux['cmd_cnt_rpc']))
                self.assertTrue(a.motor.status_aux['cmd_cnt_exec'] - stat_last['cmd_cnt_exec'] == 0)
                self.assertTrue(a.motor.status_aux['cmd_rpc_overflow'] - stat_last['cmd_rpc_overflow'] == 0)
                #self.assertTrue(a.motor.status_aux['sync_irq_overflow'] - stat_last['sync_irq_overflow'] == 0)
                #self.assertTrue(a.motor.status_aux['sync_irq_cnt'] - stat_last['sync_irq_cnt'] == 0)
                self.assertTrue(p.status['motor_sync_cnt'] - stat_last['motor_sync_cnt'] == 0)
                #if not a.motor.status['waiting_on_sync']:

                self.assertTrue(a.motor.status['waiting_on_sync'])


            p.trigger_motor_sync()
            time.sleep(.01) #Give time for stepper to respond to sync pulse

            a.pull_status()
            p.pull_status()
            a.motor.pull_status_aux()
            # print('%%%%%%%%%%%%%%%%')
            # print('pimu', p.status, p.status_aux)
            # print('arm', a.motor.status, a.motor.status_aux)



            if stat_last is not None: #Check that sync made it through
                #print('3',a.motor.status_aux, 'motor_sync_cnt',p.status['motor_sync_cnt'],'waiting_on_sync',a.motor.status['waiting_on_sync'],'queues', p.status['motor_sync_queues'])
                self.assertTrue(self.is_incremented_uint16_t(stat_last['cmd_cnt_rpc'], a.motor.status_aux['cmd_cnt_rpc']))
                self.assertTrue(self.is_incremented_uint16_t(stat_last['cmd_cnt_exec'], a.motor.status_aux['cmd_cnt_exec']))
                self.assertTrue(a.motor.status_aux['cmd_rpc_overflow'] - stat_last['cmd_rpc_overflow'] == 0)
                #self.assertTrue(a.motor.status_aux['sync_irq_overflow'] - stat_last['sync_irq_overflow'] == 0)
                #self.assertTrue(a.motor.status_aux['sync_irq_cnt'] - stat_last['sync_irq_cnt'] == 1)
                self.assertTrue(self.is_incremented_uint16_t(stat_last['motor_sync_cnt'], p.status['motor_sync_cnt']))
                self.assertFalse(a.motor.status['waiting_on_sync'])
            stat_last = a.motor.status_aux.copy()
            stat_last['motor_sync_cnt'] = p.status['motor_sync_cnt']
        a.stop()
        p.stop()
