# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
#stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.arm
import stretch_body.pimu
import stretch_body.robot
import time


class TestMotorSyncRate(unittest.TestCase):

    def xtest_pimu_sync_100Hz(self):
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


    def is_incremented_uint16_t(self,old_val,new_val):
        """
        Check that new value is old_val+1 (modulo 16 bit rollover)
        """
        if old_val == 65535:
            return new_val==0
        return old_val+1==new_val
    def xtest_arm_sync_max_rate(self):
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

    def test_robot_sync_max_rate_no_threads(self):
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