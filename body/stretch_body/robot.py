from __future__ import print_function
import threading
import time
import signal
import logging
import os
import importlib

from stretch_body.device import Device
import stretch_body.base as base
import stretch_body.arm as arm
import stretch_body.lift as lift
import stretch_body.pimu as pimu
import stretch_body.head as head
import stretch_body.wacc as wacc
import stretch_body.end_of_arm as end_of_arm
import stretch_body.hello_utils as hello_utils

from serial import SerialException

from stretch_body.robot_monitor import RobotMonitor


# #############################################################
class RobotDynamixelThread(threading.Thread):
    """
    This thread polls the status data of the Dynamixel devices
    at 15Hz
    """
    def __init__(self,robot):
        threading.Thread.__init__(self)
        self.robot=robot
        self.robot_update_rate_hz = 15.0  #Hz
        self.timer_stats = hello_utils.TimerStats()
        self.shutdown_flag = threading.Event()
        self.first_status=False

    def run(self):
        while not self.shutdown_flag.is_set():
            ts = time.time()
            self.robot._pull_status_dynamixel()
            self.first_status=True
            te = time.time()
            tsleep = max(0.001, (1 / self.robot_update_rate_hz) - (te - ts))
            if not self.shutdown_flag.is_set():
                time.sleep(tsleep)
        print('Shutting down RobotDynamixelThread')



class RobotThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry and Monitor functions
    """
    def __init__(self,robot):
        threading.Thread.__init__(self)
        self.robot=robot

        self.robot_update_rate_hz = 25.0  #Hz
        self.monitor_downrate_int = 5  # Step the monitor at every Nth iteration
        self.sentry_downrate_int = 2  # Step the sentry at every Nth iteration
        if self.robot.params['use_monitor']:
            self.robot.monitor.startup()
        self.shutdown_flag = threading.Event()
        self.timer_stats = hello_utils.TimerStats()
        self.titr=0
        self.first_status = False

    def run(self):
        while not self.shutdown_flag.is_set():
            ts = time.time()
            self.robot._pull_status_non_dynamixel()
            self.first_status = True
            if self.robot.params['use_monitor']:
                if (self.titr % self.monitor_downrate_int) == 0:
                    self.robot.monitor.step()

            if self.robot.params['use_sentry']:
                if (self.titr % self.sentry_downrate_int) == 0:
                    self.robot._step_sentry()

            self.titr=self.titr+1
            te = time.time()
            tsleep = max(0.001, (1 / self.robot_update_rate_hz) - (te - ts))
            if not self.shutdown_flag.is_set():
                time.sleep(tsleep)
        print('Shutting down RobotThread')
        print('Shutting down RobotThread')


class Robot(Device):
    """
    API to the Stretch RE1 Robot
    """
    def __init__(self):
        Device.__init__(self,'robot')
        self.monitor = RobotMonitor(self)
        self.dirty_push_command = False
        self.lock = threading.RLock() #Prevent status thread from triggering motor sync prematurely

        self.status = {'pimu': {}, 'base': {}, 'lift': {}, 'arm': {}, 'head': {}, 'wacc': {}, 'end_of_arm': {}}


        self.pimu=pimu.Pimu()
        self.status['pimu']=self.pimu.status

        self.base=base.Base()
        self.status['base']=self.base.status

        self.lift=lift.Lift()
        self.status['lift']=self.lift.status

        self.arm=arm.Arm()
        self.status['arm']=self.arm.status

        self.head=head.Head()
        self.status['head']=self.head.status

        if 'custom_wacc' in self.params:
            module_name = self.params['custom_wacc']['py_module_name']
            class_name = self.params['custom_wacc']['py_class_name']
            self.wacc=getattr(importlib.import_module(module_name), class_name)(self)
        else:
            self.wacc=wacc.Wacc()
        self.status['wacc']=self.wacc.status


        tool_name = self.params['tool']
        module_name = self.robot_params[tool_name]['py_module_name']
        class_name = self.robot_params[tool_name]['py_class_name']
        self.end_of_arm = getattr(importlib.import_module(module_name), class_name)()
        self.status['end_of_arm'] = self.end_of_arm.status

        self.devices={ 'pimu':self.pimu, 'base':self.base, 'lift':self.lift, 'arm': self.arm, 'head': self.head, 'wacc':self.wacc, 'end_of_arm':self.end_of_arm}
        self.rt=None
        self.dt=None


    # ###########  Device Methods #############

    def startup(self):
        """
        To be called once after class instantiation.
        Prepares devices for communications and motion
        """

        # Set up logging
        t = time.localtime()
        capture_date = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(
            2) + str(t.tm_min).zfill(2)
        self.log_filename = os.environ['HELLO_FLEET_PATH'] + '/log/' + self.params[
            'serial_no'] + '_monitor_' + capture_date + '.log'
        self.logger = logging.getLogger('robot')
        self.logger.setLevel(logging.DEBUG)
        fh = logging.FileHandler(self.log_filename)
        fh.setLevel(logging.DEBUG)
        formatter = logging.Formatter(
            '%(asctime)s - ' + hello_utils.get_fleet_id() + ' - %(name)s - %(levelname)s - %(message)s')
        fh.setFormatter(formatter)
        self.logger.addHandler(fh)
        if self.params['log_to_console']:
            ch = logging.StreamHandler()
            ch.setLevel(logging.DEBUG)
            ch.setFormatter(formatter)
            self.logger.addHandler(ch)

        self.logger.info('Starting up Robot')
        for k in self.devices.keys():
            if self.devices[k] is not None:
                if not self.devices[k].startup():
                    pass
                #    print('Startup failure on %s. Exiting.'%k)
                #    exit()


        # Register the signal handlers
        signal.signal(signal.SIGTERM, hello_utils.thread_service_shutdown)
        signal.signal(signal.SIGINT, hello_utils.thread_service_shutdown)
        self.rt = RobotThread(self)
        self.rt.setDaemon(True)
        self.rt.start()
        self.dt = RobotDynamixelThread(self)
        self.dt.setDaemon(True)
        self.dt.start()
        #Wait for threads to start reading data
        ts=time.time()
        while not self.rt.first_status and not self.dt.first_status and time.time()-ts<3.0:
           time.sleep(0.1)
        #if not self.rt.first_status  or not self.dt.first_status :
        #    self.logger.warning('Failed to startup up robot threads')

    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        print('Shutting down robot...')
        if self.rt is not None:
            self.rt.shutdown_flag.set()
            self.rt.join()
        if self.dt is not None:
            self.dt.shutdown_flag.set()
            self.dt.join()
        for k in self.devices.keys():
            if self.devices[k] is not None:
                print('Shutting down',k)
                self.devices[k].stop()
        print('---- Shutdown complete ----')

    def get_status(self):
        """
        Thread safe and atomic read of current Robot status data
        Returns as a dict.
        """
        with self.lock:
            return self.status.copy()

    def pretty_print(self):
        s=self.get_status()
        print('##################### HELLO ROBOT ##################### ')
        print('Time',time.time())
        print('Serial No',self.params['serial_no'])
        print('Batch', self.params['batch_name'])
        self._pretty_print_dict('Status',s)


    def push_command(self):
        """
        Cause all queued up RPC commands to be sent down to Devices
        """
        with self.lock:
            self.base.push_command()
            self.arm.push_command()
            self.lift.push_command()
            self.pimu.push_command()
            self.wacc.push_command()
            self.pimu.trigger_motor_sync()

# ##################Home and Stow #######################################

    def is_calibrated(self):
        """
        Returns true if homing-calibration has been run all joints that require it
        """
        ready = self.lift.motor.status['pos_calibrated']
        ready = ready and self.arm.motor.status['pos_calibrated']
        for j in self.end_of_arm.joints:
            req = self.end_of_arm.motors[j].params['req_calibration'] and not self.end_of_arm.motors[j].is_calibrated
            ready = ready and not req
        return ready

    def stow(self):
        """
        Cause the robot to move to its stow position
        Blocking.
        """
        self.head.move_to('head_pan', self.params['stow']['head_pan'])
        self.head.move_to('head_tilt',self.params['stow']['head_tilt'])

        lift_stowed=False
        pos_lift = self.params['stow']['lift']
        if 'stow' in self.end_of_arm.params:  # Allow tool defined stow position to take precedence
            if 'lift' in self.end_of_arm.params['stow']:
                pos_lift = self.end_of_arm.params['stow']['lift']
        if self.lift.status['pos']<=self.params['stow']['lift']: #Needs to come up before bring in arm
            print('--------- Stowing Lift ----')
            self.lift.move_to(pos_lift)
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 3.0:
                time.sleep(0.1)
            lift_stowed=True

        #Bring in arm before bring down
        print('--------- Stowing Arm ----')
        pos_arm = self.params['stow']['arm']
        if 'stow' in self.end_of_arm.params:  # Allow tool defined stow position to take precedence
            if 'arm' in self.end_of_arm.params['stow']:
                pos_arm = self.end_of_arm.params['stow']['arm']

        self.arm.move_to(pos_arm)
        self.push_command()
        time.sleep(0.25)
        ts = time.time()
        while not self.arm.motor.status['near_pos_setpoint'] and time.time() - ts < 3.0:
            time.sleep(0.1)

        self.end_of_arm.stow()
        time.sleep(0.25)


        #Now bring lift down
        if not lift_stowed:
            print('--------- Stowing Lift ----')
            self.lift.move_to(pos_lift)
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 10.0:
                time.sleep(0.1)

        #Make sure wrist yaw is done before exiting
        while self.end_of_arm.motors['wrist_yaw'].motor.is_moving():
            time.sleep(0.1)

    def home(self):
        """
        Cause the robot to home its joints by moving to hardstops
        Blocking.
        """
        if self.head is not None:
            print('--------- Homing Head ----')
            self.head.home()

        # Home the lift
        if self.lift is not None:
            print('--------- Homing Lift ----')
            self.lift.home()

        # Home the arm
        if self.arm is not None:
            print('--------- Homing Arm ----')
            self.arm.home()

        # Home the end-of-arm
        if self.end_of_arm is not None:
            self.end_of_arm.home()
        #Let user know it is done
        self.pimu.trigger_beep()
        self.push_command()
    # ################ Helpers #################################

    def _pretty_print_dict(self, t, d):
        print('--------', t, '--------')
        for k in d.keys():
            if type(d[k]) != dict:
                print(k, ' : ', d[k])
        for k in d.keys():
            if type(d[k]) == dict:
                self._pretty_print_dict(k, d[k])

    def _pull_status_dynamixel(self):
        try:
            self.end_of_arm.pull_status()
            self.head.pull_status()
        except SerialException:
            print('Serial Exception on Robot Step_Dynamixel')

    def _pull_status_non_dynamixel(self):
        self.wacc.pull_status()
        self.base.pull_status()
        self.lift.pull_status()
        self.arm.pull_status()
        self.pimu.pull_status()

    def _step_sentry(self):
        self.head.step_sentry(self)
        self.base.step_sentry(self)
        self.arm.step_sentry(self)
        self.lift.step_sentry(self)
        self.end_of_arm.step_sentry(self)