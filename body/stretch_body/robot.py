from __future__ import print_function
import threading
import time
import signal
import importlib

from stretch_body.device import Device
import stretch_body.base as base
import stretch_body.arm as arm
import stretch_body.lift as lift
import stretch_body.pimu as pimu
import stretch_body.head as head
import stretch_body.wacc as wacc
import stretch_body.hello_utils as hello_utils

from serial import SerialException

from stretch_body.robot_monitor import RobotMonitor
from stretch_body.robot_collision import RobotCollision


# #############################################################
class DXLStatusThread(threading.Thread):
    """
    This thread polls the status data of the Dynamixel devices
    at 15Hz
    """
    def __init__(self, robot, target_rate_hz=15.0):
        threading.Thread.__init__(self)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.stats = hello_utils.LoopStats(loop_name='DXLStatusThread',target_loop_rate=self.robot_update_rate_hz)
        self.shutdown_flag = threading.Event()
        self.first_status=False

    def run(self):
        while not self.shutdown_flag.is_set():
            self.stats.mark_loop_start()
            self.robot._pull_status_dynamixel()
            self.robot._update_trajectory_dynamixel()
            self.first_status=True
            self.stats.mark_loop_end()
            if not self.shutdown_flag.is_set():
                time.sleep(self.stats.get_loop_sleep_time())
        self.robot.logger.debug('Shutting down DXLStatusThread')


class NonDXLStatusThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry, Monitor, and Collision functions
    """
    def __init__(self, robot, target_rate_hz=25.0):
        threading.Thread.__init__(self)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.monitor_downrate_int = int(robot.params['rates']['NonDXLStatusThread_monitor_downrate_int'])  # Step the monitor at every Nth iteration
        self.collision_downrate_int = int(robot.params['rates']['NonDXLStatusThread_collision_downrate_int']) # Step the monitor at every Nth iteration
        self.sentry_downrate_int = int(robot.params['rates']['NonDXLStatusThread_sentry_downrate_int']) # Step the sentry at every Nth iteration
        self.trajectory_downrate_int = int(robot.params['rates']['NonDXLStatusThread_trajectory_downrate_int'])  # Update hardware with waypoint trajectory segments at every Nth iteration
        if self.robot.params['use_monitor']:
            self.robot.monitor.startup()
        if self.robot.params['use_collision_manager']:
            self.robot.collision.startup()
        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='NonDXLStatusThread',target_loop_rate=self.robot_update_rate_hz)
        self.titr=0
        self.first_status = False

    def run(self):
        while not self.shutdown_flag.is_set():
            self.stats.mark_loop_start()
            self.robot._pull_status_non_dynamixel()
            self.first_status = True

            if self.robot.params['use_monitor']:
                if (self.titr % self.monitor_downrate_int) == 0:
                    self.robot.monitor.step()

            if self.robot.params['use_collision_manager'] and self.robot.is_calibrated():
                    self.robot.collision.step()

            if self.robot.params['use_sentry']:
                if (self.titr % self.sentry_downrate_int) == 0:
                    self.robot._step_sentry()

            if (self.titr % self.trajectory_downrate_int) == 0:
                self.robot._update_trajectory_non_dynamixel()

            self.stats.mark_loop_end()
            if not self.shutdown_flag.is_set():
                time.sleep(self.stats.get_loop_sleep_time())
        self.robot.logger.debug('Shutting down NonDXLStatusThread')


class Robot(Device):
    """
    API to the Stretch Robot
    """
    def __init__(self):
        Device.__init__(self, 'robot')
        self.monitor = RobotMonitor(self)
        self.collision = RobotCollision(self)
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
        self.non_dxl_thread=None
        self.dxl_thread=None

    # ###########  Device Methods #############

    def startup(self):
        """
        To be called once after class instantiation.
        Prepares devices for communications and motion

        Returns
        -------
        bool
            true if startup of robot succeeded
        """
        self.logger.debug('Starting up Robot {0} of batch {1}'.format(self.params['serial_no'], self.params['batch_name']))
        success = True
        for k in self.devices.keys():
            if self.devices[k] is not None:
                if not self.devices[k].startup(threaded=False):
                    success = False

        # Register the signal handlers
        signal.signal(signal.SIGTERM, hello_utils.thread_service_shutdown)
        signal.signal(signal.SIGINT, hello_utils.thread_service_shutdown)

        self.non_dxl_thread = NonDXLStatusThread(self,target_rate_hz=self.params['rates']['NonDXLStatusThread_Hz'])
        self.non_dxl_thread.setDaemon(True)
        self.non_dxl_thread.start()

        self.dxl_thread = DXLStatusThread(self,target_rate_hz=self.params['rates']['DXLStatusThread_Hz'])
        self.dxl_thread.setDaemon(True)
        self.dxl_thread.start()

        # Wait for status reading threads to start reading data
        ts=time.time()
        while not self.non_dxl_thread.first_status and not self.dxl_thread.first_status and time.time()-ts<3.0:
           time.sleep(0.1)

        return success

    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        self.logger.debug('---- Shutting down robot ----')
        if self.non_dxl_thread is not None:
            self.non_dxl_thread.shutdown_flag.set()
            self.non_dxl_thread.join(1)
        if self.dxl_thread is not None:
            self.dxl_thread.shutdown_flag.set()
            self.dxl_thread.join(1)
        for k in self.devices.keys():
            if self.devices[k] is not None:
                self.logger.debug('Shutting down %s'%k)
                self.devices[k].stop()
        self.logger.debug('---- Shutdown complete ----')

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
        hello_utils.pretty_print_dict('Status',s)


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

    # ######### Waypoint Trajectory Interface ##############################


    def is_trajectory_active(self):
        return self.arm.is_trajectory_active() or self.lift.is_trajectory_active() or \
               self.base.is_trajectory_active() or self.end_of_arm.is_trajectory_active() or \
               self.head.is_trajectory_active()


    def follow_trajectory(self):
        success = True
        success = success and self.arm.follow_trajectory(move_to_start_point=False)
        success = success and self.lift.follow_trajectory(move_to_start_point=False)
        success = success and self.base.follow_trajectory()
        self.pimu.trigger_motor_sync()
        success = success and self.end_of_arm.follow_trajectory(move_to_start_point=False)
        success = success and self.head.follow_trajectory(move_to_start_point=False)
        return success

    def stop_trajectory(self):
        self.arm.stop_trajectory()
        self.lift.stop_trajectory()
        self.base.stop_trajectory()
        self.end_of_arm.stop_trajectory()
        self.head.stop_trajectory()

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

    def get_stow_pos(self,joint):
        """
        Return the stow position of a joint.
        Allow the end_of_arm to override the defaults in order to accomodate stowing different tools
        """
        if 'stow' in self.end_of_arm.params:
            if joint in self.end_of_arm.params['stow']:
                return self.end_of_arm.params['stow'][joint]
        return self.params['stow'][joint]

    def stow(self):
        """
        Cause the robot to move to its stow position
        Blocking.
        """
        self.head.move_to('head_pan', self.get_stow_pos('head_pan'))
        self.head.move_to('head_tilt',self.get_stow_pos('head_pan'))

        lift_stowed=False
        pos_lift = self.get_stow_pos('lift')
        if self.lift.status['pos']<=pos_lift: #Needs to come up before bring in arm
            print('--------- Stowing Lift ----')
            self.lift.move_to(pos_lift)
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 4.0:
                time.sleep(0.1)
            lift_stowed=True

        #Bring in arm before bring down
        print('--------- Stowing Arm ----')
        self.arm.move_to(self.get_stow_pos('arm'))
        self.push_command()
        time.sleep(0.25)
        ts = time.time()
        while not self.arm.motor.status['near_pos_setpoint'] and time.time() - ts < 6.0:
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
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 12.0:
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

    def _pull_status_dynamixel(self):
        try:
            self.end_of_arm.pull_status()
            self.head.pull_status()
        except SerialException:
            self.logger.warning('Serial Exception on Robot Step_Dynamixel')

    def _update_trajectory_dynamixel(self):
        try:
            self.end_of_arm.update_trajectory()
            self.head.update_trajectory()
        except SerialException:
            self.logger.warning('Serial Exception on Robot._update_trajectory_dynamixel()')

    def _pull_status_non_dynamixel(self):
        self.wacc.pull_status()
        self.base.pull_status()
        self.lift.pull_status()
        self.arm.pull_status()
        self.pimu.pull_status()

    def _update_trajectory_non_dynamixel(self):
        self.arm.update_trajectory()
        self.lift.update_trajectory()
        self.base.update_trajectory()

    def _step_sentry(self):
        self.head.step_sentry(self)
        self.base.step_sentry(self)
        self.arm.step_sentry(self)
        self.lift.step_sentry(self)
        self.end_of_arm.step_sentry(self)
