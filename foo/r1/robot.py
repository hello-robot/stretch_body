from __future__ import print_function
import threading
import time
import signal
import importlib
import asyncio
import traceback

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
from stretch_body.robot_trace import RobotTrace
from stretch_body.robot_collision import RobotCollisionMgmt

# #############################################################
class DXLHeadStatusThread(threading.Thread):
    """
    This thread polls the status data of the Dynamixel devices
    at 15Hz
    """
    def __init__(self, robot, target_rate_hz=15.0):
        threading.Thread.__init__(self, name = self.__class__.__name__)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.stats = hello_utils.LoopStats(loop_name='DXLHeadStatusThread',target_loop_rate=self.robot_update_rate_hz)
        self.shutdown_flag = threading.Event()
        self.running=False

    def step(self):
        self.stats.mark_loop_start()
        self.robot._pull_status_head_dynamixel()
        self.robot._update_trajectory_head_dynamixel()
        self.stats.mark_loop_end()

    def run(self):
        self.running=True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
        self.robot.logger.debug('Shutting down DXLHeadStatusThread')

class DXLEndOfArmStatusThread(threading.Thread):
    """
    This thread polls the status data of the Dynamixel devices
    at 15Hz
    """
    def __init__(self, robot, target_rate_hz=15.0):
        threading.Thread.__init__(self, name = self.__class__.__name__)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.stats = hello_utils.LoopStats(loop_name='DXLEndOfArmStatusThread',target_loop_rate=self.robot_update_rate_hz)
        self.shutdown_flag = threading.Event()
        self.running = False
    def step(self):
        self.stats.mark_loop_start()
        self.robot._pull_status_end_of_arm_dynamixel()
        self.robot._update_trajectory_end_of_arm_dynamixel()
        self.stats.mark_loop_end()
    def run(self):
        self.running=True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
        self.robot.logger.debug('Shutting down DXLEndOfArmStatusThread')

class NonDXLStatusThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry, Monitor, and Collision functions
    """
    def __init__(self, robot, target_rate_hz=25.0):
        threading.Thread.__init__(self, name = self.__class__.__name__)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='NonDXLStatusThread',target_loop_rate=self.robot_update_rate_hz)
        self.titr=0
        self.first_status = False
        self.loop = asyncio.new_event_loop()
        self.running = False

    def step(self):
        asyncio.set_event_loop(self.loop)
        self.stats.mark_loop_start()
        if self.robot.params['use_asyncio']:
            asyncio.get_event_loop().run_until_complete(asyncio.gather(
                self.robot.wacc.pull_status_async(),
                self.robot.base.pull_status_async(),
                self.robot.lift.pull_status_async(),
                self.robot.arm.pull_status_async(),
                self.robot.pimu.pull_status_async()))
        else:
            self.robot._pull_status_non_dynamixel()
        self.stats.mark_loop_end()
    def run(self):
        self.running=True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
            self.first_status = True
        self.stop()
        self.robot.logger.debug('Shutting down NonDXLStatusThread')
    
    def stop(self):
        self.loop.stop()

class SystemMonitorThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry, Monitor, and Collision functions
    """
    def __init__(self, robot, target_rate_hz=25.0):
        threading.Thread.__init__(self, name = self.__class__.__name__)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.monitor_downrate_int = int(robot.params['rates']['SystemMonitorThread_monitor_downrate_int'])  # Step the monitor at every Nth iteration
        self.trace_downrate_int = int(robot.params['rates']['SystemMonitorThread_trace_downrate_int'])  # Step the trace at every Nth iteration
        self.sentry_downrate_int = int(robot.params['rates']['SystemMonitorThread_sentry_downrate_int']) # Step the sentry at every Nth iteration
        #self.collision_downrate_int = int(robot.params['rates']['SystemMonitorThread_collision_downrate_int'])  # Step the monitor at every Nth iteration
        self.trajectory_downrate_int = int(robot.params['rates']['SystemMonitorThread_nondxl_trajectory_downrate_int'])  # Update hardware with waypoint trajectory segments at every Nth iteration

        if self.robot.params['use_monitor']:
            self.robot.monitor.startup()
        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='SystemMonitorThread',target_loop_rate=self.robot_update_rate_hz)
        self.titr=0
        self.running=False

    def step(self):
        self.titr = self.titr + 1
        self.stats.mark_loop_start()
        if self.robot.params['use_monitor']:
            if (self.titr % self.monitor_downrate_int) == 0:
                self.robot.monitor.step()
        if self.robot.params['use_trace']:
            if (self.titr % self.trace_downrate_int) == 0:
                self.robot.trace.step()
        if self.robot.params['use_sentry']:
            if (self.titr % self.sentry_downrate_int) == 0:
                self.robot._step_sentry()
        # if self.robot.is_homed():
        #     self.robot.collision.step()
        if (self.titr % self.trajectory_downrate_int) == 0:
            self.robot._update_trajectory_non_dynamixel()
        self.stats.mark_loop_end()

    def run(self):
        self.running=True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
        self.robot.logger.debug('Shutting down SystemMonitorThread')

class CollisionMonitorThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry, Monitor, and Collision functions
    """
    def __init__(self, robot, target_rate_hz=25.0):
        threading.Thread.__init__(self, name = self.__class__.__name__)
        self.robot=robot
        self.robot_update_rate_hz = target_rate_hz
        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='CollisionMonitorThread',target_loop_rate=self.robot_update_rate_hz)
        self.titr=0
        self.running=False

    def step(self):
        self.titr = self.titr + 1
        self.stats.mark_loop_start()
        if self.robot.is_homed():
            self.robot.collision.step()
        self.stats.mark_loop_end()

    def run(self):
        self.running=True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
        self.robot.logger.debug('Shutting down CollisionMonitorThread')

class Robot(Device):
    """
    API to the Stretch Robot
    """
    def __init__(self):
        Device.__init__(self, 'robot')
        self.monitor = RobotMonitor(self)
        self.trace = RobotTrace(self)
        self.collision = RobotCollisionMgmt(self)
        self.dirty_push_command = False
        self.lock = threading.RLock() #Prevent status thread from triggering motor sync prematurely
        self.status = {'pimu': {}, 'base': {}, 'lift': {}, 'arm': {}, 'head': {}, 'wacc': {}, 'end_of_arm': {}}
        self.async_event_loop = None

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

        self.non_dxl_thread = None
        self.dxl_end_of_arm_thread = None
        self.sys_thread = None
        self.dxl_head_thread = None
        self.event_loop_thread = None
        self.collision_mgmt_thread = None

        self.eoa_name= self.params['tool']
        module_name = self.robot_params[self.eoa_name]['py_module_name']
        class_name = self.robot_params[self.eoa_name]['py_class_name']
        self.end_of_arm = getattr(importlib.import_module(module_name), class_name)()
        self.status['end_of_arm'] = self.end_of_arm.status
        self.devices={ 'pimu':self.pimu, 'base':self.base, 'lift':self.lift, 'arm': self.arm, 'head': self.head, 'wacc':self.wacc, 'end_of_arm':self.end_of_arm}

        self.GLOBAL_EXCEPTIONS_LIST = []
        threading.excepthook = self.custom_excepthook

        
    def custom_excepthook(self, args):
        thread_name = args.thread.name
        Exec = {}
        Exec[thread_name] = {
            'thread': args.thread,
            'exception': {
                'type': args.exc_type,
                'value': args.exc_value,
                'traceback': args.exc_traceback
            }
        }

        # Filter RuntimeError
        if Exec[thread_name]['exception']['type'] == RuntimeError:
            pass
        else:
            print(f"Caught Exception in Thread: {thread_name}")
            traceback.print_exception(args.exc_value)

        self.logger.debug(f"Caught Exception in Thread: {thread_name}")
        self.logger.debug(traceback.format_exception(args.exc_value))

        self.GLOBAL_EXCEPTIONS_LIST.append(Exec[thread_name])

    # ###########  Device Methods #############

    def startup(self,start_non_dxl_thread=True,start_dxl_thread=True,start_sys_mon_thread=True):
        """
        To be called once after class instantiation.
        Prepares devices for communications and motion

        Returns
        -------
        bool
            true if startup of robot succeeded
        """
        did_acquire, self._file_lock = hello_utils.acquire_body_filelock()
        if not did_acquire:
            print('Another process is already using Stretch. Try running "stretch_free_robot_process.py"')
            return False

        self.logger.debug('Starting up Robot {0} of batch {1}'.format(self.params['serial_no'], self.params['batch_name']))
        success = True
        for k in self.devices:
            if self.devices[k] is not None:
                if not self.devices[k].startup(threaded=False):
                    success = False

        if (self.arm.motor.transport.version==0 \
                or self.lift.motor.transport.version==0 \
                or self.base.right_wheel.transport.version == 0 \
                or self.base.left_wheel.transport.version == 0 \
                or self.wacc.transport.version==0 \
                or self.pimu.transport.version==0) and self.params['use_asyncio'] :
            self.logger.warning('Not able to use asyncio for transport communications. Defaulting to sync.')
            self.params['use_asyncio']=0
        else:
            self.start_event_loop()

        #Always startup to load URDFs now and not while thread is running
        self.collision.startup()
        if not self.params['use_collision_manager']: #Turn it off here but allow user to enable it via SW later
            self.disable_collision_mgmt()
        else:
            self.enable_collision_mgmt()

        # Register the signal handlers
        signal.signal(signal.SIGTERM, hello_utils.thread_service_shutdown)
        signal.signal(signal.SIGINT, hello_utils.thread_service_shutdown)

        self.non_dxl_thread = NonDXLStatusThread(self, target_rate_hz=self.params['rates']['NonDXLStatusThread_Hz'])
        self.dxl_end_of_arm_thread = DXLEndOfArmStatusThread(self,target_rate_hz=self.params['rates']['DXLStatusThread_Hz'])
        self.sys_thread = SystemMonitorThread(self, target_rate_hz=self.params['rates']['SystemMonitorThread_Hz'])
        self.dxl_head_thread = DXLHeadStatusThread(self, target_rate_hz=self.params['rates']['DXLStatusThread_Hz'])
        self.collision_mgmt_thread = CollisionMonitorThread(self, target_rate_hz=100)

        if start_non_dxl_thread:
            self.non_dxl_thread.daemon = True
            self.non_dxl_thread.start()
            ts = time.time()
            while not self.non_dxl_thread.first_status and time.time() - ts < 3.0:
                time.sleep(0.01)

        if start_dxl_thread:
            self.dxl_head_thread.daemon = True
            self.dxl_head_thread.start()
            self.dxl_end_of_arm_thread.daemon = True
            self.dxl_end_of_arm_thread.start()

        if start_sys_mon_thread:
            self.sys_thread.daemon = True
            self.sys_thread.start()

        if start_sys_mon_thread and self.collision_mgmt_thread:
            self.collision_mgmt_thread.daemon = True
            self.collision_mgmt_thread.start()

        return success

    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        self.logger.debug('---- Shutting down robot ----')
        self._file_lock.release()
        if self.non_dxl_thread:
            if self.non_dxl_thread.running:
                self.non_dxl_thread.shutdown_flag.set()
                self.non_dxl_thread.join(1)
        if self.dxl_head_thread:
            if self.dxl_head_thread.running:
                self.dxl_head_thread.shutdown_flag.set()
                self.dxl_head_thread.join(1)
        if self.dxl_end_of_arm_thread:
            if self.dxl_end_of_arm_thread.running:
                self.dxl_end_of_arm_thread.shutdown_flag.set()
                self.dxl_end_of_arm_thread.join(1)
        if self.sys_thread:
            if self.sys_thread.running:
                self.sys_thread.shutdown_flag.set()
                self.sys_thread.join(1)
        if self.collision_mgmt_thread:
            if self.collision_mgmt_thread.running:
                self.collision_mgmt_thread.shutdown_flag.set()
                self.collision_mgmt_thread.join(1)
                self.collision.stop()
        for k in self.devices:
            if self.devices[k] is not None:
                self.logger.debug('Shutting down %s'%k)
                self.devices[k].stop()
        self.stop_event_loop()
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
    
    async def push_command_coro(self):
        tasks = [self.base.left_wheel.push_command_async(),
                 self.base.right_wheel.push_command_async(),
                 self.arm.push_command_async(),
                 self.lift.push_command_async(),
                 self.pimu.push_command_async(),
                 self.wacc.push_command_async()]
        results = await asyncio.gather(*tasks)
        return results
        
    def push_command(self):
        """
        Cause all queued up RPC commands to be sent down to Devices
        """
        
        with self.lock:
            ready = self.pimu.is_ready_for_sync()
            if self.params['use_asyncio'] and self.async_event_loop is not None:
                future = asyncio.run_coroutine_threadsafe(self.push_command_coro(),self.async_event_loop)
                future.result()
            else:
                self.base.push_command()
                self.arm.push_command()
                self.lift.push_command()
                self.pimu.push_command()
                self.wacc.push_command()

            # Check if need to do a motor sync by looking at if there's been a pimu sync signal sent
            # since the last stepper.set_command for each joint
            sync_required = (self.pimu.ts_last_motor_sync is not None) and (
                    self.arm.motor.is_sync_required(self.pimu.ts_last_motor_sync)
                    or self.lift.motor.is_sync_required(self.pimu.ts_last_motor_sync)
                    or self.base.right_wheel.is_sync_required(self.pimu.ts_last_motor_sync)
                        or self.base.left_wheel.is_sync_required(self.pimu.ts_last_motor_sync))

            if (self.pimu.ts_last_motor_sync is None or ( ready and sync_required)):
                self.pimu.trigger_motor_sync()

    def enable_collision_mgmt(self):
        self.collision.enable()

    def disable_collision_mgmt(self):
        self.collision.disable()

    def wait_command(self, timeout=15.0, use_motion_generator=True):
        """Pause program execution until all motion complete.

        Queuing up motion and pushing it to the hardware with
        push_command() is designed to be asynchronous, enabling
        reactive control of the robot. However, you might want
        sychronous control, where each command's motion is completed
        entirely before the program moves on to the next command.
        This is where you would use wait_command()

        Parameters
        ----------
        timeout : float
            How long to wait for motion to complete. Must be > 0.1 sec.

        Returns
        -------
        bool
            True if motion completed, False if timed out before motion completed
        """
        time.sleep(0.1)
        timeout = max(0.0, timeout - 0.1)
        done = []
        def check_wait(wait_method):
            done.append(wait_method(timeout, use_motion_generator))
        start = time.time()
        threads = []
        threads.append(threading.Thread(target=check_wait, args=(self.base.wait_while_is_moving,)))
        threads.append(threading.Thread(target=check_wait, args=(self.arm.wait_while_is_moving,)))
        threads.append(threading.Thread(target=check_wait, args=(self.lift.wait_while_is_moving,)))
        threads.append(threading.Thread(target=check_wait, args=(self.head.wait_until_at_setpoint,)))
        threads.append(threading.Thread(target=check_wait, args=(self.end_of_arm.wait_until_at_setpoint,)))
        [thread.start() for thread in threads]
        [thread.join() for thread in threads]
        return all(done)

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

        # Check if need to do a motor sync by looking at if there's been a pimu sync signal sent
        # since the last stepper.set_command for each joint
        sync_required = (self.pimu.ts_last_motor_sync is not None) and (
                self.arm.motor.ts_last_syncd_motion > self.pimu.ts_last_motor_sync
                or self.lift.motor.ts_last_syncd_motion > self.pimu.ts_last_motor_sync
                or self.base.right_wheel.ts_last_syncd_motion > self.pimu.ts_last_motor_sync
                or self.base.left_wheel.ts_last_syncd_motion > self.pimu.ts_last_motor_sync)
        if self.pimu.ts_last_motor_sync is None or sync_required:
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
        DEPRECATED: replaced by Robot.is_homed()
        """
        self.logger.warn('is_calibrated() has been replaced by is_homed()')
        return self.is_homed()

    def is_homed(self):
        """
        Returns true if homing has been run all joints that require it
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
        self.disable_collision_mgmt()
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
        
        # Run pre stow specific to each end of arm
        self.end_of_arm.pre_stow(self)

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
        self.enable_collision_mgmt()

    def home(self):
        """
        Cause the robot to home its joints by moving to hardstops
        Blocking.
        """
        self.disable_collision_mgmt()
        if self.head is not None:
            print('--------- Homing Head ----')
            self.head.home()

        # Wrist pitch should be angled somewhere in between the lift leaving
        # the base of its range and before it reaches the top of its range.
        def _angle_pitch(angle):
            time.sleep(1) # wait 1 sec to leave bottom of lift's range
            if 'wrist_pitch' in self.end_of_arm.joints:
                self.end_of_arm.move_to('wrist_pitch', angle)
        
        # If gripper present
        if 'stretch_gripper' in self.end_of_arm.joints and 'wrist_pitch' in self.end_of_arm.joints:
            threading.Thread(target=_angle_pitch,args=(self.end_of_arm.params['stow']['wrist_pitch'],)).start()

        # No gripper but tablet or accessory present in dw3 wrist, wrist pitch should face downward
        elif 'wrist_pitch' in self.end_of_arm.joints:
            threading.Thread(target=_angle_pitch,args=(-1.57,)).start()

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
        
        self.enable_collision_mgmt()
        #Let user know it is done
        self.pimu.trigger_beep()
        self.push_command()
    # ################ Helpers #################################

    def _pull_status_head_dynamixel(self):
        try:
            self.head.pull_status()
        except SerialException:
            self.logger.warning('Serial Exception on Robot._pull_status_head_dynamixel')

    def _update_trajectory_head_dynamixel(self):
        try:
            self.head.update_trajectory()
        except SerialException:
            self.logger.warning('Serial Exception on Robot._update_trajectory_head_dynamixel()')


    def _pull_status_end_of_arm_dynamixel(self):
        try:
            self.end_of_arm.pull_status()
        except SerialException:
            self.logger.warning('Serial Exception on Robot._pull_status_end_of_arm_dynamixel')

    def _update_trajectory_end_of_arm_dynamixel(self):
        try:
            self.end_of_arm.update_trajectory()
        except SerialException:
            self.logger.warning('Serial Exception on Robot._update_trajectory_end_of_arm_dynamixel()')

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
    
    def start_event_loop(self):
        self.async_event_loop = asyncio.new_event_loop()
        self.event_loop_thread = threading.Thread(target=self._event_loop, name='AsyncEvenLoopThread')
        self.event_loop_thread.daemon = True
        self.event_loop_thread.start()
        
    def _event_loop(self):
            asyncio.set_event_loop(self.async_event_loop)
            self.async_event_loop.run_forever()

    def stop_event_loop(self):
        try:
            if self.event_loop_thread:
                asyncio.run_coroutine_threadsafe(self.async_event_loop.stop())
                self.event_loop_thread.join()
        except TypeError as e:
            pass