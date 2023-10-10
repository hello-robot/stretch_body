from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
import textwrap
import threading
import sys
import time
import array as arr



# ######################## STEPPER #################################

class StepperBase(Device):
    """
    API to the Stretch Stepper Board
    """
    RPC_SET_COMMAND = 1
    """@private"""
    RPC_REPLY_COMMAND = 2
    """@private"""
    RPC_GET_STATUS = 3
    """@private"""
    RPC_REPLY_STATUS = 4
    """@private"""
    RPC_SET_GAINS = 5
    """@private"""
    RPC_REPLY_GAINS = 6
    """@private"""
    RPC_LOAD_TEST_PUSH = 7
    """@private"""
    RPC_REPLY_LOAD_TEST_PUSH = 8
    """@private"""
    RPC_SET_TRIGGER = 9
    """@private"""
    RPC_REPLY_SET_TRIGGER = 10
    """@private"""
    RPC_SET_ENC_CALIB = 11
    """@private"""
    RPC_REPLY_ENC_CALIB = 12
    """@private"""
    RPC_READ_GAINS_FROM_FLASH = 13
    """@private"""
    RPC_REPLY_READ_GAINS_FROM_FLASH = 14
    """@private"""
    RPC_SET_MENU_ON = 15
    """@private"""
    RPC_REPLY_MENU_ON = 16
    """@private"""
    RPC_GET_STEPPER_BOARD_INFO = 17
    """@private"""
    RPC_REPLY_STEPPER_BOARD_INFO = 18
    """@private"""
    RPC_SET_MOTION_LIMITS = 19
    """@private"""
    RPC_REPLY_MOTION_LIMITS = 20
    """@private"""
    RPC_SET_NEXT_TRAJECTORY_SEG =21
    """@private"""
    RPC_REPLY_SET_NEXT_TRAJECTORY_SEG =22
    """@private"""
    RPC_START_NEW_TRAJECTORY =23
    """@private"""
    RPC_REPLY_START_NEW_TRAJECTORY =24
    """@private"""
    RPC_RESET_TRAJECTORY =25
    """@private"""
    RPC_REPLY_RESET_TRAJECTORY =26
    """@private"""
    RPC_READ_TRACE =27
    """@private"""
    RPC_REPLY_READ_TRACE =28
    """@private"""
    RPC_GET_STATUS_AUX = 29
    """@private"""
    RPC_REPLY_STATUS_AUX = 30
    """@private"""
    RPC_LOAD_TEST_PULL = 31
    """@private"""
    RPC_REPLY_LOAD_TEST_PULL = 32
    """@private"""

    MODE_SAFETY = 0
    """@private"""
    MODE_FREEWHEEL = 1
    """@private"""
    MODE_HOLD = 2
    """@private"""
    MODE_POS_PID = 3
    """@private"""
    MODE_VEL_PID = 4
    """@private"""
    MODE_POS_TRAJ = 5
    """@private"""
    MODE_VEL_TRAJ = 6
    """@private"""
    MODE_CURRENT = 7
    """@private"""
    MODE_POS_TRAJ_INCR = 8
    """@private"""
    MODE_POS_TRAJ_WAYPOINT=9
    """@private"""
    
    MODE_NAMES = {
        MODE_SAFETY: 'MODE_SAFETY',
        MODE_FREEWHEEL: 'MODE_FREEWHEEL',
        MODE_HOLD: 'MODE_HOLD',
        MODE_POS_PID: 'MODE_POS_PID',
        MODE_VEL_PID: 'MODE_VEL_PID',
        MODE_POS_TRAJ: 'MODE_POS_TRAJ',
        MODE_VEL_TRAJ: 'MODE_VEL_TRAJ',
        MODE_CURRENT: 'MODE_CURRENT',
        MODE_POS_TRAJ_INCR: 'MODE_POS_TRAJ_INCR',
        MODE_POS_TRAJ_WAYPOINT: 'MODE_POS_TRAJ_WAYPOINT',
    }
    """@private"""

    DIAG_POS_CALIBRATED = 1  # Has a pos zero RPC been received since powerup
    """@private"""
    DIAG_RUNSTOP_ON = 2  # Is controller in runstop mode
    """@private"""
    DIAG_NEAR_POS_SETPOINT = 4  # Is pos controller within gains.pAs_d of setpoint
    """@private"""
    DIAG_NEAR_VEL_SETPOINT = 8  # Is vel controller within gains.vAs_d of setpoint
    """@private"""
    DIAG_IS_MOVING = 16  # Is measured velocity greater than gains.vAs_d
    """@private"""
    DIAG_AT_CURRENT_LIMIT = 32  # Is controller current saturated
    """@private"""
    DIAG_IS_MG_ACCELERATING = 64  # Is controller motion generator acceleration non-zero
    """@private"""
    DIAG_IS_MG_MOVING = 128  # Is controller motion generator velocity non-zero
    """@private"""
    DIAG_CALIBRATION_RCVD = 256  # Is calibration table in flash
    """@private"""
    DIAG_IN_GUARDED_EVENT = 512  # Guarded event occurred during motion
    """@private"""
    DIAG_IN_SAFETY_EVENT = 1024  # Is it forced into safety mode
    """@private"""
    DIAG_WAITING_ON_SYNC = 2048  # Command received but no sync yet
    """@private"""
    DIAG_TRAJ_ACTIVE     = 4096     # Whether a waypoint trajectory is actively executing
    """@private"""
    DIAG_TRAJ_WAITING_ON_SYNC = 8192 # Currently waiting on a sync signal before starting trajectory
    """@private"""
    DIAG_IN_SYNC_MODE = 16384        # Currently running in sync mode
    """@private"""
    DIAG_IS_TRACE_ON = 32768   #Is trace recording
    """@private"""

    CONFIG_SAFETY_HOLD = 1  # Hold position in safety mode? Otherwise freewheel
    """@private"""
    CONFIG_ENABLE_RUNSTOP = 2  # Recognize runstop signal?
    """@private"""
    CONFIG_ENABLE_SYNC_MODE = 4  # Commands are synchronized from digital trigger
    """@private"""
    CONFIG_ENABLE_GUARDED_MODE = 8  # Stops on current threshold
    """@private"""
    CONFIG_FLIP_ENCODER_POLARITY = 16
    """@private"""
    CONFIG_FLIP_EFFORT_POLARITY = 32
    """@private"""
    CONFIG_ENABLE_VEL_WATCHDOG = 64 #Timeout velocity commands
    """@private"""

    TRIGGER_MARK_POS = 1
    """@private"""
    TRIGGER_RESET_MOTION_GEN = 2
    """@private"""
    TRIGGER_BOARD_RESET = 4
    """@private"""
    TRIGGER_WRITE_GAINS_TO_FLASH = 8
    """@private"""
    TRIGGER_RESET_POS_CALIBRATED = 16
    """@private"""
    TRIGGER_POS_CALIBRATED = 32
    """@private"""
    TRIGGER_MARK_POS_ON_CONTACT=64
    """@private"""
    TRIGGER_ENABLE_TRACE=128
    """@private"""
    TRIGGER_DISABLE_TRACE=256
    """@private"""

    TRACE_TYPE_STATUS = 0
    """@private"""
    TRACE_TYPE_DEBUG = 1
    """@private"""
    TRACE_TYPE_PRINT = 2
    """@private"""


    def __init__(self, usb,name=None):
        if name is None:
            name=usb[5:] #Pull from usb device name
        Device.__init__(self, name=name)
        self.usb=usb
        self.transport = Transport(usb=self.usb, logger=self.logger)

        self._command = {'mode':0, 'x_des':0,'v_des':0,'a_des':0,'stiffness':1.0,'i_feedforward':0.0,'i_contact_pos':0,'i_contact_neg':0,'incr_trigger':0}
        self.status = {'mode': 0, 'effort_ticks': 0, 'effort_pct':0,'current':0,'pos': 0, 'vel': 0, 'err':0,'diag': 0,'timestamp': 0, 'debug':0,'guarded_event':0,
                       'transport': self.transport.status,'pos_calibrated':0,'runstop_on':0,'near_pos_setpoint':0,'near_vel_setpoint':0,
                       'is_moving':0,'is_moving_filtered':0,'at_current_limit':0,'is_mg_accelerating':0,'is_mg_moving':0,'calibration_rcvd': 0,'in_guarded_event':0,
                       'in_safety_event':0,'waiting_on_sync':0,'in_sync_mode':0,'trace_on':0,'ctrl_cycle_cnt':0,
                       'waypoint_traj':{'state':'idle','setpoint':None, 'segment_id':0,}}

        self.status_aux={'cmd_cnt_rpc':0,'cmd_cnt_exec':0,'cmd_rpc_overflow':0,'sync_irq_cnt':0,'sync_irq_overflow':0}

        self.status_zero=self.status.copy()

        self.board_info={'board_variant':None, 'firmware_version':None,'protocol_version':None,'hardware_id':0}
        self.motion_limits=[0,0]
        self.is_moving_history = [False] * 10

        self._waypoint_traj_segment = [0] * 8
        self._waypoint_traj_start_success = False
        self._waypoint_traj_set_next_traj_success = False
        self._waypoint_traj_start_error_msg = ""
        self._waypoint_traj_set_next_error_msg = ""
        self._waypoint_ts = None

        self.ts_last_syncd_motion=0

        self._dirty_command = False
        self._dirty_gains = False
        self._dirty_trigger = False
        self._dirty_read_gains_from_flash=False
        self._trigger=0
        self._trigger_data=0
        self.load_test_payload = arr.array('B', range(256)) * 4
        self.hw_valid=False
        self.gains = self.params['gains'].copy()
        self.gains_flash = {}


    # ###########  Device Methods #############
    def startup(self, threaded=False):
        try:
            Device.startup(self, threaded=threaded)
            self.hw_valid = self.transport.startup()
            if self.hw_valid:
                # Pull board info
                payload = arr.array('B', [self.RPC_GET_STEPPER_BOARD_INFO])
                self.transport.do_pull_rpc_sync(payload, self.rpc_board_info_reply)
                self.transport.configure_version(self.board_info['firmware_version'])
                return True
            return False
        except KeyError:
            self.hw_valid =False
            return False

    #Configure control mode prior to calling this on process shutdown (or default to freewheel)
    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        self.logger.debug('Shutting down Stepper on: ' + self.usb)
        self.enable_safety()
        self.push_command(exiting=True)
        self.transport.stop()
        self.hw_valid = False

    def is_sync_required(self, ts_last_sync):
        """Determines if synchronization is required base on current the status.

        Parameters
        ----------
        ts_last_sync : float.
            The timestamp of the last synchronized motion.

        Returns
        -------
        bool:
            True if synchronization is required, False otherwise.
        """
        return self.status['in_sync_mode'] and self.ts_last_syncd_motion > ts_last_sync

    def push_command(self, exiting=False):
        """Push the current commands to the robot's hardware
        """
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()

        if self._dirty_trigger:
            payload[0] = self.RPC_SET_TRIGGER
            sidx = self.pack_trigger(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_trigger_reply, exiting=exiting)
            self._trigger=0
            self._dirty_trigger = False

        if self._dirty_gains:
            payload[0] = self.RPC_SET_GAINS
            sidx = self.pack_gains(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_gains_reply, exiting=exiting)
            self._dirty_gains = False

        if self._dirty_command:
            if self.status['in_sync_mode']:  # Mark the time of latest new motion command sent
                self.ts_last_syncd_motion = time.time()
            else:
                self.ts_last_syncd_motion = 0

            payload[0] = self.RPC_SET_COMMAND
            sidx = self.pack_command(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_command_reply, exiting=exiting)
            self._dirty_command = False

    async def push_command_async(self,exiting=False):
        """Push the current commands to the robot's hardware in an asynchronous way
        """
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()

        if self._dirty_trigger:
            payload[0] = self.RPC_SET_TRIGGER
            sidx = self.pack_trigger(payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_trigger_reply, exiting=exiting)
            self._trigger=0
            self._dirty_trigger = False

        if self._dirty_gains:
            payload[0] = self.RPC_SET_GAINS
            sidx = self.pack_gains(payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_gains_reply, exiting=exiting)
            self._dirty_gains = False

        if self._dirty_command:
            if self.status['in_sync_mode']: #Mark the time of latest new motion command sent
                self.ts_last_syncd_motion=time.time()
            else:
                self.ts_last_syncd_motion = 0

            payload[0] = self.RPC_SET_COMMAND
            sidx = self.pack_command(payload, 1)
            await self.transport.do_push_rpc_async(payload[:sidx], self.rpc_command_reply, exiting=exiting)
            self._dirty_command = False

    def pull_status(self, exiting=False):
        """Pull the current status of the robot's hardware
        """
        if not self.hw_valid:
            return
        if self._dirty_read_gains_from_flash:
            payload = arr.array('B', [self.RPC_READ_GAINS_FROM_FLASH])
            self.transport.do_pull_rpc_sync(payload, self.rpc_read_gains_from_flash_reply)
            self._dirty_read_gains_from_flash = False
        payload = arr.array('B', [self.RPC_GET_STATUS])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_reply, exiting=exiting)

    async def pull_status_async(self, exiting=False):
        """Pull the current status of the robot's hardware in an asynchronous way
        """
        if not self.hw_valid:
            return

        if self._dirty_read_gains_from_flash:
            payload = arr.array('B', [self.RPC_READ_GAINS_FROM_FLASH])
            await self.transport.do_pull_rpc_async(payload, self.rpc_read_gains_from_flash_reply, exiting=exiting)
            self._dirty_read_gains_from_flash = False

        payload = arr.array('B', [self.RPC_GET_STATUS])
        await self.transport.do_pull_rpc_async(payload, self.rpc_status_reply, exiting=exiting)

    def push_load_test(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def pull_load_test(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def pretty_print(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def step_sentry(self, robot):
        if self.hw_valid and self.robot_params['robot_sentry']['stepper_is_moving_filter']:
            self.is_moving_history.pop(0)
            self.is_moving_history.append(self.status['is_moving'])
            self.status['is_moving_filtered'] = max(set(self.is_moving_history), key=self.is_moving_history.count)

    # ###########################################################################
    # ###########################################################################


    def set_motion_limits(self,limit_neg, limit_pos):
        """Set motion limits for the robot's motion range

        Parameters
        ----------
        limit_neg : float.
            The new negative motion limit
        limit_pos : float.
            The new positive motion limit
        """
        if limit_neg!=self.motion_limits[0] or limit_pos!=self.motion_limits[1]:
            #Push out immediately
            self.motion_limits=[limit_neg, limit_pos]
            payload=self.transport.get_empty_payload()
            payload[0] = self.RPC_SET_MOTION_LIMITS
            sidx = self.pack_motion_limits(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_motion_limits_reply)

    def set_gains(self,g):
        """Set the gains

        Parameters
        ----------
        g : array?
            An array containing the new gains to set.
        """
        self.gains=g.copy()
        self._dirty_gains = True

    def write_gains_to_YAML(self):
        """@private"""
        raise DeprecationWarning('This method has been deprecated since v0.3.0')

    def write_gains_to_flash(self):
        """Trigger writing gains to flash memory.
        """
        self._trigger = self._trigger | self.TRIGGER_WRITE_GAINS_TO_FLASH
        self._dirty_trigger = True

    def read_gains_from_flash(self):
        """Gains should be read from flash memory.
        """
        self._dirty_read_gains_from_flash=True

    def board_reset(self):
        """Trigger a board reset.
        """
        self._trigger = self._trigger | self.TRIGGER_BOARD_RESET
        self._dirty_trigger=True

    def mark_position_on_contact(self,x):
        """Trigger mark a position on contact.

        Parameters
        ----------
        x : float.
            The position of the mark contact.
        """
        self._trigger_data = x
        self._trigger = self._trigger | self.TRIGGER_MARK_POS_ON_CONTACT
        self._dirty_trigger=True

    def mark_position(self,x):
        """Mark a position.

        Parameters
        ----------
        x : float.
            The position to mark.
        
        Notes:
        ------
        This method only performs the operation if the mode is MODE_SAFETY.
        """
        if self.status['mode']!=self.MODE_SAFETY:
            self.logger.warning('Can not mark position. Must be in MODE_SAFETY for %s'%self.usb)
            return
        self._trigger_data=x
        self._trigger = self._trigger | self.TRIGGER_MARK_POS
        self._dirty_trigger=True

    def reset_motion_gen(self):
        """Trigger a reset of the motion generator.
        """
        self._trigger = self._trigger | self.TRIGGER_RESET_MOTION_GEN
        self._dirty_trigger = True

    def reset_pos_calibrated(self):
        """Trigger a reset of the calibrated position.  
        """
        self._trigger = self._trigger | self.TRIGGER_RESET_POS_CALIBRATED
        self._dirty_trigger = True

    def set_pos_calibrated(self):
        """Trigger seting the calibrated position.
        """
        self._trigger = self._trigger | self.TRIGGER_POS_CALIBRATED
        self._dirty_trigger = True



    # ###########################################################################
    def enable_safety(self):
            """Enable safety mode
            """
            self.set_command(mode=self.MODE_SAFETY)

    def enable_freewheel(self):
            """Enable freewheel mode.
            """
            self.set_command(mode=self.MODE_FREEWHEEL)

    def enable_hold(self):
        """Enable hold mode.
        """
        self.set_command(mode=self.MODE_HOLD)

    def enable_vel_pid(self):
        """Enable velocity PID control mode.
        """
        self.set_command(mode=self.MODE_VEL_PID, v_des=0)

    def enable_pos_pid(self):
        """Enable position PID control mode.
        """
        self.set_command(mode=self.MODE_POS_PID, x_des=self.status['pos'])

    def enable_vel_traj(self):
        """Enable velocity trajectory control mode.
        """
        self.set_command(mode=self.MODE_VEL_TRAJ, v_des=0)

    def enable_pos_traj(self):
        """Enable position trajectory control mode.
        """
        self.set_command(mode=self.MODE_POS_TRAJ, x_des=self.status['pos'])

    def enable_pos_traj_waypoint(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def enable_pos_traj_incr(self):
        """Enable incremental position trajectory control mode.
        """
        self.set_command(mode=self.MODE_POS_TRAJ_INCR, x_des=0)

    def enable_current(self):
        """Enable current mode.
        """
        self.set_command(mode=self.MODE_CURRENT, i_des=0)

    def enable_sync_mode(self):
        """Enable synchrous mode.
        """
        self.gains['enable_sync_mode'] = 1
        self._dirty_gains = 1

    def disable_sync_mode(self):
        """Disable asynchronous mode.
        """
        self.gains['enable_sync_mode']=0
        self._dirty_gains=1

    def enable_runstop(self):
        """Enable runstop mode.
        """
        self.gains['enable_runstop'] = 1
        self._dirty_gains = 1

    def disable_runstop(self):
        """Disable runstop mode.
        """
        self.gains['enable_runstop']=0
        self._dirty_gains=1

    def enable_guarded_mode(self):
        """Enable guarded mode.
        """
        self.gains['enable_guarded_mode'] = 1
        self._dirty_gains = 1

    def disable_guarded_mode(self):
        """Disable guarded mode.
        """
        self.gains['enable_guarded_mode'] = 0
        self._dirty_gains = 1



    # ######################################################################
    #Primary interface to controlling the stepper
    #YAML defaults are used if values not provided
    #This allows user to override defaults every control cycle and then easily revert to defaults
    def set_command(self,mode=None, x_des=None, v_des=None, a_des=None,i_des=None, stiffness=None,i_feedforward=None, i_contact_pos=None, i_contact_neg=None  ):
        """Set a command.

        Parameters
        ----------
        mode : int or None, optional.
            The command mode to set, if not provided the current mode remains unchanged, by default None.
        
        x_des : float or None, optional.
            The desired position to change, if not provided the position remains unchanged. If the mode is MODE_POS_TRAJ_INCR, this parameter
        increments the 'incr_trigger' value, by default None.
        
        v_des : float or None, optional.
            The desired velocity set, if not provided the value is determined based on the current mode or set 0 if the mode is velocity-based, by default None.
        
        a_des : float or None, optional.
            The desired acceleration to set, if not provided the value is determined based on the current motion configuration, by default None.
        
        i_des : float or None, optional.
            The desired current to set, if not provided the current value remains unchanged unless the mode is MODE_CURRENT, in which case it updates the `i_feedforward` value, by default None.
        
        stiffness : float or None, optional.
            The stiffnes value to set. If not provided it remains unchanged, by default None.
        
        i_feedforward : float or None, optional.
            The feedforward current to set, if not provided it remains unchanged unless the mode is `MODE_CURRENT`, by default None.
        
        i_contact_pos : float or None, optional.
            The positive contact current to set, if not provided it is determined based on the current gains configuration, by default None.
        
        i_contact_neg : float or None, optional.
            The negative contact current to set, if not provided it is determined based on the current gains configuration, by default None.

        Notes:
        ------
        This method updates the internal command dictionary and marks it as dirty,
        indicating that it has been modified.
        """
        if mode is not None:
            self._command['mode'] = mode

        if x_des is not None:
            self._command['x_des'] = x_des
            if self._command['mode'] == self.MODE_POS_TRAJ_INCR:
                self._command['incr_trigger'] = (self._command['incr_trigger']+1)%255

        if v_des is not None:
            self._command['v_des'] = v_des
        else:
            if mode == self.MODE_VEL_PID or mode == self.MODE_VEL_TRAJ:
                self._command['v_des'] = 0
            else:
                self._command['v_des'] = self.params['motion']['vel']

        if a_des is not None:
            self._command['a_des'] = a_des
        else:
            self._command['a_des'] = self.params['motion']['accel']

        if stiffness is not None:
            self._command['stiffness'] = max(0.0, min(1.0, stiffness))
        else:
            self._command['stiffness'] =1

        if i_feedforward is not None:
            self._command['i_feedforward'] = i_feedforward
        else:
            self._command['i_feedforward'] = 0

        if i_des is not None and mode == self.MODE_CURRENT:
            self._command['i_feedforward'] =i_des


        if i_contact_pos is not None:
            self._command['i_contact_pos'] = i_contact_pos
        else:
            self._command['i_contact_pos']=self.params['gains']['i_contact_pos']

        if i_contact_neg is not None:
            self._command['i_contact_neg'] = i_contact_neg
        else:
            self._command['i_contact_neg'] = self.params['gains']['i_contact_neg']
        #print(time.time(), i_des, self._command['i_feedforward'],mode == self.MODE_CURRENT)
        #print(time.time(),self._command['x_des'],self._command['incr_trigger'],self._command['v_des'],self._command['a_des'])
        self._dirty_command=True


    def wait_while_is_moving(self,timeout=15.0):
        """
        Poll until is moving flag is false.
        
        Return True if success.
        
        Return False if timeout.
        """
        ts = time.time()
        self.pull_status()
        while self.status['is_moving'] and time.time() - ts < timeout:
            time.sleep(0.1)
            self.pull_status()
        return not self.status['is_moving']

    def wait_until_at_setpoint(self,timeout=15.0):
        """
        Poll until near setpoint.
        
        Return True if success.
        
        Return False if timeout.
        """
        ts = time.time()
        self.pull_status()
        while not self.status['near_pos_setpoint'] and time.time() - ts < timeout:
            time.sleep(0.1)
            self.pull_status()
        return self.status['near_pos_setpoint']

    ########### Handle current and effort conversions  ###########

    #Effort_ticks are in the units of the uC current controller (0-255 8 bit)
    #Conversion to A is based on the sense resistor and motor driver (see firmware)
    def current_to_effort_ticks(self,i_A):
        """Convert current to motor effort ticks.

        Parameters
        ----------
        i_A : float.
            The current in amperes to convert.

        Returns
        -------
        int:
            The motor effort values in ticks, between -255 to 255.
        """
        if self.board_info['hardware_id']==0: # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info['hardware_id']>=1: # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        effort_ticks = (i_A * 1000.0) / mA_per_tick
        return min(255,max(-255,int(effort_ticks)))

    def effort_ticks_to_current(self,e):
        """Convert motor effort ticks to current.

        Parameters
        ----------
        e : int.
            The motor effort value in ticks to convert.

        Returns
        -------
        float:
            The current value in amperes.
        """
        if self.board_info['hardware_id'] == 0:  # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info['hardware_id'] >= 1:  # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        return e * mA_per_tick / 1000.0

    # Effort_pct is defined as a percentage of the maximum allowable motor winding current
    # Range is -100.0 to 100.0
    def current_to_effort_pct(self,i_A):
        """Convert current to motor effor percentage.

        Parameters
        ----------
        i_A : float.
            The current in amperes to convert.

        Returns
        -------
        float:
            The motor effort percentage.
        """
        if i_A>0:
            return 100*max(0.0,min(1.0,i_A/self.gains['iMax_pos']))
        else:
            return 100*min(0.0, max(-1.0, i_A/ abs(self.gains['iMax_neg'])))

    def effort_pct_to_current(self,e_pct):
        """Convert motor effort percentage to current.

        Parameters
        ----------
        e_pct : float.
            The motor effort percentage to convert.

        Returns
        -------
        float:
            The current value in amperes.
        """
        if e_pct>0:
            return min(1.0,e_pct/100.0)*self.gains['iMax_pos']
        else:
            return max(-1.0, e_pct/100.0) * abs(self.gains['iMax_neg'])

    def current_to_torque(self,i):
        """@private"""
        raise DeprecationWarning('Method current_to_torque has been deprecated since v0.3.5')

    def torque_to_current(self, tq):
        """@private"""
        raise DeprecationWarning('Method torque_to_current has been deprecated since v0.3.5')


        # ####################### Encoder Calibration ######################

    def get_chip_id(self):
        """Get the chip ID.

        Returns
        -------
        str:
            The chip ID as a UTF-8 decoded string.
        """
        self.turn_menu_interface_on()
        time.sleep(0.5)
        cid = self.menu_transaction(b'b', do_print=False)[0][:-2]
        self.turn_rpc_interface_on()
        time.sleep(0.5)
        return cid.decode('utf-8')

    def read_encoder_calibration_from_YAML(self):
        """Read enconder calibration from YAML.

        Returns
        -------
        dict:
            The encoder calibration data read from the YAML file.
        """
        device_name=self.usb[5:]
        sn=self.robot_params[device_name]['serial_no']
        fn='calibration_steppers/'+device_name+'_'+sn+'.yaml'
        enc_data=read_fleet_yaml(fn)
        return enc_data

    def write_encoder_calibration_to_YAML(self,data,filename=None, fleet_dir=None):
        """Write encoder calibration data to YAML file.

        Parameters
        ----------
        data : dict.
            The encoder data to write.
        
        filename : str or None, optional.
            The filename of the YAML file. If not provided, it is constructed based on the device name and serial number, by default None.
        
        fleet_dir : str or None, optional.
            The directory where the file should be saved, if not provided the default directory is used, by default None.
        """
        device_name = self.usb[5:]
        if filename is None:
            sn = self.robot_params[device_name]['serial_no']
            filename = 'calibration_steppers/'+device_name + '_' + sn + '.yaml'
        print('Writing encoder calibration: %s'%filename)
        write_fleet_yaml(filename,data,fleet_dir=fleet_dir)

    def read_encoder_calibration_from_flash(self):
        """Read encoder calibration data from flash memory.

        Returns
        -------
        list:
            The encoder calibration data as a list of floats.
        """
        self.turn_menu_interface_on()
        time.sleep(0.5)
        time.sleep(0.5)
        self.logger.debug('Reading encoder calibration...')
        e = self.menu_transaction(b'q',do_print=False)[19]
        self.turn_rpc_interface_on()
        self.push_command()
        self.logger.debug('Reseting board')
        self.board_reset()
        self.push_command()
        e = e[:-4].decode('utf-8')  # We now have string of floats, convert to list of floats
        enc_calib = []
        while len(e):
            ff = e.find(',')
            if ff != -1:
                enc_calib.append(float(e[:ff]))
                e = e[ff + 2:]
            else:
                enc_calib.append(float(e))
                e = []
        if len(enc_calib)==16384:
            self.logger.debug('Successful read of encoder calibration')
        else:
            self.logger.debug('Failed to read encoder calibration')
        return enc_calib

    def write_encoder_calibration_to_flash(self,data):
        """Write encoder calibration data to flash memory.

        Parameters
        ----------
        data : list.
            The encoder calirbation data to write.
        """
        if not self.hw_valid:
            return
        #This will take a few seconds. Blocks until complete.
        if len(data)!=16384:
            self.logger.warning('Bad encoder data')
        else:
            self.logger.debug('Writing encoder calibration...')
            payload=self.transport.get_empty_payload()
            for p in range(256):
                if p%10==0:
                    sys.stdout.write('.')
                    sys.stdout.flush()
                payload[0] = self.RPC_SET_ENC_CALIB
                payload[1] = p
                sidx=2
                for i in range(64):
                    pack_float_t(payload, sidx, data[p*64+i])
                    sidx += 4
                # self.logger.debug('Sending encoder calibration rpc of size',sidx)
                self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_enc_calib_reply)

    def rpc_enc_calib_reply(self,reply):
        """Handle the rpc reply for encoder calibration.

        Parameters
        ----------
        reply : list.
            The RPC reply message.
        """
        if reply[0] != self.RPC_REPLY_ENC_CALIB:
            self.logger.debug('Error RPC_REPLY_ENC_CALIB', reply[0])

    # ######################Menu Interface ################################3


    def turn_rpc_interface_on(self):
        """Turn on the RPC interface.
        """
        self.menu_transaction(b'zyx')


    def turn_menu_interface_on(self):
        """Turn on the menu interface.
        """
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_SET_MENU_ON])
        self.transport.do_push_rpc_sync(payload, self.rpc_menu_on_reply)


    def print_menu(self):
        """Print the menu
        """
        self.menu_transaction(b'm')

    def menu_transaction(self,x,do_print=True):
        """Perform a menu transaction.

        Parameters
        ----------
        x : bytes.
            The command to send for the menu transaction.
        
        do_print : bool, optional
            A flag to control whether to print the reply in the console, by default True

        Returns
        -------
        list:
            A list containing the reply messages received during the transaction.
        """
        if not self.hw_valid:
            return
        self.transport.ser.write(x)
        time.sleep(0.1)
        reply=[]
        while self.transport.ser.inWaiting():
            r=self.transport.ser.readline()
            if do_print:
                if type(r)==bytes:
                    print(r.decode('UTF-8'), end=' ')
                else:
                    print(r, end=' ')
            reply.append(r)
        return reply

    # ################Waypoint Trajectory Interface #####################
    def start_waypoint_trajectory(self, first_segment):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def set_next_trajectory_segment(self, next_segment):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def stop_waypoint_trajectory(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    # ################Transport Callbacks #####################
    def unpack_board_info(self,s):
        """Unpack board information and updates the `self.board_info`.

        Parameters
        ----------
        s : bytes.
            A byte string containing packed board information.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx=0
        self.board_info['board_variant'] = unpack_string_t(s[sidx:], 20).strip('\x00')
        self.board_info['hardware_id'] = 0
        if len(self.board_info['board_variant'])==9: #New format of Stepper.x  Older format of Stepper.BoardName.Vx' If older format,default to 0
            self.board_info['hardware_id']=int(self.board_info['board_variant'][-1])
        sidx += 20
        self.board_info['firmware_version'] = unpack_string_t(s[sidx:], 20).strip('\x00')
        self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]
        sidx += 20
        return sidx

    def unpack_status(self,s):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def unpack_gains(self,s):
        """Unpack gains data from a byte string and updates the `self.gains_flash` dictionary.

        Parameters
        ----------
        s : bytes.
            A byte string containing packed gains data.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx=0
        self.gains_flash['pKp_d'] = unpack_float_t(s[sidx:]);sidx+=4
        self.gains_flash['pKi_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pKd_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pLPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pKi_limit'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKp_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKi_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKd_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vLPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vKi_limit'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vTe_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['iMax_pos'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['iMax_neg'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['phase_advance_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['pos_near_setpoint_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vel_near_setpoint_d'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['vel_status_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['effort_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['safety_stiffness'] = unpack_float_t(s[sidx:]);sidx += 4
        self.gains_flash['i_safety_feedforward'] = unpack_float_t(s[sidx:]);sidx += 4
        config = unpack_uint8_t(s[sidx:]);sidx += 1
        self.gains_flash['safety_hold']= int(config & self.CONFIG_SAFETY_HOLD>0)
        self.gains_flash['enable_runstop'] = int(config & self.CONFIG_ENABLE_RUNSTOP>0)
        self.gains_flash['enable_sync_mode'] = int(config & self.CONFIG_ENABLE_SYNC_MODE>0)
        self.gains_flash['enable_guarded_mode'] = int(config & self.CONFIG_ENABLE_GUARDED_MODE > 0)
        self.gains_flash['flip_encoder_polarity'] = int(config & self.CONFIG_FLIP_ENCODER_POLARITY > 0)
        self.gains_flash['flip_effort_polarity'] = int(config & self.CONFIG_FLIP_EFFORT_POLARITY > 0)
        self.gains_flash['enable_vel_watchdog'] = int(config & self.CONFIG_ENABLE_VEL_WATCHDOG > 0)
        return sidx

    def pack_motion_limits(self, s, sidx):
        """Packs motion limits data starting at index sidx.

        Parameters
        ----------
        s : bytes.
            A byte string to which motion limits data will be packed.
        
        sidx : int.
            The starting index in the byte string `s`.

        Returns
        -------
        int:
            The index of the next unused byte in the byte string `s` after packing.
        """
        pack_float_t(s, sidx, self.motion_limits[0])
        sidx += 4
        pack_float_t(s, sidx, self.motion_limits[1])
        sidx += 4
        return sidx

    def pack_command(self, s, sidx):
        """Packs command data starting at index sidx.

        Parameters
        ----------
        s : bytes.
            A byte string to which command data will be packed.
        
        sidx : int.
            The starting index in the byte string `s`.

        Returns
        -------
        int:
            The index of the next unused byte in the byte string `s` after packing.
        """
        pack_uint8_t(s, sidx, self._command['mode'])
        sidx += 1
        pack_float_t(s, sidx, self._command['x_des'])
        sidx += 4
        pack_float_t(s, sidx, self._command['v_des'])
        sidx += 4
        pack_float_t(s, sidx, self._command['a_des'])
        sidx += 4
        pack_float_t(s, sidx, self._command['stiffness'])
        sidx += 4
        pack_float_t(s, sidx, self._command['i_feedforward'])
        sidx += 4
        pack_float_t(s, sidx, self._command['i_contact_pos'])
        sidx += 4
        pack_float_t(s, sidx, self._command['i_contact_neg'])
        sidx += 4
        pack_uint8_t(s, sidx, self._command['incr_trigger'])
        sidx += 1
        return sidx

    def pack_gains(self,s,sidx):
        """Packs gains data starting at index sidx.

        Parameters
        ----------
        s : bytes.
            A byte string to which gains data will be packed.
        
        sidx : int.
            The starting index in the byte string `s`.

        Returns
        -------
        int:
            The index of the next unused byte in the byte string `s` after packing.
        """
        pack_float_t(s, sidx, self.gains['pKp_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pKi_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pKd_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pLPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['pKi_limit']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKp_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKi_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKd_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['vLPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['vKi_limit']); sidx += 4
        pack_float_t(s, sidx, self.gains['vTe_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['iMax_pos']);sidx += 4
        pack_float_t(s, sidx, self.gains['iMax_neg']);sidx += 4
        pack_float_t(s, sidx, self.gains['phase_advance_d']);sidx += 4
        pack_float_t(s, sidx, self.gains['pos_near_setpoint_d']); sidx += 4
        pack_float_t(s, sidx, self.gains['vel_near_setpoint_d']); sidx += 4
        pack_float_t(s, sidx, self.gains['vel_status_LPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['effort_LPF']);sidx += 4
        pack_float_t(s, sidx, self.gains['safety_stiffness']);sidx += 4
        pack_float_t(s, sidx, self.gains['i_safety_feedforward']);sidx += 4
        config=0
        if self.gains['safety_hold']:
            config=config | self.CONFIG_SAFETY_HOLD
        if self.gains['enable_runstop']:
            config=config | self.CONFIG_ENABLE_RUNSTOP
        if self.gains['enable_sync_mode']:
            config=config | self.CONFIG_ENABLE_SYNC_MODE
        if self.gains['enable_guarded_mode']:
            config=config | self.CONFIG_ENABLE_GUARDED_MODE
        if self.gains['flip_encoder_polarity']:
            config = config | self.CONFIG_FLIP_ENCODER_POLARITY
        if self.gains['flip_effort_polarity']:
            config = config | self.CONFIG_FLIP_EFFORT_POLARITY
        if self.gains['enable_vel_watchdog']:
            config=config | self.CONFIG_ENABLE_VEL_WATCHDOG
        pack_uint8_t(s, sidx, config); sidx += 1
        return sidx

    def pack_trigger(self, s, sidx):
        """Packs trigger data starting at index sidx.

        Parameters
        ----------
        s : bytes.
            A byte string to which trigger data will be packed.
        sidx : int.
            The starting index in the byte string `s`.

        Returns
        -------
        int:
            The index of the next unused byte in the byte string `s` after packing.
        """
        pack_uint32_t(s, sidx, self._trigger)
        sidx += 4
        pack_float_t(s, sidx, self._trigger_data)
        sidx += 4
        return sidx
    def pack_trajectory_segment(self, s, sidx):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def rpc_board_info_reply(self, reply):
        """Handle the RPC reply for board information.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] == self.RPC_REPLY_STEPPER_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            print('Error RPC_REPLY_STEPPER_BOARD_INFO', reply[0])

    def rpc_gains_reply(self, reply):
        """Handle the RPC reply for gains.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] != self.RPC_REPLY_GAINS:
            print('Error RPC_REPLY_GAINS', reply[0])

    def rpc_trigger_reply(self, reply):
        """Handle the RPC reply for trigger.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] != self.RPC_REPLY_SET_TRIGGER:
            print('Error RPC_REPLY_SET_TRIGGER', reply[0])

    def rpc_command_reply(self, reply):
        """Handle the RPC reply for command.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] != self.RPC_REPLY_COMMAND:
            print('Error RPC_REPLY_COMMAND', reply[0])

    def rpc_motion_limits_reply(self, reply):
        """Handle the RPC reply for motion limits.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] != self.RPC_REPLY_MOTION_LIMITS:
            print('Error RPC_REPLY_MOTION_LIMITS', reply[0])

    def rpc_menu_on_reply(self, reply):
        """Handle the RPC reply for menu activation.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] != self.RPC_REPLY_MENU_ON:
            print('Error RPC_REPLY_MENU_ON', reply[0])

    def rpc_status_reply(self, reply):
        """Handle the RPC reply for status.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] == self.RPC_REPLY_STATUS:
            nr = self.unpack_status(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])

    def rpc_read_gains_from_flash_reply(self, reply):
        """Handle the RPC reply for reading gains from flash.

        Parameters
        ----------
        reply : list.
            The reply message.
        """
        if reply[0] == self.RPC_REPLY_READ_GAINS_FROM_FLASH:
            nr = self.unpack_gains(reply[1:])
        else:
            print('Error RPC_REPLY_READ_GAINS_FROM_FLASH', reply[0])


    def rpc_start_new_traj_reply(self, reply):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def rpc_set_next_traj_seg_reply(self, reply):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def rpc_reset_traj_reply(self, reply):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))


    def rpc_read_firmware_trace_reply(self, reply):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def enable_firmware_trace(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def disable_firmware_trace(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))


    def pull_status_aux(self):
        """@private"""
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

# ######################## STEPPER PROTOCOL PO #################################

class Stepper_Protocol_P0(StepperBase):
    def unpack_status(self,s):
        """Unpack status information and udpates `self.status` dictionary.

        Parameters
        ----------
        s : bytes
            A byte string containing packed status data.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx=0
        self.status['mode']=unpack_uint8_t(s[sidx:]);sidx+=1
        self.status['effort_ticks'] = unpack_float_t(s[sidx:]);sidx+=4
        self.status['current']=self.effort_ticks_to_current(self.status['effort_ticks'])
        self.status['effort_pct']=self.current_to_effort_pct(self.status['current'])
        self.status['pos'] = unpack_double_t(s[sidx:]);sidx+=8
        self.status['vel'] = unpack_float_t(s[sidx:]);sidx+=4
        self.status['err'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['diag'] = unpack_uint32_t(s[sidx:]);sidx += 4
        self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
        self.status['debug'] = unpack_float_t(s[sidx:]);sidx += 4
        self.status['guarded_event'] = unpack_uint32_t(s[sidx:]);sidx += 4

        self.status['pos_calibrated'] =self.status['diag'] & self.DIAG_POS_CALIBRATED > 0
        self.status['runstop_on'] =self.status['diag'] & self.DIAG_RUNSTOP_ON > 0
        self.status['near_pos_setpoint'] =self.status['diag'] & self.DIAG_NEAR_POS_SETPOINT > 0
        self.status['near_vel_setpoint'] = self.status['diag'] & self.DIAG_NEAR_VEL_SETPOINT > 0
        self.status['is_moving'] =self.status['diag'] & self.DIAG_IS_MOVING > 0
        self.status['at_current_limit'] =self.status['diag'] & self.DIAG_AT_CURRENT_LIMIT > 0
        self.status['is_mg_accelerating'] = self.status['diag'] & self.DIAG_IS_MG_ACCELERATING > 0
        self.status['is_mg_moving'] =self.status['diag'] & self.DIAG_IS_MG_MOVING > 0
        self.status['calibration_rcvd'] = self.status['diag'] & self.DIAG_CALIBRATION_RCVD > 0
        self.status['in_guarded_event'] = self.status['diag'] & self.DIAG_IN_GUARDED_EVENT > 0
        self.status['in_safety_event'] = self.status['diag'] & self.DIAG_IN_SAFETY_EVENT > 0
        self.status['waiting_on_sync'] = self.status['diag'] & self.DIAG_WAITING_ON_SYNC > 0
        return sidx

    def pretty_print(self):
        """Print a readable representation of the robot's status
        """
        print('-----------')
        print('Mode', self.MODE_NAMES[self.status['mode']])
        print('x_des (rad)', self._command['x_des'], '(deg)',rad_to_deg(self._command['x_des']))
        print('v_des (rad)', self._command['v_des'], '(deg)',rad_to_deg(self._command['v_des']))
        print('a_des (rad)', self._command['a_des'], '(deg)',rad_to_deg(self._command['a_des']))
        print('Stiffness',self._command['stiffness'])
        print('Feedforward', self._command['i_feedforward'])
        print('Pos (rad)', self.status['pos'], '(deg)',rad_to_deg(self.status['pos']))
        print('Vel (rad/s)', self.status['vel'], '(deg)',rad_to_deg(self.status['vel']))
        print('Effort (Ticks)', self.status['effort_ticks'])
        print('Effort (Pct)',self.status['effort_pct'])
        print('Current (A)', self.status['current'])
        print('Error (deg)', rad_to_deg(self.status['err']))
        print('Debug', self.status['debug'])
        print('Guarded Events:', self.status['guarded_event'])
        print('Diag', format(self.status['diag'], '032b'))
        print('       Position Calibrated:', self.status['pos_calibrated'])
        print('       Runstop on:', self.status['runstop_on'])
        print('       Near Pos Setpoint:', self.status['near_pos_setpoint'])
        print('       Near Vel Setpoint:', self.status['near_vel_setpoint'])
        print('       Is Moving:', self.status['is_moving'])
        print('       Is Moving Filtered:', self.status['is_moving_filtered'])
        print('       At Current Limit:', self.status['at_current_limit'])
        print('       Is MG Accelerating:', self.status['is_mg_accelerating'])
        print('       Is MG Moving:', self.status['is_mg_moving'])
        print('       Encoder Calibration in Flash:', self.status['calibration_rcvd'])
        print('       In Guarded Event:', self.status['in_guarded_event'])
        print('       In Safety Event:', self.status['in_safety_event'])
        print('       Waiting on Sync:', self.status['waiting_on_sync'])
        print('Timestamp (s)', self.status['timestamp'])
        #print('Read error', self.transport.status['read_error'])
        print('Board variant:', self.board_info['board_variant'])
        print('Firmware version:', self.board_info['firmware_version'])

# ######################## STEPPER PROTOCOL P1 #################################
class Stepper_Protocol_P1(StepperBase):

    def unpack_status(self,s,unpack_to=None):
        """Unpack status information and udpates `self.status` dictionary.

        Parameters
        ----------
        s : bytes
            A byte string containing packed status data.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        unpack_to['mode']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['effort_ticks'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['current']=self.effort_ticks_to_current(unpack_to['effort_ticks'])
        unpack_to['effort_pct'] = self.current_to_effort_pct(unpack_to['current'])
        unpack_to['pos'] = unpack_double_t(s[sidx:]);sidx+=8
        unpack_to['vel'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['err'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['diag'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['debug'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['guarded_event'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['waypoint_traj']['setpoint'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['waypoint_traj']['segment_id'] = unpack_uint16_t(s[sidx:]);sidx += 2

        unpack_to['pos_calibrated'] =unpack_to['diag'] & self.DIAG_POS_CALIBRATED > 0
        unpack_to['runstop_on'] =unpack_to['diag'] & self.DIAG_RUNSTOP_ON > 0
        unpack_to['near_pos_setpoint'] =unpack_to['diag'] & self.DIAG_NEAR_POS_SETPOINT > 0
        unpack_to['near_vel_setpoint'] = unpack_to['diag'] & self.DIAG_NEAR_VEL_SETPOINT > 0
        unpack_to['is_moving'] =unpack_to['diag'] & self.DIAG_IS_MOVING > 0
        unpack_to['at_current_limit'] =unpack_to['diag'] & self.DIAG_AT_CURRENT_LIMIT > 0
        unpack_to['is_mg_accelerating'] = unpack_to['diag'] & self.DIAG_IS_MG_ACCELERATING > 0
        unpack_to['is_mg_moving'] =unpack_to['diag'] & self.DIAG_IS_MG_MOVING > 0
        unpack_to['calibration_rcvd'] = unpack_to['diag'] & self.DIAG_CALIBRATION_RCVD > 0
        unpack_to['in_guarded_event'] = unpack_to['diag'] & self.DIAG_IN_GUARDED_EVENT > 0
        unpack_to['in_safety_event'] = unpack_to['diag'] & self.DIAG_IN_SAFETY_EVENT > 0
        unpack_to['waiting_on_sync'] = unpack_to['diag'] & self.DIAG_WAITING_ON_SYNC > 0
        unpack_to['in_sync_mode'] = unpack_to['diag'] & self.DIAG_IN_SYNC_MODE > 0
        if unpack_to['diag'] & self.DIAG_TRAJ_WAITING_ON_SYNC > 0:
            unpack_to['waypoint_traj']['state']='waiting_on_sync'
        elif unpack_to['diag'] & self.DIAG_TRAJ_ACTIVE > 0:
            unpack_to['waypoint_traj']['state']='active'
        else:
            unpack_to['waypoint_traj']['state']='idle'

        return sidx

    def pretty_print(self):
        """Print a readable representation of the robot's status
        """
        print('-----------')
        print('Mode', self.MODE_NAMES[self.status['mode']])
        print('x_des (rad)', self._command['x_des'], '(deg)',rad_to_deg(self._command['x_des']))
        print('v_des (rad)', self._command['v_des'], '(deg)',rad_to_deg(self._command['v_des']))
        print('a_des (rad)', self._command['a_des'], '(deg)',rad_to_deg(self._command['a_des']))
        print('Stiffness',self._command['stiffness'])
        print('Feedforward', self._command['i_feedforward'])
        print('Pos (rad)', self.status['pos'], '(deg)',rad_to_deg(self.status['pos']))
        print('Vel (rad/s)', self.status['vel'], '(deg)',rad_to_deg(self.status['vel']))
        print('Effort (Ticks)', self.status['effort_ticks'])
        print('Effort (Pct)', self.status['effort_pct'])
        print('Current (A)', self.status['current'])
        print('Error (deg)', rad_to_deg(self.status['err']))
        print('Debug', self.status['debug'])
        print('Guarded Events:', self.status['guarded_event'])
        print('Diag', format(self.status['diag'], '032b'))
        print('       Position Calibrated:', self.status['pos_calibrated'])
        print('       Runstop on:', self.status['runstop_on'])
        print('       Near Pos Setpoint:', self.status['near_pos_setpoint'])
        print('       Near Vel Setpoint:', self.status['near_vel_setpoint'])
        print('       Is Moving:', self.status['is_moving'])
        print('       Is Moving Filtered:', self.status['is_moving_filtered'])
        print('       At Current Limit:', self.status['at_current_limit'])
        print('       Is MG Accelerating:', self.status['is_mg_accelerating'])
        print('       Is MG Moving:', self.status['is_mg_moving'])
        print('       Encoder Calibration in Flash:', self.status['calibration_rcvd'])
        print('       In Guarded Event:', self.status['in_guarded_event'])
        print('       In Safety Event:', self.status['in_safety_event'])
        print('       Waiting on Sync:', self.status['waiting_on_sync'])
        print('       Trace recording:', self.status['trace_on'])
        print('Waypoint Trajectory')
        print('       State:', self.status['waypoint_traj']['state'])
        print('       Setpoint: (rad) %s | (deg) %s'%(self.status['waypoint_traj']['setpoint'],  rad_to_deg(self.status['waypoint_traj']['setpoint'])))
        print('       Segment ID:', self.status['waypoint_traj']['segment_id'])
        print('Timestamp (s)', self.status['timestamp'])
        #print('Read error', self.transport.status['read_error'])
        print('Board variant:', self.board_info['board_variant'])
        print('Firmware version:', self.board_info['firmware_version'])

    def enable_pos_traj_waypoint(self):
        """Enable position trajectory waypoint
        """
        self.set_command(mode=self.MODE_POS_TRAJ_WAYPOINT)

    def start_waypoint_trajectory(self, first_segment):
        """Starts execution of a waypoint trajectory on hardware.

        Parameters
        ----------
        first_segment : list.
            List of length eight, structured like [duration_s, a0, a1, a2, a3, a4, a5, segment_id].
            The hardware begins executing this first segment of a spline. The segment's duration and
            six coefficients (a0-a5) fill the first seven elements of the list. A segment ID, always
            2 for the first segment, fills the last element of the list.

        Returns
        -------
        bool:
            True if uC successfully initiated a new trajectory.
        """
        if len(first_segment) != 8:
            self.logger.warning('start_waypoint_trajectory: Invalid segment arr length (must be 8)')
            return False
        self._waypoint_traj_segment = first_segment
        self._waypoint_ts = time.time()
        if self._waypoint_traj_segment is not None:
            payload = self.transport.get_empty_payload()
            payload[0] = self.RPC_START_NEW_TRAJECTORY
            sidx = self.pack_trajectory_segment(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_start_new_traj_reply)
        if not self._waypoint_traj_start_success:
            self.logger.warning('start_waypoint_trajectory: %s' % self._waypoint_traj_start_error_msg.capitalize())
        # return self._waypoint_traj_start_success
        return self._waypoint_traj_start_success

    def set_next_trajectory_segment(self, next_segment):
        """Sets the next segment for the hardware to execute.

        This method is generally called multiple times while the prior segment is executing. This
        provides the hardware with the next segment to gracefully transition across the entire spline,
        while allowing users to preempt or modify the future trajectory in real time.

        This method will return False if there is not already an segment executing on the uC.

        Parameters
        ----------
        next_segment : list.
            List of length eight, structured like [duration_s, a0, a1, a2, a3, a4, a5, segment_id].
            Duration and six coefficients fill the first seven elements of the list. Generally, the
            coefficients are calculated to smoothly transition across a spline. The segment ID, always
            1 higher than the prior segment's ID, fills the last element of the list.

        Returns
        -------
        bool:
            True if uC successfully queued next trajectory.
        """
        if len(next_segment) != 8:
            self.logger.warning('set_next_trajectory_segment: Invalid segment arr length (must be 8)')
            return False
        self._waypoint_traj_segment = next_segment
        if self._waypoint_traj_segment is not None:
            payload = self.transport.get_empty_payload()
            payload[0] = self.RPC_SET_NEXT_TRAJECTORY_SEG
            sidx = self.pack_trajectory_segment(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_set_next_traj_seg_reply)
        if not self._waypoint_traj_set_next_traj_success:
            self.logger.warning('set_next_trajectory_segment: %s' % self._waypoint_traj_set_next_error_msg.capitalize())
        return self._waypoint_traj_set_next_traj_success

    def stop_waypoint_trajectory(self):
        """Stops execution of the waypoint trajectory.
        """
        self._waypoint_ts = None
        payload = arr.array('B', [self.RPC_RESET_TRAJECTORY])
        self.transport.do_push_rpc_sync(payload, self.rpc_reset_traj_reply)

    def pack_trajectory_segment(self, s, sidx):
        """Packs trajectory segment data starting at index sidx.

        Parameters
        ----------
        s : bytes.
            A byte string to which trajectory segment data will be packed.
        
        sidx : int.
            The starting index in the byte string `s`.

        Returns
        -------
        int:
            The index of the next unused byte in the byte string `s` after packing.
        """
        for i in range(7):
            pack_float_t(s, sidx, self._waypoint_traj_segment[i]);
            sidx += 4
        pack_uint8_t(s, sidx, self._waypoint_traj_segment[7]);
        sidx += 1
        return sidx

    def rpc_start_new_traj_reply(self, reply):
        """Process the RPC reply for starting a new trajectory.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.
        """
        if reply[0] == self.RPC_REPLY_START_NEW_TRAJECTORY:
            sidx = 1
            self._waypoint_traj_start_success = unpack_uint8_t(reply[sidx:]);
            sidx += 1
            self._waypoint_traj_start_error_msg = unpack_string_t(reply[sidx:], 100).strip('\x00')
            sidx += 100
        else:
            self.logger.error('RPC_REPLY_START_NEW_TRAJECTORY replied {0}'.format(reply[0]))

    def rpc_set_next_traj_seg_reply(self, reply):
        """Process the RPC reply for setting the next trajectory segment.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.
        """
        if reply[0] == self.RPC_REPLY_SET_NEXT_TRAJECTORY_SEG:
            sidx = 1
            self._waypoint_traj_set_next_traj_success = unpack_uint8_t(reply[sidx:]);
            sidx += 1
            self._waypoint_traj_set_next_error_msg = unpack_string_t(reply[sidx:], 100).strip('\x00')
            sidx += 100
        else:
            self.logger.error('RPC_REPLY_SET_NEXT_TRAJECTORY_SEG replied {0}'.format(reply[0]))

    def rpc_reset_traj_reply(self, reply):
        """Process the RPC reply for resetting the trajectory.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.
        """
        if reply[0] != self.RPC_REPLY_RESET_TRAJECTORY:
            self.logger.error('RPC_REPLY_RESET_TRAJECTORY replied {0}'.format(reply[0]))


# ######################## STEPPER PROTOCOL P2 #################################
class Stepper_Protocol_P2(StepperBase):

    def read_firmware_trace(self):
        """Reads firmware trace data and returns it as a list.

        Returns
        -------
        list:
            A list containing the firmware trace data.
        """
        self.trace_buf = []
        self.timestamp.reset() #Timestamp holds state, reset within lock to avoid threading issues
        self.n_trace_read=1
        ts=time.time()
        while ( self.n_trace_read) and time.time()-ts<60.0:
            payload = arr.array('B', [self.RPC_READ_TRACE])
            self.transport.do_pull_rpc_sync(payload, self.rpc_read_firmware_trace_reply)
            time.sleep(.001)
        return self.trace_buf
    def unpack_debug_trace(self,s,unpack_to):
        """Unpacks debug trace data into the `unpack_to` dictionary.

        Parameters
        ----------
        s : bytes.
            The byte string containing debug trace information.
        
        unpack_to : dict.
            A dictionary to store the unpacked information.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx=0
        unpack_to['u8_1']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['u8_2'] = unpack_uint8_t(s[sidx:]);sidx += 1
        unpack_to['f_1'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_2'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_3'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx

    def unpack_print_trace(self,s,unpack_to):
        """Unpacks and prints trace information into the `unpack_to` dictionary.

        Parameters
        ----------
        s : bytes.
            The byte string containing print trace information.
        
        unpack_to : dict.
            The index indicating the position in the byte array after unpacking.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx=0
        line_len=32
        unpack_to['timestamp']=self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['line'] = unpack_string_t(s[sidx:], line_len); sidx += line_len
        unpack_to['x'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx
    def rpc_read_firmware_trace_reply(self, reply):
        """Process the RPC reply for reading the firmware trace.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.
        """
        if len(reply)>0 and reply[0] == self.RPC_REPLY_READ_TRACE:
            self.n_trace_read=reply[1]
            self.trace_buf.append({'id': len(self.trace_buf), 'status': {},'debug':{},'print':{}})
            if reply[2]==self.TRACE_TYPE_STATUS:
                self.trace_buf[-1]['status']= self.status_zero.copy()
                self.unpack_status(reply[3:],unpack_to=self.trace_buf[-1]['status'])
            elif reply[2]==self.TRACE_TYPE_DEBUG:
                self.unpack_debug_trace(reply[3:],unpack_to=self.trace_buf[-1]['debug'])
            elif reply[2]==self.TRACE_TYPE_PRINT:
                self.unpack_print_trace(reply[3:],unpack_to=self.trace_buf[-1]['print'])
            else:
                print('Unrecognized trace type %d'%reply[2])
        else:
            print('Error RPC_REPLY_READ_TRACE')
            self.n_trace_read=0
            self.trace_buf = []
    def enable_firmware_trace(self):
        """Sets the trigger to enable firmware trace.
        """
        self._trigger = self._trigger | self.TRIGGER_ENABLE_TRACE
        self._dirty_trigger = True

    def disable_firmware_trace(self):
        """Sets the trigger to disable firmware trace.
        """
        self._trigger = self._trigger | self.TRIGGER_DISABLE_TRACE
        self._dirty_trigger = True


    def unpack_status(self,s,unpack_to=None):
        """Unpacks status information and updates the `unpack_to` dictionary.

        Parameters
        ----------
        s : bytes.
            A byte string containing packed status data.
        
        unpack_to : dict, optional.
            A dictionary to store the unpacked status information, by default None.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        unpack_to['mode']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['effort_ticks'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['current']=self.effort_ticks_to_current(unpack_to['effort_ticks'])
        unpack_to['effort_pct'] = self.current_to_effort_pct(unpack_to['current'])
        unpack_to['pos'] = unpack_double_t(s[sidx:]);sidx+=8
        unpack_to['vel'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['err'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['diag'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['debug'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['guarded_event'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['waypoint_traj']['setpoint'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['waypoint_traj']['segment_id'] = unpack_uint16_t(s[sidx:]);sidx += 2

        unpack_to['pos_calibrated'] =unpack_to['diag'] & self.DIAG_POS_CALIBRATED > 0
        unpack_to['runstop_on'] =unpack_to['diag'] & self.DIAG_RUNSTOP_ON > 0
        unpack_to['near_pos_setpoint'] =unpack_to['diag'] & self.DIAG_NEAR_POS_SETPOINT > 0
        unpack_to['near_vel_setpoint'] = unpack_to['diag'] & self.DIAG_NEAR_VEL_SETPOINT > 0
        unpack_to['is_moving'] =unpack_to['diag'] & self.DIAG_IS_MOVING > 0
        unpack_to['at_current_limit'] =unpack_to['diag'] & self.DIAG_AT_CURRENT_LIMIT > 0
        unpack_to['is_mg_accelerating'] = unpack_to['diag'] & self.DIAG_IS_MG_ACCELERATING > 0
        unpack_to['is_mg_moving'] =unpack_to['diag'] & self.DIAG_IS_MG_MOVING > 0
        unpack_to['calibration_rcvd'] = unpack_to['diag'] & self.DIAG_CALIBRATION_RCVD > 0
        unpack_to['in_guarded_event'] = unpack_to['diag'] & self.DIAG_IN_GUARDED_EVENT > 0
        unpack_to['in_safety_event'] = unpack_to['diag'] & self.DIAG_IN_SAFETY_EVENT > 0
        unpack_to['waiting_on_sync'] = unpack_to['diag'] & self.DIAG_WAITING_ON_SYNC > 0
        unpack_to['in_sync_mode'] = unpack_to['diag'] & self.DIAG_IN_SYNC_MODE > 0
        unpack_to['trace_on'] = unpack_to['diag'] & self.DIAG_IS_TRACE_ON > 0

        if unpack_to['diag'] & self.DIAG_TRAJ_WAITING_ON_SYNC > 0:
            unpack_to['waypoint_traj']['state']='waiting_on_sync'
        elif unpack_to['diag'] & self.DIAG_TRAJ_ACTIVE > 0:
            unpack_to['waypoint_traj']['state']='active'
        else:
            unpack_to['waypoint_traj']['state']='idle'

        return sidx

# ######################## STEPPER PROTOCOL P3 #################################

class Stepper_Protocol_P3(StepperBase):

    def rpc_start_new_traj_reply(self, reply):
        """Process the RPC reply for starting a new trajectory.

        Parameters
        ----------
        reply : bytes.
            The RPC reply sequence.
        """
        if reply[0] == self.RPC_REPLY_START_NEW_TRAJECTORY:
            self._waypoint_traj_start_success = unpack_uint8_t(reply[1:])
            self._waypoint_traj_start_error_msg = 'SUCCESS' if self._waypoint_traj_start_success else 'FAIL'
        else:
            self.logger.error('RPC_REPLY_START_NEW_TRAJECTORY replied {0}'.format(reply[0]))

    def rpc_set_next_traj_seg_reply(self, reply):
        """Process the RPC reply for setting the next trajectory segment.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.

        """
        if reply[0] == self.RPC_REPLY_SET_NEXT_TRAJECTORY_SEG:
            self._waypoint_traj_set_next_traj_success = unpack_uint8_t(reply[1:])
            self._waypoint_traj_set_next_traj_success = 'SUCCESS' if self._waypoint_traj_start_success else 'FAIL'
        else:
            self.logger.error('RPC_REPLY_SET_NEXT_TRAJECTORY_SEG replied {0}'.format(reply[0]))

    def push_load_test(self):
        """Push a load test payload to the hardware for testing.
        """
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self.load_test_payload
        self.transport.do_push_rpc_sync(payload, self.rpc_load_test_push_reply)

    def pull_load_test(self):
        """Pull the results of a load test.
        """
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_LOAD_TEST_PULL])
        self.transport.do_pull_rpc_sync(payload, self.rpc_load_test_pull_reply)

    def rpc_load_test_push_reply(self, reply):
        """Process the RPC reply for a load push test.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.
        """
        if reply[0] != self.RPC_REPLY_LOAD_TEST_PUSH:
            print('Error RPC_REPLY_LOAD_TEST_PUSH', reply[0])

    def rpc_load_test_pull_reply(self, reply):
        """Process the RPC reply for a load pull test.

        Parameters
        ----------
        reply : bytes.
            The RPC reply byte sequence.
        """
        if reply[0] == self.RPC_REPLY_LOAD_TEST_PULL:
            d = reply[1:]
            for i in range(1024):
                if d[i] != self.load_test_payload[(i + 1) % 1024]:
                    print('Load test pull bad data', d[i], self.load_test_payload[(i + 1) % 1024])
            self.load_test_payload = d
            print('Successful load test pull')
        else:
            print('Error RPC_REPLY_LOAD_TEST_PULL', reply[0])

    def pull_status_aux(self):
        """Pull auxiliary status information.
        """
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_GET_STATUS_AUX])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_aux_reply)

    def unpack_status_aux(self,s):
        """Unpacks and stores auxiliary status information.

        Parameters
        ----------
        s : bytes.
            The byte string containing auxiliary status information.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx = 0
        self.status_aux['cmd_cnt_rpc'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['cmd_cnt_exec'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['cmd_rpc_overflow'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['sync_irq_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        self.status_aux['sync_irq_overflow'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        return sidx

    def rpc_status_aux_reply(self, reply):
        """Handle the reply to the auxiliary status command.

        Parameters
        ----------
        reply : list.
            The reply received.
        """
        if reply[0] == self.RPC_REPLY_STATUS_AUX:
            nr = self.unpack_status_aux(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])


    def unpack_command_reply(self,s):
        """Unpacks and stores command related information 

        Parameters
        ----------
        s : bytes.
            A byte sequence containing command related information.

        Returns
        -------
        int:
            The index indicating the position in the byte array after unpacking.
        """
        sidx = 0
        self.status['ctrl_cycle_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        return sidx

    def rpc_command_reply(self, reply):
        """Process the reply for a command-related information.

        Parameters
        ----------
        reply : list.
            A list containing the reply data.
        """
        if reply[0] == self.RPC_REPLY_COMMAND:
            nr = self.unpack_command_reply(reply[1:])
        else:
            print('Error RPC_REPLY_COMMAND', reply[0])

# ######################## STEPPER #################################
class Stepper(StepperBase):
    """
    API to the Stretch Stepper Board
    """
    def __init__(self,usb, name=None):
        StepperBase.__init__(self,usb,name)
        # Order in descending order so more recent protocols/methods override less recent
        self.supported_protocols = {'p0': (Stepper_Protocol_P0,),
                                    'p1': (Stepper_Protocol_P1,Stepper_Protocol_P0,),
                                    'p2': (Stepper_Protocol_P2,Stepper_Protocol_P1,Stepper_Protocol_P0,),
                                    'p3': (Stepper_Protocol_P3,Stepper_Protocol_P2,Stepper_Protocol_P1,Stepper_Protocol_P0,)}
    
    def expand_protocol_methods(self, protocol_class):
        """Expands protocol methods for use.

        Parameters
        ----------
        protocol_class : class.
            The protocol class from which the methods should be expanded.
        """
        for attr_name, attr_value in protocol_class.__dict__.items():
            if callable(attr_value) and not attr_name.startswith("__"):
                setattr(self, attr_name, attr_value.__get__(self, Stepper))
                
    def startup(self, threaded=False):
        """
        First determine which protocol version the uC firmware is running.
        Based on that version, populates Stepper class with the supported specific Stepper_protocol_P* class methods.
        """
        StepperBase.startup(self, threaded=threaded)
        if self.hw_valid:
            if self.board_info['protocol_version'] in self.supported_protocols:
                protocol_classes = self.supported_protocols[self.board_info['protocol_version']]
                for p in protocol_classes[::-1]:
                    self.expand_protocol_methods(p)
            else:
                if self.board_info['protocol_version'] is None:
                    protocol_msg = """
                                    ----------------
                                    Failure in communications for {0} on startup.
                                    Please power cycle the robot and try again.
                                    ----------------
                                    """.format(self.name)
                else:
                    protocol_msg = """
                    ----------------
                    Firmware protocol mismatch on {0}.
                    Protocol on board is {1}.
                    Valid protocols are: {2}.
                    Disabling device.
                    Please upgrade the firmware and/or version of Stretch Body.
                    ----------------
                    """.format(self.name, self.board_info['protocol_version'], self.supported_protocols.keys())
                self.logger.warning(textwrap.dedent(protocol_msg))
                self.hw_valid = False
                self.transport.stop()

        if self.hw_valid:
            self.enable_safety()
            self._dirty_gains = True
            self.pull_status()
            self.push_command()
        return self.hw_valid

