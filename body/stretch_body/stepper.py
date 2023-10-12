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
    RPC_REPLY_COMMAND = 2
    RPC_GET_STATUS = 3
    RPC_REPLY_STATUS = 4
    RPC_SET_GAINS = 5
    RPC_REPLY_GAINS = 6
    RPC_LOAD_TEST_PUSH = 7
    RPC_REPLY_LOAD_TEST_PUSH = 8
    RPC_SET_TRIGGER = 9
    RPC_REPLY_SET_TRIGGER = 10
    RPC_SET_ENC_CALIB = 11
    RPC_REPLY_ENC_CALIB = 12
    RPC_READ_GAINS_FROM_FLASH = 13
    RPC_REPLY_READ_GAINS_FROM_FLASH = 14
    RPC_SET_MENU_ON = 15
    RPC_REPLY_MENU_ON = 16
    RPC_GET_STEPPER_BOARD_INFO = 17
    RPC_REPLY_STEPPER_BOARD_INFO = 18
    RPC_SET_MOTION_LIMITS = 19
    RPC_REPLY_MOTION_LIMITS = 20
    RPC_SET_NEXT_TRAJECTORY_SEG =21
    RPC_REPLY_SET_NEXT_TRAJECTORY_SEG =22
    RPC_START_NEW_TRAJECTORY =23
    RPC_REPLY_START_NEW_TRAJECTORY =24
    RPC_RESET_TRAJECTORY =25
    RPC_REPLY_RESET_TRAJECTORY =26
    RPC_READ_TRACE =27
    RPC_REPLY_READ_TRACE =28
    RPC_GET_STATUS_AUX = 29
    RPC_REPLY_STATUS_AUX = 30
    RPC_LOAD_TEST_PULL = 31
    RPC_REPLY_LOAD_TEST_PULL = 32

    MODE_SAFETY = 0
    MODE_FREEWHEEL = 1
    MODE_HOLD = 2
    MODE_POS_PID = 3
    MODE_VEL_PID = 4
    MODE_POS_TRAJ = 5
    MODE_VEL_TRAJ = 6
    MODE_CURRENT = 7
    MODE_POS_TRAJ_INCR = 8
    MODE_POS_TRAJ_WAYPOINT=9

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

    DIAG_POS_CALIBRATED = 1  # Has a pos zero RPC been received since powerup
    DIAG_RUNSTOP_ON = 2  # Is controller in runstop mode
    DIAG_NEAR_POS_SETPOINT = 4  # Is pos controller within gains.pAs_d of setpoint
    DIAG_NEAR_VEL_SETPOINT = 8  # Is vel controller within gains.vAs_d of setpoint
    DIAG_IS_MOVING = 16  # Is measured velocity greater than gains.vAs_d
    DIAG_AT_CURRENT_LIMIT = 32  # Is controller current saturated
    DIAG_IS_MG_ACCELERATING = 64  # Is controller motion generator acceleration non-zero
    DIAG_IS_MG_MOVING = 128  # Is controller motion generator velocity non-zero
    DIAG_CALIBRATION_RCVD = 256  # Is calibration table in flash
    DIAG_IN_GUARDED_EVENT = 512  # Guarded event occurred during motion
    DIAG_IN_SAFETY_EVENT = 1024  # Is it forced into safety mode
    DIAG_WAITING_ON_SYNC = 2048  # Command received but no sync yet
    DIAG_TRAJ_ACTIVE     = 4096     # Whether a waypoint trajectory is actively executing
    DIAG_TRAJ_WAITING_ON_SYNC = 8192 # Currently waiting on a sync signal before starting trajectory
    DIAG_IN_SYNC_MODE = 16384        # Currently running in sync mode
    DIAG_IS_TRACE_ON = 32768   #Is trace recording

    CONFIG_SAFETY_HOLD = 1  # Hold position in safety mode? Otherwise freewheel
    CONFIG_ENABLE_RUNSTOP = 2  # Recognize runstop signal?
    CONFIG_ENABLE_SYNC_MODE = 4  # Commands are synchronized from digital trigger
    CONFIG_ENABLE_GUARDED_MODE = 8  # Stops on current threshold
    CONFIG_FLIP_ENCODER_POLARITY = 16
    CONFIG_FLIP_EFFORT_POLARITY = 32
    CONFIG_ENABLE_VEL_WATCHDOG = 64 #Timeout velocity commands

    TRIGGER_MARK_POS = 1
    TRIGGER_RESET_MOTION_GEN = 2
    TRIGGER_BOARD_RESET = 4
    TRIGGER_WRITE_GAINS_TO_FLASH = 8
    TRIGGER_RESET_POS_CALIBRATED = 16
    TRIGGER_POS_CALIBRATED = 32
    TRIGGER_MARK_POS_ON_CONTACT=64
    TRIGGER_ENABLE_TRACE=128
    TRIGGER_DISABLE_TRACE=256

    TRACE_TYPE_STATUS = 0
    TRACE_TYPE_DEBUG = 1
    TRACE_TYPE_PRINT = 2


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
                       'waypoint_traj':{'state':'idle','setpoint':None, 'segment_id':0,},
                       'voltage':0}

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
        return self.status['in_sync_mode'] and self.ts_last_syncd_motion > ts_last_sync

    def push_command(self, exiting=False):
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
        if not self.hw_valid:
            return
        if self._dirty_read_gains_from_flash:
            payload = arr.array('B', [self.RPC_READ_GAINS_FROM_FLASH])
            self.transport.do_pull_rpc_sync(payload, self.rpc_read_gains_from_flash_reply)
            self._dirty_read_gains_from_flash = False
        payload = arr.array('B', [self.RPC_GET_STATUS])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_reply, exiting=exiting)

    async def pull_status_async(self, exiting=False):
        if not self.hw_valid:
            return

        if self._dirty_read_gains_from_flash:
            payload = arr.array('B', [self.RPC_READ_GAINS_FROM_FLASH])
            await self.transport.do_pull_rpc_async(payload, self.rpc_read_gains_from_flash_reply, exiting=exiting)
            self._dirty_read_gains_from_flash = False

        payload = arr.array('B', [self.RPC_GET_STATUS])
        await self.transport.do_pull_rpc_async(payload, self.rpc_status_reply, exiting=exiting)

    def push_load_test(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def pull_load_test(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def pretty_print(self): #Base
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
        if limit_neg!=self.motion_limits[0] or limit_pos!=self.motion_limits[1]:
            #Push out immediately
            self.motion_limits=[limit_neg, limit_pos]
            payload=self.transport.get_empty_payload()
            payload[0] = self.RPC_SET_MOTION_LIMITS
            sidx = self.pack_motion_limits(payload, 1)
            self.transport.do_push_rpc_sync(payload[:sidx], self.rpc_motion_limits_reply)

    def set_gains(self,g):
        self.gains=g.copy()
        self._dirty_gains = True

    def write_gains_to_YAML(self):
        raise DeprecationWarning('This method has been deprecated since v0.3.0')

    def write_gains_to_flash(self):
        self._trigger = self._trigger | self.TRIGGER_WRITE_GAINS_TO_FLASH
        self._dirty_trigger = True

    def read_gains_from_flash(self):
        self._dirty_read_gains_from_flash=True

    def board_reset(self):
        self._trigger = self._trigger | self.TRIGGER_BOARD_RESET
        self._dirty_trigger=True

    def mark_position_on_contact(self,x):
        self._trigger_data = x
        self._trigger = self._trigger | self.TRIGGER_MARK_POS_ON_CONTACT
        self._dirty_trigger=True

    def mark_position(self,x):
        if self.status['mode']!=self.MODE_SAFETY:
            self.logger.warning('Can not mark position. Must be in MODE_SAFETY for %s'%self.usb)
            return
        self._trigger_data=x
        self._trigger = self._trigger | self.TRIGGER_MARK_POS
        self._dirty_trigger=True

    def reset_motion_gen(self):
        self._trigger = self._trigger | self.TRIGGER_RESET_MOTION_GEN
        self._dirty_trigger = True

    def reset_pos_calibrated(self):
        self._trigger = self._trigger | self.TRIGGER_RESET_POS_CALIBRATED
        self._dirty_trigger = True

    def set_pos_calibrated(self):
        self._trigger = self._trigger | self.TRIGGER_POS_CALIBRATED
        self._dirty_trigger = True



    # ###########################################################################
    def enable_safety(self):
            self.set_command(mode=self.MODE_SAFETY)

    def enable_freewheel(self):
            self.set_command(mode=self.MODE_FREEWHEEL)

    def enable_hold(self):
        self.set_command(mode=self.MODE_HOLD)

    def enable_vel_pid(self):
        self.set_command(mode=self.MODE_VEL_PID, v_des=0)

    def enable_pos_pid(self):
        self.set_command(mode=self.MODE_POS_PID, x_des=self.status['pos'])

    def enable_vel_traj(self):
        self.set_command(mode=self.MODE_VEL_TRAJ, v_des=0)

    def enable_pos_traj(self):
        self.set_command(mode=self.MODE_POS_TRAJ, x_des=self.status['pos'])

    def enable_pos_traj_waypoint(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def enable_pos_traj_incr(self):
        self.set_command(mode=self.MODE_POS_TRAJ_INCR, x_des=0)

    def enable_current(self):
        self.set_command(mode=self.MODE_CURRENT, i_des=0)

    def enable_sync_mode(self):
        self.gains['enable_sync_mode'] = 1
        self._dirty_gains = 1

    def disable_sync_mode(self):
        self.gains['enable_sync_mode']=0
        self._dirty_gains=1

    def enable_runstop(self):
        self.gains['enable_runstop'] = 1
        self._dirty_gains = 1

    def disable_runstop(self):
        self.gains['enable_runstop']=0
        self._dirty_gains=1

    def enable_guarded_mode(self):
        self.gains['enable_guarded_mode'] = 1
        self._dirty_gains = 1

    def disable_guarded_mode(self):
        self.gains['enable_guarded_mode'] = 0
        self._dirty_gains = 1



    # ######################################################################
    #Primary interface to controlling the stepper
    #YAML defaults are used if values not provided
    #This allows user to override defaults every control cycle and then easily revert to defaults
    def set_command(self,mode=None, x_des=None, v_des=None, a_des=None,i_des=None, stiffness=None,i_feedforward=None, i_contact_pos=None, i_contact_neg=None  ):
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
        Poll until is moving flag is false
        Return True if success
        Return False if timeout
        """
        ts = time.time()
        self.pull_status()
        while self.status['is_moving'] and time.time() - ts < timeout:
            time.sleep(0.1)
            self.pull_status()
        return not self.status['is_moving']

    def wait_until_at_setpoint(self,timeout=15.0):
        """
        Poll until near setpoint
        Return True if success
        Return False if timeout
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
        if self.board_info['hardware_id']==0: # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info['hardware_id']>=1: # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        effort_ticks = (i_A * 1000.0) / mA_per_tick
        return min(255,max(-255,int(effort_ticks)))

    def effort_ticks_to_current(self,e):
        if self.board_info['hardware_id'] == 0:  # I = Vref / (10 * R), Rs = 0.10 Ohm, Vref = 3.3V -->3.3A
            mA_per_tick = (3300 / 255) / (10 * 0.1)
        if self.board_info['hardware_id'] >= 1:  # I = Vref / (5 * R), Rs = 0.150 Ohm, Vref = 3.3V -->4.4A
            mA_per_tick = (3300 / 255) / (5 * 0.15)
        return e * mA_per_tick / 1000.0

    # Effort_pct is defined as a percentage of the maximum allowable motor winding current
    # Range is -100.0 to 100.0
    def current_to_effort_pct(self,i_A):
        if i_A>0:
            return 100*max(0.0,min(1.0,i_A/self.gains['iMax_pos']))
        else:
            return 100*min(0.0, max(-1.0, i_A/ abs(self.gains['iMax_neg'])))

    def effort_pct_to_current(self,e_pct):
        if e_pct>0:
            return min(1.0,e_pct/100.0)*self.gains['iMax_pos']
        else:
            return max(-1.0, e_pct/100.0) * abs(self.gains['iMax_neg'])

    def current_to_torque(self,i):
        raise DeprecationWarning('Method current_to_torque has been deprecated since v0.3.5')

    def torque_to_current(self, tq):
        raise DeprecationWarning('Method torque_to_current has been deprecated since v0.3.5')


        # ####################### Encoder Calibration ######################

    def get_chip_id(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        cid = self.menu_transaction(b'b', do_print=False)[0][:-2]
        self.turn_rpc_interface_on()
        time.sleep(0.5)
        return cid.decode('utf-8')

    def read_encoder_calibration_from_YAML(self):
        device_name=self.usb[5:]
        sn=self.robot_params[device_name]['serial_no']
        fn='calibration_steppers/'+device_name+'_'+sn+'.yaml'
        enc_data=read_fleet_yaml(fn)
        return enc_data

    def write_encoder_calibration_to_YAML(self,data,filename=None, fleet_dir=None):
        device_name = self.usb[5:]
        if filename is None:
            sn = self.robot_params[device_name]['serial_no']
            filename = 'calibration_steppers/'+device_name + '_' + sn + '.yaml'
        print('Writing encoder calibration: %s'%filename)
        write_fleet_yaml(filename,data,fleet_dir=fleet_dir)

    def read_encoder_calibration_from_flash(self):
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
        if reply[0] != self.RPC_REPLY_ENC_CALIB:
            self.logger.debug('Error RPC_REPLY_ENC_CALIB', reply[0])

    # ######################Menu Interface ################################3


    def turn_rpc_interface_on(self):
        self.menu_transaction(b'zyx')


    def turn_menu_interface_on(self):
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_SET_MENU_ON])
        self.transport.do_push_rpc_sync(payload, self.rpc_menu_on_reply)


    def print_menu(self):
        self.menu_transaction(b'm')

    def menu_transaction(self,x,do_print=True):
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
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def set_next_trajectory_segment(self, next_segment):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def stop_waypoint_trajectory(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    # ################Transport Callbacks #####################
    def unpack_board_info(self,s):
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

    def unpack_status(self,s,unpack_to=None): #Base
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def unpack_gains(self,s): #Base
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
        pack_float_t(s, sidx, self.motion_limits[0])
        sidx += 4
        pack_float_t(s, sidx, self.motion_limits[1])
        sidx += 4
        return sidx

    def pack_command(self, s, sidx):
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

    def pack_gains(self,s,sidx): #Base
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
        pack_uint32_t(s, sidx, self._trigger)
        sidx += 4
        pack_float_t(s, sidx, self._trigger_data)
        sidx += 4
        return sidx
    def pack_trajectory_segment(self, s, sidx):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def rpc_board_info_reply(self, reply):
        if reply[0] == self.RPC_REPLY_STEPPER_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            print('Error RPC_REPLY_STEPPER_BOARD_INFO', reply[0])

    def rpc_gains_reply(self, reply):
        if reply[0] != self.RPC_REPLY_GAINS:
            print('Error RPC_REPLY_GAINS', reply[0])

    def rpc_trigger_reply(self, reply):
        if reply[0] != self.RPC_REPLY_SET_TRIGGER:
            print('Error RPC_REPLY_SET_TRIGGER', reply[0])

    def rpc_command_reply(self, reply):
        if reply[0] != self.RPC_REPLY_COMMAND:
            print('Error RPC_REPLY_COMMAND', reply[0])

    def rpc_motion_limits_reply(self, reply):
        if reply[0] != self.RPC_REPLY_MOTION_LIMITS:
            print('Error RPC_REPLY_MOTION_LIMITS', reply[0])

    def rpc_menu_on_reply(self, reply):
        if reply[0] != self.RPC_REPLY_MENU_ON:
            print('Error RPC_REPLY_MENU_ON', reply[0])

    def rpc_status_reply(self, reply):
        if reply[0] == self.RPC_REPLY_STATUS:
            nr = self.unpack_status(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])

    def rpc_read_gains_from_flash_reply(self, reply):
        if reply[0] == self.RPC_REPLY_READ_GAINS_FROM_FLASH:
            nr = self.unpack_gains(reply[1:])
        else:
            print('Error RPC_REPLY_READ_GAINS_FROM_FLASH', reply[0])


    def rpc_start_new_traj_reply(self, reply):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def rpc_set_next_traj_seg_reply(self, reply):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))

    def rpc_reset_traj_reply(self, reply):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
            .format(self.board_info['protocol_version']))


    def rpc_read_firmware_trace_reply(self, reply):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def enable_firmware_trace(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

    def disable_firmware_trace(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))


    def pull_status_aux(self):
        raise NotImplementedError('This method not supported for firmware on protocol {0}.'
                                  .format(self.board_info['protocol_version']))

# ######################## STEPPER PROTOCOL PO #################################

class Stepper_Protocol_P0(StepperBase):
    def unpack_status(self,s,unpack_to=None): #P0
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        unpack_to['mode']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['effort_ticks'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['current']=self.effort_ticks_to_current(unpack_to['effort_ticks'])
        unpack_to['effort_pct']=self.current_to_effort_pct(unpack_to['current'])
        unpack_to['pos'] = unpack_double_t(s[sidx:]);sidx+=8
        unpack_to['vel'] = unpack_float_t(s[sidx:]);sidx+=4
        unpack_to['err'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['diag'] = unpack_uint32_t(s[sidx:]);sidx += 4
        unpack_to['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
        unpack_to['debug'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['guarded_event'] = unpack_uint32_t(s[sidx:]);sidx += 4

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
        return sidx

    def pretty_print(self): #P0
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
        if self.board_info['hardware_id']>=3:
            print('Voltage (V)',self.status['voltage'])
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

    def unpack_status(self,s,unpack_to=None): #P1
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

    def pretty_print(self): #P1
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
        if self.board_info['hardware_id'] >= 3:
            print('Voltage (V)', self.status['voltage'])
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
        self.set_command(mode=self.MODE_POS_TRAJ_WAYPOINT)

    def start_waypoint_trajectory(self, first_segment):
        """Starts execution of a waypoint trajectory on hardware

        Parameters
        ----------
        first_segment : list
            List of length eight, structured like [duration_s, a0, a1, a2, a3, a4, a5, segment_id].
            The hardware begins executing this first segment of a spline. The segment's duration and
            six coefficients (a0-a5) fill the first seven elements of the list. A segment ID, always
            2 for the first segment, fills the last element of the list.

        Returns
        -------
        bool
            True if uC successfully initiated a new trajectory
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
        """Sets the next segment for the hardware to execute

        This method is generally called multiple times while the prior segment is executing. This
        provides the hardware with the next segment to gracefully transition across the entire spline,
        while allowing users to preempt or modify the future trajectory in real time.

        This method will return False if there is not already an segment executing on the uC

        Parameters
        ----------
        next_segment : list
            List of length eight, structured like [duration_s, a0, a1, a2, a3, a4, a5, segment_id].
            Duration and six coefficients fill the first seven elements of the list. Generally, the
            coefficients are calculated to smoothly transition across a spline. The segment ID, always
            1 higher than the prior segment's ID, fills the last element of the list.

        Returns
        -------
        bool
            True if uC successfully queued next trajectory
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
        """Stops execution of the waypoint trajectory running in hardware
        """
        self._waypoint_ts = None
        payload = arr.array('B', [self.RPC_RESET_TRAJECTORY])
        self.transport.do_push_rpc_sync(payload, self.rpc_reset_traj_reply)

    def pack_trajectory_segment(self, s, sidx):
        for i in range(7):
            pack_float_t(s, sidx, self._waypoint_traj_segment[i]);
            sidx += 4
        pack_uint8_t(s, sidx, self._waypoint_traj_segment[7]);
        sidx += 1
        return sidx

    def rpc_start_new_traj_reply(self, reply):
        if reply[0] == self.RPC_REPLY_START_NEW_TRAJECTORY:
            sidx = 1
            self._waypoint_traj_start_success = unpack_uint8_t(reply[sidx:]);
            sidx += 1
            self._waypoint_traj_start_error_msg = unpack_string_t(reply[sidx:], 100).strip('\x00')
            sidx += 100
        else:
            self.logger.error('RPC_REPLY_START_NEW_TRAJECTORY replied {0}'.format(reply[0]))

    def rpc_set_next_traj_seg_reply(self, reply):
        if reply[0] == self.RPC_REPLY_SET_NEXT_TRAJECTORY_SEG:
            sidx = 1
            self._waypoint_traj_set_next_traj_success = unpack_uint8_t(reply[sidx:]);
            sidx += 1
            self._waypoint_traj_set_next_error_msg = unpack_string_t(reply[sidx:], 100).strip('\x00')
            sidx += 100
        else:
            self.logger.error('RPC_REPLY_SET_NEXT_TRAJECTORY_SEG replied {0}'.format(reply[0]))

    def rpc_reset_traj_reply(self, reply):
        if reply[0] != self.RPC_REPLY_RESET_TRAJECTORY:
            self.logger.error('RPC_REPLY_RESET_TRAJECTORY replied {0}'.format(reply[0]))


# ######################## STEPPER PROTOCOL P2 #################################
class Stepper_Protocol_P2(StepperBase):

    def read_firmware_trace(self):
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
        sidx=0
        unpack_to['u8_1']=unpack_uint8_t(s[sidx:]);sidx+=1
        unpack_to['u8_2'] = unpack_uint8_t(s[sidx:]);sidx += 1
        unpack_to['f_1'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_2'] = unpack_float_t(s[sidx:]);sidx += 4
        unpack_to['f_3'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx

    def unpack_print_trace(self,s,unpack_to):
        sidx=0
        line_len=32
        unpack_to['timestamp']=self.timestamp.set(unpack_uint64_t(s[sidx:]));sidx += 8
        unpack_to['line'] = unpack_string_t(s[sidx:], line_len); sidx += line_len
        unpack_to['x'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx
    def rpc_read_firmware_trace_reply(self, reply):
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
        self._trigger = self._trigger | self.TRIGGER_ENABLE_TRACE
        self._dirty_trigger = True

    def disable_firmware_trace(self):
        self._trigger = self._trigger | self.TRIGGER_DISABLE_TRACE
        self._dirty_trigger = True


    def unpack_status(self,s,unpack_to=None): #P2
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
        if reply[0] == self.RPC_REPLY_START_NEW_TRAJECTORY:
            self._waypoint_traj_start_success = unpack_uint8_t(reply[1:])
            self._waypoint_traj_start_error_msg = 'SUCCESS' if self._waypoint_traj_start_success else 'FAIL'
        else:
            self.logger.error('RPC_REPLY_START_NEW_TRAJECTORY replied {0}'.format(reply[0]))

    def rpc_set_next_traj_seg_reply(self, reply):
        if reply[0] == self.RPC_REPLY_SET_NEXT_TRAJECTORY_SEG:
            self._waypoint_traj_set_next_traj_success = unpack_uint8_t(reply[1:])
            self._waypoint_traj_set_next_traj_success = 'SUCCESS' if self._waypoint_traj_start_success else 'FAIL'
        else:
            self.logger.error('RPC_REPLY_SET_NEXT_TRAJECTORY_SEG replied {0}'.format(reply[0]))

    def push_load_test(self):
        if not self.hw_valid:
            return
        payload = self.transport.get_empty_payload()
        payload[0] = self.RPC_LOAD_TEST_PUSH
        payload[1:] = self.load_test_payload
        self.transport.do_push_rpc_sync(payload, self.rpc_load_test_push_reply)

    def pull_load_test(self):
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_LOAD_TEST_PULL])
        self.transport.do_pull_rpc_sync(payload, self.rpc_load_test_pull_reply)

    def rpc_load_test_push_reply(self, reply):
        if reply[0] != self.RPC_REPLY_LOAD_TEST_PUSH:
            print('Error RPC_REPLY_LOAD_TEST_PUSH', reply[0])

    def rpc_load_test_pull_reply(self, reply):
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
        if not self.hw_valid:
            return
        payload = arr.array('B',[self.RPC_GET_STATUS_AUX])
        self.transport.do_pull_rpc_sync(payload, self.rpc_status_aux_reply)

    def unpack_status_aux(self,s):
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
        if reply[0] == self.RPC_REPLY_STATUS_AUX:
            nr = self.unpack_status_aux(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])


    def unpack_command_reply(self,s):
        sidx = 0
        self.status['ctrl_cycle_cnt'] = unpack_uint16_t(s[sidx:])
        sidx += 2
        return sidx

    def rpc_command_reply(self, reply):
        if reply[0] == self.RPC_REPLY_COMMAND:
            nr = self.unpack_command_reply(reply[1:])
        else:
            print('Error RPC_REPLY_COMMAND', reply[0])


# ######################## STEPPER PROTOCOL P4 #################################
class Stepper_Protocol_P4(StepperBase):
    def unpack_status(self,s,unpack_to=None): #P4
        if unpack_to is None:
            unpack_to=self.status
        sidx=0
        sidx=sidx+Stepper_Protocol_P2.unpack_status(self,s,unpack_to)
        unpack_to['voltage']=self.get_voltage(unpack_float_t(s[sidx:]));sidx+=4
        return sidx

    def get_voltage(self,raw):
        raw_to_V = 20.0/1024 #10bit adc, 0-20V per 0-3.3V reading
        return raw*raw_to_V

    def pack_gains(self,s,sidx):
        sidx=sidx+Stepper_Protocol_P3.pack_gains(self,s,sidx)
        pack_float_t(s, sidx, self.gains['voltage_LPF']);sidx += 4
        return sidx

    def unpack_gains(self,s):
        sidx=Stepper_Protocol_P3.unpack_gains(self,s)
        self.gains_flash['voltage_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
        return sidx
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
                                    'p3': (Stepper_Protocol_P3,Stepper_Protocol_P2,Stepper_Protocol_P1,Stepper_Protocol_P0,),
                                    'p4': (Stepper_Protocol_P4,Stepper_Protocol_P3,Stepper_Protocol_P2,Stepper_Protocol_P1,Stepper_Protocol_P0,)}
    
    def expand_protocol_methods(self, protocol_class):
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

