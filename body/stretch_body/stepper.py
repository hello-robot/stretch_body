from __future__ import print_function
from stretch_body.transport import *
from stretch_body.device import Device
from stretch_body.hello_utils import *
import textwrap
import threading
import sys

RPC_SET_COMMAND = 1
RPC_REPLY_COMMAND = 2
RPC_GET_STATUS = 3
RPC_REPLY_STATUS = 4
RPC_SET_GAINS = 5
RPC_REPLY_GAINS = 6
RPC_LOAD_TEST =7
RPC_REPLY_LOAD_TEST =8
RPC_SET_TRIGGER = 9
RPC_REPLY_SET_TRIGGER =10
RPC_SET_ENC_CALIB =11
RPC_REPLY_ENC_CALIB =12
RPC_READ_GAINS_FROM_FLASH =13
RPC_REPLY_READ_GAINS_FROM_FLASH =14
RPC_SET_MENU_ON =15
RPC_REPLY_MENU_ON=16
RPC_GET_STEPPER_BOARD_INFO =17
RPC_REPLY_STEPPER_BOARD_INFO =18
RPC_SET_MOTION_LIMITS=19
RPC_REPLY_MOTION_LIMITS =20

MODE_SAFETY=0
MODE_FREEWHEEL=1
MODE_HOLD=2
MODE_POS_PID=3
MODE_VEL_PID=4
MODE_POS_TRAJ=5
MODE_VEL_TRAJ=6
MODE_CURRENT=7
MODE_POS_TRAJ_INCR=8

DIAG_POS_CALIBRATED =1         #Has a pos zero RPC been recieved since powerup
DIAG_RUNSTOP_ON =2             #Is controller in runstop mode
DIAG_NEAR_POS_SETPOINT =4      #Is pos controller within gains.pAs_d of setpoint
DIAG_NEAR_VEL_SETPOINT =8     #Is vel controller within gains.vAs_d of setpoint
DIAG_IS_MOVING =16             #Is measured velocity greater than gains.vAs_d
DIAG_AT_CURRENT_LIMIT =32      #Is controller current saturated
DIAG_IS_MG_ACCELERATING =64   #Is controler motion generator acceleration non-zero
DIAG_IS_MG_MOVING =128         #Is controller motion generator velocity non-zero
DIAG_CALIBRATION_RCVD = 256     #Is calibration table in flash
DIAG_IN_GUARDED_EVENT = 512     # Guarded event occured during motion
DIAG_IN_SAFETY_EVENT = 1024      #Is it forced into safety mode
DIAG_WAITING_ON_SYNC = 2048     #Command received but no sync yet

CONFIG_SAFETY_HOLD =1           #Hold position in safety mode? Otherwise freewheel
CONFIG_ENABLE_RUNSTOP =2        #Recognize runstop signal?
CONFIG_ENABLE_SYNC_MODE =4      #Commands are synchronized from digital trigger
CONFIG_ENABLE_GUARDED_MODE=8    #Stops on current threshold
CONFIG_FLIP_ENCODER_POLARITY=16
CONFIG_FLIP_EFFORT_POLARITY=32

TRIGGER_MARK_POS = 1
TRIGGER_RESET_MOTION_GEN = 2
TRIGGER_BOARD_RESET = 4
TRIGGER_WRITE_GAINS_TO_FLASH = 8
TRIGGER_RESET_POS_CALIBRATED = 16
TRIGGER_POS_CALIBRATED = 32


class Stepper(Device):
    """
    API to the Stretch RE1 stepper board
    """
    def __init__(self, usb):
        name = usb[5:]
        Device.__init__(self, name)
        self.usb=usb
        self.lock=threading.RLock()
        self.transport = Transport(usb=self.usb, logger=self.logger)
        self._command = {'mode':0, 'x_des':0,'v_des':0,'a_des':0,'stiffness':1.0,'i_feedforward':0.0,'i_contact_pos':0,'i_contact_neg':0,'incr_trigger':0}
        self.status = {'mode': 0, 'effort': 0, 'current':0,'pos': 0, 'vel': 0, 'err':0,'diag': 0,'timestamp': 0, 'debug':0,'guarded_event':0,
                       'transport': self.transport.status,'pos_calibrated':0,'runstop_on':0,'near_pos_setpoint':0,'near_vel_setpoint':0,
                       'is_moving':0,'is_moving_filtered':0,'at_current_limit':0,'is_mg_accelerating':0,'is_mg_moving':0,'calibration_rcvd': 0,'in_guarded_event':0,
                       'in_safety_event':0,'waiting_on_sync':0}
        self.board_info={'board_version':None, 'firmware_version':None,'protocol_version':None}
        self.mode_names={MODE_SAFETY:'MODE_SAFETY', MODE_FREEWHEEL:'MODE_FREEWHEEL',MODE_HOLD:'MODE_HOLD',MODE_POS_PID:'MODE_POS_PID',
                         MODE_VEL_PID:'MODE_VEL_PID',MODE_POS_TRAJ:'MODE_POS_TRAJ',MODE_VEL_TRAJ:'MODE_VEL_TRAJ',MODE_CURRENT:'MODE_CURRENT', MODE_POS_TRAJ_INCR:'MODE_POS_TRAJ_INCR'}
        self.motion_limits=[0,0]
        self.is_moving_history = [False] * 10

        self._dirty_command = False
        self._dirty_gains = False
        self._dirty_trigger = False
        self._dirty_read_gains_from_flash=False
        self._dirty_load_test=False
        self._trigger=0
        self._trigger_data=0
        self.load_test_payload = arr.array('B', range(256)) * 4
        self.valid_firmware_protocol='p0'
        self.hw_valid=False
        self.gains = self.params['gains'].copy()

    # ###########  Device Methods #############
    def startup(self):
        with self.lock:
            self.hw_valid=self.transport.startup()
            if self.hw_valid:
                #Pull board info
                self.transport.payload_out[0] = RPC_GET_STEPPER_BOARD_INFO
                self.transport.queue_rpc(1, self.rpc_board_info_reply)
                self.transport.step(exiting=False)
                #Check that protocol matches
                if not(self.valid_firmware_protocol == self.board_info['protocol_version']):
                    protocol_msg = """
                    ----------------
                    Firmware protocol mismatch on {0}.
                    Protocol on board is {1}.
                    Valid protocol is: {2}.
                    Disabling device.
                    Please upgrade the firmware and/or version of Stretch Body.
                    ----------------
                    """.format(self.name, self.board_info['protocol_version'], self.valid_firmware_protocol)
                    self.logger.warning(textwrap.dedent(protocol_msg))
                    self.hw_valid=False
                    self.transport.stop()
            if self.hw_valid:
                self.enable_safety()
                self._dirty_gains = True
                self.pull_status()
                self.push_command()
                return True
            return False


    #Configure control mode prior to calling this on process shutdown (or default to freewheel)
    def stop(self):
        if not self.hw_valid:
            return
        with self.lock:
            self.logger.debug('Shutting down Stepper on: ' + self.usb)
            self.enable_safety()
            self.push_command(exiting=True)
            self.transport.stop()
            self.hw_valid = False

    def push_command(self,exiting=False):
        if not self.hw_valid:
            return
        with self.lock:
            if self._dirty_load_test:
                self.transport.payload_out[0] = RPC_LOAD_TEST
                self.transport.payload_out[1:] = self.load_test_payload
                self.transport.queue_rpc2(1024 + 1, self.rpc_load_test_reply)
                self._dirty_load_test=False

            if self._dirty_trigger:
                self.transport.payload_out[0] = RPC_SET_TRIGGER
                sidx = self.pack_trigger(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, self.rpc_trigger_reply)
                self._trigger=0
                self._dirty_trigger = False

            if self._dirty_gains:
                self.transport.payload_out[0] = RPC_SET_GAINS
                sidx = self.pack_gains(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, self.rpc_gains_reply)
                self._dirty_gains=False

            if self._dirty_command:
                self.transport.payload_out[0] = RPC_SET_COMMAND
                sidx = self.pack_command(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, self.rpc_command_reply)
                self._dirty_command=False
            self.transport.step2(exiting=exiting)


    def pull_status(self, exiting=False):
        if not self.hw_valid:
            return
        with self.lock:
            if self._dirty_read_gains_from_flash:
                self.transport.payload_out[0] = RPC_READ_GAINS_FROM_FLASH
                self.transport.queue_rpc(1, self.rpc_read_gains_from_flash_reply)
                self._dirty_read_gains_from_flash = False

            # Queue Status RPC
            self.transport.payload_out[0] = RPC_GET_STATUS
            sidx = 1
            self.transport.queue_rpc(sidx, self.rpc_status_reply)
            self.transport.step(exiting=exiting)

    def pretty_print(self):
        print('-----------')
        print('Mode',self.mode_names[self.status['mode']])
        print('x_des (rad)', self._command['x_des'], '(deg)',rad_to_deg(self._command['x_des']))
        print('v_des (rad)', self._command['v_des'], '(deg)',rad_to_deg(self._command['v_des']))
        print('a_des (rad)', self._command['a_des'], '(deg)',rad_to_deg(self._command['a_des']))
        print('Stiffness',self._command['stiffness'])
        print('Feedforward', self._command['i_feedforward'])
        print('Pos (rad)', self.status['pos'], '(deg)',rad_to_deg(self.status['pos']))
        print('Vel (rad/s)', self.status['vel'], '(deg)',rad_to_deg(self.status['vel']))
        print('Effort', self.status['effort'])
        print('Current (A)', self.status['current'])
        print('Error (deg)', rad_to_deg(self.status['err']))
        print('Debug', self.status['debug'])
        print('Guarded Events:', self.status['guarded_event'])
        print('Diag', self.status['diag'])
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
        print('Timestamp', self.status['timestamp'])
        print('Read error', self.transport.status['read_error'])
        print('Board version:', self.board_info['board_version'])
        print('Firmware version:', self.board_info['firmware_version'])

    def step_sentry(self, robot):
        if self.hw_valid and self.robot_params['robot_sentry']['stepper_is_moving_filter']:
            self.is_moving_history.pop(0)
            self.is_moving_history.append(self.status['is_moving'])
            self.status['is_moving_filtered'] = max(set(self.is_moving_history), key=self.is_moving_history.count)
    # ###########################################################################
    # ###########################################################################

    def set_load_test(self):
        self._dirty_load_test=True

    def set_motion_limits(self,limit_neg, limit_pos):
        with self.lock:
            if limit_neg!=self.motion_limits[0] or limit_pos!=self.motion_limits[1]:
                #Push out immediately
                self.motion_limits=[limit_neg, limit_pos]
                self.transport.payload_out[0] = RPC_SET_MOTION_LIMITS
                sidx = self.pack_motion_limits(self.transport.payload_out, 1)
                self.transport.queue_rpc2(sidx, self.rpc_motion_limits_reply)
                self.transport.step2()

    def set_gains(self,g):
        with self.lock:
            self.gains=g.copy()
            self._dirty_gains = True

    def write_gains_to_YAML(self):
        self.params['gains']=self.gains.copy()
        self.write_device_params(self.name,self.params)

    def write_gains_to_flash(self):
        with self.lock:
            self._trigger = self._trigger | TRIGGER_WRITE_GAINS_TO_FLASH
            self._dirty_trigger = True

    def read_gains_from_flash(self):
        self._dirty_read_gains_from_flash=True

    def board_reset(self):
        with self.lock:
            self._trigger = self._trigger | TRIGGER_BOARD_RESET
            self._dirty_trigger=True

    def mark_position(self,x):
        if self.status['mode']!=MODE_SAFETY:
            self.logger.warning('Can not mark position. Must be in MODE_SAFETY for',self.usb)
            return

        with self.lock:
            self._trigger_data=x
            self._trigger = self._trigger | TRIGGER_MARK_POS
            self._dirty_trigger=True

    def reset_motion_gen(self):
        with self.lock:
            self._trigger = self._trigger | TRIGGER_RESET_MOTION_GEN
            self._dirty_trigger = True

    def reset_pos_calibrated(self):
        with self.lock:
            self._trigger = self._trigger | TRIGGER_RESET_POS_CALIBRATED
            self._dirty_trigger = True

    def set_pos_calibrated(self):
        with self.lock:
            self._trigger = self._trigger | TRIGGER_POS_CALIBRATED
            self._dirty_trigger = True

    # ###########################################################################
    def enable_safety(self):
            self.set_command(mode=MODE_SAFETY)

    def enable_freewheel(self):
            self.set_command(mode=MODE_FREEWHEEL)

    def enable_hold(self):
        self.set_command(mode=MODE_HOLD)

    def enable_vel_pid(self):
        self.set_command(mode=MODE_VEL_PID, v_des=0)

    def enable_pos_pid(self):
        self.set_command(mode=MODE_POS_PID, x_des=self.status['pos'])

    def enable_vel_traj(self):
        self.set_command(mode=MODE_VEL_TRAJ, v_des=0)

    def enable_pos_traj(self):
        self.set_command(mode=MODE_POS_TRAJ, x_des=self.status['pos'])

    def enable_pos_traj_incr(self):
        self.set_command(mode=MODE_POS_TRAJ_INCR, x_des=0)

    def enable_current(self):
        self.set_command(mode=MODE_CURRENT, i_des=0)

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

    #Primary interface to controlling the stepper
    #YAML defaults are used if values not provided
    #This allows user to override defaults every control cycle and then easily revert to defaults
    def set_command(self,mode=None, x_des=None, v_des=None, a_des=None,i_des=None, stiffness=None,i_feedforward=None, i_contact_pos=None, i_contact_neg=None  ):
        with self.lock:
            if mode is not None:
                self._command['mode'] = mode

            if x_des is not None:
                self._command['x_des'] = x_des
                if self._command['mode'] == MODE_POS_TRAJ_INCR:
                    self._command['incr_trigger'] = (self._command['incr_trigger']+1)%255

            if v_des is not None:
                self._command['v_des'] = v_des
            else:
                if mode == MODE_VEL_PID or mode == MODE_VEL_TRAJ:
                    self._command['v_des'] = 0
                else:
                    self._command['v_des'] = self.params['motion']['vel']

            if a_des is not None:
                self._command['a_des'] = a_des
            else:
                self._command['a_des'] = self.params['motion']['accel']

            if stiffness is not None:
                self._command['stiffness'] = max(0, min(1.0, stiffness))
            else:
                self._command['stiffness'] =1

            if i_feedforward is not None:
                self._command['i_feedforward'] = i_feedforward
            else:
                self._command['i_feedforward'] = 0

            if i_des is not None and mode == MODE_CURRENT:
                self._command['i_feedforward'] =i_des


            if i_contact_pos is not None:
                self._command['i_contact_pos'] = i_contact_pos
            else:
                self._command['i_contact_pos']=self.params['gains']['i_contact_pos']

            if i_contact_neg is not None:
                self._command['i_contact_neg'] = i_contact_neg
            else:
                self._command['i_contact_neg'] = self.params['gains']['i_contact_neg']
            #print(time.time(),self._command['x_des'],self._command['incr_trigger'],self._command['v_des'],self._command['a_des'])
            self._dirty_command=True


    def wait_until_at_setpoint(self,timeout=15.0):
        ts = time.time()
        self.pull_status()
        while not self.status['near_pos_setpoint'] and time.time() - ts < timeout:
            time.sleep(0.1)
            self.pull_status()

    def current_to_effort(self,i_A):
        mA_per_tick = (3300 / 255) / (10 * 0.1)
        effort = (i_A * 1000.0) / mA_per_tick
        return min(255,max(-255,int(effort)))

    def effort_to_current(self,e):
        mA_per_tick = (3300 / 255) / (10 * 0.1)
        return e * mA_per_tick / 1000.0

    #Very rough, not accounting for motor dynamics / temp / etc
    def current_to_torque(self,i):
        k_t = self.params['holding_torque']/self.params['rated_current'] #N-m/A
        return i*k_t

    def torque_to_current(self, tq):
        k_t = self.params['holding_torque'] / self.params['rated_current']  # N-m/A
        return tq/k_t

        # ####################### Encoder Calibration ######################

    def get_chip_id(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        cid = self.menu_transaction('b', do_print=False)[0][:-2]
        self.turn_rpc_interface_on()
        time.sleep(0.5)
        return cid

    def read_encoder_calibration_from_YAML(self):
        device_name=self.usb[5:]
        sn=self.robot_params[device_name]['serial_no']
        fn='calibration_steppers/'+device_name+'_'+sn+'.yaml'
        enc_data=read_fleet_yaml(fn)
        return enc_data

    def write_encoder_calibration_to_YAML(self,data):
        device_name = self.usb[5:]
        sn = self.robot_params[device_name]['serial_no']
        fn = 'calibration_steppers/'+device_name + '_' + sn + '.yaml'
        write_fleet_yaml(fn,data)

    def read_encoder_calibration_from_flash(self):
        self.turn_menu_interface_on()
        time.sleep(0.5)
        self.logger.debug('Reading encoder calibration...')
        e = self.menu_transaction('q',do_print=False)[19]
        self.turn_rpc_interface_on()
        self.push_command()
        self.logger.debug('Reseting board')
        self.board_reset()
        self.push_command()
        e = e[:-4]  # We now have string of floats, convert to list of floats
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
            self.logger.debug('Bad encoder data')
        else:
            self.logger.debug('Writing encoder calibration...')
            for p in range(256):
                if p%10==0:
                    sys.stdout.write('.')
                    sys.stdout.flush()
                self.transport.payload_out[0] = RPC_SET_ENC_CALIB
                self.transport.payload_out[1] = p
                sidx=2
                for i in range(64):
                    pack_float_t(self.transport.payload_out, sidx, data[p*64+i])
                    sidx += 4
                # self.logger.debug('Sending encoder calibration rpc of size',sidx)
                self.transport.queue_rpc(sidx, self.rpc_enc_calib_reply)
                self.transport.step()
            self.logger.debug('')
    def rpc_enc_calib_reply(self,reply):
        if reply[0] != RPC_REPLY_ENC_CALIB:
            self.logger.debug('Error RPC_REPLY_ENC_CALIB', reply[0])

    # ######################Menu Inteface ################################3


    def turn_rpc_interface_on(self):
        with self.lock:
            self.menu_transaction('zyx')


    def turn_menu_interface_on(self):
        if not self.hw_valid:
            return
        with self.lock:
            # Run immediately rather than queue
            self.transport.payload_out[0] = RPC_SET_MENU_ON
            self.transport.queue_rpc(1, self.rpc_menu_on_reply)
            self.transport.step()

    def print_menu(self):
        with self.lock:
            self.menu_transaction('m')

    def menu_transaction(self,x,do_print=True):
        if not self.hw_valid:
            return
        with self.lock:
            self.transport.ser.write(x)
            time.sleep(0.1)
            reply=[]
            while self.transport.ser.inWaiting():
                r=self.transport.ser.readline()
                if do_print:
                    print(r, end=' ')
                reply.append(r)
            return reply


    # ################Transport Callbacks #####################
    def unpack_board_info(self,s):
        with self.lock:
            sidx=0
            self.board_info['board_version'] = unpack_string_t(s[sidx:], 20).strip('\x00')
            sidx += 20
            self.board_info['firmware_version'] = unpack_string_t(s[sidx:], 20).strip('\x00')
            self.board_info['protocol_version'] = self.board_info['firmware_version'][self.board_info['firmware_version'].rfind('p'):]
            sidx += 20
            return sidx

    def unpack_status(self,s):
        with self.lock:
            sidx=0
            self.status['mode']=unpack_uint8_t(s[sidx:]);sidx+=1
            self.status['effort'] = unpack_float_t(s[sidx:]);sidx+=4
            self.status['current']=self.effort_to_current(self.status['effort'])
            self.status['pos'] = unpack_double_t(s[sidx:]);sidx+=8
            self.status['vel'] = unpack_float_t(s[sidx:]);sidx+=4
            self.status['err'] = unpack_float_t(s[sidx:]);sidx += 4
            self.status['diag'] = unpack_uint32_t(s[sidx:]);sidx += 4
            self.status['timestamp'] = self.timestamp.set(unpack_uint32_t(s[sidx:]));sidx += 4
            self.status['debug'] = unpack_float_t(s[sidx:]);sidx += 4
            self.status['guarded_event'] = unpack_uint32_t(s[sidx:]);sidx += 4
            self.status['pos_calibrated'] =self.status['diag'] & DIAG_POS_CALIBRATED > 0
            self.status['runstop_on'] =self.status['diag'] & DIAG_RUNSTOP_ON > 0
            self.status['near_pos_setpoint'] =self.status['diag'] & DIAG_NEAR_POS_SETPOINT > 0
            self.status['near_vel_setpoint'] = self.status['diag'] & DIAG_NEAR_VEL_SETPOINT > 0
            self.status['is_moving'] =self.status['diag'] & DIAG_IS_MOVING > 0
            self.status['at_current_limit'] =self.status['diag'] & DIAG_AT_CURRENT_LIMIT > 0
            self.status['is_mg_accelerating'] = self.status['diag'] & DIAG_IS_MG_ACCELERATING > 0
            self.status['is_mg_moving'] =self.status['diag'] & DIAG_IS_MG_MOVING > 0
            self.status['calibration_rcvd'] = self.status['diag'] & DIAG_CALIBRATION_RCVD > 0
            self.status['in_guarded_event'] = self.status['diag'] & DIAG_IN_GUARDED_EVENT > 0
            self.status['in_safety_event'] = self.status['diag'] & DIAG_IN_SAFETY_EVENT > 0
            self.status['waiting_on_sync'] = self.status['diag'] & DIAG_WAITING_ON_SYNC > 0
            return sidx


    def unpack_gains(self,s):
        with self.lock:
            sidx=0
            self.gains['pKp_d'] = unpack_float_t(s[sidx:]);sidx+=4
            self.gains['pKi_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['pKd_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['pLPF'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['pKi_limit'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vKp_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vKi_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vKd_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vLPF'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vKi_limit'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vTe_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['iMax_pos'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['iMax_neg'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['phase_advance_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['pos_near_setpoint_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vel_near_setpoint_d'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['vel_status_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['effort_LPF'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['safety_stiffness'] = unpack_float_t(s[sidx:]);sidx += 4
            self.gains['i_safety_feedforward'] = unpack_float_t(s[sidx:]);sidx += 4
            config = unpack_uint8_t(s[sidx:]);sidx += 1
            self.gains['safety_hold']= int(config & CONFIG_SAFETY_HOLD>0)
            self.gains['enable_runstop'] = int(config & CONFIG_ENABLE_RUNSTOP>0)
            self.gains['enable_sync_mode'] = int(config & CONFIG_ENABLE_SYNC_MODE>0)
            self.gains['enable_guarded_mode'] = int(config & CONFIG_ENABLE_GUARDED_MODE > 0)
            self.gains['flip_encoder_polarity'] = int(config & CONFIG_FLIP_ENCODER_POLARITY > 0)
            self.gains['flip_effort_polarity'] = int(config & CONFIG_FLIP_EFFORT_POLARITY > 0)
            return sidx

    def pack_motion_limits(self,s,sidx):
        with self.lock:
            pack_float_t(s, sidx, self.motion_limits[0])
            sidx += 4
            pack_float_t(s, sidx, self.motion_limits[1])
            sidx += 4
            return sidx

    def pack_command(self,s,sidx):
        with self.lock:
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
        with self.lock:
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
                config=config | CONFIG_SAFETY_HOLD
            if self.gains['enable_runstop']:
                config=config | CONFIG_ENABLE_RUNSTOP
            if self.gains['enable_sync_mode']:
                config=config | CONFIG_ENABLE_SYNC_MODE
            if self.gains['enable_guarded_mode']:
                config=config | CONFIG_ENABLE_GUARDED_MODE
            if self.gains['flip_encoder_polarity']:
                config = config | CONFIG_FLIP_ENCODER_POLARITY
            if self.gains['flip_effort_polarity']:
                config = config | CONFIG_FLIP_EFFORT_POLARITY

            pack_uint8_t(s, sidx, config); sidx += 1
            return sidx

    def pack_trigger(self, s, sidx):
        with self.lock:
            pack_uint32_t(s, sidx, self._trigger);
            sidx += 4
            pack_float_t(s, sidx, self._trigger_data);
            sidx += 4
            return sidx

    def rpc_load_test_reply(self, reply):
        if reply[0] == RPC_REPLY_LOAD_TEST:
            d = reply[1:]
            for i in range(1024):
                if d[i] != self.load_test_payload[(i + 1) % 1024]:
                    print('Load test bad data', d[i], self.load_test_payload[(i + 1) % 1024])
            self.load_test_payload = d
        else:
            print('Error RPC_REPLY_LOAD_TEST', reply[0])

    def rpc_board_info_reply(self, reply):
        if reply[0] == RPC_REPLY_STEPPER_BOARD_INFO:
            self.unpack_board_info(reply[1:])
        else:
            print('Error RPC_REPLY_STEPPER_BOARD_INFO', reply[0])

    def rpc_gains_reply(self, reply):
        if reply[0] != RPC_REPLY_GAINS:
            print('Error RPC_REPLY_GAINS', reply[0])

    def rpc_trigger_reply(self, reply):
        if reply[0] != RPC_REPLY_SET_TRIGGER:
            print('Error RPC_REPLY_SET_TRIGGER', reply[0])

    def rpc_command_reply(self, reply):
        if reply[0] != RPC_REPLY_COMMAND:
            print('Error RPC_REPLY_COMMAND', reply[0])

    def rpc_motion_limits_reply(self, reply):
        if reply[0] != RPC_REPLY_MOTION_LIMITS:
            print('Error RPC_REPLY_MOTION_LIMITS', reply[0])

    def rpc_menu_on_reply(self, reply):
        if reply[0] != RPC_REPLY_MENU_ON:
            print('Error RPC_REPLY_MENU_ON', reply[0])

    def rpc_status_reply(self, reply):
        if reply[0] == RPC_REPLY_STATUS:
            nr = self.unpack_status(reply[1:])
        else:
            print('Error RPC_REPLY_STATUS', reply[0])

    def rpc_read_gains_from_flash_reply(self, reply):
        if reply[0] == RPC_REPLY_READ_GAINS_FROM_FLASH:
            nr = self.unpack_gains(reply[1:])
        else:
            print('Error RPC_REPLY_READ_GAINS_FROM_FLASH', reply[0])