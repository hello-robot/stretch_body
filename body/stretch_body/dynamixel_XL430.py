from __future__ import print_function
import struct
import array as arr
import logging

from dynamixel_sdk.robotis_def import *
import dynamixel_sdk.port_handler as prh
import dynamixel_sdk.packet_handler as pch
import threading
from stretch_body.device import Device
import serial

# The code can be found in the following directory:
# /opt/ros/melodic/lib/python2.7/dist-packages/dynamixel_sdk/
import dynamixel_sdk.group_bulk_read as gbr
import dynamixel_sdk.group_sync_read as gsr


# #########################
# XL430-W250
# http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table


# Control table address
#EEPROM
XL430_ADDR_MODEL_NUMBER = 0
XL430_ADDR_MODEL_INFORMATION = 2
XL430_ADDR_FIRMWARE_VERSION = 6
XL430_ADDR_ID = 7
XL430_ADDR_BAUD_RATE = 8
XL430_ADDR_RETURN_DELAY_TIME = 9
XL430_ADDR_DRIVE_MODE = 10
XL430_ADDR_OPERATING_MODE = 11
XL430_ADDR_SECONDARY_ID = 12
XL430_ADDR_PROTOCOL_VERSION =13
XL430_ADDR_HOMING_OFFSET = 20
XL430_ADDR_MOVING_THRESHOLD =24
XL430_ADDR_TEMPERATURE_LIMIT = 31
XL430_ADDR_MAX_VOLTAGE_LIMIT = 32
XL430_ADDR_MIN_VOLTAGE_LIMIT = 34
XL430_ADDR_PWM_LIMIT = 36
XL430_ADDR_VELOCITY_LIMIT = 44
XL430_ADDR_MAX_POS_LIMIT = 48
XL430_ADDR_MIN_POS_LIMIT = 52
XL430_ADDR_SHUTDOWN = 63

#RAM
XL430_ADDR_TORQUE_ENABLE      = 64
XL430_ADDR_LED = 65
XL430_ADDR_STATUS_RETURN_LEVEL = 68
XL430_ADDR_REGISTERED_INSTRUCTION = 69
XL430_ADDR_HARDWARE_ERROR_STATUS = 70
XL430_ADDR_VELOCITY_I_GAIN = 76
XL430_ADDR_VELOCITY_P_GAIN = 78
XL430_ADDR_POS_D_GAIN = 80
XL430_ADDR_POS_I_GAIN = 82
XL430_ADDR_POS_P_GAIN = 84
XL430_ADDR_FF_2ND_GAIN = 88
XL430_ADDR_FF_1ST_GAIN =90
XL430_ADDR_BUS_WATCHDOG = 98
XL430_ADDR_GOAL_PWM     = 100
XL430_ADDR_GOAL_VEL     = 104
XL430_ADDR_PROFILE_ACCELERATION = 108
XL430_ADDR_PROFILE_VELOCITY = 112
XL430_ADDR_GOAL_POSITION      = 116
XL430_ADDR_REALTIME_TICK = 120
XL430_ADDR_MOVING = 122
XL430_ADDR_MOVING_STATUS=123
XL430_ADDR_PRESENT_PWM = 124
XL430_ADDR_PRESENT_LOAD = 126
XL430_ADDR_PRESENT_VELOCITY =128
XL430_ADDR_PRESENT_POSITION   = 132
XL430_ADDR_VELOCITY_TRAJECTORY = 136
XL430_ADDR_POSITION_TRAJECTORY=140
XL430_ADDR_PRESENT_INPUT_VOLTATE = 144
XL430_ADDR_PRESENT_TEMPERATURE = 146
XL430_ADDR_HELLO_CALIBRATED = 661 #Appropriate Indirect Data 56 to store calibrated flag

XM430_ADDR_GOAL_CURRENT = 102
XM430_ADDR_CURRENT_LIMIT = 38

COMM_CODES = {
    COMM_SUCCESS: "COMM_SUCCESS",
    COMM_PORT_BUSY: "COMM_PORT_BUSY",
    COMM_TX_FAIL: "COMM_TX_FAIL",
    COMM_RX_FAIL: "COMM_RX_FAIL",
    COMM_TX_ERROR: "COMM_TX_ERROR",
    COMM_RX_WAITING: "COMM_RX_WAITING",
    COMM_RX_TIMEOUT: "COMM_RX_TIMEOUT",
    COMM_RX_CORRUPT: "COMM_RX_CORRUPT",
    COMM_NOT_AVAILABLE: "COMM_NOT_AVAILABLE"
}

BAUD_MAP = {
    9600: 0,
    57600: 1,
    115200: 2,
    1000000: 3,
    2000000: 4,
    3000000: 5,
    4000000: 6,
    4500000: 7
}

class DynamixelCommError(Exception):
    pass


class DynamixelXL430():
    """
    Wrapping of Dynamixel X-Series interface
    """
    def __init__(self, dxl_id, usb, port_handler=None, pt_lock=None, baud=57600, logger=logging.getLogger()):
        self.dxl_id = dxl_id
        self.usb = usb
        self.comm_errors = 0
        self.last_comm_success = True
        self.logger = logger
        self.baud=baud
        # Make access to portHandler threadsafe
        self.pt_lock = threading.RLock() if pt_lock is None else pt_lock

        # Allow sharing of port handler across multiple servos
        self.packet_handler=None
        try:
            if port_handler is None:
                self.port_handler = prh.PortHandler(usb)
                self.port_handler.openPort()
                self.port_handler.setBaudRate(baud)
            else:
                self.port_handler = port_handler
            self.packet_handler = pch.PacketHandler(2.0)
        except serial.SerialException as e:
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))
        self.hw_valid = self.packet_handler is not None

    @staticmethod
    def identify_baud_rate(dxl_id, usb):
        """Identify the baud rate a Dynamixel servo is communicating at.

        Parameters
        ----------
        dxl_id : int
            Dynamixel ID on chain. Must be [0, 25]
        usb : str
            the USB port, typically "/dev/something"

        Returns
        -------
        int
            the baud rate the Dynamixel is communicating at
        """
        for b in BAUD_MAP.keys():
            port_h = prh.PortHandler(usb)
            port_h.openPort()
            port_h.setBaudRate(b)
            packet_h = pch.PacketHandler(2.0)
            _, dxl_comm_result, _ = packet_h.ping(port_h, dxl_id)
            port_h.closePort()
            if dxl_comm_result == COMM_SUCCESS:
                return b
        return -1

    def startup(self):
        if self.hw_valid:
            try:
                self.enable_torque()
            except DynamixelCommError:
                baud=self.identify_baud_rate(self.dxl_id,self.usb)
                if baud!=self.baud:
                    self.logger.error('DynamixelCommError. Mismatched baud rate. Expected %d but servo is set to %d.'%(self.baud,baud))
                else:
                    self.logger.error('DynamixelCommError. Unable to enable torque')
                self.hw_valid=False
                return False
            return True
        return False

    def stop(self):
        if self.hw_valid:
            self.hw_valid = False
            self.disable_torque()
            self.port_handler.closePort()

    def pretty_print(self):
        print('-----XL430------')
        print('ID', self.get_id())
        print('Operating Mode',self.get_operating_mode())
        print('Drive Mode', self.get_drive_mode())
        print('Temp: ', self.get_temp())
        print('Position', self.get_pos())
        print('Velocity',self.get_vel())
        print('Load ', self.get_load())
        print('PWM', self.get_pwm())
        print('Is Moving', self.is_moving())
        print('Is Calibrated',self.is_calibrated())
        print('Profile Velocity', self.get_profile_velocity())
        print('Profile Acceleration', self.get_profile_acceleration())
        h=self.get_hardware_error()
        print('Hardware Error Status', h)
        print('Hardware Error: Input Voltage Error: ', h & 1 != 0)
        print('Hardware Error: Overheating Error: ',h & 4 != 0)
        print('Hardware Error: Motor Encoder Error: ', h & 8 != 0)
        print('Hardware Error: Electrical Shock Error: ', h & 16 != 0)
        print('Hardware Error: Overload Error: ', h & 32 != 0)
        print('Comm errors', self.comm_errors)

    # ##########################################

    def handle_comm_result(self, fx, dxl_comm_result, dxl_error):
        """Handles comm result and tracks comm errors.

        Parameters
        ----------
        fx : str
            control table address label
        dxl_comm_result : int
            communication result from options `COMM_CODES`
        dxl_error : int
            hardware errors sent by the dynamixel

        Returns
        -------
        bool
            True if successful result, False otherwise
        """
        if dxl_comm_result==COMM_SUCCESS:
            self.last_comm_success=True
            return True

        self.last_comm_success = False
        self.comm_errors = self.comm_errors + 1
        self.logger.debug('DXL Comm Error on %s ID %d. Attempted %s. Result %d. Error %d. Code %s. Total Errors %d.' %
            (self.usb, self.dxl_id, fx, dxl_comm_result, dxl_error, COMM_CODES[dxl_comm_result], self.comm_errors))
        raise DynamixelCommError


    def get_comm_errors(self):
        return self.comm_errors

    def read_int32_t(self,addr):
        with self.pt_lock:
            x, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, addr)
        xn = struct.unpack('i', arr.array('B',[DXL_LOBYTE(DXL_LOWORD(x)), DXL_HIBYTE(DXL_LOWORD(x)), DXL_LOBYTE(DXL_HIWORD(x)),DXL_HIBYTE(DXL_HIWORD(x))]))[0]
        return xn, dxl_comm_result, dxl_error

    def read_int16_t(self,addr):
        with self.pt_lock:
            x, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, addr)
        xn = struct.unpack('h', arr.array('B',[DXL_LOBYTE(x), DXL_HIBYTE(x)]))[0]
        return xn, dxl_comm_result, dxl_error

    def do_ping(self,verbose=True):
        if not self.hw_valid:
            return False
        try:
            with self.pt_lock:
                dxl_model_number, dxl_comm_result, dxl_error = self.packet_handler.ping(self.port_handler, self.dxl_id)
            if self.handle_comm_result('XL430_PING', dxl_comm_result, dxl_error):
                self.logger.debug("[Dynamixel ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.dxl_id, dxl_model_number))
                if verbose:
                    print("[Dynamixel ID:%03d] ping Succeeded. Dynamixel model number : %d" % (self.dxl_id, dxl_model_number))
                return True
            else:
                self.logger.debug("[Dynamixel ID:%03d] ping Failed." % (self.dxl_id))
                if verbose:
                    print("[Dynamixel ID:%03d] ping Failed." % (self.dxl_id))
                return False
        except DynamixelCommError:
            self.logger.debug("[Dynamixel ID:%03d] Comm Error. Ping Failed." % (self.dxl_id))
            return False

    def get_id(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_ID)
        self.handle_comm_result('XL430_ADDR_ID', dxl_comm_result, dxl_error)
        return p

    def set_id(self,id):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_ID, id)
        self.handle_comm_result('XL430_ADDR_ID', dxl_comm_result, dxl_error)

    def get_baud_rate(self):
        """Retrieves the baud rate of Dynamixel communication.

        Returns
        -------
        int
            baud rate from `BAUD_MAP` if successful communication, else -1
        """
        if not self.hw_valid:
            return -1
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_BAUD_RATE)
        if not self.handle_comm_result('XL430_ADDR_BAUD_RATE', dxl_comm_result, dxl_error):
            return -1
        return BAUD_MAP.keys()[BAUD_MAP.values().index(p)]

    def set_baud_rate(self, rate):
        """Sets the baud rate of Dynamixel communication.

        Parameters
        ----------
        rate : int
            baud rate option from `BAUD_MAP`

        Returns
        -------
        bool
            True if the baud rate was set successfully, else False
        """
        if not self.hw_valid:
            return -1
        if rate not in BAUD_MAP:
            self.logger.debug("Invalid baud rate")
            return False

        self.disable_torque()
        with self.pt_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_BAUD_RATE, BAUD_MAP[rate])
        return self.handle_comm_result('XL430_ADDR_BAUD_RATE', dxl_comm_result, dxl_error)

    #Hello Robot Specific
    def is_calibrated(self):
        if not self.hw_valid:
            return False
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_HELLO_CALIBRATED)
        self.handle_comm_result('XL430_ADDR_HELLO_CALIBRATED', dxl_comm_result, dxl_error)
        return p

    # Hello Robot Specific
    def set_calibrated(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_HELLO_CALIBRATED, x)
        self.handle_comm_result('XL430_ADDR_HELLO_CALIBRATED', dxl_comm_result, dxl_error)

    def do_reboot(self):
        if not self.hw_valid:
            return False
        with self.pt_lock:
            dxl_comm_result, dxl_error = self.packet_handler.reboot(self.port_handler, self.dxl_id)
        if self.handle_comm_result('XL430_REBOOT', dxl_comm_result, dxl_error):
            print("[Dynamixel ID:%03d] Reboot Succeeded." % (self.dxl_id))
            return True
        else:
            print("[Dynamixel ID:%03d] Reboot Failed." % (self.dxl_id))
            return False

    def get_shutdown(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_SHUTDOWN)
        self.handle_comm_result('XL430_ADDR_SHUTDOWN', dxl_comm_result, dxl_error)
        return p

    def set_shutdown(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_SHUTDOWN, id)
        self.handle_comm_result('XL430_ADDR_SHUTDOWN', dxl_comm_result, dxl_error)

    def get_hardware_error(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id,
                                                                              XL430_ADDR_HARDWARE_ERROR_STATUS)
        self.handle_comm_result('XL430_ADDR_HARDWARE_ERROR_STATUS', dxl_comm_result, dxl_error)
        return p


    def enable_torque(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =  self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_TORQUE_ENABLE, 1)
        self.handle_comm_result('XL430_ADDR_TORQUE_ENABLE', dxl_comm_result, dxl_error)


    def disable_torque(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_TORQUE_ENABLE, 0)
        self.handle_comm_result('XL430_ADDR_TORQUE_ENABLE', dxl_comm_result, dxl_error)

    def set_return_delay_time(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id,XL430_ADDR_RETURN_DELAY_TIME, x)
        self.handle_comm_result('XL430_ADDR_RETURN_DELAY_TIME', dxl_comm_result, dxl_error)

    def set_pwm(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_GOAL_PWM, x)
        self.handle_comm_result('XL430_ADDR_GOAL_PWM', dxl_comm_result, dxl_error)

    def set_current_limit(self,i):
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XM430_ADDR_CURRENT_LIMIT, i)
        self.handle_comm_result('XM430_ADDR_CURRENT_LIMIT', dxl_comm_result, dxl_error)

    def enable_multiturn(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_OPERATING_MODE, 4)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)

    def enable_pwm(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_OPERATING_MODE, 16)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)

    def enable_pos(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_OPERATING_MODE, 3)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)

    def enable_vel(self):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_OPERATING_MODE, 1)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)

    # XM Series
    def enable_pos_current(self):
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_OPERATING_MODE, 5)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)
    #XM Series
    def enable_current(self):
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_OPERATING_MODE, 0)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)


    def get_operating_mode(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id,XL430_ADDR_OPERATING_MODE)
        self.handle_comm_result('XL430_ADDR_OPERATING_MODE', dxl_comm_result, dxl_error)
        return p

    def get_drive_mode(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id,XL430_ADDR_DRIVE_MODE)
        self.handle_comm_result('XL430_ADDR_DRIVE_MODE', dxl_comm_result, dxl_error)
        return p

    def set_drive_mode(self,vel_based=True, reverse=False):
        if not self.hw_valid:
            return
        #defaults to vel_based, not forward at factory
        x=0
        if not vel_based:
            x=x|0x01
        if reverse:
            x=x|0x4
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_DRIVE_MODE, x)
        self.handle_comm_result('XL430_ADDR_DRIVE_MODE', dxl_comm_result, dxl_error)

    #XM Series
    def set_goal_current(self,i):
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XM430_ADDR_GOAL_CURRENT, i)
        self.handle_comm_result('XM430_ADDR_GOAL_CURRENT', dxl_comm_result, dxl_error)


    def go_to_pos(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_GOAL_POSITION, x)
        self.handle_comm_result('XL430_ADDR_GOAL_POSITION', dxl_comm_result, dxl_error)

    def set_vel(self, x):
        with self.pt_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id,
                                                                            XL430_ADDR_GOAL_VEL, x)
        self.handle_comm_result('XL430_ADDR_GOAL_VEL', dxl_comm_result, dxl_error)

    def get_pos(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            xn, dxl_comm_result, dxl_error= self.read_int32_t(XL430_ADDR_PRESENT_POSITION)
        self.handle_comm_result('XL430_ADDR_PRESENT_POSITION', dxl_comm_result, dxl_error)
        return xn



    def get_load(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            xn, dxl_comm_result, dxl_error=  self.read_int16_t(XL430_ADDR_PRESENT_LOAD)
        self.handle_comm_result('XL430_ADDR_PRESENT_LOAD', dxl_comm_result, dxl_error)
        return xn

    def get_pwm(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            xn, dxl_comm_result, dxl_error=  self.read_int16_t(XL430_ADDR_PRESENT_PWM)
        self.handle_comm_result('XL430_ADDR_PRESENT_PWM', dxl_comm_result, dxl_error)
        return xn


    def set_profile_velocity(self,v):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PROFILE_VELOCITY, v)
        self.handle_comm_result('XL430_ADDR_PROFILE_VELOCITY', dxl_comm_result, dxl_error)

    def set_profile_acceleration(self, a):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id,XL430_ADDR_PROFILE_ACCELERATION, a)
        self.handle_comm_result('XL430_ADDR_PROFILE_ACCELERATION', dxl_comm_result, dxl_error)


    def get_profile_velocity(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            v, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PROFILE_VELOCITY)
        self.handle_comm_result('XL430_ADDR_PROFILE_VELOCITY', dxl_comm_result, dxl_error)
        return v

    def get_profile_acceleration(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            a, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PROFILE_ACCELERATION)
        self.handle_comm_result('XL430_ADDR_PROFILE_ACCELERATION', dxl_comm_result, dxl_error)
        return a

    def get_vel(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            v, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PRESENT_VELOCITY)
        self.handle_comm_result('XL430_ADDR_PRESENT_VELOCITY', dxl_comm_result, dxl_error)
        if v > 2 ** 24:
            v = v - 2 ** 32
        return v

    def get_P_gain(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_POS_P_GAIN)
        self.handle_comm_result('XL430_ADDR_POS_P_GAIN', dxl_comm_result, dxl_error)
        return p

    def set_P_gain(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_POS_P_GAIN, x)
        self.handle_comm_result('XL430_ADDR_POS_P_GAIN', dxl_comm_result, dxl_error)

    def get_D_gain(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_POS_D_GAIN)
        self.handle_comm_result('XL430_ADDR_POS_D_GAIN', dxl_comm_result, dxl_error)
        return p

    def set_D_gain(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_POS_D_GAIN, x)
        self.handle_comm_result('XL430_ADDR_POS_D_GAIN', dxl_comm_result, dxl_error)

    def get_I_gain(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_POS_I_GAIN)
        self.handle_comm_result('XL430_ADDR_POS_I_GAIN', dxl_comm_result, dxl_error)
        return p

    def set_I_gain(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_POS_I_GAIN, x)
        self.handle_comm_result('XL430_ADDR_POS_I_GAIN', dxl_comm_result, dxl_error)

    def get_temperature_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_TEMPERATURE_LIMIT)
        self.handle_comm_result('XL430_ADDR_TEMPERATURE_LIMIT', dxl_comm_result, dxl_error)
        return p

    def set_temperature_limit(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_TEMPERATURE_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_TEMPERATURE_LIMIT', dxl_comm_result, dxl_error)


    def get_max_voltage_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MAX_VOLTAGE_LIMIT)
        self.handle_comm_result('XL430_ADDR_MAX_VOLTAGE_LIMIT', dxl_comm_result, dxl_error)
        return p

    def set_max_voltage_limit(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MAX_VOLTAGE_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_MAX_VOLTAGE_LIMIT', dxl_comm_result, dxl_error)

    def get_min_voltage_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MIN_VOLTAGE_LIMIT)
        self.handle_comm_result('XL430_ADDR_MIN_VOLTAGE_LIMIT', dxl_comm_result, dxl_error)
        return p

    def set_min_voltage_limit(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MIN_VOLTAGE_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_MIN_VOLTAGE_LIMIT', dxl_comm_result, dxl_error)

    def get_vel_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_VELOCITY_LIMIT)
        self.handle_comm_result('XL430_ADDR_VELOCITY_LIMIT', dxl_comm_result, dxl_error)
        return p

    def set_vel_limit(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_VELOCITY_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_VELOCITY_LIMIT', dxl_comm_result, dxl_error)

    def get_max_pos_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MAX_POS_LIMIT)
        self.handle_comm_result('XL430_ADDR_MAX_POS_LIMIT', dxl_comm_result, dxl_error)
        return p

    def set_max_pos_limit(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MAX_POS_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_MAX_POS_LIMIT', dxl_comm_result, dxl_error)

    def set_min_pos_limit(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MIN_POS_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_MIN_POS_LIMIT', dxl_comm_result, dxl_error)


    def get_min_pos_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MIN_POS_LIMIT)
        self.handle_comm_result('XL430_ADDR_MIN_POS_LIMIT', dxl_comm_result, dxl_error)
        return p

    def get_temp(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PRESENT_TEMPERATURE)
        self.handle_comm_result('XL430_ADDR_PRESENT_TEMPERATURE', dxl_comm_result, dxl_error)
        return p

    def set_moving_threshold(self,x): #unit of 0.229 rev/min, default 10
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MOVING_THRESHOLD, x)
        self.handle_comm_result('XL430_ADDR_MOVING_THRESHOLD', dxl_comm_result, dxl_error)

    def set_pwm_limit(self,x): #0(0%) ~ 885(100%
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error =   self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PWM_LIMIT, x)
        self.handle_comm_result('XL430_ADDR_PWM_LIMIT', dxl_comm_result, dxl_error)

    def get_pwm_limit(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_PWM_LIMIT)
        self.handle_comm_result('XL430_ADDR_PWM_LIMIT', dxl_comm_result, dxl_error)
        return p

    def is_moving(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            p, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_MOVING)
        self.handle_comm_result('XL430_ADDR_MOVING', dxl_comm_result, dxl_error)
        return p

    def zero_position(self,verbose=False):
        if not self.hw_valid:
            return
        self.logger.debug('Previous HOMING_OFFSET in EEPROM {0}'.format(self.get_homing_offset()))
        if verbose:
            print('Previous HOMING_OFFSET in EEPROM', self.get_homing_offset())
        self.set_homing_offset(0)
        h=-1*self.get_pos()
        self.logger.debug('Setting homing offset to {0}'.format(h))
        if verbose:
            print('Setting homing offset to',h)
        self.set_homing_offset(h)
        self.logger.debug('New HOMING_OFFSET in EEPROM {0}'.format(self.get_homing_offset()))
        self.logger.debug('Current position after homing {0}'.format(self.get_pos()))
        if verbose:
            print('New HOMING_OFFSET in EEPROM', self.get_homing_offset())
            print('Current position after homing',self.get_pos())
        return

    def get_homing_offset(self):
        if not self.hw_valid:
            return 0
        with self.pt_lock:
            xn, dxl_comm_result, dxl_error = self.read_int32_t(XL430_ADDR_HOMING_OFFSET)
        self.handle_comm_result('XL430_ADDR_HOMING_OFFSET', dxl_comm_result, dxl_error)
        return xn

    def set_homing_offset(self,x):
        if not self.hw_valid:
            return
        with self.pt_lock:
            dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, XL430_ADDR_HOMING_OFFSET, x)
        self.handle_comm_result('XL430_ADDR_HOMING_OFFSET', dxl_comm_result, dxl_error)
