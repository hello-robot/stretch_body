from __future__ import print_function
import stretch_body.hello_utils as hello_utils
from stretch_body.device import Device
import time

# The code can be found in the following directory:
# /opt/ros/melodic/lib/python2.7/dist-packages/dynamixel_sdk/
from dynamixel_sdk.robotis_def import *
from stretch_body.dynamixel_XL430 import *
from stretch_body.dynamixel_hello_XL430 import DynamixelCommErrorStats
import dynamixel_sdk.port_handler as prh
import dynamixel_sdk.packet_handler as pch
import dynamixel_sdk.group_sync_read as gsr


class DynamixelXChain(Device):
    """
    This class manages a daisy chain of Dynamixel X-Series servos
    It allows adding more than one servo at run time
    It allows manage group reading of status data from servos so as to not overload the control bus
    """
    def __init__(self, usb, name):
        Device.__init__(self, name)
        self.usb = usb
        self.pt_lock = threading.RLock()
        self.thread_rate_hz = 15.0

        self.packet_handler = None
        self.port_handler = None
        self.hw_valid = False

        self.status={}
        self.motors = {}
        self.readers={}
        self.comm_errors = DynamixelCommErrorStats(name, logger=self.logger)
        self.status_mux_id = 0

    def add_motor(self,m):
        self.motors[m.name]=m

    def get_motor(self,motor_name):
        try:
            return self.motors[motor_name]
        except (AttributeError, KeyError):
            return None
    
    def create_port_handler(self):
        try:
            prh.LATENCY_TIMER = self.params['dxl_latency_timer']
            self.port_handler = prh.PortHandler(self.usb)
            self.port_handler.openPort()
            self.port_handler.setBaudRate(int(self.params['baud']))
            self.packet_handler = pch.PacketHandler(2.0)
            self.hw_valid = True
        except serial.SerialException as e:
            self.packet_handler = None
            self.port_handler = None
            self.hw_valid =False
            self.logger.error("SerialException({0}): {1}".format(e.errno, e.strerror))

    def startup(self, threaded=False):
        self.create_port_handler()
        for mk in self.motors.keys():  # Provide nop data in case comm failures
            self.status[mk] = self.motors[mk].status
        if not self.hw_valid:
            print("HW Not Valid")
            return False
        if len(self.motors.keys()):
            try:
                if self.params['use_group_sync_read']:
                    self.readers['pos'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                            XL430_ADDR_PRESENT_POSITION, 4)
                    self.readers['effort'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                               XL430_ADDR_PRESENT_LOAD, 2)
                    self.readers['vel'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                            XL430_ADDR_PRESENT_VELOCITY, 4)
                    self.readers['temp'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                             XL430_ADDR_PRESENT_TEMPERATURE, 1)
                    self.readers['hardware_error'] = gsr.GroupSyncRead(self.port_handler, self.packet_handler,
                                                                       XL430_ADDR_HARDWARE_ERROR_STATUS, 1)
                    for mk in self.motors.keys():
                        for k in self.readers.keys():
                            if not self.readers[k].addParam(self.motors[mk].motor.dxl_id):
                                self.logger.error('Dynamixel X sync read initialization failed.')
                                raise DynamixelCommError
                for mk in self.motors.keys():
                    if not self.motors[mk].startup(threaded=False):
                        raise DynamixelCommError
                    self.status[mk] = self.motors[mk].status
                self.pull_status()
            except DynamixelCommError as e:
                print(f"Dnamixel Com error: {e}")
                self.comm_errors.add_error(rx=True,gsr=True)
                self.hw_valid = False
                return False
        Device.startup(self, threaded=threaded)
        return True

    def _thread_loop(self):
        self.pull_status()
        self.update_trajectory()

    def stop(self):
        Device.stop(self)
        if not self.hw_valid:
            return
        for motor in self.motors:
            self.motors[motor]._waypoint_ts = None
            self.motors[motor]._waypoint_vel = None
            self.motors[motor]._waypoint_accel = None
            self.motors[motor].stop(close_port=False)
        self.port_handler.closePort()
        self.hw_valid = False

    def wait_until_at_setpoint(self, timeout=15.0, use_motion_generator=True):
        at_setpoint = []
        def check_wait(wait_method):
            at_setpoint.append(wait_method(timeout,use_motion_generator))
        threads = []
        for motor in self.motors:
            threads.append(threading.Thread(target=check_wait, args=(self.motors[motor].wait_until_at_setpoint,)))
        [done_thread.start() for done_thread in threads]
        [done_thread.join() for done_thread in threads]
        return all(at_setpoint)

    def is_trajectory_active(self):
        for motor in self.motors:
            if self.motors[motor].is_trajectory_active():
                return True
        return False

    def follow_trajectory(self, v_r=None, a_r=None, req_calibration=False, move_to_start_point=True):
        success = True
        for motor in self.motors:
            success = success and self.motors[motor].follow_trajectory(v_r, a_r, req_calibration, move_to_start_point)
        return success

    def update_trajectory(self):
        for motor in self.motors:
            self.motors[motor].update_trajectory()

    def stop_trajectory(self):
        for motor in self.motors:
            self.motors[motor].stop_trajectory()

    def pull_status(self):
        if not self.hw_valid:
            return
        try:
            ts = time.time()
            error=False
            if self.params['use_group_sync_read']:
                pos = self.sync_read(self.readers['pos'])
                if pos==None and self.params['retry_on_comm_failure']:
                    pos = self.sync_read(self.readers['pos'])
                error= error or (pos==None)

                vel = self.sync_read(self.readers['vel'])
                if vel == None and self.params['retry_on_comm_failure']:
                    vel = self.sync_read(self.readers['vel'])
                error = error or (vel == None)

                if self.status_mux_id == 0:
                    effort = self.sync_read(self.readers['effort'])
                    if effort == None and self.params['retry_on_comm_failure']:
                        effort = self.sync_read(self.readers['effort'])
                    error = error or (effort == None)
                else:
                    effort = None

                if self.status_mux_id == 1:
                    temp = self.sync_read(self.readers['temp'])
                    if temp == None and self.params['retry_on_comm_failure']:
                        temp = self.sync_read(self.readers['temp'])
                    error = error or (temp == None)
                else:
                    temp = None

                if self.status_mux_id == 2:
                    hardware_error = self.sync_read(self.readers['hardware_error'])
                    if hardware_error == None and self.params['retry_on_comm_failure']:
                        hardware_error = self.sync_read(self.readers['hardware_error'])
                    error = error or (hardware_error == None)
                else:
                    hardware_error = None

                self.status_mux_id = (self.status_mux_id + 1) % 3

                if error:
                    self.comm_errors.add_error(rx=True, gsr=True)
                    self.logger.warning('Dynamixel communication error during pull_status on %s: ' % self.name)
                    #self.port_handler.ser.reset_output_buffer()
                    #self.port_handler.ser.reset_input_buffer()

                idx = 0
                # Build dictionary of status data and push to each motor status
                # None may indicate comm error or the field wasn't read on this mux cycle
                for mk in self.motors.keys():
                    data = {'ts': time.time()}
                    if pos is not None:
                        data['x'] = pos[idx]
                    else:
                        data['x'] = self.motors[mk].status['pos_ticks']
                    if vel is not None:
                        data['v'] = vel[idx]
                    else:
                        data['v'] = self.motors[mk].status['vel_ticks']
                    if effort is not None:
                        data['eff'] = effort[idx]
                    else:
                        data['eff'] = self.motors[mk].status['effort_ticks']
                    if temp is not None:
                        data['temp'] = temp[idx]
                    else:
                        data['temp'] = self.motors[mk].status['temp']
                    if hardware_error is not None:
                        data['err'] = hardware_error[idx]
                    else:
                        data['err'] = self.motors[mk].status['hardware_error']
                    self.motors[mk].pull_status(data)
                    idx = idx + 1
            else:
                for m in self.motors:
                    with self.pt_lock:
                        self.motors[m].pull_status()
        except(DynamixelCommError, IOError):
            self.comm_errors.add_error(rx=True, gsr=True)
            self.logger.warning('Dynamixel communication error during pull_status on %s: ' % self.name)

    def pretty_print(self):
        print('--- Dynamixel X Chain ---')
        print('USB', self.usb)
        for mk in self.motors.keys():
            self.motors[mk].pretty_print()

    def sync_read(self, reader):
        if not self.hw_valid:
            return None
        with self.pt_lock:
            result = reader.txRxPacket()
        if result != COMM_SUCCESS:
            self.logger.debug('Dynamixel X sync read txRxPacket failed with error code = ' + str(result))
            return None
            #raise DynamixelCommError

        def get_val(id_num):
            try:
                with self.pt_lock:
                    b = reader.getData(id_num, reader.start_address, reader.data_length)
            except IndexError:
                #Bad data struct size possible to raise Index Error
                return None #raise DynamixelCommError
            # The error code seems to be 0, yet there are also
            # circumstances where b will be 0 without an error, such
            # as being at position = 0. For now, I am commenting out
            # this error reporting code. One possibility is to edit
            # the SDK module. Another possibility is to use the
            # Available command separately.
            #
            # if (b == 0):
            #     print('Dynamixel X sync read getData failed.')
            #     return None

            if reader.data_length == 4:
                val = struct.unpack('i', arr.array('B', [DXL_LOBYTE(DXL_LOWORD(b)), DXL_HIBYTE(DXL_LOWORD(b)),
                                                         DXL_LOBYTE(DXL_HIWORD(b)), DXL_HIBYTE(DXL_HIWORD(b))]))[0]
            if reader.data_length == 2:
                val = struct.unpack('h', arr.array('B', [DXL_LOBYTE(b), DXL_HIBYTE(b)]))[0]
            if reader.data_length == 1:
                val = struct.unpack('b', arr.array('B', [b]))[0]
            return val
        values = [get_val(self.motors[mk].motor.dxl_id) for mk in self.motors.keys()]
        return values


    def step_sentry(self,robot):
        """This sentry places the Dynamixel servos in torque_disabled
        mode when the runstop is enabled.
        """
        for k in self.motors.keys():
            self.motors[k].step_sentry(robot)
