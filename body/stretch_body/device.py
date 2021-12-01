from __future__ import print_function
from stretch_body.robot_params import RobotParams
import stretch_body.hello_utils as hello_utils
import time
import logging, logging.config


class DeviceTimestamp:
    def __init__(self):
        self.timestamp_last = None
        self.timestamp_base = 0
        self.timestamp_first= None
        self.ts_start=time.time()

    def set(self, ts): #take a timestamp from a uC in uS and put in terms of system clock
        if self.timestamp_last is None:  # First time
            self.timestamp_last = ts
            self.timestamp_first=ts
        if ts - self.timestamp_last < 0:  # rollover
            self.timestamp_base = self.timestamp_base + 0xFFFFFFFF
        self.timestamp_last = ts
        s=(self.timestamp_base + ts - self.timestamp_first) / 1000000.0
        return self.ts_start+s


class Device:
    logging_params = RobotParams.get_params()[1]['logging']
    logging.config.dictConfig(logging_params)
    """
    Generic base class for all custom Stretch hardware
    """
    def __init__(self, name=''):
        self.name = name
        self.user_params, self.robot_params = RobotParams.get_params()
        self.params = self.robot_params.get(self.name, {})
        self.logger = logging.getLogger(self.name)
        self.timestamp = DeviceTimestamp()

    # ########### Primary interface #############

    def startup(self):
        return True

    def stop(self):
        pass

    def push_command(self):
        pass

    def pull_status(self):
        pass

    def home(self):
        pass

    def step_sentry(self,robot):
        pass

    def pretty_print(self):
        print('----- {0} ------ '.format(self.name))
        hello_utils.pretty_print_dict("params", self.params)

    def write_device_params(self,device_name, params,fleet_dir=None):
        rp=hello_utils.read_fleet_yaml(self.user_params['factory_params'],fleet_dir=fleet_dir)
        rp[device_name]=params
        hello_utils.write_fleet_yaml(self.user_params['factory_params'],rp,fleet_dir=fleet_dir)

