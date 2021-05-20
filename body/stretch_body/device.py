from __future__ import print_function
from stretch_body.robot_params import RobotParams
import stretch_body.hello_utils as hello_utils
import time



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
    """
    Generic base class for all custom Stretch hardware
    """
    def __init__(self,name):
        self.name=name
        self.user_params, self.robot_params = RobotParams.get_params()
        try:
            self.params=self.robot_params[self.name]
        except KeyError:
            print('No device params found for %s'%name)
            self.params={}
        self.timestamp = DeviceTimestamp()

    # ########### Primary interface #############

    def startup(self):
        return True

    def stop(self):
        pass

    def pretty_print(self):
        pass

    def write_device_params(self,device_name, params):
        rp=hello_utils.read_fleet_yaml(self.user_params['factory_params'])
        rp[device_name]=params
        hello_utils.write_fleet_yaml(self.user_params['factory_params'],rp)

