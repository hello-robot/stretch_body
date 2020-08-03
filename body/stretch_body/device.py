
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
    def __init__(self):
        #Factory + Tool params form the robot_params
        #User params can overwrite the resulting robot_params
        self.user_params=hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
        self.robot_params=hello_utils.read_fleet_yaml(self.user_params['factory_params'])
        self.robot_params.update(hello_utils.read_fleet_yaml(self.user_params['tool_params']))
        self.overwrite_params(self.robot_params,self.user_params)
        self.timestamp = DeviceTimestamp()


    def overwrite_params(self,factory_dict,user_dict):
        for k in user_dict.keys():
            if factory_dict.has_key(k):
                if type(factory_dict[k])==type(user_dict[k]):
                    if type(factory_dict[k])==dict:
                        self.overwrite_params(factory_dict[k],user_dict[k])
                    else:
                        factory_dict[k]=user_dict[k]
                else:
                    print('Overwritting Factory Params with User Params. Type mismatch for key:',k)
            else: #If key not present, add anyhow (useful for adding new end_of_arm)
                factory_dict[k] = user_dict[k]


    # ########### Primary interface #############

    def startup(self):
        pass

    def stop(self):
        pass

    def push_command(self):
        pass
    def pull_status(self):
        pass

    def home(self):
        pass

    def pretty_print(self):
        pass

    def write_device_params(self,device_name, params):
        rp=hello_utils.read_fleet_yaml(self.user_params['factory_params'])
        rp[device_name]=params
        hello_utils.write_fleet_yaml(self.user_params['factory_params'],rp)

