from __future__ import print_function
from stretch_body.robot_params import RobotParams
import stretch_body.hello_utils as hello_utils
import time
import logging, logging.config
import threading
import sys

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
    def __init__(self, name='',req_params=True):
        self.name = name
        self.user_params, self.robot_params = RobotParams.get_params()
        self.params = self.robot_params.get(self.name, {})
        self.logger = logging.getLogger(self.name)

        if self.params == {} and req_params:
                self.logger.warning('Parameters for device %s not found. Check parameter YAML and device name. Exiting...' % self.name)
                sys.exit(1)

        self.timestamp = DeviceTimestamp()
        self.thread_active = False
        self.thread_rate_hz = 25.0
        self.thread_stats = None
        self.thread = None
        self.thread_shutdown_flag = threading.Event()

    # ########### Primary interface #############

    def startup(self, threaded=False):
        """Starts machinery required to interface with this device

        Parameters
        ----------
        threaded : bool
            whether a thread manages hardware polling/pushing in the background

        Returns
        -------
        bool
            whether the startup procedure succeeded
        """
        self.thread_active = threaded
        if self.thread_active:
            if self.thread is not None:
                self.thread_shutdown_flag.set()
                self.thread.join(1)
            self.thread_stats = hello_utils.LoopStats(loop_name='{0}_thread'.format(self.name), target_loop_rate=self.thread_rate_hz)
            self.thread = threading.Thread(target=self._thread_target)
            self.thread.setDaemon(True)
            self.thread_shutdown_flag.clear()
            self.thread.start()
        return True

    def stop(self):
        """Shuts down machinery started in `startup()`
        """
        if self.thread_active:
            self.thread_shutdown_flag.set()
            self.thread.join(1)

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
        raise DeprecationWarning('This method has been deprecated since v0.3.0')

    def write_configuration_param_to_YAML(self,param_name,value,fleet_dir=None):
        """
        Update the robot configuration YAML with a new value
        The param_name has the form device.key, or for a nested dictionary, device.key1.key2...
        For example, write_configuration_param_to_YAML('pimu.config.cliff_zero',100) will set this value to 100 in the YAML
        """
        cp = hello_utils.read_fleet_yaml('stretch_configuration_params.yaml', fleet_dir=fleet_dir)
        param_keys=param_name.split('.')
        d=cp
        for param_key in param_keys:
            if param_key in d:
                if param_key==param_keys[-1]:
                    d[param_key] = value
                else:
                    d = d[param_key]
            else:
                print('Improper param_name in write_configuration_param. Not able to update %s' % param_name)
        hello_utils.write_fleet_yaml('stretch_configuration_params.yaml', cp, fleet_dir=fleet_dir)

    # ########### Thread interface #############

    def _thread_loop(self):
        # self.step_sentry(robot=None) TODO: Support step_sentry here
        self.pull_status()

    def _thread_target(self):
        self.logger.debug('Starting {0}'.format(self.thread_stats.loop_name))
        while not self.thread_shutdown_flag.is_set():
            self.thread_stats.mark_loop_start()
            self._thread_loop()
            self.thread_stats.mark_loop_end()
            if not self.thread_shutdown_flag.is_set():
                time.sleep(self.thread_stats.get_loop_sleep_time())
        self.logger.debug('Shutting down {0}'.format(self.thread_stats.loop_name))
