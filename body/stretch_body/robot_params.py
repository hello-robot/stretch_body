import stretch_body.hello_utils as hello_utils
import importlib

# Do not override factory params here
factory_params = {
    # ######### Robot Level ###################
    'robot':{
        'verbose':0
    },
    'robot_sentry': {
        'dynamixel_stop_on_runstop': 1,
        'verbose':0
    },
    'robot_monitor': {
        'verbose':0
    },
    # ######### End of Arm ###################
    'end_of_arm': {
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud':57600,
        'verbose':0
    },
    'wrist_yaw':{
        'retry_on_comm_failure': 1,
        'baud':57600,
        'verbose':0
    },
    'stretch_gripper':{
        'retry_on_comm_failure': 1,
        'baud':57600,
        'verbose':0
    },
    # ######### HEAD ###################
    'head': {
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud':57600,
        'verbose':0
    },
    'head_pan':{
        'verbose':0
    },
    'head_tilt':{
        'verbose':0
    },
    # ######### 4DOF Body ###################
    'arm':{
        'verbose':0
    },
    'lift':{
        'verbose':0
    },
    'base':{
        'verbose':0
    },
    'pimu':{
        'verbose':0
    },
    'imu':
        {

        },
    'wacc':{
        'verbose':0
    },
    'hello-motor-arm':{
        'verbose':0
    },
    'hello-motor-lift':{
        'verbose':0
    },
    'hello-motor-right-wheel':{
        'verbose':0
    },
    'hello-motor-left-wheel':{
        'verbose':0
    }
}

class RobotParams:
    _user_params = None
    _robot_params = None

    @classmethod
    def get_params(self):
        if self._user_params is None:
            self._user_params = hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
        if self._robot_params is None:
            self._robot_params = hello_utils.read_fleet_yaml(self._user_params['factory_params'])
            hello_utils.overwrite_dict(self._robot_params, factory_params)
            for outside_params_module in self._user_params.get('params', []):
                hello_utils.overwrite_dict(self._robot_params,
                    getattr(importlib.import_module(outside_params_module), 'params'))
            hello_utils.overwrite_dict(self._robot_params, self._user_params)
        return (self._user_params, self._robot_params)
