import stretch_body.hello_utils as hello_utils
import importlib

# Do not override factory params here
factory_params = {
    # ######### Robot Level ###################
    'robot':{
        'verbose':0,
        'tool': 'tool_stretch_gripper'
    },
    'robot_sentry': {
        'dynamixel_stop_on_runstop': 1,
        'verbose':0
    },
    'robot_monitor': {
        'verbose':0
    },
    # ######### End of Arm ###################
    'wrist_yaw':{
        'retry_on_comm_failure': 1,
        'baud':115200,
        'verbose':0
    },
    'stretch_gripper':{
        'retry_on_comm_failure': 1,
        'baud':115200,
        'verbose':0
    },
    "tool_none": {
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud':115200,
        'verbose':0,
        'py_class_name': 'ToolNone',
        'py_module_name': 'stretch_body.end_of_arm_tools',
        'stow': {'wrist_yaw': 3.4},
        'devices': {
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch_body.wrist_yaw'
            }
        }
    },
    "tool_stretch_gripper": {
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud':115200,
        'verbose':0,
        'py_class_name': 'ToolStretchGripper',
        'py_module_name': 'stretch_body.end_of_arm_tools',
        'stow': {'stretch_gripper': 0, 'wrist_yaw': 3.4},
        'devices': {
            'stretch_gripper': {
                'py_class_name': 'StretchGripper',
                'py_module_name': 'stretch_body.stretch_gripper'
            },
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch_body.wrist_yaw'
            }
        }
    },
    # ######### HEAD ###################
    'head': {
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud':115200,
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
        """
        Build the parameter dictionary that is common to all Devices as device.robot_params.
        Overwrite dictionaries in order of ascending priorty
        1. stretch_re1_factory_params.yaml | Factory YAML settings that shipped with the robot. (Including robot specific calibrations)
        2. stretch_body.robot_params.params | Factory Python settings (Common across robots. Factory may modify these via Pip updates)
        3. Outside parameters | (eg, from stretch_tool_share.stretch_dex_wrist.params)
        4. stretch_re1_user_params.yaml | Place to override all of the above
        """
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
