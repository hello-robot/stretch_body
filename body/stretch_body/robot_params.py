import stretch_body.hello_utils as hello_utils
import importlib

# Do not override factory params here
factory_params = {
    "head": {
        "use_group_sync_read": 1,
        "retry_on_comm_failure": 1,
        "baud": 57600,
    },
    "end_of_arm": {
        "use_group_sync_read": 1,
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "tool": "tool_stretch_gripper"
    },
    "head_pan": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
    },
    "head_tilt": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
    },
    "wrist_yaw": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
    },
    "stretch_gripper": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
    },
    "tool_none": {
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
}

class RobotParams:
    _user_params = None
    _robot_params = None

    @classmethod
    def get_params(self):
        if self._user_params is None:
            self._user_params = hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
        if self._robot_params is None:
            self._robot_params = hello_utils.read_fleet_yaml(self._user_params.get('factory_params', ''))
            self._robot_params.update(hello_utils.read_fleet_yaml(self._user_params.get('tool_params', '')))
            hello_utils.overwrite_dict(self._robot_params, factory_params)
            hello_utils.overwrite_dict(self._robot_params, self._user_params)
            for outside_params_module in self._user_params.get('params', []):
                hello_utils.overwrite_dict(self._robot_params,
                    getattr(importlib.import_module(outside_params_module), 'params'))

        return (self._user_params, self._robot_params)
