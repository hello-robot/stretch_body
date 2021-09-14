import stretch_body.hello_utils as hello_utils
import importlib
import logging

# Do not override factory params here
factory_params = {
    "dxl_comm_errors":{
        "warn_every_s":1.0,
        "warn_above_rate":0.1,
        'verbose':0
    },
    "robot": {
        "tool": "tool_stretch_gripper",
        "use_collision_manager": 0,
    },
    "robot_sentry": {
        "dynamixel_stop_on_runstop": 1,
        "base_fan_control": 1,
        "base_max_velocity": 1,
        "stretch_gripper_overload": 1,
        "wrist_yaw_overload": 1,
        "stepper_is_moving_filter": 1,
    },
    "robot_collision": {
        'models': ['collision_arm_camera']
    },
    'hello-motor-arm':{
        'gains': {'vel_near_setpoint_d': 3.5}
    },
    'hello-motor-lift':{
        'gains': {'vel_near_setpoint_d': 3.5}
    },
    'hello-motor-right-wheel':{
        'gains': {'vel_near_setpoint_d': 3.5}
    },
    'hello-motor-left-wheel':{
        'gains': {'vel_near_setpoint_d': 3.5}
    },
    "head": {
        "use_group_sync_read": 1,
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "dxl_latency_timer":64
    },
    "end_of_arm": {
        "use_group_sync_read": 1,
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "dxl_latency_timer": 64,
        'stow': {'wrist_yaw': 3.4},
        'devices': {
            'wrist_yaw': {
                'py_class_name': 'WristYaw',
                'py_module_name': 'stretch_body.wrist_yaw'
            }
        }
    },
    "head_pan": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "enable_runstop": 1
    },
    "head_tilt": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "enable_runstop": 1
    },
    "wrist_yaw": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "enable_runstop": 1
    },
    "stretch_gripper": {
        "retry_on_comm_failure": 1,
        "baud": 57600,
        "enable_runstop": 1
    },
    "base": {
        "sentry_max_velocity": {
            "limit_accel_m": 0.15,
            "limit_vel_m": 0.1,
            "max_arm_extension_m": 0.03,
            "max_lift_height_m": 0.3,
            "min_wrist_yaw_rad": 2.54,
        }
    },
    "tool_none": {
        'use_group_sync_read': 1,
        'retry_on_comm_failure': 1,
        'baud':57600,
        "dxl_latency_timer": 64,
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
        'baud':57600,
        "dxl_latency_timer": 64,
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
        },
        'collision_models': ['collision_stretch_gripper']
    },
    "collision_arm_camera": {
        'enabled': 1,
        'py_class_name': 'CollisionArmCamera',
        'py_module_name': 'stretch_body.robot_collision_models'
    },
    "collision_stretch_gripper": {
        'enabled': 1,
        'py_class_name': 'CollisionStretchGripper',
        'py_module_name': 'stretch_body.robot_collision_models',
    },
    "logging": {
        "version": 1,
        "disable_existing_loggers": True,
        "root": {
            "level": "DEBUG",
            "handlers": ["console_handler", "file_handler"],
            "propagate": False
        },
        "handlers": {
            "console_handler": {
                "class": "logging.StreamHandler",
                "level": "INFO",
                "formatter": "default_console_formatter",
            },
            "file_handler": {
                "class": "logging.FileHandler",
                "level": "DEBUG",
                "formatter": "default_file_formatter",
                "filename": hello_utils.get_stretch_directory('log/') + 'stretchbody_{0}.log'.format(hello_utils.create_time_string())
            }
        },
        "formatters": {
            "default_console_formatter": {
                "format": "[%(levelname)s] [%(name)s]: %(message)s"
            },
            "brief_console_formatter": {
                "format": "%(message)s"
            },
            "default_file_formatter": {
                "format": "[%(levelname)-8s] [%(asctime)s.%(msecs)03d] [%(name)s]: %(message)s",
                "datefmt": "%m/%d/%Y %H:%M:%S"
            }
        }
    },
}

class RobotParams:
    """Build the parameter dictionary that is availale as stretch_body.Device().robot_params.

    Overwrite dictionaries in order of ascending priorty
    1. stretch_re1_factory_params.yaml | Factory YAML settings that shipped with the robot. (Including robot specific calibrations)
    2. stretch_body.robot_params.factory_params | Factory Python settings (Common across robots. Factory may modify these via Pip updates)
    3. Outside parameters | (eg, from stretch_tool_share.stretch_dex_wrist.params)
    4. stretch_re1_user_params.yaml | Place to override factory defaults
    """
    _user_params = hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
    _robot_params = hello_utils.read_fleet_yaml(_user_params.get('factory_params', ''))
    hello_utils.overwrite_dict(_robot_params, factory_params)
    for external_params_module in _user_params.get('params', []):
        hello_utils.overwrite_dict(_robot_params, getattr(importlib.import_module(external_params_module), 'params'))
    hello_utils.overwrite_dict(_robot_params, _user_params)

    @classmethod
    def get_params(cls):
        return (cls._user_params, cls._robot_params)

    @classmethod
    def add_params(cls, new_params):
        hello_utils.overwrite_dict(cls._robot_params, new_params)

    @classmethod
    def set_logging_level(cls, level, handler='console_handler'):
        level_names={0: 'NOTSET', 10: 'DEBUG', 'WARN': 30, 20: 'INFO', 'ERROR': 40, 'DEBUG': 10, 30:
            'WARNING', 'INFO': 20, 'WARNING': 30, 40: 'ERROR', 50: 'CRITICAL', 'CRITICAL': 50, 'NOTSET': 0}
        if level in level_names and handler in cls._robot_params['logging']['handlers']:
            cls._robot_params['logging']['handlers'][handler]['level'] = level
