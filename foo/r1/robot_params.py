import stretch_body.hello_utils as hello_utils
from os.path import exists
import importlib
import logging
import sys
import click


#System parameters that are common across models. May be updated by the factory via Pip.
nominal_system_params={
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
                    "filename": hello_utils.get_stretch_directory('log/stretch_body_logger/') + 'stretchbody_{0}.log'.format(
                        hello_utils.create_time_string())
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
    "system_check": {
            "show_sw_exc": False
        },
}

class RobotParams:
    """Build the parameter dictionary that is available as stretch_body.Device().robot_params.
    Overwrite dictionaries in order of ascending priority
    1. stretch_body.robot_params.nominal_system_params  | Generic systems settings (Common across all robot models. Factory may modify these via Pip updates)
    2. stretch_body.robot_params_XXXX.py                | Nominal robot paramters for this robot model (e.g., RE1V0) as defined in stretch_user_params.yaml. Factory may modify these via Pip updates
    3. Outside parameters                               | Include other sourcesthrough 'params' field. (eg, from stretch_tool_share.stretch_dex_wrist.params). Factory may modify these via Pip updates.
    4. stretch_configuration_params.yaml                | Robot specific data (eg, serial numbers and calibrations). Calibration tools may update these.
    5. stretch_user_params.yaml                         | User specific data (eg, contact thresholds, controller tunings, etc)
    """
    user_params_fn = hello_utils.get_fleet_directory()+'stretch_user_params.yaml'
    config_params_fn = hello_utils.get_fleet_directory()+'stretch_configuration_params.yaml'
    if not hello_utils.check_file_exists(user_params_fn) or not hello_utils.check_file_exists(config_params_fn):
        _valid_params=False
        print('Please run tool RE1_migrate_params.py or verify if Stretch configuration YAML files are present before continuing.\nFor more details, see https://forum.hello-robot.com/t/425')
        sys.exit(1)
    else:
        _user_params = hello_utils.read_fleet_yaml('stretch_user_params.yaml')
        _config_params = hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
        _robot_params=nominal_system_params

        #Check for user / config overrides that impact what data is loaded
        #Get the name of the robot model
        if 'robot' in _user_params and 'model_name' in _user_params['robot']:
            param_module_name = 'stretch_body.robot_params_'+_user_params['robot']['model_name']
        else:
            param_module_name = 'stretch_body.robot_params_' + _config_params['robot']['model_name']

        _nominal_params = getattr(importlib.import_module(param_module_name), 'nominal_params')

        #Get the name of the current end-of-arm
        if 'robot' in _user_params and 'tool' in _user_params['robot']:
            eoa_name = _user_params['robot']['tool']
        elif 'robot' in _config_params and 'tool' in _config_params['robot']:
            eoa_name = _config_params['robot']['tool']
        else:
            eoa_name = _nominal_params['robot']['tool']

        if not eoa_name in _nominal_params['supported_eoa'] or not eoa_name in _nominal_params:
            _valid_params = False
            print('%s not supported for robot %s'%(eoa_name.upper(), param_module_name))
            print('Check your YAML definition of robot.tool')
            sys.exit(1)

        #Now expand the params for each EOA
        for d in _nominal_params[eoa_name]['devices']:
            g=getattr(importlib.import_module(param_module_name),_nominal_params[eoa_name]['devices'][d]['device_params'])
            _nominal_params[d]=g
        #     _nominal_params[d]=_nominal_params[eoa_name]['devices'][d]['device_params']

        def check_for_dexwrist_toolshare(external_params_module):
            if external_params_module == 'stretch_tool_share.stretch_dex_wrist.params':
                print('')
                click.secho('----------- Deprecation Warning -----------', fg="cyan", bold=True)
                click.secho('Your robot params are configured to load DexWrist2 params from Stretch Tool Share', fg="cyan", bold=True)
                click.secho('Support for the DexWrist2 has moved to Stretch Body' , fg="cyan", bold=True)
                click.secho(' 1) Open stretch_user_params.yaml and stretch_configuration_params.yaml in', fg="cyan", bold=True)
                click.secho('    the ~/stretch_user/stretch-yyy-xxxx directory.', fg="cyan", bold=True)
                click.secho(' 2) Locate the following text in one of those files and remove it:',fg="cyan", bold=True)
                click.secho('     params: stretch_tool_share.stretch_dex_wrist.params',fg="cyan", bold=True)
                click.secho('     or',fg="cyan", bold=True)
                click.secho('     params:',fg="cyan", bold=True)
                click.secho('      - stretch_tool_share.stretch_dex_wrist.params',fg="cyan", bold=True)
                click.secho('More information can be found at: https://github.com/hello-robot/stretch_body/pull/272',fg="cyan", bold=True)
                click.secho('-------------------------------------------', fg="cyan", bold=True)
                print('')
                return True
            return False


        hello_utils.overwrite_dict(_robot_params, _nominal_params)

        for external_params_module in _config_params.get('params', []):
            if not check_for_dexwrist_toolshare(external_params_module):
                hello_utils.overwrite_dict(_robot_params,getattr(importlib.import_module(external_params_module), 'params'))

        for external_params_module in _user_params.get('params', []):
            if not check_for_dexwrist_toolshare(external_params_module):
                hello_utils.overwrite_dict(_robot_params,getattr(importlib.import_module(external_params_module), 'params'))

        hello_utils.overwrite_dict(_robot_params, _config_params)

        hello_utils.overwrite_dict(_robot_params, _user_params)

        _valid_params=True

    @classmethod
    def get_user_params_header(cls):
        return getattr(importlib.import_module(cls.param_module_name), 'user_params_header')

    @classmethod
    def get_configuration_params_header(cls):
        return getattr(importlib.import_module(cls.param_module_name), 'configuration_params_header')

    @classmethod
    def are_params_valid(cls):
        return (cls._valid_params)

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

    @classmethod
    def set_logging_formatter(cls, formatter, handler='console_handler'):
        formatter_names = ["default_console_formatter", "brief_console_formatter", "default_file_formatter"]
        if formatter in formatter_names and handler in cls._robot_params['logging']['handlers']:
            cls._robot_params['logging']['handlers'][handler]['formatter'] = formatter

# For Reference, the parameter loading organization prior to release of RE2
# class RobotParams:
#     """Build the parameter dictionary that is available as stretch_body.Device().robot_params.
#
#     Overwrite dictionaries in order of ascending priority
#     1. stretch_re1_factory_params.yaml | Factory YAML settings that shipped with the robot. (Including robot specific calibrations)
#     2. stretch_body.robot_params.factory_params | Factory Python settings (Common across robots. Factory may modify these via Pip updates)
#     3. Outside parameters | (eg, from stretch_tool_share.stretch_dex_wrist.params)
#     4. stretch_re1_user_params.yaml | Place to override factory defaults
#     """
#     _user_params = hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
#     _robot_params = hello_utils.read_fleet_yaml(_user_params.get('factory_params', ''))
#     hello_utils.overwrite_dict(_robot_params, factory_params)
#     for external_params_module in _user_params.get('params', []):
#         hello_utils.overwrite_dict(_robot_params, getattr(importlib.import_module(external_params_module), 'params'))
#     hello_utils.overwrite_dict(_robot_params, _user_params)
#
#     @classmethod
#     def get_params(cls):
#         return (cls._user_params, cls._robot_params)
#
#     @classmethod
#     def add_params(cls, new_params):
#         hello_utils.overwrite_dict(cls._robot_params, new_params)
#
#     @classmethod
#     def set_logging_level(cls, level, handler='console_handler'):
#         level_names={0: 'NOTSET', 10: 'DEBUG', 'WARN': 30, 20: 'INFO', 'ERROR': 40, 'DEBUG': 10, 30:
#             'WARNING', 'INFO': 20, 'WARNING': 30, 40: 'ERROR', 50: 'CRITICAL', 'CRITICAL': 50, 'NOTSET': 0}
#         if level in level_names and handler in cls._robot_params['logging']['handlers']:
#             cls._robot_params['logging']['handlers'][handler]['level'] = level
