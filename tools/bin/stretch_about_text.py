#!/usr/bin/env python3
import os
import yaml

factory_params_filename = os.environ['HELLO_FLEET_PATH']+'/'+os.environ['HELLO_FLEET_ID']+'/stretch_re1_factory_params.yaml'

with open(factory_params_filename, 'r') as fid:
    factory_params = yaml.safe_load(fid)
    robot_info = factory_params['robot']
    batch_name = robot_info['batch_name']
    serial_number = robot_info['serial_no']

batch_name_string = 'Stretch Batch Name: {0}'.format(batch_name)
serial_number_string = 'Stretch Serial Number: {0}'.format(serial_number)

print(batch_name_string)
print(serial_number_string)
