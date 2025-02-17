#!/usr/bin/env python3
import os
import yaml

import argparse
parser=argparse.ArgumentParser(description='Display model and serial number information')
args=parser.parse_args()


params_filename = os.environ['HELLO_FLEET_PATH']+'/'+os.environ['HELLO_FLEET_ID']+'/stretch_configuration_params.yaml'

with open(params_filename, 'r') as fid:
    params = yaml.safe_load(fid)
    robot_info = params['robot']
    batch_name = robot_info['batch_name']
    serial_number = robot_info['serial_no']
    model=robot_info['model_name']

batch_name_string = 'Stretch Batch Name: {0}'.format(batch_name)
serial_number_string = 'Stretch Serial Number: {0}'.format(serial_number)
model_string = 'Stretch Model: {0}'.format(model)

print(batch_name_string)
print(serial_number_string)
print(model_string)
