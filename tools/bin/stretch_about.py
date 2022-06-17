#!/usr/bin/env python
from __future__ import print_function
import os
import yaml
import cv2

image_file = "/etc/hello-robot/stretch_about.png"
image = cv2.imread(image_file)
if image is None:
    print("Unable to find stretch_about.png in /etc/hello-robot")
    exit()

configuration_params_filename = os.environ['HELLO_FLEET_PATH']+'/'+os.environ['HELLO_FLEET_ID']+'/stretch_configuration_params.yaml'

with open(configuration_params_filename, 'r') as fid:
    configuration_params = yaml.safe_load(fid)
    robot_info = configuration_params['robot']
    batch_name = robot_info['batch_name']
    serial_number = robot_info['serial_no']
    model=robot_info['model_name']

# Write information on the image
font = cv2.FONT_HERSHEY_SIMPLEX #cv2.FONT_HERSHEY_PLAIN
font_scale = 0.7
line_color = [18, 40, 55]
line_width = 1

batch_name_string = 'Batch Name: {0}'.format(batch_name)
serial_number_string = 'Serial Number: {0}'.format(serial_number)
model_string = 'Model: {0}'.format(model)
color = (0, 0, 255)

# see the following page for a helpful reference
# https://stackoverflow.com/questions/51285616/opencvs-gettextsize-and-puttext-return-wrong-size-and-chop-letters-with-low

text_x = 310
text_y = 375

vertical_spacing_pix = 40

cv2.rectangle(image, (280, 333), (720, 443), (245, 241, 253), cv2.FILLED)
cv2.putText(image, batch_name_string, (text_x, text_y), font, font_scale, line_color, line_width, cv2.LINE_AA)
cv2.putText(image, serial_number_string, (text_x, text_y + vertical_spacing_pix), font, font_scale, line_color, line_width, cv2.LINE_AA)
cv2.putText(image, model_string, (text_x, text_y + vertical_spacing_pix*2), font, font_scale, line_color, line_width, cv2.LINE_AA)
window_name = 'S T R E T C H RESEARCH EDITION'
cv2.imshow(window_name, image)
# Used exiting advice found via the following link:
# https://stackoverflow.com/questions/35003476/opencv-python-how-to-detect-if-a-window-is-closed
while cv2.getWindowProperty(window_name, 0) >= 0:
    key_code = cv2.waitKey(1)
    if key_code > 0:
        # key pressed
        cv2.destroyAllWindows()
        break        
    elif cv2.getWindowProperty(window_name,cv2.WND_PROP_VISIBLE) < 1:
        # window closed
        break     
