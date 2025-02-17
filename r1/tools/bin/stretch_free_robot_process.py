#!/usr/bin/env python3
from stretch_body.hello_utils import free_body_filelock

did_free = free_body_filelock()
if did_free:
    print('Done!')
else:
    print('Failed because robot is being used by another user. To force kill the process, try running "sudo -E env PATH=$PATH stretch_free_robot_process.py"')
