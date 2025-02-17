#!/usr/bin/env python3
from stretch_body.hello_utils import *
import stretch_body.wrist_yaw as wrist_yaw
import stretch_body.wrist_pitch as wrist_pitch
import stretch_body.wrist_roll as wrist_roll

print_stretch_re_use()

w=wrist_yaw.WristYaw()
w.startup(threaded=False)
w.stop()
w=wrist_pitch.WristPitch()
w.startup(threaded=False)
w.stop()
w = wrist_roll.WristRoll()
w.startup(threaded=False)
w.stop()


