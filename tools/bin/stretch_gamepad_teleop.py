#!/usr/bin/env python3
from __future__ import print_function
from stretch_body.gamepad_teleop import GamePadTeleop
from stretch_body.hello_utils import print_stretch_re_use

print_stretch_re_use()

if __name__ == "__main__":
   gamepad_teleop = GamePadTeleop(collision_mgmt=True)
   gamepad_teleop.startup()
   gamepad_teleop.mainloop()
