import time
import random
import stretch_body.robot
# Required Hack whenever importing cv2 (stretch_body does it implicitly). Details: https://forum.hello-robot.com/t/1047
import os; os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'
from multiprocessing_plotter import NBPlot

r = stretch_body.robot.Robot()
assert r.startup()
assert r.is_homed()

def low_write():
    r.lift.move_to(0.3)
    r.arm.move_to(0.0)
    r.end_of_arm.move_to('wrist_roll', -1.0)
    r.head.move_to('head_pan', -1.0)
    r.push_command()

def high_write():
    r.lift.move_to(1.1)
    r.arm.move_to(0.4)
    r.end_of_arm.move_to('wrist_roll', 1.0)
    r.head.move_to('head_pan', 1.0)
    r.push_command()

def read(plotter):
    lift_pos = r.lift.status['pos']
    arm_pos = r.arm.status['pos']
    roll_pos = r.end_of_arm.get_joint('wrist_roll').status['pos']
    pan_pos = r.head.get_joint('head_pan').status['pos']
    plotter.plot(lift_pos, arm_pos, roll_pos, pan_pos)

def low_rw_runner():
    """read-and-write runner"""
    pl=NBPlot()
    while True:
        low_write()
        read(pl)
        time.sleep(0.1)

def high_rw_runner():
    """read-and-write runner"""
    pl=NBPlot()
    s = time.time()
    while time.time() - s < 10:
        high_write()
        read(pl)
        time.sleep(0.1)

if __name__ == "__main__":
    import threading
    threading.Thread(target=low_rw_runner).start()
    threading.Thread(target=high_rw_runner).start()
