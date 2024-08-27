import time
import random
import stretch_body.robot
# Required Hack whenever importing cv2 (stretch_body does it implicitly). Details: https://forum.hello-robot.com/t/1047
import os; os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = '/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so'
from multiprocessing_plotter import NBPlot

r = stretch_body.robot.Robot()
assert r.startup()
assert r.is_homed()

def write():
    r.lift.move_to(random.uniform(0.3, 1.1)) # not thread-safe
    r.arm.move_to(random.uniform(0.0, 0.4))  # not thread-safe
    r.end_of_arm.move_to('wrist_roll', random.uniform(-1.0, 1.0))
    r.head.move_to('head_pan', random.uniform(-1.0, 1.0))
    r.push_command()

def read(plotter):
    lift_pos = r.lift.status['pos']
    arm_pos = r.arm.status['pos']
    roll_pos = r.end_of_arm.get_joint('wrist_roll').status['pos']
    pan_pos = r.head.get_joint('head_pan').status['pos']
    plotter.plot(lift_pos, arm_pos, roll_pos, pan_pos)

def ro_runner():
    """read-only runner"""
    pl=NBPlot()
    while True:
        read(pl)
        time.sleep(0.1)

def rw_runner():
    """read-and-write runner"""
    pl=NBPlot()
    while True:
        write()
        read(pl)
        time.sleep(0.1)

def wo_runner():
    """write-only runner"""
    while True:
        write()
        time.sleep(0.1)

def read_wo_plotting():
    lift_pos = r.lift.status['pos']; print('lift:', lift_pos)
    arm_pos = r.arm.status['pos']; print('arm:', arm_pos)
    roll_pos = r.end_of_arm.get_joint('wrist_roll').status['pos']; print('roll:', roll_pos)
    pan_pos = r.head.get_joint('head_pan').status['pos']; print('pan:', pan_pos)

def ro_woplotter_runner():
    """read-only runner"""
    while True:
        read_wo_plotting()
        time.sleep(0.1)

if __name__ == "__main__":
    import threading
    threading.Thread(target=ro_woplotter_runner).start()
