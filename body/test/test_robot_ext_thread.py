# Logging level must be set before importing any stretch_body class
from collections.abc import Callable, Iterable, Mapping
from typing import Any
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
from stretch_body.robot import Robot
import time
import traceback
import threading 


GLOBAL_EXCEPTIONS_LIST = []

def custom_excepthook(args):
    global GLOBAL_EXCEPTIONS_LIST
    thread_name = args.thread.name
    exec = {}
    exec[thread_name] = {
        'thread': args.thread,
        'exception': {
            'type': args.exc_type,
            'value': args.exc_value,
            'traceback': args.exc_traceback
            }
        }
    GLOBAL_EXCEPTIONS_LIST.append(exec[thread_name])



class ExternalThread(threading.Thread):
    def __init__(self, robot):
        threading.Thread.__init__(self)
        self.robot = robot
    
    def run(self):
        for i in range(100):
            self.robot.push_command()
            self.robot.pull_status()
            self.robot.end_of_arm.get_joint("wrist_yaw").set_velocity(0)
            time.sleep(0.01)
            
# https://stackoverflow.com/questions/2829329/catch-a-threads-exception-in-the-caller-thread

class TestRobotExtThreads(unittest.TestCase):
    """
    Test to catch fails/excepts in an external when robot class is shared to the external thread
    """
    @classmethod
    def setUpClass(self):
        self.exceptions_caught_in_threads = []
        self.robot = Robot()
        threading.excepthook = custom_excepthook
        self.thread = ExternalThread(self.robot)
        self.thread.start()

    @classmethod
    def tearDownClass(self):
        global GLOBAL_EXCEPTIONS_LIST
        self.robot.stop()
        if self.thread:
            self.thread.join()
        if len(GLOBAL_EXCEPTIONS_LIST):
            for e in GLOBAL_EXCEPTIONS_LIST: 
                self.raise_custom_exception(self,exception_type=e['exception']['type'],
                                            exception_value=e['exception']['value'],
                                            tb=e['exception']['traceback'])
        
    def test_robot_ext_thread_hooks(self):
        self.assertTrue(self.robot.startup()) 
        for i in range(100):
            time.sleep(0.01)
    
    def raise_custom_exception(self,exception_type, exception_value, tb):
        try:
            raise exception_type(exception_value)
        except Exception as e:
            raise e.with_traceback(tb)
            