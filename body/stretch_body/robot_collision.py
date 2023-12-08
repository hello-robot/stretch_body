#! /usr/bin/env python

from stretch_body.device import Device
import importlib
import urchin as urdf_loader
import numpy as np
import os

# #######################################################################

class RobotCollision(Device):
    """
    Deprecated
    """
    def __init__(self,robot):
        print('This version of the collision system has been deprecated')
        print('See forum post https://forums.hello-robot.com/xxx')
    def startup(self):
        pass
    def step(self):
        pass
