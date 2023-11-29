#! /usr/bin/env python

from stretch_body.device import Device

# #######################################################################

class RobotCollision(Device):
    """
        Deprecated. Keep shell class (for now) to avoid breaking legacy code.
    """
    def __init__(self,robot):
        print('-----------')
        print('RobotCollision has been deprecated.')
        print('Use RobotCollisionMgmt instead')
        print('See KB post forum.hello-robot.com/xxx')
        print('-----------')

    def startup(self):
        pass

    def enable_model(self,name):
        pass

    def disable_model(self,name):
        pass


    def step(self):
        pass

