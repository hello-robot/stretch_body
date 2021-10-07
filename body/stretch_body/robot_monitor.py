from __future__ import print_function
from stretch_body.device import Device


class RobotMonitor(Device):
    """
    The RobotMonitor class provide system level monitoring of Status data
    for important events. These events are logged to file (and optionally console)
    The events to be monitored may be turned on/off/configured via YAML
    The RobotMonitor is managed by the Robot class
    It runs at 5Hz
    """
    def __init__(self, robot):
        Device.__init__(self, 'robot_monitor')
        self.robot=robot

    def startup(self):
        Device.startup(self, threaded=False)
        if self.robot.wacc is not None:
            stc=self.robot.wacc.status['single_tap_count']
        else:
            stc=None

        if self.robot.pimu is not None:
            lva=self.robot.pimu.config['low_voltage_alert']
            hca=self.robot.pimu.config['high_current_alert']
            bec=self.robot.pimu.status['bump_event_cnt']
        else:
            lva=None
            hca=None
            bec=None

        self.monitor_history={'monitor_dynamixel_flags':{}, 'monitor_voltage':lva,'monitor_current':hca,
                              'monitor_guarded_contact':{},'monitor_wrist_single_tap':stc,
                              'monitor_base_cliff_event':0,'monitor_base_bump_event':bec,'monitor_over_tilt_alert':0,'monitor_runstop':0}
        return True

    def step(self):
        if self.params['monitor_voltage']:
            self.monitor_voltage()
        if self.params['monitor_current']:
            self.monitor_current()
        if self.params['monitor_runstop']:
            self.monitor_runstop()
        if self.params['monitor_dynamixel_flags']:
            self.monitor_dynamixel_flags()
        if self.params['monitor_guarded_contact']:
            self.monitor_guarded_contact()
        if self.params['monitor_wrist_single_tap']:
            self.monitor_wrist_single_tap()
        if self.params['monitor_base_cliff_event']:
            self.monitor_base_cliff_event()
        if self.params['monitor_base_bump_event']:
            self.monitor_base_bump_event()
        if self.params['monitor_over_tilt_alert']:
            self.monitor_over_tilt_alert()


    # ##################################
    def monitor_base_cliff_event(self):
        if self.robot.pimu is not None:
            if self.robot.pimu.status['cliff_event'] and  self.monitor_history['monitor_base_cliff_event']==0:
                self.logger.debug("Base cliff event")
            self.monitor_history['monitor_base_cliff_event'] = self.robot.pimu.status['cliff_event']

    # ##################################
    def monitor_base_bump_event(self):
        if self.robot.pimu is not None:
            if self.robot.pimu.status['bump_event_cnt'] != self.monitor_history['monitor_base_bump_event']:
                self.logger.debug("Base bump event")
            self.monitor_history['monitor_base_bump_event'] = self.robot.pimu.status['bump_event_cnt']

    # ##################################
    def monitor_over_tilt_alert(self):
        if self.robot.pimu is not None:
            if self.robot.pimu.status['over_tilt_alert'] and self.monitor_history['monitor_over_tilt_alert'] == 0:
                self.logger.debug("Over Tilt Alert")
            self.monitor_history['monitor_over_tilt_alert'] = self.robot.pimu.status['over_tilt_alert']

    # ##################################
    def monitor_wrist_single_tap(self):
        if self.robot.wacc is not None:
            if self.robot.wacc.status['single_tap_count']!=self.monitor_history['monitor_wrist_single_tap']:
                self.logger.debug("Wrist single tap: %d" % self.robot.wacc.status['single_tap_count'])
            self.monitor_history['monitor_wrist_single_tap']=self.robot.wacc.status['single_tap_count']

    # ##################################
    def monitor_guarded_contact(self):
        joints=[self.robot.lift, self.robot.arm]
        mn='monitor_guarded_contact'
        for j in joints:
            if j is not None:
                if j.name not in self.monitor_history[mn]:# Init history
                    self.monitor_history[mn][j.name] = 0
                if j is not None:
                    if self.monitor_history[mn][j.name]==0 and j.motor.status['in_guarded_event']:
                        self.logger.debug("Guarded contact %s"%j.name)
                self.monitor_history[mn][j.name] =j.motor.status['in_guarded_event']

    # ##################################
    def monitor_dynamixel_flags(self):
        chains=[self.robot.head,self.robot.end_of_arm]
        mn='monitor_dynamixel_flags'
        errs = ['overheating_error', 'overload_error']
        for c in chains:
            if c is not None:
                #Init history
                if c.name not in self.monitor_history[mn]:
                    self.monitor_history[mn][c.name]={}
                    for k in c.motors.keys():
                        self.monitor_history[mn][c.name][k] = {}
                        for e in errs:
                            self.monitor_history[mn][c.name][k][e]=0
                #Flag changes from 0 to 1 on error
                for k in c.motors.keys():
                    for e in errs:
                        if c.motors[k].status[e] and not self.monitor_history[mn][c.name][k][e]:
                            self.logger.debug("Dynamixel %s on %s:%s"%(e,c.name,k))
                        self.monitor_history[mn][c.name][k][e] = c.motors[k].status[e]

    # ##################################
    def monitor_runstop(self):
        if self.robot.status['pimu']['runstop_event'] != self.monitor_history['monitor_runstop']:
            if self.robot.status['pimu']['runstop_event']:
                self.logger.debug("Runstop activated")
            else:
                self.logger.debug("Runstop deactivated")
        self.monitor_history['monitor_runstop']=self.robot.status['pimu']['runstop_event']

    # ##################################
    def monitor_voltage(self):
        v=self.robot.pimu.status['voltage']
        if v < self.robot.pimu.config['low_voltage_alert']:
            if v-self.monitor_history['monitor_voltage']<-0.5:#every 0.5V of drop allow to report
                self.logger.debug('Low voltage of: %.2f'%v)
                self.monitor_history['monitor_voltage']=v
        else:
            self.monitor_history['monitor_voltage'] =v

    # ##################################
    def monitor_current(self):
        i=self.robot.pimu.status['current']
        if i > self.robot.pimu.config['high_current_alert']:
            if i-self.monitor_history['monitor_current']>0.25:#every 0.25A of rise allow to report
                self.logger.debug('High current of: %.2f'%i)
                self.monitor_history['monitor_current']=i
        else:
            self.monitor_history['monitor_current'] =i
