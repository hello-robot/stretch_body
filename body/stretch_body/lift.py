from __future__ import print_function
from stretch_body.prismatic_joint import PrismaticJoint
from stretch_body.device import Device
from stretch_body.trajectories import PrismaticTrajectory

import time
import math


class Lift(PrismaticJoint):
    """
    API to the Stretch RE1 Lift
    """
    def __init__(self):
        PrismaticJoint.__init__(self, 'lift')

    # ######### Utilties ##############################

    def motor_rad_to_translate_m(self,ang): #input in rad
        d=self.params['pinion_t']*self.params['belt_pitch_m']/math.pi
        lift_m = (math.degrees(ang)/180.0)*math.pi*(d/2)
        return lift_m

    def translate_m_to_motor_rad(self, x):
        d = self.params['pinion_t'] * self.params['belt_pitch_m'] / math.pi
        ang = 180*x/((d/2)*math.pi)
        return math.radians(ang)


    def home4(self, end_pos=0.6,to_positive_stop=True, measuring=False):
        """
        to_positive_stop:
        -- True: Move to the positive direction stop and mark to range_m[1]
        -- False: Move to the negative direction stop and mark to range_m[0]
        measuring: After homing to stop, move to opposite stop and report back measured distance
        return measured range-of-motion if measuring. Return None if not a valide measurement
        """
        if not self.motor.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present'%self.name.capitalize())
            return None
        #Legacy configurations will use pseudo_N. New configurations will be using current_A
        if self.params['contact_model_homing'] == 'pseudo_N':
            contact_thresh_pos=self.params['homing_force_N'][1]
            contact_thresh_neg=self.params['homing_force_N'][0]
        elif self.params['contact_model_homing'] == 'current_A':
            contact_thresh_neg=min(self.params['contact_models']['current_A']['contact_thresh_homing'][0],self.params['contact_models']['current_A']['contact_thresh_default'][0])
            contact_thresh_pos=max(self.params['contact_models']['current_A']['contact_thresh_homing'][1],self.params['contact_models']['current_A']['contact_thresh_default'][1])
        else:
            self.logger.warning('Invalid contact model for %s. Unable to home arm.' % self.name.capitalize())
            return None

        success=True
        print('Homing %s...'%self.name.capitalize())
        prev_guarded_mode = self.motor.gains['enable_guarded_mode']
        prev_sync_mode = self.motor.gains['enable_sync_mode']
        self.motor.enable_guarded_mode()
        self.motor.disable_sync_mode()
        self.motor.reset_pos_calibrated()
        self.push_command()

        pos_direction_m=None
        if to_positive_stop:
            x_goal_1=5.0 #Well past the stop
            x_goal_2=-5.0
        else:
            x_goal_1=-5.0
            x_goal_2=5.0

        #Move to stop
        self.pull_status()
        self.move_by(x_m=x_goal_1, contact_thresh_pos=contact_thresh_pos,contact_thresh_neg=contact_thresh_neg, req_calibration=False,contact_model=self.params['contact_model_homing'])
        self.push_command()

        if self.wait_for_contact(timeout=15.0):
            self.motor.set_command(mode=self.motor.MODE_CURRENT, i_des=self.i_feedforward)
            self.push_command()
            print('Hardstop detected at motor position (rad)',self.motor.status['pos'])
            time.sleep(4.0)
            self.pull_status()
            x_dir_1=self.status['pos']
            if to_positive_stop:
                x=self.translate_m_to_motor_rad(self.params['range_m'][1])
                print('Marking %s position to %f (m)'%(self.name.upper(),self.params['range_m'][1]))
            else:
                x = self.translate_m_to_motor_rad(self.params['range_m'][0])
                print('Marking %s position to %f (m)'%(self.name.upper(),self.params['range_m'][0]))
            if not measuring:
                self.motor.mark_position(x)
                self.motor.set_pos_calibrated()
                self.push_command()

            time.sleep(1.0) #Allow to settle

            #Second direction
            if measuring:
                # Move to other direction
                self.pull_status()
                self.move_by(x_m=x_goal_2, contact_thresh_pos=contact_thresh_pos,contact_thresh_neg=contact_thresh_neg, req_calibration=False,contact_model=self.params['contact_model_homing'])
                self.push_command()
                if self.wait_for_contact(timeout=15.0):
                    print('Second hardstop detected at motor position (rad)', self.motor.status['pos'])
                    time.sleep(1.0)
                    self.pull_status()
                    x_dir_2=self.status['pos']
                else:
                    self.logger.warning('%s homing failed. Failed to detect contact'%self.name.upper())
                    success = False
        else:
            self.logger.warning('%s homing failed. Failed to detect contact'%self.name.upper())
            success = False

        if success:
            self.pretty_print()
            self.move_to(x_m=end_pos,req_calibration=False)
            self.push_command()
            time.sleep(1.0)
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('%s failed to reach final position' % self.name.upper())
                success=False

        # Restore previous modes
        if not prev_guarded_mode:
            self.motor.disable_guarded_mode()
        if prev_sync_mode:
            self.motor.enable_sync_mode()
        self.push_command()
        if success:
            if measuring:
                print('%s range measuing successful: %f (m)' % (self.name.upper(), abs(x_dir_1 - x_dir_2)))
                return abs(x_dir_1 - x_dir_2)
            else:
                print('%s homing successful' % self.name.upper())
        return None


    def homeX(self,end_pos=0.6,to_positive_stop=True, measuring=False):
        """
        to_positive_stop:
        -- True: Move to the positive direction stop and mark to range_m[1]
        -- False: Move to the negative direction stop and mark to range_m[0]
        measuring: After homing to stop, move to opposite stop and report back measured distance
        return measured range-of-motion if measuring. Return None if not a valide measurement
        """
        if not self.motor.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present'%self.name.capitalize())
            return None
        #Legacy configurations will use pseudo_N. New configurations will be using current_A
        if self.params['contact_model_homing'] == 'pseudo_N':
            i_contact_pos, i_contact_neg = self.contact_thresh_to_motor_current(self.params['homing_force_N'][1], self.params['homing_force_N'][0], 'pseudo_N')
        elif self.params['contact_model_homing'] == 'current_A':
            #Pick larger of contact defaults and homing default so that if recalibrated, homing will succeed
            contact_thresh_neg = min(self.params['contact_models']['current_A']['contact_thresh_homing'][0],
                                     self.params['contact_models']['current_A']['contact_thresh_default'][0])
            contact_thresh_pos = max(self.params['contact_models']['current_A']['contact_thresh_homing'][1],
                                     self.params['contact_models']['current_A']['contact_thresh_default'][1])

            i_contact_pos, i_contact_neg = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg,'current_A')

        else:
            self.logger.warning('Invalid contact model for %s. Unable to home arm.' % self.name.capitalize())
            return None


        success=True
        print('Homing %s...'%self.name.capitalize())
        prev_sync_mode = self.motor.gains['enable_sync_mode']
        self.motor.disable_sync_mode()
        self.motor.reset_pos_calibrated()
        self.push_command()

        pos_direction_m=None
        if to_positive_stop:
            i_goal_1=i_contact_pos #Well past the stop
            i_goal_2=-i_contact_neg
        else:
            i_goal_1=i_contact_neg
            i_goal_2=i_contact_pos

        v= self.translate_m_to_motor_rad(0.1)
        self.motor.set_command(mode=self.motor.MODE_VEL_TRAJ,v_des=v)
        self.push_command()

        time.sleep(0.5)

        self.motor.gains['safety_stiffness']=1.0
        self.motor.set_gains(self.motor.gains)
        self.push_command()

        if self.wait_for_contact(timeout=15.0):
            print('Hardstop detected at motor position (rad)', self.motor.status['pos'])
            x_dir_1 = self.status['pos']
            if to_positive_stop:
                x = self.translate_m_to_motor_rad(self.params['range_m'][1])
                print('Marking %s position to %f (m)' % (self.name.upper(), self.params['range_m'][1]))
            else:
                x = self.translate_m_to_motor_rad(self.params['range_m'][0])
                print('Marking %s position to %f (m)' % (self.name.upper(), self.params['range_m'][0]))
            if not measuring:
                self.motor.mark_position(x)
                self.motor.set_pos_calibrated()
                self.push_command()

        self.motor.gains['safety_stiffness'] = 0.0
        self.motor.set_gains(self.motor.gains)
        self.push_command()

    def home_I(self,end_pos=0.6,to_positive_stop=True, measuring=False):
        """
        to_positive_stop:
        -- True: Move to the positive direction stop and mark to range_m[1]
        -- False: Move to the negative direction stop and mark to range_m[0]
        measuring: After homing to stop, move to opposite stop and report back measured distance
        return measured range-of-motion if measuring. Return None if not a valide measurement
        """
        if not self.motor.hw_valid:
            self.logger.warning('Not able to home %s. Hardware not present'%self.name.capitalize())
            return None
        #Legacy configurations will use pseudo_N. New configurations will be using current_A
        if self.params['contact_model_homing'] == 'pseudo_N':
            i_contact_pos, i_contact_neg = self.contact_thresh_to_motor_current(self.params['homing_force_N'][1], self.params['homing_force_N'][0], 'pseudo_N')
        elif self.params['contact_model_homing'] == 'current_A':
            #Pick larger of contact defaults and homing default so that if recalibrated, homing will succeed
            contact_thresh_neg = min(self.params['contact_models']['current_A']['contact_thresh_homing'][0],
                                     self.params['contact_models']['current_A']['contact_thresh_default'][0])
            contact_thresh_pos = max(self.params['contact_models']['current_A']['contact_thresh_homing'][1],
                                     self.params['contact_models']['current_A']['contact_thresh_default'][1])

            i_contact_pos, i_contact_neg = self.contact_thresh_to_motor_current(contact_thresh_pos,contact_thresh_neg,'current_A')

        else:
            self.logger.warning('Invalid contact model for %s. Unable to home arm.' % self.name.capitalize())
            return None


        success=True
        print('Homing %s...'%self.name.capitalize())
        prev_sync_mode = self.motor.gains['enable_sync_mode']
        self.motor.disable_sync_mode()
        self.motor.reset_pos_calibrated()
        self.push_command()

        pos_direction_m=None
        if to_positive_stop:
            i_goal_1=i_contact_pos #Well past the stop
            i_goal_2=-i_contact_neg
        else:
            i_goal_1=i_contact_neg
            i_goal_2=i_contact_pos
        i_goal_1=1.5
        print('GOAL',i_goal_1)
        for i in range(20):
            g=self.params['i_feedforward']+(i_goal_1*(i+1)/20.0)
            print(g)
            self.motor.set_command(mode=self.motor.MODE_CURRENT,i_des=g)
            self.push_command()
            time.sleep(0.1)
        time.sleep(0.5)
        if self.motor.wait_while_is_moving():
            print('Hardstop detected at motor position (rad)', self.motor.status['pos'])
            x_dir_1 = self.status['pos']
            if to_positive_stop:
                x = self.translate_m_to_motor_rad(self.params['range_m'][1])
                print('Marking %s position to %f (m)' % (self.name.upper(), self.params['range_m'][1]))
            else:
                x = self.translate_m_to_motor_rad(self.params['range_m'][0])
                print('Marking %s position to %f (m)' % (self.name.upper(), self.params['range_m'][0]))
            if not measuring:
                self.motor.enable_safety()
                self.push_command()
                self.pull_status()
                self.motor.mark_position(x)
                self.motor.set_pos_calibrated()
                self.push_command()

            #Second direction
            if measuring:
                # Move to other direction
                self.motor.set_command(mode=self.motor.MODE_CURRENT, i_des=i_goal_2)
                self.push_command()
                time.sleep(0.5)

                if self.motor.wait_while_is_moving():
                    print('Second hardstop detected at motor position (rad)', self.motor.status['pos'])
                    time.sleep(1.0)
                    x_dir_2=self.status['pos']
                else:
                    self.logger.warning('%s homing failed. Failed to detect contact'%self.name.upper())
                    success = False
        else:
            self.logger.warning('%s homing failed. Failed to detect contact'%self.name.upper())
            success = False

        time.sleep(1.0) #Allow to settle
        if success:
            self.move_to(x_m=end_pos,req_calibration=False)
            self.push_command()
            time.sleep(1.0)
            if not self.motor.wait_until_at_setpoint():
                self.logger.warning('%s failed to reach final position' % self.name.upper())
                success=False

        # Restore previous modes
        if prev_sync_mode:
            self.motor.enable_sync_mode()
        self.push_command()
        if success:
            if measuring:
                print('%s range measuing successful: %f (m)' % (self.name.upper(), abs(x_dir_1 - x_dir_2)))
                return abs(x_dir_1 - x_dir_2)
            else:
                print('%s homing successful' % self.name.upper())
        return None

    def home(self, end_pos=1.0,to_positive_stop=True, measuring=False):
        return PrismaticJoint.home(self,end_pos=end_pos,to_positive_stop=to_positive_stop,measuring=measuring)


