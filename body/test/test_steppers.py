# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.stepper
import stretch_body.pimu
from stretch_body.hello_utils import evaluate_polynomial_at

import time


class TestSteppers(unittest.TestCase):
    @unittest.skip(reason='Test fails because encoders give noisy reading of is_moving')
    def test_is_moving(self):
        """Test that is_moving is False when no motion
        """
        motors=['/dev/hello-motor-left-wheel','/dev/hello-motor-right-wheel','/dev/hello-motor-arm','/dev/hello-motor-lift']
        for m in motors:
            print('Testing is moving for %s'%m)
            s = stretch_body.stepper.Stepper(m)
            self.assertTrue(s.startup())
            for i in range(50):
                s.pull_status()
                self.assertFalse(s.status['is_moving'])
                time.sleep(0.1)
            s.stop()

    def test_is_moving_filtered(self):
        """Test that is_moving_filtered is False when no motion
        """
        motors=['/dev/hello-motor-left-wheel','/dev/hello-motor-right-wheel','/dev/hello-motor-arm','/dev/hello-motor-lift']
        for m in motors:
            print('Testing is moving filtered for %s'%m)
            s = stretch_body.stepper.Stepper(m)
            self.assertTrue(s.startup())
            for i in range(50):
                s.pull_status()
                s.step_sentry(robot=None)
                self.assertFalse(s.status['is_moving_filtered'])
                time.sleep(0.1)
            s.stop()

    def test_runstop_status(self):
        """Verify that runstop status doesn't fluctuate
        """
        p = stretch_body.pimu.Pimu()
        p.startup()
        s = stretch_body.stepper.Stepper('/dev/hello-motor-arm')
        s.startup()

        p.runstop_event_reset()
        p.push_command()
        time.sleep(1)
        for _ in range(50):
            time.sleep(0.1)
            s.pull_status()
            self.assertFalse(s.status['runstop_on'])

        p.runstop_event_trigger()
        p.push_command()
        time.sleep(1)
        for _ in range(50):
            time.sleep(0.1)
            s.pull_status()
            self.assertTrue(s.status['runstop_on'])

        p.runstop_event_reset()
        p.push_command()
        time.sleep(1)
        for _ in range(50):
            time.sleep(0.1)
            s.pull_status()
            self.assertFalse(s.status['runstop_on'])

        p.stop()
        s.stop()

    def test_position_trajectory_interface(self):
        """Verify correct behavior of the position waypoint interface
        """
        s = stretch_body.stepper.Stepper('/dev/hello-motor-lift')
        self.assertTrue(s.startup())
        limits_rad = (0.0, 115.14478302001953) # lift motor limits
        position_rad = 62.425
        velocity_rad = 9.948
        acceleration_rad = 15.707
        stiffness = 1.0
        i_feedforward = 0.54
        i_contact_neg = -1.46
        i_contact_pos = 2.54
        MODE_POS_TRAJ = 5

        s.set_command(mode=MODE_POS_TRAJ,
                      x_des=position_rad,
                      v_des=velocity_rad,
                      a_des=acceleration_rad,
                      stiffness=stiffness,
                      i_feedforward=i_feedforward,
                      i_contact_pos=i_contact_pos,
                      i_contact_neg=i_contact_neg)
        s.push_command()
        time.sleep(7)

        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], position_rad, places=2)

        s.stop()

    def test_stop_waypoint_trajectory_interface(self):
        """Verify that waypoint trajectories stop as expected
        """
        s = stretch_body.stepper.Stepper('/dev/hello-motor-lift')
        s.disable_sync_mode()
        self.assertTrue(s.startup())
        limits_rad = (0.0, 115.14478302001953)  # lift motor limits


        position_rad = 62.425
        velocity_rad = 9.948
        acceleration_rad = 15.707
        stiffness = 1.0
        i_feedforward = 0.54
        i_contact_neg = -1.46
        i_contact_pos = 2.54
        MODE_POS_TRAJ = 5
        ################################################################
        #stop by sending a duration=0 second segment
        #bring motor to starting position
        print('Waypoint stop test 1')
        s.set_command(mode=MODE_POS_TRAJ,
                      x_des=position_rad,
                      v_des=velocity_rad,
                      a_des=acceleration_rad,
                      stiffness=stiffness,
                      i_feedforward=i_feedforward,
                      i_contact_pos=i_contact_pos,
                      i_contact_neg=i_contact_neg)
        s.push_command()
        s.wait_until_at_setpoint()
        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], position_rad, places=1)

        # send waypoint trajectory
        first_segment = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249, 2]
        second_segment = [0.0, 52.35, 0, 0, 3.731, -1.866, 0.249, 3] #duration=O marks stop
        s.enable_pos_traj_waypoint()
        s.push_command()
        time.sleep(0.1)
        start_time = time.time()
        self.assertTrue(s.start_waypoint_trajectory(first_segment))

        # first segment executing,
        for _ in range(10):
            s.pull_status()
            self.assertEqual(s.status['waypoint_traj']['segment_id'], 2)
            time.sleep(0.1)
            self.assertTrue(s.set_next_trajectory_segment(second_segment))
            s.pull_status()
            self.assertLessEqual(s.status['pos'], 62.425 + 1.0)
            self.assertGreaterEqual(s.status['pos'], 52.35 - 1.0)
            pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], time.time() - start_time)
            self.assertAlmostEqual(s.status['pos'], pos, places=-1)
        time.sleep(2)  # let the remainder of the first segment complete
        s.pull_status()
        pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], first_segment[0])
        self.assertAlmostEqual(s.status['pos'], pos, places=1)
        self.assertEqual(s.status['waypoint_traj']['state'], 'idle')

        # ################################################################
        # stop by sending only one segment
        # bring motor to starting position
        print('Waypoint stop test 2')
        s.set_command(mode=MODE_POS_TRAJ,
                      x_des=position_rad,
                      v_des=velocity_rad,
                      a_des=acceleration_rad,
                      stiffness=stiffness,
                      i_feedforward=i_feedforward,
                      i_contact_pos=i_contact_pos,
                      i_contact_neg=i_contact_neg)
        s.push_command()
        s.wait_until_at_setpoint()
        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], position_rad, places=1)

        # send waypoint trajectory
        first_segment = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249, 2]
        s.enable_pos_traj_waypoint()
        s.push_command()
        time.sleep(0.1)
        start_time = time.time()
        self.assertTrue(s.start_waypoint_trajectory(first_segment))

        # first segment executing, stop by sending a duration=0 second segment
        for _ in range(10):
            s.pull_status()
            self.assertEqual(s.status['waypoint_traj']['segment_id'], 2)
            time.sleep(0.1)
            self.assertLessEqual(s.status['pos'], 62.425 + 1.0)
            self.assertGreaterEqual(s.status['pos'], 52.35 - 1.0)
            pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], time.time() - start_time)
            self.assertAlmostEqual(s.status['pos'], pos, places=-1)
        time.sleep(2)  # let the remainder of the first segment complete
        s.pull_status()
        pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], first_segment[0])
        self.assertAlmostEqual(s.status['pos'], pos, places=1)
        self.assertEqual(s.status['waypoint_traj']['state'], 'idle')

        # ################################################################
        print('Waypoint stop test 3')
        # stop by terminating directly
        # bring motor to starting position
        s.set_command(mode=MODE_POS_TRAJ,
                      x_des=position_rad,
                      v_des=velocity_rad,
                      a_des=acceleration_rad,
                      stiffness=stiffness,
                      i_feedforward=i_feedforward,
                      i_contact_pos=i_contact_pos,
                      i_contact_neg=i_contact_neg)
        s.push_command()
        s.wait_until_at_setpoint()
        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], position_rad, places=1)
        print('Moved to pos')
        # send waypoint trajectory
        first_segment = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249, 2]
        s.enable_pos_traj_waypoint()
        s.push_command()
        time.sleep(0.1)
        start_time = time.time()
        self.assertTrue(s.start_waypoint_trajectory(first_segment))

        # first segment executing, stop by sending a duration=0 second segment
        for _ in range(10):
            s.pull_status()
            self.assertEqual(s.status['waypoint_traj']['segment_id'], 2)
            time.sleep(0.1)
            self.assertLessEqual(s.status['pos'], 62.425 + 1.0)
            self.assertGreaterEqual(s.status['pos'], 52.35 - 1.0)
            pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], time.time() - start_time)
            self.assertAlmostEqual(s.status['pos'], pos, places=-1)
        s.stop_waypoint_trajectory() #End prematurely
        dt=time.time() - start_time
        s.pull_status()
        pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], dt)
        self.assertAlmostEqual(s.status['pos'], pos, delta=1.0)
        self.assertEqual(s.status['waypoint_traj']['state'], 'idle')
        s.stop()


    def test_malicious_waypoint_trajectory_interface(self):
        #Send bad values to trajectory interface
        #Joint should not move
        s = stretch_body.stepper.Stepper('/dev/hello-motor-lift')
        s.disable_sync_mode()
        self.assertTrue(s.startup())

        # send waypoint trajectory
        first_segment = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249, 0] #Bad ID
        second_segment = [3.0, 52.35, 0, 0, 3.731, -1.866, 0.249, 1] #Bad ID
        s.enable_pos_traj_waypoint()
        s.push_command()
        x_start = s.status['pos']
        self.assertEqual(s.status['waypoint_traj']['state'],'idle')
        self.assertFalse(s.start_waypoint_trajectory(first_segment))
        self.assertFalse(s.start_waypoint_trajectory(second_segment))
        self.assertFalse(s.set_next_trajectory_segment(first_segment))
        self.assertFalse(s.set_next_trajectory_segment(second_segment))
        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], x_start, places=1)
        self.assertEqual(s.status['waypoint_traj']['state'], 'idle')


    def test_waypoint_trajectory_interface(self):
        """Verify correct behavior of the waypoint trajectory

        Spline coefficients calculated in Desmos at:
        https://www.desmos.com/calculator/atv5ilhodq
        """
        s = stretch_body.stepper.Stepper('/dev/hello-motor-lift')
        s.disable_sync_mode()
        self.assertTrue(s.startup())
        limits_rad = (0.0, 115.14478302001953)  # lift motor limits

        # bring motor to starting position
        position_rad = 62.425
        velocity_rad = 9.948
        acceleration_rad = 15.707
        stiffness = 1.0
        i_feedforward = 0.54
        i_contact_neg = -1.46
        i_contact_pos = 2.54
        MODE_POS_TRAJ = 5
        s.set_command(mode=MODE_POS_TRAJ,
                      x_des=position_rad,
                      v_des=velocity_rad,
                      a_des=acceleration_rad,
                      stiffness=stiffness,
                      i_feedforward=i_feedforward,
                      i_contact_pos=i_contact_pos,
                      i_contact_neg=i_contact_neg)
        s.push_command()
        s.wait_until_at_setpoint()
        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], position_rad, places=1)

        # send waypoint trajectory
        first_segment = [3.0, 62.425, 0, 0, -3.731, 1.866, -0.249, 2]
        second_segment = [3.0, 52.35, 0, 0, 3.731, -1.866, 0.249, 3]
        s.enable_pos_traj_waypoint()
        # s.set_command(v_des=velocity_rad,
        #               a_des=acceleration_rad,
        #               i_feedforward=i_feedforward,
        #               i_contact_pos=i_contact_pos,
        #               i_contact_neg=i_contact_neg)
        s.push_command()
        time.sleep(0.1)
        start_time = time.time()
        self.assertTrue(s.start_waypoint_trajectory(first_segment))

        # first segment executing
        for _ in range(10):
            s.pull_status()
            self.assertEqual(s.status['waypoint_traj']['segment_id'], 2)
            print(s.status['waypoint_traj']['segment_id'])
            time.sleep(0.1)
            self.assertTrue(s.set_next_trajectory_segment(second_segment))
            s.pull_status()
            self.assertLessEqual(s.status['pos'], 62.425 + 1.0)
            self.assertGreaterEqual(s.status['pos'], 52.35 - 1.0)
            pos, _, _ = evaluate_polynomial_at(first_segment[1:-1], time.time() - start_time)
            self.assertAlmostEqual(s.status['pos'], pos, places=-1)
        time.sleep(2)  # let the remainder of the first segment complete

        # second segment executing
        for _ in range(10):
            s.pull_status()
            self.assertEqual(s.status['waypoint_traj']['segment_id'], 3)
            print(s.status['waypoint_traj']['segment_id'])
            time.sleep(0.1)
            s.pull_status()
            self.assertLessEqual(s.status['pos'], 62.425 + 1.0)
            self.assertGreaterEqual(s.status['pos'], 52.35 - 1.0)
            pos, _, _ = evaluate_polynomial_at(second_segment[1:-1], (time.time() - start_time) - 3.0)
            self.assertAlmostEqual(s.status['pos'], pos, places=-1)
        time.sleep(2)  # let the remainder of the second segment complete

        s.pull_status()
        self.assertAlmostEqual(s.status['pos'], position_rad, places=1)
        s.stop()
