import unittest
from stretch_body import robot as rb
import copy

class TestRobotMovement(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.robot = rb.Robot()
        cls.robot.startup()
        cls.robot.home()

    @classmethod
    def tearDownClass(cls):
        cls.robot.stop() 

    def test_movement_accuracy(self):
        lift_inc = 0.4
        arm_inc = 0.3
        base_inc = 0.5
        wrist_inc = 0.5

        self.robot.pull_status()
        status_before = copy.deepcopy(self.robot.status)

        self.robot.base.translate_by(base_inc)
        self.robot.lift.move_by(lift_inc, 0.05)
        self.robot.arm.move_by(arm_inc, 0.05)
        self.robot.end_of_arm.move_by('wrist_yaw', wrist_inc, 0.05)
        self.robot.end_of_arm.move_by('wrist_pitch', wrist_inc, 0.05)
        self.robot.end_of_arm.move_by('wrist_roll', wrist_inc, 0.05)
        self.robot.push_command()
        
        self.robot.wait_command()

        self.robot.pull_status()
        status_after = copy.deepcopy(self.robot.status)

        errors = {}
        errors['base'] = abs(status_after['base']['x'] - status_before['base']['x'])
        errors['lift'] = abs(status_after['lift']['pos'] - status_before['lift']['pos'])
        errors['arm'] = abs(status_after['arm']['pos'] - status_before['arm']['pos'])
        errors['wrist_yaw'] = abs(status_after['end_of_arm']['wrist_yaw']['pos'] - status_before['end_of_arm']['wrist_yaw']['pos'])
        errors['wrist_pitch'] = abs(status_after['end_of_arm']['wrist_pitch']['pos'] - status_before['end_of_arm']['wrist_pitch']['pos'])
        errors['wrist_roll'] = abs(status_after['end_of_arm']['wrist_roll']['pos'] - status_before['end_of_arm']['wrist_roll']['pos'])

        print("Set Point Errors:")
        print(errors)

        self.assertAlmostEqual(errors['base'], base_inc, 2,"base movement error exceeds tolerance")
        self.assertAlmostEqual(errors['lift'], lift_inc, 2,"lift movement error exceeds tolerance")
        self.assertAlmostEqual(errors['arm'], arm_inc, 2,"arm movement error exceeds tolerance")
        self.assertAlmostEqual(errors['wrist_yaw'], wrist_inc, 1,"wrist_yaw movement error exceeds tolerance")
        self.assertAlmostEqual(errors['wrist_pitch'], wrist_inc, 1,"wrist_pitch movement error exceeds tolerance")
        self.assertAlmostEqual(errors['wrist_roll'], wrist_inc, 1,"wrist_roll movement error exceeds tolerance")


if __name__ == '__main__':
    unittest.main()