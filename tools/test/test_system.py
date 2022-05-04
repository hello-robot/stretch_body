import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import os, fnmatch
from pprint import pformat
import stretch_body.device
import stretch_body.pimu
import stretch_body.hello_utils


def system_check_warn(warning=None):
    def decorator(test_item):
        test_item.__system_check_warn__ = True
        test_item.__system_check_warning__ = warning
        return test_item
    return decorator


class TestUSBDevices(unittest.TestCase):
    """Checking USB devices
    """

    def test_usb_aliases_present(self):
        """All USB aliases present
        """
        listOfDevices = os.listdir('/dev')
        usb_aliases = {'hello-wacc': False,
                       'hello-pimu': False,
                       'hello-dynamixel-head': False,
                       'hello-dynamixel-wrist': False,
                       'hello-motor-arm': False,
                       'hello-motor-left-wheel': False,
                       'hello-motor-right-wheel': False,
                       'hello-motor-lift': False,
                       'hello-lrf': False,
                       'hello-respeaker': False}
        for entry in listOfDevices:
            if fnmatch.fnmatch(entry, "hello*"):
                if entry in usb_aliases.keys():
                    usb_aliases[entry] = True
        self.assertTrue(all(list(usb_aliases.values())), msg='missing usb aliases={0}'.format([k for k, v in usb_aliases.items() if not v]))

    def test_num_acm_devices_present(self):
        """Six ACM devices present
        """
        dummy_device = stretch_body.device.Device('dummy')
        listOfDevices = os.listdir('/dev')

        # debug log ACM aliases and their linked devices
        acm_aliases = {'hello-wacc': None,
                       'hello-pimu': None,
                       'hello-motor-arm': None,
                       'hello-motor-left-wheel': None,
                       'hello-motor-right-wheel': None,
                       'hello-motor-lift': None}
        for entry in listOfDevices:
            if fnmatch.fnmatch(entry, "hello*"):
                if entry in acm_aliases.keys():
                    acm_aliases[entry] = os.readlink('/dev/{0}'.format(entry))
        dummy_device.logger.debug(pformat(acm_aliases))

        # expect 6 ACM devices present
        acm_devices = [entry for entry in listOfDevices if fnmatch.fnmatch(entry, "ttyACM*")]
        num_acm_devices = len(acm_devices)
        self.assertEqual(num_acm_devices, 6, msg="found={0}".format(acm_devices))


class TestPIMU(unittest.TestCase):
    """Checking Power + IMU Board (PIMU)
    """

    @classmethod
    def setUpClass(self):
        self.p = stretch_body.pimu.Pimu()
        self.p.startup()
        self.dummy = stretch_body.device.Device('dummy')

    @classmethod
    def tearDownClass(self):
        self.p.stop()

    def test_valid_voltage(self):
        """Voltage in range
        """
        voltage = self.p.status['voltage']
        self.dummy.logger.debug('voltage={0}'.format(voltage))
        self.assertGreater(voltage, self.p.config['low_voltage_alert'])
        self.assertLess(voltage, 14.5)

    def test_valid_current(self):
        """Current in range
        """
        current = self.p.status['current']
        self.dummy.logger.debug('current={0}'.format(current))
        self.assertGreater(current, 0.5)
        self.assertLess(current, self.p.config['high_current_alert'])

    def test_valid_temperature(self):
        """Temperature in range
        """
        temperature = self.p.status['temp']
        self.dummy.logger.debug('temperature={0}'.format(temperature))
        self.assertGreater(temperature, 10)
        self.assertLess(temperature, 40)

    @system_check_warn(warning="run RE1_cliff_sensor_calibrate.py")
    def test_valid_cliff0(self):
        """Cliff-0 in range
        """
        cliff = self.p.status['cliff_range'][0]
        self.dummy.logger.debug('cliff-0={0}'.format(cliff))
        self.assertGreater(cliff, self.p.config['cliff_thresh'])
        self.assertLess(cliff, 20)

    @system_check_warn(warning="run RE1_cliff_sensor_calibrate.py")
    def test_valid_cliff1(self):
        """Cliff-1 in range
        """
        cliff = self.p.status['cliff_range'][1]
        self.dummy.logger.debug('cliff-1={0}'.format(cliff))
        self.assertGreater(cliff, self.p.config['cliff_thresh'])
        self.assertLess(cliff, 20)

    @system_check_warn(warning="run RE1_cliff_sensor_calibrate.py")
    def test_valid_cliff2(self):
        """Cliff-2 in range
        """
        cliff = self.p.status['cliff_range'][2]
        self.dummy.logger.debug('cliff-2={0}'.format(cliff))
        self.assertGreater(cliff, self.p.config['cliff_thresh'])
        self.assertLess(cliff, 20)

    @system_check_warn(warning="run RE1_cliff_sensor_calibrate.py")
    def test_valid_cliff3(self):
        """Cliff-3 in range
        """
        cliff = self.p.status['cliff_range'][3]
        self.dummy.logger.debug('cliff-3={0}'.format(cliff))
        self.assertGreater(cliff, self.p.config['cliff_thresh'])
        self.assertLess(cliff, 20)

    def test_valid_imu_az(self):
        """Accelerometer Z-axis in range
        """
        az = self.p.status['imu']['az']
        self.dummy.logger.debug('accelerometer z-axis={0}'.format(az))
        self.assertGreater(az, -10.1)
        self.assertLess(az, -9.5)

    def test_valid_imu_pitch(self):
        """Gyroscope pitch in range
        """
        pitch = stretch_body.hello_utils.rad_to_deg(self.p.status['imu']['pitch'])
        self.dummy.logger.debug('gyroscope pitch (deg)={0}'.format(pitch))
        self.assertGreater(pitch, -12)
        self.assertLess(pitch, 12)

    def test_valid_imu_roll(self):
        """Gyroscope roll in range
        """
        roll = stretch_body.hello_utils.rad_to_deg(self.p.status['imu']['roll'])
        self.dummy.logger.debug('gyroscope roll (deg)={0}'.format(roll))
        self.assertGreater(roll, -12)
        self.assertLess(roll, 12)
