import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import distro
import unittest
import requests
import xmltodict
import importlib
import subprocess
import os, fnmatch
from pprint import pformat
import stretch_body.device
import stretch_body.pimu
import stretch_body.hello_utils
import stretch_body.head
import stretch_body.wacc
import stretch_body.base
import stretch_body.lift
import stretch_body.arm
import stretch_factory.firmware_updater


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
        dummy_device = stretch_body.device.Device('dummy', req_params=False)

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
        usb_links = {}
        listOfDevices = os.listdir('/dev')
        for entry in listOfDevices:
            if fnmatch.fnmatch(entry, "hello*"):
                if entry in usb_aliases.keys():
                    usb_aliases[entry] = True
                    usb_links[entry] = os.readlink('/dev/{0}'.format(entry))
        dummy_device.logger.debug(pformat(usb_links))
        self.assertTrue(all(list(usb_aliases.values())), msg='missing usb aliases={0}'.format([k for k, v in usb_aliases.items() if not v]))

    def test_num_acm_devices_present(self):
        """Six ACM devices present
        """
        listOfDevices = os.listdir('/dev')
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
        self.dummy = stretch_body.device.Device('dummy', req_params=False)

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
        self.dummy.logger.debug('pimu accelerometer z-axis={0}'.format(az))
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


class TestEndOfArm(unittest.TestCase):
    """Checking End of Arm
    """

    @classmethod
    def setUpClass(self):
        dummy = stretch_body.device.Device('dummy', req_params=False)
        tool_name = dummy.robot_params['robot']['tool']
        module_name = dummy.robot_params[tool_name]['py_module_name']
        class_name = dummy.robot_params[tool_name]['py_class_name']
        self.e = getattr(importlib.import_module(module_name), class_name)()
        self.e.startup()

    @classmethod
    def tearDownClass(self):
        self.e.stop()

    def test_joints_pingable(self):
        """All joints pingable
        """
        joints_pinged = {}
        for k in self.e.joints:
            joints_pinged[k] = self.e.get_joint(k).do_ping(verbose=False)
        unpingable_joints = [k for k, v in joints_pinged.items() if not v]
        self.assertEqual(len(unpingable_joints), 0, msg="ping failed for joints={0}".format(unpingable_joints))

    @system_check_warn(warning="run stretch_robot_home.py")
    def test_joints_homed(self):
        """All joints homed
        """
        joints_homed = {}
        for k in self.e.joints:
            joint = self.e.get_joint(k)
            if joint.params['req_calibration']:
                joints_homed[k] = joint.motor.is_calibrated()
        self.e.logger.debug('homing status for endofarm joints that require homing={0}'.format(joints_homed))
        unhomed_joints = [k for k, v in joints_homed.items() if not v]
        self.assertEqual(len(unhomed_joints), 0, msg="not yet homed joints={0}".format(unhomed_joints))


class TestHead(unittest.TestCase):
    """Checking Head
    """

    @classmethod
    def setUpClass(self):
        self.h = stretch_body.head.Head()
        self.h.startup()

    @classmethod
    def tearDownClass(self):
        self.h.stop()

    def test_joints_pingable(self):
        """All joints pingable
        """
        joints_pinged = {}
        for k in self.h.joints:
            joints_pinged[k] = self.h.get_joint(k).do_ping(verbose=False)
        unpingable_joints = [k for k, v in joints_pinged.items() if not v]
        self.assertEqual(len(unpingable_joints), 0, msg="ping failed for joints={0}".format(unpingable_joints))

    @system_check_warn(warning="run stretch_robot_home.py")
    def test_joints_homed(self):
        """All joints homed
        """
        joints_homed = {}
        for k in self.h.joints:
            joint = self.h.get_joint(k)
            if joint.params['req_calibration']:
                joints_homed[k] = joint.motor.is_calibrated()
        self.h.logger.debug('homing status for head joints that require homing={0}'.format(joints_homed))
        unhomed_joints = [k for k, v in joints_homed.items() if not v]
        self.assertEqual(len(unhomed_joints), 0, msg="not yet homed joints={0}".format(unhomed_joints))

    def test_head_camera_present(self):
        """Head camera present
        """
        exit_code = subprocess.call("lsusb -d 8086:0b3a", stdout=open(os.devnull, 'w'), shell=True)
        self.assertEqual(exit_code, 0, msg="camera not connected")

class TestWACC(unittest.TestCase):
    """Checking Wrist + Accelerometer Board (WACC)
    """

    @classmethod
    def setUpClass(self):
        self.w = stretch_body.wacc.Wacc()
        self.w.startup()

    @classmethod
    def tearDownClass(self):
        self.w.stop()

    def test_valid_accelx(self):
        """Accelerometer X-axis in range
        """
        ax = self.w.status['ax']
        self.w.logger.debug('wacc accelerometer x-axis={0}'.format(ax))
        self.assertGreater(ax, 8.0)
        self.assertLess(ax, 11.0)

    @system_check_warn(warning="ignore if external hardware connected")
    def test_valid_digital_inputs(self):
        """Pins D0 and D1 read high
        """
        d0 = self.w.status['d0']
        d1 = self.w.status['d1']
        self.w.logger.debug('digital input 0={0}'.format(d0))
        self.w.logger.debug('digital input 1={0}'.format(d1))
        self.assertEqual(d0, 1, msg="D0 reads low")
        self.assertEqual(d1, 1, msg="D1 reads low")


class TestBase(unittest.TestCase):
    """Checking Mobile Base
    """

    @classmethod
    def setUpClass(self):
        self.b = stretch_body.base.Base()
        self.b.startup()

    @classmethod
    def tearDownClass(self):
        self.b.stop()

    def test_valid_lwheel_status(self):
        """Left wheel status valid
        """
        pos = self.b.left_wheel.status['pos']
        self.b.logger.debug('left wheel pos={0}'.format(pos))
        self.assertNotEqual(pos, 0, msg='odometry not initialized')

    def test_valid_rwheel_status(self):
        """Right wheel status valid
        """
        pos = self.b.right_wheel.status['pos']
        self.b.logger.debug('right wheel pos={0}'.format(pos))
        self.assertNotEqual(pos, 0, msg='odometry not initialized')


class TestLift(unittest.TestCase):
    """Checking Lift
    """

    @classmethod
    def setUpClass(self):
        self.l = stretch_body.lift.Lift()
        self.l.startup()

    @classmethod
    def tearDownClass(self):
        self.l.stop()

    def test_valid_status(self):
        """Status valid
        """
        pos = self.l.motor.status['pos']
        self.l.logger.debug('lift pos={0}'.format(pos))
        self.assertNotEqual(pos, 0, msg='odometry not initialized')

    @system_check_warn(warning="run stretch_robot_home.py")
    def test_lift_homed(self):
        """Homed
        """
        is_homed = self.l.motor.status['pos_calibrated']
        self.assertTrue(is_homed)


class TestArm(unittest.TestCase):
    """Checking Telescoping Arm
    """

    @classmethod
    def setUpClass(self):
        self.a = stretch_body.arm.Arm()
        self.a.startup()

    @classmethod
    def tearDownClass(self):
        self.a.stop()

    def test_valid_status(self):
        """Status valid
        """
        pos = self.a.motor.status['pos']
        self.a.logger.debug('arm pos={0}'.format(pos))
        self.assertNotEqual(pos, 0, msg='odometry not initialized')

    @system_check_warn(warning="run stretch_robot_home.py")
    def test_arm_homed(self):
        """Homed
        """
        is_homed = self.a.motor.status['pos_calibrated']
        self.assertTrue(is_homed)


class TestSoftware(unittest.TestCase):
    """Checking Software
    """

    @system_check_warn()
    def test_latest_hello_pip_packages(self):
        """Latest Stretch Python libraries
        """
        dummy_device = stretch_body.device.Device('dummy', req_params=False)
        ubuntu_to_pip_mapping = {'18.04': 'pip2', '20.04': 'pip3'}
        pip_str = ubuntu_to_pip_mapping[distro.version()]

        def get_latest_version(url):
            resp = requests.get(url)
            if resp.status_code == 200:
                releases = xmltodict.parse(resp.text)['rss']['channel']['item']
                for i in range(len(releases)):
                    if 'dev' not in releases[i]['title']:
                        return releases[i]['title']
            return None

        # Stretch Body
        try:
            from stretch_body.version import __version__ as installed_stretch_body_version
        except:
            installed_stretch_body_version = None
        latest_stretch_body_version = get_latest_version("https://pypi.org/rss/project/hello-robot-stretch-body/releases.xml")
        dummy_device.logger.debug("hello-robot-stretch-body installed={0}, latest available={1}".format(installed_stretch_body_version, latest_stretch_body_version))
        self.assertEqual(installed_stretch_body_version, latest_stretch_body_version, msg="run {} install -U hello-robot-stretch-body".format(pip_str))

        # Stretch Body Tools
        try:
            from stretch_body_tools.version import __version__ as installed_stretch_body_tools_version
        except:
            installed_stretch_body_tools_version = None
        latest_stretch_body_tools_version = get_latest_version("https://pypi.org/rss/project/hello-robot-stretch-body-tools/releases.xml")
        dummy_device.logger.debug("hello-robot-stretch-body-tools installed={0}, latest available={1}".format(installed_stretch_body_tools_version, latest_stretch_body_tools_version))
        self.assertEqual(installed_stretch_body_tools_version, latest_stretch_body_tools_version, msg="run {} install -U hello-robot-stretch-body-tools".format(pip_str))

        # Stretch Factory
        try:
            from stretch_factory.version import __version__ as installed_stretch_factory_version
        except:
            installed_stretch_factory_version = None
        latest_stretch_factory_version = get_latest_version("https://pypi.org/rss/project/hello-robot-stretch-factory/releases.xml")
        dummy_device.logger.debug("hello-robot-stretch-factory installed={0}, latest available={1}".format(installed_stretch_factory_version, latest_stretch_factory_version))
        self.assertEqual(installed_stretch_factory_version, latest_stretch_factory_version, msg="run {} install -U hello-robot-stretch-factory".format(pip_str))

        # Stretch Tool Share
        try:
            from stretch_tool_share.version import __version__ as installed_stretch_tool_share_version
        except:
            installed_stretch_tool_share_version = None
        latest_stretch_tool_share_version = get_latest_version("https://pypi.org/rss/project/hello-robot-stretch-tool-share/releases.xml")
        dummy_device.logger.debug("hello-robot-stretch-tool-share installed={0}, latest available={1}".format(installed_stretch_tool_share_version, latest_stretch_tool_share_version))
        self.assertEqual(installed_stretch_tool_share_version, latest_stretch_tool_share_version, msg="run {} install -U hello-robot-stretch-tool-share".format(pip_str))

    @system_check_warn(warning="run REx_firmware_updater.py --recommended")
    def test_latest_firmware(self):
        """Latest Stretch firmware
        """
        hello_devices = {'hello-motor-lift': True, 'hello-motor-arm': True, 'hello-motor-right-wheel': True, 'hello-motor-left-wheel': True, 'hello-pimu': True, 'hello-wacc': True}
        r = stretch_factory.firmware_updater.RecommendedFirmware(hello_devices)
        is_latest = {'hello-motor-lift': False, 'hello-motor-arm': False, 'hello-motor-right-wheel': False, 'hello-motor-left-wheel': False, 'hello-pimu': False, 'hello-wacc': False}
        for device_name in r.recommended.keys():
            if r.fw_installed.is_device_valid(device_name):
                installed_version = r.fw_installed.get_version(device_name)
                if r.recommended[device_name] == installed_version:
                    is_latest[device_name] = True
        self.assertTrue(all(list(is_latest.values())), msg='updateable devices={0}'.format([k for k, v in is_latest.items() if not v]))
