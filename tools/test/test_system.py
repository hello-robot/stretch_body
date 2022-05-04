import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import os, fnmatch
from pprint import pformat
import stretch_body.device


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
        self.assertTrue(all(list(usb_aliases.values())), msg='test_usb_aliases_present MESSAGE')

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
        num_acm_devices = sum([fnmatch.fnmatch(entry, "ttyACM*") for entry in listOfDevices])
        self.assertEqual(num_acm_devices, 6, msg='test_num_acm_devices_present MESSAGE')
