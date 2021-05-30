# Logging level must be set before importing any stretch_body class
import stretch_body.robot_params
stretch_body.robot_params.RobotParams.set_logging_level("DEBUG")

import unittest
import stretch_body.pimu

import time


class TestPimu(unittest.TestCase):

    def test_invalid_protocol(self):
        """Simulate an invalid protocol and verify the correct error.
        """
        p = stretch_body.pimu.Pimu()
        p.valid_firmware_protocol = 'p-1' # valid protocols are p0 and up
        p.startup()
        # TODO: capture logging with https://testfixtures.readthedocs.io/en/latest/logging.html
        #       verify output is '[WARNING] [pimu]: \n----------------\nFirmware protocol mismatch on hello-pimu. [...]'

        p.stop()
