import unittest
import stretch_body.device


class TestDevice(unittest.TestCase):

    def test_disable_existing_loggers(self):
        """Test multiple instances of device class all have enabled loggers.

        The logging param 'disable_existing_loggers' is set True so that loggers
        from imported python libraries don't print to the console. However, this
        means stretch_body classes can only initialize loggers after the config
        is loaded. We initialize config as class variables of Device for this.
        """
        d1 = stretch_body.device.Device('wrist_yaw') # loads logging config
        d2 = stretch_body.device.Device('stretch_gripper')
        d1.logger.info('hi')
        d2.logger.info('hi')
        # TODO: capture logging with https://testfixtures.readthedocs.io/en/latest/logging.html
        #       verify output is '[INFO][wrist_yaw]hi\n[INFO][stretch_gripper]hi\n'
