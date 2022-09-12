import unittest
import stretch_body.device
import stretch_body.hello_utils
import time


class TestDevice(unittest.TestCase):


    def test_write_configuration_params(self):
        """
        Check that we don't mangle the format when writing config params to YAML
        """

        print('\nUnittest: test_write_configuration_params')

        d=stretch_body.device.Device(req_params=False)
        cp = stretch_body.hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
        for k in cp.keys(): #Write existing values up to 3 dict layers deep
            key1=k
            d.write_configuration_param_to_YAML(key1,cp[k])
            if type(cp[k]) is dict:
                for j in cp[k].keys():
                    key2=key1+'.'+j
                    d.write_configuration_param_to_YAML(key2, cp[k][j])
                    if type(cp[k][j]) is dict:
                        for l in cp[k][j].keys():
                            key3 = key2 + '.' + l
                            d.write_configuration_param_to_YAML(key3, cp[k][j][l])
        new_cp=stretch_body.hello_utils.read_fleet_yaml('stretch_configuration_params.yaml')
        self.assertTrue(cp==new_cp)


    def test_disable_existing_loggers(self):
        """Test multiple instances of device class all have enabled loggers.

        The logging param 'disable_existing_loggers' is set True so that loggers
        from imported python libraries don't print to the console. However, this
        means stretch_body classes can only initialize loggers after the config
        is loaded. We initialize config as class variables of Device for this.
        """
        print('\nUnittest: test_disable_existing_loggers')
        d1 = stretch_body.device.Device('wrist_yaw') # loads logging config
        d2 = stretch_body.device.Device('stretch_gripper')
        d1.logger.info('hi')
        d2.logger.info('hi')
        # TODO: capture logging with https://testfixtures.readthedocs.io/en/latest/logging.html
        #       verify output is '[INFO][wrist_yaw]hi\n[INFO][stretch_gripper]hi\n'

    def test_threaded(self):
        print('\nUnittest: test_threaded')
        class SometimesThreadedDevice(stretch_body.device.Device):
            def __init__(self):
                stretch_body.device.Device.__init__(self,req_params=False)
            def startup(self, threaded=False):
                self.pulling_status = False
                stretch_body.device.Device.startup(self, threaded=threaded)
            def pull_status(self):
                self.pulling_status = True

        # not threaded
        d = SometimesThreadedDevice()
        d.startup()
        time.sleep(0.1)
        self.assertFalse(d.pulling_status)
        time.sleep(0.1)
        self.assertFalse(d.pulling_status)
        d.stop()

        # threaded
        d.startup(threaded=True)
        time.sleep(0.1)
        self.assertTrue(d.pulling_status)
        time.sleep(0.1)
        self.assertTrue(d.pulling_status)
        d.stop()

        # not threaded
        d.startup()
        time.sleep(0.1)
        self.assertFalse(d.pulling_status)
        time.sleep(0.1)
        self.assertFalse(d.pulling_status)
        d.stop()
