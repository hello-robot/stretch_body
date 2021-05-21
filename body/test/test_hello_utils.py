import unittest
import warnings
import stretch_body.hello_utils


class TestHelloUtils(unittest.TestCase):

    def test_yaml_file_released(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            # read yaml, generating a ResourceWarning if the file is not released
            yaml = stretch_body.hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
            self.assertTrue(len(w) == 0)
