import unittest
import stretch_body.hello_utils

import time
import warnings


class TestHelloUtils(unittest.TestCase):

    def test_yaml_file_released(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            # read yaml, generating a ResourceWarning if the file is not released
            yaml = stretch_body.hello_utils.read_fleet_yaml('stretch_re1_user_params.yaml')
            self.assertTrue(len(w) == 0)

    def test_reading_invalid_yaml(self):
        """Verify that read_fleet_yaml returns empty dict on invalid file.
        """
        read_params = stretch_body.hello_utils.read_fleet_yaml('invalid_file123.yaml')
        self.assertEqual(read_params, {})
        read_params1 = stretch_body.hello_utils.read_fleet_yaml('')
        self.assertEqual(read_params1, {})

    def test_overwriting_params(self):
        """Test the behavior of the overwrite_dict method.
        """
        dee1 = {'param1': 1}
        der1 = {'param2': 2}
        stretch_body.hello_utils.overwrite_dict(dee1, der1)
        self.assertEqual(dee1, {'param1': 1, 'param2': 2})

        dee2 = {'param1': 'to_override'}
        der2 = {'param1': 'over'}
        stretch_body.hello_utils.overwrite_dict(dee2, der2)
        self.assertEqual(dee2, {'param1': 'over'})

        dee3 = {'param1': {'motion': 'to_override', 'no_change': 1}}
        der3 = {'param1': {'motion': 'over'}}
        stretch_body.hello_utils.overwrite_dict(dee3, der3)
        self.assertEqual(dee3, {'param1': {'motion': 'over', 'no_change': 1}})

        dee4 = {'param1': {'motion': 'same', 'no_change': 1}}
        der4 = {'param1': {'motion': {}}}
        stretch_body.hello_utils.overwrite_dict(dee4, der4)
        self.assertEqual(dee4, {'param1': {'motion': 'same', 'no_change': 1}})

        dee5 = {'param1': {'motion': {}, 'no_change': 1}}
        der5 = {'param1': {'motion': 2}}
        stretch_body.hello_utils.overwrite_dict(dee5, der5)
        self.assertEqual(dee5, {'param1': {'motion': {}, 'no_change': 1}})

    def test_overwriting_vs_updating_params(self):
        """Verify the difference between overwrite_dict and updating a dict.
        """
        overider1 = {"robot": {"motion": {"max": 100}}}
        overidee1 = {"robot": {"motion": {"min": -100}}}
        stretch_body.hello_utils.overwrite_dict(overidee1, overider1)
        self.assertEqual(overidee1, {"robot": {"motion": {"max": 100, "min": -100}}})

        overider2 = {"robot": {"motion": {"max": 100}}}
        overidee2 = {"robot": {"motion": {"min": -100}}}
        overidee2.update(overider2)
        self.assertNotEqual(overidee1, overidee2)

    def test_pretty_print_dict(self):
        dict1 = {"param1": 1, "param2": 2}
        stretch_body.hello_utils.pretty_print_dict("params", dict1)

        dict2 = {"robot": {"motion": {"max": 100, "min": -100}, "retry": True}}
        stretch_body.hello_utils.pretty_print_dict("Stretch", dict2)

    def test_create_time_string(self):
        """Verify time strings match
        """
        t = time.localtime()
        expected_time_string = str(t.tm_year) + str(t.tm_mon).zfill(2) + str(t.tm_mday).zfill(2) + str(t.tm_hour).zfill(2) + str(t.tm_min).zfill(2) + str(t.tm_sec).zfill(2)
        actual_time_string = stretch_body.hello_utils.create_time_string()
        self.assertEqual(expected_time_string, actual_time_string)

    def test_get_stretch_directory(self):
        """
        """
        import os
        if os.environ.get('HELLO_FLEET_PATH', None) is not None:
            self.assertNotEqual(stretch_body.hello_utils.get_stretch_directory(), "/tmp/")
            original_fleet_path = os.environ['HELLO_FLEET_PATH']
            del os.environ['HELLO_FLEET_PATH']
            self.assertEqual(stretch_body.hello_utils.get_stretch_directory(), "/tmp/")
            os.environ['HELLO_FLEET_PATH'] = original_fleet_path
        else:
            self.assertEqual(stretch_body.hello_utils.get_stretch_directory(), "/tmp/")
