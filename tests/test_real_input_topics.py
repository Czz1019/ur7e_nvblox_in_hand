"""Protect real/sim input selection and explicit user topic overrides."""
import importlib.util
from pathlib import Path
import unittest

launch_file = Path(__file__).resolve().parents[1] / 'src/ur7e_nvblox_in_hand/launch/bringup.launch.py'
spec = importlib.util.spec_from_file_location('in_hand_bringup_test', launch_file)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class RealInputTopicsTests(unittest.TestCase):
    def setUp(self):
        self.base = dict(depth_topic='/camera/depth/image_rect_raw',
                         depth_info_topic='/camera/depth/camera_info',
                         color_topic='/camera/color/image_raw',
                         color_info_topic='/camera/color/camera_info')

    def test_real_selects_driver_namespace(self):
        result = module.resolve_input_topics(self.base, {}, False)
        self.assertEqual(result, {key: value.replace('/camera/', '/camera/camera/', 1)
                                  for key, value in self.base.items()})

    def test_sim_keeps_original_topics(self):
        self.assertEqual(module.resolve_input_topics(self.base, {}, True), self.base)

    def test_explicit_topics_win_in_both_modes(self):
        for sim in (False, True):
            supplied = dict(color_topic='/custom/rgb', color_info_topic='/custom/info')
            result = module.resolve_input_topics(self.base, supplied, sim)
            self.assertEqual(result['color_topic'], '/custom/rgb')
            self.assertEqual(result['color_info_topic'], '/custom/info')


if __name__ == '__main__':
    unittest.main()
