import importlib.util
from pathlib import Path
import unittest

import numpy as np

spec = importlib.util.spec_from_file_location('alignment', Path(__file__).parents[1]/'scripts'/'grasp_alignment.py')
alignment = importlib.util.module_from_spec(spec)
spec.loader.exec_module(alignment)


class AlignmentTests(unittest.TestCase):
    def test_projection_does_not_approve_depth(self):
        mask = np.zeros((100, 100), np.uint8)
        mask[40:60, 20:80] = 255
        result = alignment.inspect(mask, {'jaw_tips_px': [[50, 20], [50, 80]]})
        self.assertTrue(result['centered_in_projection'])
        self.assertTrue(result['orientation_ok_in_projection'])
        self.assertFalse(result['grasp_ready'])

    def test_rejects_clipped_target(self):
        mask = np.zeros((100, 100), np.uint8)
        mask[40:60, :80] = 255
        with self.assertRaises(ValueError):
            alignment.inspect(mask, {'jaw_tips_px': [[50, 20], [50, 80]]})

    def test_correction_is_bounded_and_uses_measured_response(self):
        mask = np.zeros((100, 100), np.uint8)
        mask[40:60, 10:50] = 255
        result = alignment.inspect(mask, {
            'jaw_tips_px': [[50, 20], [50, 80]],
            'local_response': {'joints': ['pan', 'reach'], 'pixels_per_radian': [[100, 0], [0, 100]]},
        })
        self.assertAlmostEqual(result['suggested_joint_deltas_rad']['pan'], .08)
        self.assertLessEqual(abs(result['suggested_joint_deltas_rad']['reach']), .08)


if __name__ == '__main__':
    unittest.main()
