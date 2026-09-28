import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros"))
from kendra_robot.calibration import project, validate


class GeometryTests(unittest.TestCase):
    def test_projection_and_independent_threshold(self):
        h = [[0.0005, 0, 0.05], [0, 0.0005, -0.1], [0, 0, 1]]
        self.assertAlmostEqual(project(h, 200, 200)[0], 0.15)
        self.assertLess(validate(h, [[100, 100], [200, 300]],
                                 [[0.10, -0.05], [0.15, 0.05]]), 0.01)
        with self.assertRaises(ValueError):
            validate(h, [[200, 200]], [[0.17, 0.0]])

    def test_singular_projection_rejected(self):
        with self.assertRaises(ValueError):
            project([[1, 0, 0], [0, 1, 0], [0, 0, 0]], 1, 1)


if __name__ == "__main__":
    unittest.main()
