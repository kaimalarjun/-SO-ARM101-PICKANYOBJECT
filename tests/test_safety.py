import sys
from pathlib import Path
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros"))
from kendra_robot.control import MockBackend, Point, Rejected, RobotController


CONFIG = {"mode": "mock", "operator_enabled": False, "command_lease_s": 0.1,
          "max_command_s": 1, "max_observation_age_s": 2,
          "approach_clearance_m": 0.05,
          "workspace_m": {"x": [0.05, 0.25], "y": [-0.18, 0.18], "z": [0.02, 0.20]},
          "home_m": {"x": 0.15, "y": 0, "z": 0.15}}


class SafetyTests(unittest.TestCase):
    def setUp(self):
        self.backend = MockBackend()
        self.robot = RobotController(CONFIG, self.backend)

    def test_nan_and_outside_never_command(self):
        for data in ({"x": float("nan"), "y": 0, "z": 0.1},
                     {"x": 0.9, "y": 0, "z": 0.1}):
            with self.assertRaises(Rejected):
                self.robot.move_to(Point.parse(data))
        self.assertEqual(self.backend.commands, [])

    def test_pick_requires_fresh_unique_matching_observation(self):
        point = Point(0.12, 0.02, 0.04)
        with self.assertRaises(Rejected): self.robot.pick(point, "missing")
        obs = {"id": "one", "point": {"x": 0.12, "y": 0.02, "z": 0.04},
               "monotonic_time": time.monotonic(), "unique": False,
               "calibrated": True, "approach_clear": True, "zone_clear": True}
        self.robot.set_observation(obs)
        with self.assertRaises(Rejected): self.robot.pick(point, "one")
        obs["unique"] = True
        self.assertEqual(self.robot.pick(point, "one")["task_success"], "uncertain")
        self.assertEqual(len(self.backend.commands), 5)
        with self.assertRaises(Rejected): self.robot.pick(point, "one")

    def test_real_mode_requires_operator_enable(self):
        config = dict(CONFIG, mode="real")
        robot = RobotController(config, self.backend)
        with self.assertRaises(Rejected): robot.move_to(Point(0.15, 0, 0.1))
        self.assertEqual(self.backend.commands, [])

    def test_stop_latches_and_bypasses_queue(self):
        self.robot.stop()
        with self.assertRaises(Rejected): self.robot.home()

    def test_lease_expiry_stops_active_command(self):
        self.robot._busy = True
        self.robot._lease_deadline = time.monotonic() - 1
        self.assertTrue(self.robot.watchdog_tick())
        self.assertTrue(self.robot.cancel.is_set())


if __name__ == "__main__":
    unittest.main()
