"""Run LeRobot's standard SO-101 calibration without enabling motor torque."""
from __future__ import annotations

import argparse
from pathlib import Path

from lerobot.robots.so_follower.config_so_follower import SOFollowerRobotConfig
from lerobot.robots.so_follower.so_follower import SOFollower


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("calibration_dir", type=Path)
    args = parser.parse_args()
    robot = SOFollower(SOFollowerRobotConfig(
        port=args.port, id="follower", calibration_dir=args.calibration_dir,
    ))
    # Robot.connect() configures runtime gains and briefly enables torque.
    # Calibration needs only the bus handshake and the upstream calibrate method.
    robot.bus.connect()
    try:
        robot.bus.disable_torque()
        robot.calibrate()
    finally:
        robot.bus.disconnect(disable_torque=True)


if __name__ == "__main__":
    main()
