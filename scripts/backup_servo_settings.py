"""Read settings changed by the standard calibration helper; never write motors."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import time


def main() -> None:
    from lerobot.motors import Motor, MotorNormMode
    from lerobot.motors.feetech import FeetechMotorsBus

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port")
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    names = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")
    registers = (
        "Model_Number", "ID", "Baud_Rate", "Homing_Offset", "Min_Position_Limit", "Max_Position_Limit",
        "Operating_Mode", "P_Coefficient", "I_Coefficient", "D_Coefficient", "Return_Delay_Time",
        "Phase", "Acceleration", "Maximum_Acceleration", "Max_Torque_Limit", "Protection_Current",
        "Overload_Torque", "Torque_Enable", "Present_Position",
    )
    bus = FeetechMotorsBus(args.port, {name: Motor(i, "sts3215", MotorNormMode.DEGREES)
                                     for i, name in enumerate(names, 1)})
    bus.connect()
    try:
        settings = {name: {register: bus.read(register, name, normalize=False) for register in registers}
                    for name in names}
    finally:
        bus.disconnect(disable_torque=False)
    args.output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    fd = os.open(args.output, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "w") as output:
        json.dump({"created_unix": time.time(), "motors": settings}, output, indent=2)
    print(f"Read-only settings backup saved to {args.output}")


if __name__ == "__main__":
    main()
