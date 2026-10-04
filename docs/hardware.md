# Hardware inventory and power gate

The **follower** is the arm that picks objects. An optional **leader** is a second arm moved by hand to record demonstrations. Use this checklist beside the actual hardware before opening the follower's USB port.

1. Record the follower and leader controller model, servo count and labels, motor IDs, mechanical assembly, gripper orientation and observed end stops in a private operator log.
2. Read the **actual** power-adapter label and controller input rating. Do not infer voltage from a product listing. Confirm polarity and connector fit before connection.
3. Mount the follower rigidly, with a clear workspace and no person within reach. Mount the camera rigidly overhead, with the whole reachable table and four fixed ArUco markers visible.
4. Put a reachable physical cutoff in series with follower motor power, rated for the measured adapter. Test the cutoff with the arm supported. Power loss can drop the arm or a held object.
5. Identify the two USB cables by briefly unplugging one at a time, then assign the follower and leader distinct `/dev/serial/by-id/` paths in private settings. The Linux operator needs serial-port permission (usually membership in `dialout`). ROS `ros2_control` opens only the follower; LeRobot may read the leader during supervised recording.
6. Save factory offsets and controller settings privately before calibration. Do not launch ROS in `real` mode just to identify a port: the Feetech driver changes torque state and may write servo parameters when it starts.
7. With power off, run `scripts/preflight.sh`. Then launch mock hardware and verify the joint ordering, base frame, gripper direction and MoveIt collision model in RViz.

Once the arm is clamped, the supply label is checked, and the operator is at the power cutoff, run `bash scripts/servo_inventory.sh /dev/serial/by-id/DEVICE` using the follower's stable USB path. This utility reads servo IDs 1–6, model numbers, present positions, stored homing offsets, and range limits. It sends no torque or position commands. Keep the output with private hardware notes; do not reset or overwrite existing offsets just because they are nonzero.

## Seeed Studio setup guide

Use the [Seeed SO-ARM100/101 guide](https://wiki.seeedstudio.com/lerobot_so100m_new/) for kit identification, power ratings and the physical calibration procedure. This project uses ROS 2 to control the follower. A preassembled kit whose six motor IDs already respond should not need servo ID and baud-rate setup. Check the actual adapter and motor labels: the Standard kit uses 5 V for both arms, while the Pro kit uses 5 V for the leader and 12 V for the follower.

At the pinned source revisions, the SO-101 description supplies old `offset` values that the Feetech driver ignores. The driver instead uses homing offsets stored in each servo. Do not copy another robot's offsets or assume nonzero stored offsets prove calibration matches the ROS model.

The [ROS framework's hardware guide](https://github.com/legalaspro/so101-ros-physical-ai/blob/main/docs/hardware.md) recommends LeRobot's standard one-time calibration helper. Stop ROS and preserve the existing servo settings before using it. Follow the [SO-101 calibration procedure](https://huggingface.co/docs/lerobot/so101#calibrate), then verify the measured angles, directions and travel against the ROS model before motion. LeRobot and ROS must never open the follower bus at the same time. ROS owns the bus during ordinary robot operation.

Install the optional helper with `bash scripts/install_calibration_helper.sh`. With ROS stopped, the arm supported and an operator at the power cutoff, run `bash scripts/calibrate_follower.sh /dev/serial/by-id/DEVICE` using the follower's actual path. The wrapper makes a read-only backup of affected settings, then calls LeRobot 0.6.1's standard calibration method directly. It skips runtime motor setup, which briefly enables torque in the stock CLI. Torque stays disabled during this wrapper's calibration. Both backups and calibration files stay in a restricted host-local directory. Follow the prompts on the picking/follower arm to centre its travel and gently record each joint's available range. Do not force the joints or rotate the wrist into its cables. The helper's calibration must still be checked against the ROS geometry before any pick.

The installer applies a small patch that increases the pinned Feetech serial read timeout from 5 ms to 20 ms. This gives USB transfers more time to complete; a failed read still stops control. A passive hold test does not establish reliability while both cameras are streaming.

A second patch keeps torque off during driver setup. Activation must read fresh joint positions and write a hold target at those positions before enabling torque. A failed read or write aborts activation. This avoids enabling torque against an old motor target after manual calibration.

Real motion is withheld until the operator has measured workspace bounds, verified a working physical cutoff, tested stop behavior, and authorized the first powered movement. Use slow single-joint tests with no object before any Cartesian pick. Stop at once on unexpected direction, heat, collision, or missing feedback.
