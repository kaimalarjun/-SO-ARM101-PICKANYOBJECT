# Hardware inventory and power gate

The **follower** is the arm that picks objects. An optional **leader** is a second arm moved by hand to record demonstrations. Use this checklist beside the actual hardware before opening the follower's USB port.

1. Record the follower and leader controller model, servo count and labels, motor IDs, mechanical assembly, gripper orientation and observed end stops in a private operator log.
2. Read the **actual** power-adapter label and controller input rating. Do not infer voltage from a product listing. Confirm polarity and connector fit before connection.
3. Mount the follower rigidly, with a clear workspace and no person within reach. Mount the camera rigidly overhead, with the whole reachable table and four fixed ArUco markers visible.
4. Put a reachable physical cutoff in series with follower motor power, rated for the measured adapter. Test the cutoff with the arm supported. Power loss can drop the arm or a held object.
5. Identify the two USB cables by briefly unplugging one at a time, then assign the follower and leader distinct `/dev/serial/by-id/` paths in private settings. The Linux operator needs serial-port permission (usually membership in `dialout`). ROS `ros2_control` opens only the follower; LeRobot may read the leader during supervised recording.
6. Save factory offsets and controller settings privately before calibration. Do not launch ROS in `real` mode just to identify a port: the Feetech driver changes torque state and may write servo parameters when it starts.
7. With power off, run `scripts/preflight.sh`. Then launch mock hardware and verify the joint ordering, base frame, gripper direction and MoveIt collision model in RViz.

## Seeed Studio setup guide

Use the [Seeed SO-ARM100/101 guide](https://wiki.seeedstudio.com/lerobot_so100m_new/) here only for kit identification, servo labels and power ratings. This project uses ROS 2 to control the follower; do not run the guide's LeRobot motor setup, calibration or teleoperation commands during ROS bringup. A preassembled kit should not need servo ID and baud-rate setup. Check the actual adapter and motor labels: the Standard kit uses 5 V for both arms, while the Pro kit uses 5 V for the leader and 12 V for the follower.

At the pinned source revisions, the SO-101 description supplies `offset` values that the Feetech driver ignores. Verify each joint's center, direction and limits using a ROS-compatible calibration before launching `mode:=real`. LeRobot is reserved for the separate, optional training stage and never opens the follower while ROS owns it.

Real motion is withheld until the operator has measured workspace bounds, verified a working physical cutoff, tested stop behavior, and authorized the first powered movement. Use slow single-joint tests with no object before any Cartesian pick. Stop at once on unexpected direction, heat, collision, or missing feedback.
