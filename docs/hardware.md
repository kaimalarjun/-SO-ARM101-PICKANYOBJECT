# Hardware inventory and power gate

The **follower** is the arm that picks objects. An optional **leader** is a second arm moved by hand to record demonstrations. Use this checklist beside the actual hardware before opening the follower's USB port.

1. Record the follower and leader controller model, servo count and labels, motor IDs, mechanical assembly, gripper orientation and observed end stops in a private operator log.
2. Read the **actual** power-adapter label and controller input rating. Do not infer voltage from a product listing. Confirm polarity and connector fit before connection.
3. Mount the follower rigidly, with a clear workspace and no person within reach. Mount the camera rigidly overhead, with the whole reachable table and four fixed ArUco markers visible.
4. Put a reachable physical cutoff in series with follower motor power, rated for the measured adapter. Test the cutoff with the arm supported. Power loss can drop the arm or a held object.
5. Give follower and leader distinct stable `/dev` paths. The follower bus is opened by ROS `ros2_control` only. LeRobot may read the leader's separate bus during supervised recording.
6. Save factory offsets and any controller EEPROM values privately before calibration. The Feetech driver can write homing offsets; do not issue that operation during inventory.
7. With power off, run `scripts/preflight.sh`. Then launch mock hardware and verify the joint ordering, base frame, gripper direction and MoveIt collision model in RViz.

Real motion is withheld until the operator has measured workspace bounds, verified a working physical cutoff, tested stop behavior, and authorized the first powered movement. Use slow single-joint tests with no object before any Cartesian pick. Stop at once on unexpected direction, heat, collision, or missing feedback.
