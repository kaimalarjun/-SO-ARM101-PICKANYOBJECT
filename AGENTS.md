# Robot commissioning instructions

- Keep this project simple and iterative. Use the existing ROS controller as the
  sole owner of the follower's serial bus.
- Object positions may change between commands and after release. Obtain fresh
  camera evidence and locate each requested object before every grasp or placement.
  Saved joint poses are approach hints, not proof of the object's current location.
- Use active vision when the wrist camera cannot see enough: first raise to a
  previously checked clear observation pose, then make small coordinated wrist
  bend/rotation or nearby arm-joint movements to scan. Observe each view and keep
  camera frames paired with joint feedback and timestamps. Prefer the smallest
  scan that resolves the target and placement clearance. Gripper opening itself
  does not reposition a camera mounted on the wrist.
- Treat the camera like eyes and wrist joints like head/neck movements, while
  remembering that the whole arm and cables move too. Check the external view
  for clearance throughout scans. Do not scan near an object by blindly rotating
  the wrist, and do not rotate into cables or recorded joint limits.
- Refresh localization after camera motion, object motion, contact, a failed
  grasp, or release. A moving-camera image response estimate applies only near
  the pose at which it was measured. Two cameras alone do not establish depth.
- Before timed trials, establish the route and object clearance. Include current
  localization, feedback, and outcome verification in the measured trial time.
  Use smooth simultaneous arm trajectories and minimize idle gaps once validated.
- Record joint-specific stalls/undertravel with load, pose, direction, targets,
  actual movement, and both camera views. Keep candidate contextual movement
  limits separate from confirmed mechanical travel limits.
- Report a pick as successful only when camera evidence confirms the held object
  lifted off its support. Keep torque-on hold between actions to prevent folding.
- Keep real poses, calibration, camera recordings, host/device identities, and
  credentials in ignored host-local files. Publish reusable code and generic docs.
