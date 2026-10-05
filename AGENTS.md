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
- After a failed motion, stuck joint, or slipped grasp, cancel the active command
  and inspect fresh joint feedback and both cameras before retrying. Identify
  whether the cause is contact, load sag, a travel limit, communication loss,
  lost localization, or still uncertain. Do not keep pushing the failed target.
- If feedback is healthy and the retreat path is visibly clear, reverse a small
  part of the last successful movement toward the last known clear pose. Check
  whether the joint responds and the obstruction clears before retreating further.
  Use bounded coordinated movements; do not jump blindly to home or disable
  torque where the arm or held object could fall. Human proximity, trapped cables,
  missing feedback, or an uncertain retreat path require holding/stopping instead.
- Record the failed target, measured pose and motion, load/grip state, camera
  evidence, suspected cause, retreat attempted, and recovery outcome in ignored
  host-local records. Add the observed obstacle, contextual limit, or failed grasp
  region to the next plan so the same approach is not repeated unchanged. A single
  failure does not prove a permanent mechanical limit or guarantee future avoidance.
- After recovery, reacquire the objects and placement space, choose a revised
  route, grasp, clearance, or motion profile, and test its first correction before
  resuming the full goal. Count failed timed trials as unmet; keep the original
  success criteria and record what changed between attempts.
- Report a pick as successful only when camera evidence confirms the held object
  lifted off its support. Keep torque-on hold between actions to prevent folding.
- Keep real poses, calibration, camera recordings, host/device identities, and
  credentials in ignored host-local files. Publish reusable code and generic docs.

## Lessons from observed commissioning failures

- A gripper position goal or a stopped closing motion is not proof of a grasp.
  Top-rim and end grips can slide or rotate a box, then slip during lifting.
  Centre the contact along the object, check both jaw tips are beside its sides,
  and make a small camera-verified lift before transporting it.
- An open moving jaw can be much higher than the fixed jaw. Seeing one tip below
  the object's top does not establish two-sided contact. Inspect the closed grasp
  and fresh camera views; changes in camera viewpoint can resemble object motion.
- A held object changes the collision footprint. Check its complete underside,
  corners, and swept volume, not just the claw centre. Raise its lowest point
  above neighbouring objects before moving across them.
- A rear placement descent shifted a neighbouring bottle even though the claw
  centre appeared clear. Treat object displacement/rotation as suspected contact,
  reject that route, retreat, and increase separation before another descent.
- Moving farther back cleared the bottle but put part of the box beyond the
  tabletop edge. Check full support at the destination before release. A clear
  background or a provisional XYZ height does not prove there is table underneath.
- A side placement on a supported clear region succeeded after the rear routes
  failed. Follow the user's latest placement constraint; do not keep pursuing a
  rejected location after an alternative nearby location is authorized.
- Held payloads can increase tracking error. Record actual movement and use a
  revised bounded profile or another joint; do not silently widen tolerances or
  infer a new mechanical limit from sag. Extra gripper closure during lifting can
  indicate slipping contact and must trigger inspection of the stopped state.
- Camera evidence of a hand near the held object requires stationary hold and
  a fresh clearance check before continuing. Old statements about an empty area
  do not override current images.
- After each attempt, append a concise lesson with evidence and its scope to these
  instructions and update the ignored host-local attempt record. Keep observations
  separate from hypotheses, record unsuccessful trials honestly, and reuse the
  lesson when planning the next attempt. This is persistent procedural memory,
  not automatic model training or a guarantee against recurring failures.
