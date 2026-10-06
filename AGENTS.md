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

- A box retry achieved a verified lift after centering its complete silhouette,
  correcting reach and pan together while clear, and lowering with coupled
  shoulder/wrist motion before closing. The gripper stopped at body contact;
  fresh external and wrist images confirmed retention before release onto the mat.
  Record complete elapsed time honestly: this successful retry exceeded its timed
  target. Re-localize after release because the box moved while landing.

- Before a grasp, explicitly locate the object's middle, its narrow dimension,
  and both fingertip positions. Use `scripts/grasp_alignment.py` for image-only
  alignment and bounded suggestions from measured local camera response. A clipped
  target or unreliable silhouette requires another clear view. Confirm opposed
  contact depth separately; an aligned overlay cannot authorize closing or motion.

- Continue adapting the pickup position after a displaced object or failed grasp.
  Reacquire it, change the approach based on measured camera response, and retry
  while feedback and clearance remain valid. Do not require the object to return
  to a saved location or repeat an unchanged failed approach.
- A fallen bottle was successfully lifted after changing wrist rotation to grasp
  across its width, correcting reach and pan while clear, and descending in small
  shoulder/wrist steps that approximately preserved pitch. The closing gripper
  stopped against the body before its requested target; a short lift followed by
  a second lift confirmed retention without adding squeeze. Save the measured
  pose privately as a scene-specific hint, then re-localize before reuse.
- A phone hidden behind the bottle appeared during descent. Check both views
  again before closing; an earlier clear view can miss an occluded obstacle.

- For a toppled bottle, reacquire its body axis and rotate clear of the table
  so the jaws approach across the body, rather than along the cap/body axis.
  Neck or surface contacts can roll it without retaining it. Repeated small
  lift checks in this scene failed even after firmer closure; do not treat a
  plausible two-dimensional jaw overlap as a verified opposed grasp.
- A wrist rotation request under-travelled substantially in one unloaded pose.
  Record this as a contextual limit and investigate its
  cause; do not force it or treat the full encoder range as usable travel.
  An alternate orientation was reached by raising first and rotating the
  opposite way. Its successful motion did not establish a successful grasp.
- A forward correction displaced a fallen bottle. Re-localize after every
  contact and account for rolling; stop sideways nudges as table support becomes
  uncertain. If a person enters the movement area, hold and inspect a fresh view.

- After a scene reset, a simultaneous pan and height-changing approach tipped
  an upright bottle before grasping. The exact contact point was uncertain;
  include the wrist camera, controller board, and cables in clearance checks.
  First raise at the existing pan, inspect clearance, then transfer horizontally.
  A reset does not validate a diagonal swept path. Record the failed timed
  attempt and re-localize the fallen object before attempting recovery.

- A deep side-facing descent showed shoulder undertravel and a visible tilt of
  the base relative to the table. A small reverse movement succeeded, but the
  tilt remained. Treat this as a possible mounting shift: hold, request physical
  inspection, and revalidate the base reference before further motion. Joint
  feedback alone cannot detect a loose mount or validate saved world coordinates.

- Upright bottle attempts closed the jaws successfully but left the bottle on
  the mat during small lift checks. Some closures pushed or rotated it instead.
  Image overlap between fingertips and body does not prove contact at the same
  depth. Reacquire a displaced bottle, inspect both opposed contacts, and change
  the approach orientation after repeated misses. Do not keep nudging it toward
  a table edge or compensate by exceeding the recorded gripper travel.
- Calculate commissioning duration from the largest measured joint delta and
  the configured peak-speed bound, including gripper moves. A rejected duration
  sends no motion; correct timing without widening the speed bound.
- Even empty-arm coordinated retreats showed small shoulder/elbow undertravel.
  Use measured final poses for subsequent localization and planning. Record the
  error and inspect clearance; a rejected completion result does not mean the
  arm stayed at its starting pose.

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
- For the tested rectangular box, a shallow edge grasp allowed rotation during
  descent. Center the jaw gap on the body's middle, lower both fingers farther
  along its sides without table contact, and verify retention with a short lift.
  The deeper grasp and a staged return succeeded: first level the held box at an
  airborne waypoint, then lower onto support. Direct high-to-table interpolation
  previously produced rotation and a gripper-drift stop. Do not reuse this route
  for another object or location without fresh localization and clearance checks.
- An operator-demonstrated top pose is a measured reference, not a guarantee of
  maximum reach. Verify powered tracking and retained payload with the wrist
  camera when the external view cannot see the top. Save actual complete trial
  time, including failures and verification; a sequence that finishes correctly
  after the time limit has not met a timed goal.
- With the deeper center grasp validated, direct coordinated returns also passed
  on the unchanged scene. A local three-pick sequence finished just under its
  time budget with camera checkpoints and retained the final payload at the top.
  This was one trial with almost no timing margin, not reliable throughput.
  Keep gripper contact errors separate from verified grasp success, preserve the
  existing speed/feedback limits, and calculate duration from fresh joint travel.
  Use scene-specific image references: an old edge-grasp reference produced a
  false retention rejection after the center grasp changed the camera appearance.
- Torque status must come from fresh motor-register readback when confirming
  manual positioning. A ROS transition to inactive previously reported success
  while motor torque registers remained enabled. Do not announce torque off or
  display green from that lifecycle response alone. Support the arm before
  disabling torque; verify all six registers are zero. Unknown/stale status is
  amber. A read-only helper may open the bus only after ROS has closed it, and
  must stop before ROS restarts. Never run two serial owners together.
- USB camera device numbers can change after reconnecting. Check device identity
  and capabilities before rebinding the preview; a camera that returns images
  may now point at a different scene. Verify both current views before using
  prior object localization, tabletop geometry, or arm clearance assumptions.
- Drawing with an attached pencil requires a calibrated pencil-tip transform,
  reachable paper boundary, paper-plane height, tool orientation, and a small
  contact test. Claw coordinates and pickup poses do not describe the pencil tip.
  Camera appearance alone does not measure contact force. Keep the gripper fixed
  around the attachment; lift the tip between disconnected strokes. Recalibrate
  if the paper, camera, robot base, or pencil mounting moves. Store real images,
  transforms, and stroke execution records only in ignored host-local files.
