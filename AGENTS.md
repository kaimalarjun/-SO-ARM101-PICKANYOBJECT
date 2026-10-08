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
- Current drawing setup: the operator taped the pen into the gripper, which must
  remain fixed. Do not issue gripper open/close commands or reuse pickup routines
  that actuate the gripper. Treat the pen as a fixed tool until the operator
  explicitly confirms the tape has been removed and gripper operation restored.
- Drawing boundaries: every pencil-down segment, including any written label,
  must stay inside the currently verified white paper with an inset margin that
  accounts for tip/mapping uncertainty. Reject paths outside that boundary; stop
  and recalibrate if the paper moves or its boundary becomes uncertain.
- After drawing or an interrupted/stuck stroke, lift the pen to a verified clear
  paper-change pose before continuing or inviting a paper change. If feedback is
  lost, the arm is jammed, or lifting would worsen contact, cancel motion and
  request physical recovery instead of forcing an upward move. Keep the taped
  gripper fixed. Revalidate the paper plane and boundary after replacement.
- Compare completed drawings with the reference after aligning/correcting their
  image perspectives. Assess missing/extra strokes, proportions, placement,
  contours, detail, line continuity, and the English object label. Record specific
  discrepancies, causes with confidence, corrective actions, and evidence from
  a follow-up check. Do not claim an exact replica from visual plausibility alone
  or redraw blindly over errors. State physical/measurement limits honestly.
- Identify the depicted object and name it in plain English; disclose ambiguity.
  Report the name in the response and, when planning an on-paper label, reserve
  space inside the white paper and include its strokes in the boundary checks.
- Persist drawing failures and validated corrections as procedural memory. Read
  relevant lessons before the next attempt, change the failing approach, and
  verify that the correction worked. Distinguish observations from hypotheses;
  recorded lessons reduce recurrence but do not guarantee mistakes never repeat.
- In the taped-pen setup, an attempted shoulder/elbow lift with wrist compensation
  left an unintended line; predicted claw height did not prove pen clearance.
  The operator requested lifting through the middle arm servo instead. A small
  shoulder-only raise with wrist and gripper targets unchanged produced clearer
  tip/shadow separation, and the operator confirmed the pen lifted off the paper.
  Prefer this verified shoulder-only lift for the unchanged attachment and setup;
  keep wrist and gripper targets fixed. Treat it as a setup-specific recovery,
  not calibrated pen-tip height. Verify actual separation before lateral travel.
- Drawing commissioning produced faint test marks, but two small antenna-stroke
  executions did not yield a clearly verifiable outline even after one bounded
  contact-depth correction. A completed joint trajectory is not proof of ink or
  graphite deposition. Lift and clear the view to inspect marks; verify the
  writing tip works before increasing pressure or executing the entire image.
  Keep motion success, visible-mark success, and reference fidelity separate.
- Image-space tracing can omit filled pupils and short outline connections when
  skeleton strokes are filtered too aggressively. Review the stroke preview,
  retain short connections, and explicitly hatch compact filled regions before
  mapping to robot motion. This preview does not validate physical drawing.

- The operator subsequently confirmed the commissioning pen marks were visible.
  Continue with bounded central-paper strokes and lifted transfers, retaining
  fresh feedback checks. This confirms writing contact in this setup, not exact
  picture fidelity or uniform pressure across the paper.

- Central-paper drawing continued after the operator confirmed ink. Multiple
  outlines executed, but pen-up transfers repeatedly showed shoulder endpoint
  undertravel despite slower motion. Some rejected moves still lifted the tip;
  inspect fresh images and measured positions before revising a transfer.
  Small segmented lifts did not reliably resolve the error. A higher lift
  cleared the paper in one tested pose, but did not eliminate later failures.
  Preserve the existing feedback tolerance; do not infer a mechanical limit
  or exact drawing fidelity from these observations. Save stroke success before
  attempting the lift so a failed lift does not erase completed-stroke records.

- Continuous drawing observation consumes the existing two MJPEG feeds; do not
  open duplicate USB capture owners. Pair frame sequences and host timestamps
  with actual ROS feedback. OpenCV forward/backward optical flow is a diagnostic,
  not calibrated tip depth, contact pressure, or motion authorization. Host JPEG
  timestamps are not hardware exposure timestamps. Keep recordings outside Git.
- A serial read timeout deactivated the ROS hardware during camera-service
  recovery. A lifecycle reactivation then received an unexpected serial reply.
  The camera feeds remained fresh while joint feedback became stale; reject
  motion in that state. The observed timing does not prove the camera restart
  caused the timeout. Support the arm before restarting its sole ROS serial
  owner, since shutdown/recovery may remove holding torque. Preserve the fixed
  taped gripper and inspect both views after feedback returns.

- The operator reported that pen drag/compliance can shorten the actual mark
  relative to arm travel. Treat this as a hypothesis to measure with observed
  pen-tip motion and ink length, not a fixed scale factor. Prefer a firm tool
  mounting and a low-force writing tip; a sketch pen is a candidate, not a
  guaranteed cure. Do not blindly extend strokes or press harder. Any tool
  change invalidates the previous tip transform and contact-depth profile.
- During a subsequent drawing trial, encoder-aligned targets completed several
  additional strokes after unquantized transfers had narrowly failed. The driver
  truncates radian requests to encoder steps; plan representable targets without
  widening tracking tolerances. This result does not eliminate load sag, friction,
  or contact uncertainty. The batch was canceled for a proposed tool change;
  the tip was visibly raised and holding torque remained enabled.

- Operator review of the first drawing: too small, broken lines, nonsmooth
  shapes, and unclear/unfilled eyes. The run was incomplete; missing eye strokes
  must not be explained solely by friction. The nominal canvas was only about
  30 by 40 mm, so joint quantization/tracking error was large relative to detail.
  Improve by increasing size only within a newly verified reachable paper inset,
  using continuous connected outlines and smooth time parameterization, validating
  ink continuity, and completing bounded pupil hatching. Do not claim these
  corrections succeeded until a physical drawing is compared with the reference.
- A replacement sheet invalidates the previous paper boundary/plane. In the
  reviewed replacement setup, the external view was too low to see all corners
  and the wrist view mainly showed the tip. Reposition/recheck the camera or
  obtain another reliable boundary measurement before expanding the drawing.
  A warped sheet and tool contact require a new short stroke/contact check.

- The operator supplied a clear full-sheet photo of the first attempt. It shows
  a small drawing low in the bounded region, disconnected head/body/limb contours,
  distorted proportions, and no visible eyes or English label. Uneven ink density
  is observed; drag, tool flex, varying contact, and tracking error remain candidate
  causes. The eye/label stages had not executed, so their absence is also an
  execution-completeness failure. Stray marks exist; attribution of every mark
  is uncertain. Do not call this an adequate replica or infer a single scale fix.
- Before the next full drawing, validate a short horizontal and vertical line,
  a rectangle, and a circle within the newly checked paper inset. Compare visible
  ink lengths, continuity, aspect ratio, closure, and curvature with intended paths.
  Use the results to separate scale/mapping distortion from intermittent contact.
  Increase drawing size only after reach and full boundary checks; complete and
  inspect pupils and the label, then compare the whole result with the reference.

- Try 2 started after the operator enabled it. ROS hold and a nominal 10 mm
  straight test succeeded under the existing limits; the test used one continuous
  three-second segment rather than stops at each tracing vertex. Ink remained
  faint and short. Contact snapshots showed greater apparent shaft displacement
  than nib displacement, consistent with drag/flex but not proof of its cause.
  The nib still appeared ballpoint-like; confirm the tool before assuming a
  felt-tip replacement. Do not enlarge strokes to compensate blindly. Verify
  a low-force, firmly mounted tip and a continuous visible mark before continuing
  the full picture. The arm finished the test with the pen visibly lifted.

- Reviewing the original rectified reference revealed that an earlier binary
  image omitted portions of the eye contours. Adaptive thresholding of the
  photographed reference restored closed eye rings; reviewed border exclusion
  and component filtering removed a glare artifact while retaining pupils and
  smile. Inspect the rendered stroke plan against the original before blaming
  all missing details on motion. This fixes image-space planning, not physical
  ink continuity or tip calibration.

- A longer sensitive felt-tip replacement invalidates the previous tool/contact
  mapping. Keep the taped gripper fixed and use small contact corrections, not
  extra pressure to darken ink. In the first replacement-tool trial purple marks
  were visible, while rectangle/circle and head passes remained partial
  or distorted despite successful controller completion. Motion success does
  not establish ink continuity, shape fidelity, or correct tip height. Review
  fresh ink evidence before repeating or extending the drawing.
- The wrist view in this setup shows a grey finger more clearly than the actual
  purple nib. Do not track that finger as the pen tip. Use the external view for
  nib/ink evidence and the wrist view for clearance until actual tip visibility
  is established. Reject a proposed drawing canvas that approaches a paper edge;
  raised corner checks can reveal a reach-feasible plan is poorly placed.
- A shorter supervised line-and-lift sequence completed in about seven seconds
  under the existing speed/tolerance bounds, but its ink result remained poor.
  Faster completion is not a drawing success. Color-only nib candidates were
  ambiguous or inconsistent in some video frames; never use them as proof of
  paper contact or calibrated motion. Pair each recorded video-frame index with
  its timestamp and feedback; nominal video FPS alone can misalign evidence when
  repeated/stale camera frames are skipped.
- Raised-tool response checks showed that short joint commands and returns can
  complete within tolerance while accumulating enough undertravel/drift to alter
  tip clearance. Measure the actual return pose and inspect both views; a nominal
  return is not restoration of the original tip height. In this trial a larger
  bounded shoulder-only lift restored visible paper clearance. The colored-tip
  diagnostic was too intermittent to calibrate a camera response matrix; retain
  the recordings, reject that calibration and check the attachment/contact setup.
- The operator reported a rigid taped attachment and high contact friction.
  Contact tests located faint dark ink; housing color did not establish ink color.
  A straight pan line and a coordinated shoulder/elbow/wrist line became visible,
  but rectangle/circle fidelity remained poor. Raising the planned contact plane
  by 0.5 mm retained a visible short line in a fresh patch. This is a target
  adjustment, not verified force or submillimetre physical accuracy. Joint error
  varied with movement direction, so a single fitted joint bias is not universal.
  External framing changed during the last check; recheck current paper bounds
  and invalidate old image coordinates before using the plan for a full drawing.
