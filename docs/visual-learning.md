# Learning small movements from the cameras

Use a measured feedback loop: observe, predict, make one small movement, measure
what happened, then correct. This creates a local relationship between joint
movement and image movement. It does not establish a calibrated 3D grasp pose or
train a general picking policy.

## Record each experiment

Keep actual evidence outside Git, under a restricted local directory. Save:

- Initial joint readings, requested target, final readings, and timestamps.
- Controller reference, feedback, error, and action result throughout the move.
- Both camera images before and after the move, while motor hold is still active.
- Calibration and robot-model hashes, camera resolution, and target tracking ROI.
- Measured image displacement, tracking quality, and whether the return succeeded.
- Any serial timeout, stale feedback, unexpected joint movement, or missing image.

The external view checks clearance and physical movement. The wrist view measures
the target relative to the claw. Read frames from the existing preview stream;
opening and closing camera devices during control has caused a serial timeout.

## First experiments

1. Keep the camera mounts, robot base, and object fixed. Check the current scene.
2. Stop the separate serial telemetry reader before starting ROS. ROS must be the
   sole owner of the follower connection during motion.
3. Use fresh feedback to establish the current-position hold. Check every joint
   against its measured encoder range. Validate the model mapping before using IK.
4. Change one joint by a bounded small amount over several seconds. Use explicit
   trajectory goal tolerances; controller success alone is insufficient evidence.
5. Reject stale feedback, missing images, unexpected motion, or tracking failure.
6. Save the measured result before shutdown. Do not compare a powered pose to an
   unsupported pose after torque release: gravity can move the arm.
7. Return to the measured starting angle with the same checks. Record return error.
8. Repeat at nearby poses only when the first result and clearance are understood.

Software cancellation and torque release cannot keep an unsupported arm upright.
Plan a supported resting pose and keep the operator's cutoff accessible.

## Estimate a local image response

For each joint, divide observed image displacement by **measured** joint movement:

```text
local response = [change in image x, change in image y] / change in joint angle
```

Use `scripts/measure_visual_step.py` on the saved wrist images to measure target
feature movement. Choose an ROI containing only the stationary target. Background
or claw features can contaminate the estimate. Inspect images as well as numbers.

Several independent joint experiments form a local image Jacobian. Use it to
propose another small correction, observe again, and update the estimate. Bound
steps, reject ill-conditioned estimates, and stop if the observed response differs
from the prediction. One base experiment cannot determine depth or a full grasp.

Moving a camera invalidates the visual baseline. Changing pose, object height, or
gripper orientation requires a new local measurement. A side view helps distinguish
height from forward movement; two ordinary cameras are not automatically calibrated
stereo cameras.

## Progress toward a grasp

First verify lateral alignment above the object. Then verify vertical clearance
from the side view. Approach in small increments, close the claw within its tested
range, and attempt only a small initial lift. Require visual evidence that the
object moved with the claw before reporting a successful pick. Release over the
observed clear tabletop. Record failed and uncertain outcomes as well as successes.

Raw images, calibration, device identities, and experiment logs remain local and
must not be included in the public repository.

## Repeat the learned skill

Keep ROS running between movements so gravity cannot fold the arm when a command
finishes. `visual_learning_session.py` sends coordinated arm trajectories lasting
1.5 seconds and records both camera views before and after each step. Each joint
change is limited to 0.12 radians. The gripper moves separately. Measured travel,
fresh joint feedback, controller state, and interpolated height checks remain active.
The height estimate is provisional; it does not replace camera clearance checks.

The successful experiment required jaws beside the box below its upper rim.
Closing on the top face merely slid or rotated it. A stopped closing motion can
indicate contact, but only a lift observed by both cameras proves a grasp. Raise
with elbow steps when a loaded shoulder cannot meet its tracking tolerance.

Store successful approach, placement, retreat, and original-pose waypoints in an
ignored host-local JSON file. Replay them with `scripts/replay_visual_skill.py`.
Use separate gripper waypoints and `inspect: true` at grasp and release boundaries.
The script checks measured convergence and stops on a failed action. Its output
never claims task success without visual verification. Camera or object movement
requires another alignment check; recorded poses are not a general picking policy.

Arm trajectories specify zero endpoint velocity and acceleration so ROS can
interpolate smooth starts and stops across coordinated joints. An unsuccessful
position target writes `observed-movement-limits.jsonl` with the affected axis,
requested and measured positions, direction, complete pose, and evidence directory.
These are provisional limits for that load and pose. They do not overwrite motor
calibration: a payload, contact, or weak tracking can all cause undertravel. Inspect
the feedback trace and camera evidence before classifying a mechanical limit.
