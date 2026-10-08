# Camera-guided drawing

The pen is a fixed tool. Its tip, rather than the claw origin, must be mapped to
the paper plane. Keep a taped gripper fixed throughout drawing and recovery.

## Use the existing stack

- Use OpenCV camera calibration and planar homography for the paper. A homography
  maps points on one plane; it does not measure the height of a raised pen.
  [OpenCV calibration](https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html).
- Track visible features with OpenCV pyramidal Lucas-Kanade optical flow, checking
  forward/backward consistency and rejecting lost tracks. Reacquire after an
  occlusion, camera movement, or large change. Feature flow alone is not pen-tip
  localization or contact verification.
  [OpenCV optical flow](https://docs.opencv.org/4.x/d4/dee/tutorial_optical_flow.html).
- Follow image-based visual servoing principles: measure image error, use a locally
  measured image response, apply a bounded correction, and observe again. ViSP
  provides established tracking and visual servoing examples; a separate stack
  is unnecessary for initial OpenCV commissioning.
  [ViSP](https://github.com/lagadic/visp).
- Let the existing ROS joint trajectory controller interpolate coordinated
  movements. Supplying zero velocity at every tracing waypoint makes the robot
  stop at every waypoint. Future continuous strokes need checked derivatives and
  time parameterization; do not simply remove fields or increase speed limits.
  [ROS interpolation](https://control.ros.org/jazzy/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html).

## Continuous observation

`scripts/camera_preview.py` owns the two USB cameras. Its MJPEG feeds include
frame sequence numbers and host timestamps; `/camera-state` exposes freshness.
These timestamps are assigned when the JPEG becomes available, not measured
sensor exposure times. USB buffering and transport latency remain uncertain.

`scripts/drawing_observer.py` consumes those existing streams without opening
another camera device or the motor bus. It records both views at approximately
10 Hz, tracks image features, and pairs the latest frames with the ROS telemetry
file. Store its output outside Git:

```bash
python3 scripts/drawing_observer.py \
  --telemetry-file /path/to/local/preview-telemetry.json \
  --output /path/to/local/new-drawing-trial --seconds 300
```

Each trial needs a new output directory. `observations.jsonl` contains the actual
timestamps, frame sequences, joint feedback, tracking diagnostics, and freshness.
The AVI files are nominal 10 Hz previews; use the index for timing analysis.
`status.json` is replaced atomically. `paired_fresh` requires both cameras younger
than 0.5 seconds, joint feedback younger than 0.4 seconds, camera skew below 0.15
seconds, and camera/feedback skew below 0.4 seconds. These are commissioning
checks, not proof of hardware synchronization. The observer is passive: it neither
authorizes nor cancels motion and does not yet track a calibrated pen tip.

## Resume drawing

1. Check both current views, the complete reachable paper boundary and inset,
   pen attachment, and fresh actual joint feedback.
2. Verify a shoulder-only lift with the fixed wrist/gripper. Confirm physical
   separation before lateral transfer. Use actual feedback after undertravel.
3. Measure pen-tip response while raised, then validate a short contact stroke.
   Preserve tool orientation and paper-plane contact; camera appearance cannot
   establish pressure. Do not increase pressure to compensate for faint ink.
4. Execute small checked batches, recording stroke completion before the lift.
   Re-localize after contact or a failed transfer. Preserve existing joint limits
   and endpoint tolerances.
5. Compare a perspective-corrected drawing with the saved reference, including
   the English label, and finish at a visibly clear paper-change pose.

When serial feedback disappears, hold/cancel and inspect the ROS hardware state.
Camera streams do not substitute for motor feedback. A failed lifecycle recovery
can leave torque status unknown. Support the arm before a controller restart that
could remove holding torque; keep ROS as the sole follower serial owner.

The observer can optionally log a colored, elongated nib candidate with
`--tip-hsv-low H S V --tip-hsv-high H S V --tip-roi x0 y0 x1 y1`.
This is an experimental image diagnostic: ink, shadows and tool edges can confuse
it. Missing or inconsistent candidates must not authorize contact or motion.
Each newly recorded frame now has a `video_frame_index` in the observation log;
use that index and its timestamp to inspect motion, rather than nominal AVI FPS.

Use `--simplify-px` when preparing strokes to reduce redundant contour points.
The default is 0.6 reference-image pixels. Inspect the generated preview before
using a larger tolerance; graph junctions and closed contours are preserved.
This reduces waypoint stops, but does not fix tool mounting or paper contact.
