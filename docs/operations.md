# Supervised operations and acceptance

## Before each session

- Check follower mounting, power cutoff, clear table, lighting, stable device paths and camera marker visibility.
- Confirm ROS joint feedback and controller state. Start with `mode:=mock` and `operator_enabled:=false`.
- Check camera calibration against independent points. Confirm the vision tunnel stays on loopback and fails closed.
- Keep a human at the cutoff for the first powered movement and all initial picks.

`robot.stop()` latches a stop and asks the controller to cancel. Restart and inspect after a stop; do not auto-reset. ROS watchdog requests trajectory cancellation if command heartbeat expires. These software mechanisms cannot guarantee a safe stop after complete host/ROS failure; the physical cutoff is required.

## Tool semantics

`robot_look` returns current candidate objects and an observation ID. `robot_pick` must use its measured coordinates and that fresh ID. Reject ambiguous/missing objects, stale frames, invalid calibration, occupied approaches, and paths outside measured workspace. `robot_place` requires a separately checked empty zone. Tool coordinates are metres in `base_link`; no raw servo commands are exposed.

Results carry `motion_success` and `task_success`. Motion execution alone does not prove the object was lifted or placed. If the overhead view cannot verify a grasp, report `task_success: uncertain` and ask the operator; never say “Done” for uncertain or failed outcomes.

## Acceptance sequence

1. Mock: state, home, gripper, safe XYZ and rejected XYZ without serial access.
2. Real, supervised: identify devices and ratings; read joint state; verify each direction and limit slowly; test cutoff and stop; preserve offsets.
3. Camera: fresh frame and ≤10 mm independent table calibration after restart.
4. Vision: replay and live named/unseen object detection; reject ambiguity and outage.
5. Full fake-hardware command path through OpenClaw, gateway and ROS; stop and lease-loss cases.
6. Real known-location pick; then Kendra-invoked look, pick, visual verification and place. Only then try the three spoken requests.
7. Record local leader demonstrations and train SmolVLA separately. Keep trained-policy evaluation out of Kendra's spoken flow. Supervise ten first-attempt picks of held-out eligible objects; promotion threshold is at least eight verified successes and zero unsafe commands.

If a gate fails, disable the real service and return to the last passing stage. Keep camera frames, logs, calibration, local datasets and checkpoints outside this repository.
