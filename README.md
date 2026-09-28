# SO-ARM101 tabletop pick prototype

Public, Apache 2.0 code for a safety-gated SO-ARM101 follower, overhead camera, local vision service, and Kendra/OpenClaw tools. The robot host owns the follower serial bus through ROS 2 Jazzy and `ros2_control`. A separate vision host can serve Qwen3.5-4B and Grounding DINO Tiny through a private loopback tunnel. No credentials, host addresses, calibration, images, datasets, or checkpoints belong in this repository.

## Current implementation state

The mock gateway, safety checks, HTTP contract, ROS launch/configuration, camera calibration CLI, vision-service code, and OpenClaw plugin are included. **Physical motion is disabled by default.** The real path requires an operator-enabled local configuration and has not been commissioned. Calibrated picks, visual outcome verification, and SmolVLA's held-out benchmark require the arm and camera to be connected and tested by an operator.

The initial object envelope is small, rigid, isolated objects on a flat contrasting table, with top-down approach. A single overhead camera cannot determine arbitrary object height or see hidden gripper contact. An uncertain outcome is reported as `uncertain`; Kendra must only say “Done” after `task_success: verified`.

## Source and host setup

1. Inspect both arms, servo/controller labels, adapter ratings and follower power cutoff. See [hardware](docs/hardware.md).
2. On Ubuntu 24.04, run `bash scripts/preflight.sh`, then `bash scripts/install_robot_host.sh`. The script requests sudo for ROS packages. External ROS checkouts are pinned in `sources.repos`.
3. Copy `config/*.example.yaml` to a **restricted host-local** directory such as `/var/lib/kendra-robot`, replace every placeholder with measured values, and keep `mode: mock` and `operator_enabled: false`.
4. Source `/opt/ros/jazzy/setup.bash` and the workspace `install/setup.bash`. Start `ros2 launch kendra_robot bringup.launch.py repo_path:=$PWD mode:=mock`. Start `kendra-robot-gateway --robot /var/lib/kendra-robot/robot.yaml`.
5. Exercise `curl -s http://127.0.0.1:8765/health` and the mock tests below. Do not connect follower power during mock testing.
6. Mount the overhead camera, calibrate with independent points, and validate no more than 10 mm table error. See [calibration](docs/calibration.md).
7. Install the isolated vision service with `scripts/install_vision_host.sh` on the vision host. Bind its HTTP listener to `127.0.0.1`; use only a private SSH tunnel from the robot host.
8. Build the OpenClaw plugin under `openclaw/`, validate its generated manifest, then allowlist its tools **only** for the intended agent. Keep live chat-model services untouched. See [operations](docs/operations.md).

ROS [SO-101 description](https://github.com/ros-physical-ai/ros2_so_arm), [Feetech driver](https://github.com/ros-physical-ai/feetech_ros2_driver), [usb_cam](https://github.com/ros-drivers/usb_cam), [MoveIt 2](https://moveit.picknik.ai/), [Qwen3.5-4B](https://huggingface.co/Qwen/Qwen3.5-4B), [Grounding DINO Tiny](https://huggingface.co/IDEA-Research/grounding-dino-tiny), and [LeRobot](https://huggingface.co/docs/lerobot/index) are external dependencies. Their licenses and model terms apply separately. This repository does not vendor their source or weights.

## Local tests

```bash
python3 -m unittest discover -s tests -v
```

The tests cover coordinate bounds, stale/ambiguous observation rejection, stop latching, lease expiry, and a fake-camera HTTP look/pick path. They do not certify collision safety or real hardware behavior. Before physical trials, run the acceptance gates in [operations](docs/operations.md).

## Public data boundary

Only generic code and examples are committed. `.gitignore` excludes host settings, calibration, camera frames, datasets, checkpoints, keys and logs. Scan staged content and **all commits** before first push; a later deletion leaves sensitive bytes in Git history. The private Kendra integration should link here and retain all site-specific configuration privately.
