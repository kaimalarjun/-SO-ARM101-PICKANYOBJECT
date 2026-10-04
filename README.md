# SO-ARM101 tabletop pick prototype

This project is a test setup for picking small objects from a table with an SO-ARM101 robot arm and an overhead USB camera. ROS 2 controls the arm; a separate vision program can identify objects in camera images. You can try the software with a simulated arm before connecting hardware. The code is public under Apache 2.0.

**Kendra** is the name of the original project's voice assistant. It uses [OpenClaw](https://github.com/openclaw/openclaw) to call robot tools. You do **not** need Kendra to use this repository: the local robot API can be called directly, and the OpenClaw plugin is optional. Some package, command, and folder names still contain `kendra` for compatibility; they are ordinary local names.

## What runs where

- **Follower:** the SO-ARM101 arm that moves and picks objects. ROS 2 owns its USB controller during operation. The one-time calibration helper runs separately, with ROS stopped.
- **Leader:** an optional second arm that a person moves by hand to record demonstrations for later training.
- **Robot computer:** an Ubuntu 24.04 computer connected to the follower and camera. It runs ROS 2 Jazzy, motion planning, and a small local HTTP service called the *gateway*.
- **Vision computer:** an optional second computer for describing images and locating objects. It can run Qwen3.5-4B and Grounding DINO Tiny; the robot computer connects to it privately.
- **Mock mode:** simulated arm hardware. It lets you test ROS and the gateway without opening a robot USB port or powering the arm.

The first target is a small, rigid, isolated object on a flat, contrasting table. A single overhead camera cannot measure every object's height or confirm a hidden grasp. The API reports `task_success: uncertain` when it cannot verify the result; a voice assistant should say “Done” only for `task_success: verified`.

## Start with mock mode

On the Ubuntu robot computer, from this repository:

```bash
bash scripts/preflight.sh
bash scripts/install_robot_host.sh
source /opt/ros/jazzy/setup.bash
source "$HOME/so101_ws/install/setup.bash"
ros2 launch kendra_robot bringup.launch.py repo_path:="$PWD" mode:=mock
```

In a second terminal, open the same repository, source the same two setup files, then start the gateway with the example mock settings:

```bash
ros2 run kendra_robot kendra-robot-gateway --robot config/robot.example.yaml
```

In another terminal, check its status:

```bash
curl -s http://127.0.0.1:8765/health
```

The installer needs `sudo` for Ubuntu packages. The ROS source versions are pinned in `sources.repos`. Mock mode does not require a connected arm or camera. The ROS stack has been built and started in mock mode on Ubuntu 24.04; physical motion has not been commissioned.

## Add hardware and optional features

1. Before powering the follower, check its controller and adapter labels, mounting, and physical power cutoff. Back up its settings and calibrate it before motion. See [hardware setup](docs/hardware.md) and [supervised operation](docs/operations.md).
2. Mount the camera overhead and measure its coordinates relative to the robot. Store calibration outside Git and check table position error is at most 10 mm. See [camera calibration](docs/calibration.md).
3. For object recognition, run `scripts/install_vision_host.sh` on a vision computer and connect its local service to the robot computer through a private tunnel.
4. If using OpenClaw, build the plugin in `openclaw/` and enable its robot tools only for the assistant you choose. The gateway listens on `127.0.0.1`, not a public network address.
5. Leader demonstrations and SmolVLA training are optional and separate from basic spoken commands. See `training/`.

Real settings belong in a private directory on the robot computer, such as `/var/lib/kendra-robot`. The `config/*.example.yaml` files are templates; their values are not measurements. Keep `mode: mock` and `operator_enabled: false` until the physical checks are complete.

ROS [arm description](https://github.com/ros-physical-ai/ros2_so_arm), [Feetech driver](https://github.com/ros-physical-ai/feetech_ros2_driver), [USB camera driver](https://github.com/ros-drivers/usb_cam), [MoveIt 2](https://moveit.picknik.ai/), [vision models](https://huggingface.co/docs/transformers/model_doc/grounding-dino), and [LeRobot](https://huggingface.co/docs/lerobot/index) are separate dependencies with their own licenses and terms.

## Tests and private data

Run `python3 -m unittest discover -s tests -v` for the small software test suite. It checks bounds, stale observations, stop behavior, and the gateway's fake-camera flow. These tests do not prove real hardware safety.

Do not commit credentials, network addresses, device identifiers, calibration, camera images, datasets, or model checkpoints. `.gitignore` excludes common local files; check changes before publishing. Your assistant's private configuration stays outside this repository.
