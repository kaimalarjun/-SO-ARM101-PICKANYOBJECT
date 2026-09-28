"""Operator-supervised leader demonstration capture through the ROS backend.

Only the leader serial bus is opened by LeRobot. The follower remains owned by
ros2_control and every target passes through MoveIt planning and a step bound.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros"))
from kendra_robot.ros_backend import RosBackend
from kendra_robot.vision import VisionClient

JOINTS = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--camera", type=Path, required=True)
    parser.add_argument("--repo-path", type=Path, required=True)
    parser.add_argument("--leader-port", required=True)
    parser.add_argument("--leader-id", required=True)
    parser.add_argument("--leader-map", type=Path, required=True,
                        help="private measured raw-leader to ROS-radian scale/offset JSON")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--task", required=True)
    parser.add_argument("--fps", type=int, default=2)
    parser.add_argument("--seconds", type=int, default=20)
    parser.add_argument("--operator-enable", action="store_true")
    args = parser.parse_args()
    if not args.operator_enable or args.fps < 1 or args.fps > 5:
        parser.error("operator enable required; capture rate must be 1-5 Hz")
    import yaml
    from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig
    robot_cfg = yaml.safe_load(args.robot.read_text())
    camera_cfg = yaml.safe_load(args.camera.read_text())
    if (robot_cfg["mode"] != "real" or not robot_cfg["operator_enabled"] or
            args.leader_port == robot_cfg["follower_port"]):
        parser.error("real robot, enabled operator and separate leader bus required")
    if args.output.exists():
        parser.error("output exists; refusing overwrite")
    mapping = json.loads(args.leader_map.read_text())
    if set(mapping) != set(JOINTS) or any(set(mapping[name]) != {"scale", "offset"}
                                          for name in JOINTS):
        parser.error("all six measured leader mappings required")
    args.output.mkdir(mode=0o700, parents=True)
    backend = RosBackend(robot_cfg, args.repo_path)
    leader = SO101Leader(SO101LeaderConfig(port=args.leader_port, id=args.leader_id))
    camera = VisionClient(camera_cfg, {})
    cancel = threading.Event()
    leader.connect()
    try:
        deadline = time.monotonic() + args.seconds
        frame_index = 0
        with (args.output / "episode.partial").open("w") as log:
            while time.monotonic() < deadline:
                started = time.monotonic()
                action = leader.get_action()
                target = {name + "_joint": float(action[name + ".pos"]) * mapping[name]["scale"]
                          + mapping[name]["offset"]
                          for name in JOINTS}
                before = [backend.joint_positions[name + "_joint"] for name in JOINTS]
                stamp = time.monotonic_ns()
                image = camera.capture()
                frame_path = args.output / f"frame-{frame_index:06d}.jpg"
                frame_path.write_bytes(image)
                backend.guarded_joint_step(target, cancel, time.monotonic() + 2.0)
                log.write(json.dumps({"task": args.task, "monotonic_ns": stamp,
                                      "state": before,
                                      "action": [target[name + "_joint"] for name in JOINTS],
                                      "image_path": str(frame_path)}) + "\n")
                log.flush()
                frame_index += 1
                elapsed = time.monotonic() - started
                if elapsed > 1.5 / args.fps:
                    raise RuntimeError("capture missed synchronization deadline; episode left partial")
                time.sleep(max(0, 1 / args.fps - elapsed))
        (args.output / "episode.partial").rename(args.output / "episode.jsonl")
    except BaseException:
        cancel.set()
        backend.stop()
        raise
    finally:
        leader.disconnect()


if __name__ == "__main__":
    main()
