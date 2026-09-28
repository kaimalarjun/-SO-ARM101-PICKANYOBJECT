"""Convert reviewed ROS-guarded demonstration frames to a local LeRobotDataset.

Input is a private JSONL episode captured by an operator-controlled ROS session.
Each line has task, monotonic_ns, state (6 joints), action (6 joints), and
image_path. This converter does not open or command the follower serial bus.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def validate_episode(rows: list[dict], fps: int) -> None:
    if not rows:
        raise ValueError("empty demonstration")
    last = -1
    for row in rows:
        if len(row["state"]) != 6 or len(row["action"]) != 6:
            raise ValueError("expected five arm joints and gripper")
        if row["monotonic_ns"] <= last:
            raise ValueError("timestamps must increase")
        last = row["monotonic_ns"]
        if not row["task"]:
            raise ValueError("task label required")
    gaps = [(b["monotonic_ns"] - a["monotonic_ns"]) / 1e9 for a, b in zip(rows, rows[1:])]
    if gaps and max(gaps) > 1.5 / fps:
        raise ValueError("missing synchronized frames")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("episodes", type=Path, help="private directory of reviewed .jsonl episodes")
    parser.add_argument("output", type=Path, help="private dataset directory outside Git")
    parser.add_argument("--fps", type=int, default=10)
    parser.add_argument("--repo-id", default="local/so101_tabletop")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output already exists; refusing overwrite")
    files = sorted(args.episodes.rglob("*.jsonl"))
    if not files:
        parser.error("no reviewed episodes found")
    episodes = []
    for file in files:
        rows = [json.loads(line) for line in file.read_text().splitlines() if line.strip()]
        validate_episode(rows, args.fps)
        episodes.append(rows)
    from lerobot.datasets import LeRobotDataset
    import numpy as np
    from PIL import Image
    with Image.open(episodes[0][0]["image_path"]) as image:
        width, height = image.size
    features = {
        "observation.images.top": {"dtype": "video", "shape": (height, width, 3),
                                   "names": ["height", "width", "channels"]},
        "observation.state": {"dtype": "float32", "shape": (6,), "names": None},
        "action": {"dtype": "float32", "shape": (6,), "names": None},
    }
    dataset = LeRobotDataset.create(repo_id=args.repo_id, root=args.output, fps=args.fps,
                                    features=features, robot_type="so101_ros_guarded", use_videos=True)
    try:
        for rows in episodes:
            for row in rows:
                with Image.open(row["image_path"]) as image:
                    if image.size != (width, height):
                        raise ValueError("camera dimensions changed")
                    pixels = np.asarray(image.convert("RGB"))
                dataset.add_frame({"observation.images.top": pixels,
                                   "observation.state": np.asarray(row["state"], dtype=np.float32),
                                   "action": np.asarray(row["action"], dtype=np.float32),
                                   "task": row["task"]})
            dataset.save_episode()
    finally:
        dataset.finalize()
    print(f"Saved {len(episodes)} local episodes at {args.output}")


if __name__ == "__main__":
    main()
