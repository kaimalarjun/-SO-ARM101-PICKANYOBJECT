"""Local SmolVLA inference for offline inspection; no robot command endpoint."""
from __future__ import annotations

import argparse
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("checkpoint", type=Path)
    parser.add_argument("observation", type=Path, help="private NPZ with image HWC and state[6]")
    args = parser.parse_args()
    if not args.checkpoint.is_dir():
        parser.error("checkpoint directory missing")
    import numpy as np
    import torch
    from lerobot.policies.smolvla.modeling_smolvla import SmolVLAPolicy
    policy = SmolVLAPolicy.from_pretrained(str(args.checkpoint)).eval().to("cuda")
    with np.load(args.observation) as data:
        image = torch.from_numpy(data["image"]).permute(2, 0, 1).float()[None].to("cuda") / 255.0
        state = torch.from_numpy(data["state"]).float()[None].to("cuda")
    with torch.inference_mode():
        action = policy.select_action({"observation.images.top": image,
                                       "observation.state": state})
    print(action.cpu().numpy().tolist())


if __name__ == "__main__":
    main()
