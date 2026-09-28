"""Score operator-verified held-out picks; never commands hardware."""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def score(rows: list[dict]) -> dict:
    if len(rows) != 10:
        raise ValueError("exactly ten first-attempt held-out objects required")
    ids = [row["object_id"] for row in rows]
    if len(set(ids)) != 10:
        raise ValueError("objects must be distinct")
    if any(not row.get("eligible") or not row.get("previously_unseen") for row in rows):
        raise ValueError("all objects must be eligible and unseen")
    unsafe = sum(bool(row["unsafe_command"]) for row in rows)
    verified = sum(row["task_success"] == "verified" for row in rows)
    return {"verified_first_attempt": verified, "unsafe_commands": unsafe,
            "passes": verified >= 8 and unsafe == 0}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("results", type=Path, help="private JSON with ten operator-reviewed outcomes")
    args = parser.parse_args()
    print(json.dumps(score(json.loads(args.results.read_text())), indent=2))


if __name__ == "__main__":
    main()
