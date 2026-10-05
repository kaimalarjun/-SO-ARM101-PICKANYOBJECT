#!/usr/bin/env python3
"""Replay a host-local, camera-validated joint waypoint skill through ROS.

The JSON file contains waypoints [{"positions": {"joint_name": radians}}].
Never distribute actual poses as universally valid robot settings.
"""
import argparse
import json
import math
import time
import urllib.request


def request(url, data=None):
    body = None if data is None else json.dumps(data).encode()
    return json.load(urllib.request.urlopen(urllib.request.Request(
        url, data=body, headers={'Content-Type': 'application/json'}), timeout=5))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('skill')
    parser.add_argument('--gateway', default='http://127.0.0.1:8766')
    args = parser.parse_args()
    skill = json.load(open(args.skill, encoding='utf-8'))
    started = time.monotonic()
    for waypoint in skill['waypoints']:
        targets = waypoint['positions']
        if not targets or any(not math.isfinite(float(v)) for v in targets.values()):
            raise ValueError('Invalid waypoint')
        if 'gripper_joint' in targets and len(targets) > 1:
            raise ValueError('Gripper must have its own waypoint')
        for _ in range(40):
            state = request(args.gateway + '/state')
            if not state['fresh'] or state['latched'] or state['busy']:
                raise RuntimeError('Gateway not ready')
            positions = state['raw_ros_positions']
            # 0.02 rad convergence avoids repeated tiny steps against hold error.
            deltas = {n: max(-.12, min(.12, float(v)-positions[n]))
                      for n, v in targets.items() if abs(float(v)-positions[n]) > .02}
            if not deltas:
                break
            result = request(args.gateway + '/step', {'deltas': deltas})
            if not result.get('success') or result.get('capture_error'):
                raise RuntimeError('Motion or camera evidence failed')
            print(json.dumps(result), flush=True)
        else:
            raise RuntimeError('Waypoint failed to converge')
        if waypoint.get('inspect'):
            input('Inspect both live camera views, then press Enter to continue: ')
    print(json.dumps({'elapsed_seconds': time.monotonic()-started,
                      'task_success': 'requires visual verification'}))


if __name__ == '__main__':
    main()
