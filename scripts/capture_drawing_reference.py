#!/usr/bin/env python3
"""Save a manually demonstrated pen pose while torque is confirmed off.

Run against the existing preview/read-only telemetry helper. Does not move the
arm or open a serial port. Output belongs in an ignored host-local directory.
Recorded corner poses are calibration observations, not a validated mapping.
"""
import argparse
import json
from pathlib import Path
import time
import urllib.request


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('label', choices=['setup', 'upper-left', 'upper-right',
                                        'lower-right', 'lower-left', 'center'])
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--preview', default='http://127.0.0.1:8765')
    args = parser.parse_args()
    with urllib.request.urlopen(args.preview + '/telemetry', timeout=3) as response:
        telemetry = json.load(response)
    if (not telemetry.get('fresh') or telemetry.get('torque_off') is not True
            or time.time() - telemetry.get('timestamp', 0) > 1):
        raise RuntimeError('Fresh motor torque-off readback is required')
    if len(telemetry.get('joints', [])) != 6:
        raise RuntimeError('All six encoder readings are required')
    frames = {}
    for role in ('external', 'wrist'):
        with urllib.request.urlopen(args.preview + '/snapshot/' + role,
                                    timeout=3) as response:
            frames[role] = response.read()
    args.output.mkdir(parents=True, exist_ok=True, mode=0o700)
    stamp = str(time.time_ns()) + '-' + args.label
    for role, frame in frames.items():
        path = args.output / (stamp + '-' + role + '.jpg')
        path.write_bytes(frame)
        path.chmod(0o600)
    path = args.output / (stamp + '.json')
    path.write_text(json.dumps({'label': args.label, 'telemetry': telemetry,
                               'mapping_validated': False}, indent=2))
    path.chmod(0o600)
    print(path)


if __name__ == '__main__':
    main()
