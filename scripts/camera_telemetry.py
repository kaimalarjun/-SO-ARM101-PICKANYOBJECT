#!/usr/bin/env python3
"""Read encoders while ROS is stopped; write local preview telemetry.

Stop this process before starting ROS: only one process may own the follower bus.
Uses the installed calibration-helper environment. Never writes motor registers.
"""
import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def forward(model, positions, base, tip):
    joints = {j.find('child').get('link'): j for j in model.findall('joint')}
    chain = []
    link = tip
    while link != base:
        joint = joints[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    transform = np.eye(4)
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
        rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
        local = np.eye(4)
        local[:3, 3] = xyz
        local[:3, :3] = rotation([0, 0, 1], rpy[2]) @ rotation([0, 1, 0], rpy[1]) @ rotation([1, 0, 0], rpy[0])
        if joint.get('type') in ('revolute', 'continuous'):
            local[:3, :3] = local[:3, :3] @ rotation([float(v) for v in joint.find('axis').get('xyz').split()], positions[joint.get('name')])
        transform = transform @ local
    return dict(zip(('x', 'y', 'z'), transform[:3, 3].tolist()))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--urdf', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--calibration', type=Path, required=True)
    args = parser.parse_args()
    model = ET.parse(args.urdf).getroot()
    calibration = json.loads(args.calibration.read_text())
    control = model.findall('.//ros2_control/joint')
    names = [j.get('name').removesuffix('_joint') for j in control]
    offsets = {j.get('name'): float(j.find("param[@name='position_offset_rad']").text) if j.find("param[@name='position_offset_rad']") is not None else 0 for j in control}
    motors = {name: Motor(int(j.find("param[@name='id']").text), 'sts3215', MotorNormMode.DEGREES) for name, j in zip(names, control)}
    def ros_active():
        return subprocess.run(['pgrep', '-f', r'(^|/)ros2_control_node( |$)'], stdout=subprocess.DEVNULL).returncode == 0
    if ros_active():
        raise RuntimeError('Stop ROS before opening the follower bus')
    bus = FeetechMotorsBus(args.port, motors)
    bus.connect()
    try:
        while not ros_active():
            ticks = bus.sync_read('Present_Position', normalize=False, num_retry=0)
            torque = bus.sync_read('Torque_Enable', normalize=False, num_retry=0)
            positions = {name + '_joint': (ticks[name] - 2048) * 2 * math.pi / 4096 + offsets[name + '_joint'] for name in names}
            within = all(float(j.find('limit').get('lower')) <= positions[j.get('name')] <= float(j.find('limit').get('upper')) for j in model.findall('joint') if j.get('name') in positions)
            measurements = []
            for n in names:
                cal = calibration[n]
                fraction = (ticks[n] - cal['range_min']) / (cal['range_max'] - cal['range_min'])
                fraction = min(1, max(0, fraction))
                measurements.append({'name': n, 'degrees': math.degrees(positions[n + '_joint']),
                                     'ticks': ticks[n], 'normalized': fraction * 100 if n == 'gripper' else fraction * 200 - 100,
                                     'range_min': cal['range_min'], 'range_max': cal['range_max'],
                                     'in_range': cal['range_min'] <= ticks[n] <= cal['range_max']})
            data = {'timestamp': time.time(), 'xyz_m': forward(model, positions, 'base_link', 'gripper_frame_link'),
                    'torque_off': not any(torque.values()), 'within_model_limits': within,
                    'joints': measurements}
            temporary = args.output.with_suffix('.tmp')
            temporary.write_text(json.dumps(data))
            os.chmod(temporary, 0o600)
            os.replace(temporary, args.output)
            time.sleep(0.2)
    finally:
        bus.disconnect(disable_torque=False)


if __name__ == '__main__':
    main()
