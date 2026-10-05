#!/usr/bin/env python3
"""Measure local target motion between recorded images; does not move hardware."""
import argparse
import json

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('before')
    parser.add_argument('after')
    parser.add_argument('--roi', nargs=4, type=int, required=True, metavar=('X', 'Y', 'W', 'H'))
    parser.add_argument('--measured-joint-delta', type=float, required=True, help='Radians from encoder feedback')
    args = parser.parse_args()
    if not np.isfinite(args.measured_joint_delta) or abs(args.measured_joint_delta) < 0.005:
        raise ValueError('Joint movement too small or invalid for a response estimate')
    before = cv2.imread(args.before, cv2.IMREAD_GRAYSCALE)
    after = cv2.imread(args.after, cv2.IMREAD_GRAYSCALE)
    if before is None or after is None or before.shape != after.shape:
        raise ValueError('Images missing or dimensions differ')
    x, y, w, h = args.roi
    if x < 0 or y < 0 or w <= 0 or h <= 0 or x+w > before.shape[1] or y+h > before.shape[0]:
        raise ValueError('ROI outside image')
    mask = np.zeros_like(before)
    mask[y:y+h, x:x+w] = 255
    points = cv2.goodFeaturesToTrack(before, 80, 0.02, 5, mask=mask)
    if points is None or len(points) < 6:
        raise ValueError('Insufficient target features')
    moved, status, _ = cv2.calcOpticalFlowPyrLK(before, after, points, None)
    back, back_status, _ = cv2.calcOpticalFlowPyrLK(after, before, moved, None)
    good = (status[:, 0] == 1) & (back_status[:, 0] == 1) & (np.linalg.norm(back[:, 0]-points[:, 0], axis=1) < 1)
    if good.sum() < 6:
        raise ValueError('Insufficient consistent feature matches')
    flow = moved[good, 0] - points[good, 0]
    shift = np.median(flow, axis=0)
    print(json.dumps({'roi_px': args.roi, 'tracked_features': int(good.sum()),
                      'measured_joint_delta_rad': args.measured_joint_delta,
                      'median_target_shift_px': shift.tolist(),
                      'pixels_per_joint_radian': (shift/args.measured_joint_delta).tolist(),
                      'median_flow_residual_px': float(np.median(np.linalg.norm(flow-shift, axis=1))),
                      'scope': 'Local image response only; not a calibrated depth or grasp estimate'}, indent=2))


if __name__ == '__main__':
    main()
