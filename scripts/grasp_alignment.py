#!/usr/bin/env python3
"""Inspect a selected object mask against visible jaw tips; never commands motion.

Supply a binary object mask and a host-local profile. A profile contains jaw_tips_px,
and optionally a measured local image Jacobian (2 rows, one column per joint).
The mask must exclude the fingers, table and other objects. Re-select after contact.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def inspect(mask, profile):
    height, width = mask.shape
    count, labels, stats, _ = cv2.connectedComponentsWithStats((mask > 0).astype('uint8'))
    if count != 2:
        raise ValueError('Select exactly one connected object silhouette')
    if stats[1, cv2.CC_STAT_AREA] < 100:
        raise ValueError('Object silhouette too small')
    ys, xs = np.nonzero(labels == 1)
    if xs.min() == 0 or ys.min() == 0 or xs.max() == width-1 or ys.max() == height-1:
        raise ValueError('Object is clipped; obtain a clear view before alignment')
    points = np.column_stack((xs, ys)).astype(float)
    center = points.mean(axis=0)
    eigenvalues, vectors = np.linalg.eigh(np.cov(points.T))
    body_axis = vectors[:, -1]
    tips = np.asarray(profile['jaw_tips_px'], float)
    if tips.shape != (2, 2) or not np.isfinite(tips).all():
        raise ValueError('Profile needs two finite visible jaw-tip coordinates')
    if (tips < 0).any() or (tips >= [width, height]).any():
        raise ValueError('Jaw tips outside image')
    gap = float(np.linalg.norm(tips[1]-tips[0]))
    if gap < 10:
        raise ValueError('Open the jaws and obtain a valid visible gap')
    closing_axis = (tips[1]-tips[0])/gap
    gap_center = tips.mean(axis=0)
    error = gap_center-center
    across = (points-center) @ closing_axis
    body_width = float(np.percentile(across, 98)-np.percentile(across, 2))
    # A near-round silhouette does not have a reliable principal body axis.
    axis_reliable = eigenvalues[-1]/max(eigenvalues[0], 1) > 1.5
    cross_error = float(np.degrees(np.arcsin(np.clip(abs(body_axis @ closing_axis), 0, 1))))
    centered = float(np.linalg.norm(error)) <= gap*.08
    fits = body_width < gap*.85
    orientation_ok = axis_reliable and cross_error < 20
    result = {
        'object_center_px': center.tolist(), 'jaw_gap_center_px': gap_center.tolist(),
        'image_correction_px': error.tolist(), 'body_axis': body_axis.tolist(),
        'body_width_px': body_width, 'jaw_gap_px': gap,
        'axis_reliable': bool(axis_reliable), 'cross_axis_error_deg': cross_error,
        'centered_in_projection': bool(centered), 'fits_in_projection': bool(fits),
        'orientation_ok_in_projection': bool(orientation_ok),
        'depth_status': 'unverified', 'grasp_ready': False,
        'next': 'rotate while clear' if not orientation_ok else
                'correct alignment while clear' if not centered else
                'check width and jaw opening' if not fits else
                'verify two-sided depth with external view; descend incrementally',
    }
    response = profile.get('local_response')
    if response and not centered and orientation_ok:
        jacobian = np.asarray(response['pixels_per_radian'], float)
        joints = response['joints']
        if jacobian.shape != (2, len(joints)) or not np.isfinite(jacobian).all():
            raise ValueError('Invalid measured local image response')
        if np.linalg.matrix_rank(jacobian) < 2 or np.linalg.cond(jacobian) > 20:
            result['response_status'] = 'insufficient independent measurements'
        else:
            change = np.linalg.pinv(jacobian) @ error
            change *= min(1., .08/max(float(np.max(abs(change))), 1e-9))
            result['suggested_joint_deltas_rad'] = dict(zip(joints, change.tolist()))
            result['response_status'] = 'local suggestion; clearance and gateway checks still required'
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('mask', type=Path)
    parser.add_argument('profile', type=Path)
    parser.add_argument('--overlay', type=Path, required=True)
    args = parser.parse_args()
    frame = cv2.imread(str(args.image))
    mask = cv2.imread(str(args.mask), cv2.IMREAD_GRAYSCALE)
    if frame is None or mask is None or frame.shape[:2] != mask.shape:
        raise ValueError('Image and mask must exist and have matching dimensions')
    profile = json.loads(args.profile.read_text())
    result = inspect(mask, profile)
    tips = np.asarray(profile['jaw_tips_px'], int)
    center = np.asarray(result['object_center_px'])
    cv2.line(frame, tuple(tips[0]), tuple(tips[1]), (0, 220, 0), 2)
    cv2.circle(frame, tuple(np.asarray(result['jaw_gap_center_px'], int)), 5, (0, 220, 0), -1)
    cv2.circle(frame, tuple(center.astype(int)), 5, (255, 0, 255), -1)
    axis = np.asarray(result['body_axis'])*50
    cv2.line(frame, tuple((center-axis).astype(int)), tuple((center+axis).astype(int)), (255, 0, 255), 2)
    cv2.putText(frame, 'Green: jaw gap / Magenta: body middle', (10, 20), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
    cv2.putText(frame, 'Depth UNVERIFIED', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 180, 255), 1)
    if not cv2.imwrite(str(args.overlay), frame):
        raise RuntimeError('Could not save overlay')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
