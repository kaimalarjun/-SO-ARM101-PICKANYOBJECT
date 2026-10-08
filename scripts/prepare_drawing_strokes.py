#!/usr/bin/env python3
"""Trace a rectified line image into image-space strokes; never moves a robot.

Requires OpenCV contrib thinning. Output is not a calibrated robot trajectory.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def join_touching(strokes):
    """Join exact shared endpoints without adding a gap-spanning mark."""
    paths = [list(path) for path in strokes if len(path) >= 2]
    while True:
        best = None
        for i, left in enumerate(paths):
            if np.allclose(left[0], left[-1], atol=1e-6, rtol=0):
                continue
            for j in range(i+1, len(paths)):
                right = paths[j]
                for flip_left in (False, True):
                    a = left[::-1] if flip_left else left
                    for flip_right in (False, True):
                        b = right[::-1] if flip_right else right
                        if not np.allclose(a[-1], b[0], atol=1e-6, rtol=0):
                            continue
                        incoming = np.asarray(a[-1])-a[-2]
                        outgoing = np.asarray(b[1])-b[0]
                        norm = np.linalg.norm(incoming)*np.linalg.norm(outgoing)
                        score = float(incoming @ outgoing)/norm if norm else -1
                        if best is None or score > best[0]:
                            best = (score, i, j, a+b[1:])
        if best is None:
            return paths
        _, i, j, merged = best
        paths[i] = merged
        paths.pop(j)


def trace(image, min_area=50):
    binary = (image < 128).astype(np.uint8) * 255
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    clean = np.zeros_like(binary)
    fills = []
    for i in range(1, count):
        x, y, width, height, area = stats[i]
        if area < min_area:
            continue
        clean[labels == i] = 255
        if width < 40 and height < 40 and area / (width * height) > .65:
            hatch = []
            for row in range(y, y + height, 2):
                xs = np.flatnonzero(labels[row] == i)
                if len(xs) > 1:
                    line = [[int(xs[0]), int(row)], [int(xs[-1]), int(row)]]
                    if hatch:
                        if np.linalg.norm(np.asarray(hatch[-1])-line[-1]) < np.linalg.norm(np.asarray(hatch[-1])-line[0]):
                            line.reverse()
                        bridge = np.linspace(hatch[-1], line[0],
                                             int(np.linalg.norm(np.asarray(hatch[-1])-line[0]))+2).round().astype(int)
                        if not all(labels[v, u] == i for u, v in bridge):
                            fills.append(hatch)
                            hatch = []
                    hatch.extend(line)
            if hatch:
                fills.append(hatch)
    binary = clean
    skeleton = cv2.ximgproc.thinning(binary)
    pixels = {tuple(p) for p in np.argwhere(skeleton > 0)}
    def neighbors(p):
        y, x = p
        result = []
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                q = (y + dy, x + dx)
                if q == p or q not in pixels:
                    continue
                if dy and dx and ((y + dy, x) in pixels or (y, x + dx) in pixels):
                    continue
                result.append(q)
        return result
    graph = {p: neighbors(p) for p in pixels}
    visited, strokes = set(), []
    starts = sorted(p for p in pixels if len(graph[p]) != 2) + sorted(pixels)
    for start in starts:
        for following in graph[start]:
            edge = frozenset((start, following))
            if edge in visited:
                continue
            path = [start]
            previous, current = start, following
            visited.add(edge)
            while True:
                path.append(current)
                if current == start or len(graph[current]) != 2:
                    break
                nxt = next(p for p in graph[current] if p != previous)
                edge = frozenset((current, nxt))
                if edge in visited:
                    break
                visited.add(edge)
                previous, current = current, nxt
            if len(path) >= 2:
                points = np.array([(x, y) for y, x in path], np.float32)
                simplified = cv2.approxPolyDP(points, .6, False).reshape(-1, 2)
                strokes.append(simplified.tolist())
    return join_touching(strokes) + fills


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--adaptive', action='store_true',
                        help='Normalize photographed references with uneven lighting')
    parser.add_argument('--border-fraction', type=float, default=0,
                        help='Exclude a reviewed photograph border before tracing')
    parser.add_argument('--min-area', type=int, default=50,
                        help='Minimum connected ink component area in pixels')
    args = parser.parse_args()
    image = cv2.imread(str(args.image), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError('Cannot read reference')
    if not 0 <= args.border_fraction < .2 or args.min_area < 1:
        parser.error('Invalid border fraction or component area')
    working = image.copy()
    if args.adaptive:
        working = cv2.adaptiveThreshold(working, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, 31, 8)
    if args.border_fraction:
        h, w = working.shape
        y, x = int(h*args.border_fraction), int(w*args.border_fraction)
        if y:
            working[:y] = 255
            working[-y:] = 255
        if x:
            working[:, :x] = 255
            working[:, -x:] = 255
    strokes = trace(working, args.min_area)
    # Reference-image label; robot mapping and boundary validation are separate.
    letters = [
        [[0,20],[0,0],[10,0],[10,10],[0,10],[10,20]],
        [[0,0],[10,0],[10,20],[0,20],[0,0]],
        [[0,20],[0,0],[9,0],[11,5],[9,10],[0,10],[9,10],[11,15],[9,20],[0,20]],
        [[0,0],[10,0],[10,20],[0,20],[0,0]],
        [[0,0],[12,0]], [[6,0],[6,20]],
    ]
    origins = [140,157,174,191,208,208]
    strokes.extend([[[x + origin, y + image.shape[0] - 35] for x,y in letter]
                    for origin,letter in zip(origins,letters)])
    args.output.mkdir(parents=True, exist_ok=True)
    preview = np.full_like(image, 255)
    for stroke in strokes:
        cv2.polylines(preview, [np.int32(stroke)], False, 0, 1, cv2.LINE_AA)
    cv2.imwrite(str(args.output/'stroke-preview.png'), preview)
    (args.output/'image-strokes.json').write_text(json.dumps({
        'image_size': [image.shape[1],image.shape[0]], 'object_english': 'Robot',
        'strokes_px': strokes, 'robot_mapping_validated': False,
        'note': 'Image-space plan only; calibrate and validate before motion.'}, indent=2))
    print('Image-space strokes:', len(strokes))


if __name__ == '__main__':
    main()
