#!/usr/bin/env python3
"""Trace a rectified line image into image-space strokes; never moves a robot.

Requires OpenCV contrib thinning. Output is not a calibrated robot trajectory.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np


def trace(image):
    binary = (image < 128).astype(np.uint8) * 255
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary)
    clean = np.zeros_like(binary)
    fills = []
    for i in range(1, count):
        x, y, width, height, area = stats[i]
        if area < 50:
            continue
        clean[labels == i] = 255
        if width < 40 and height < 40 and area / (width * height) > .65:
            for row in range(y, y + height, 2):
                xs = np.flatnonzero(labels[row] == i)
                if len(xs) > 1:
                    fills.append([[int(xs[0]), int(row)], [int(xs[-1]), int(row)]])
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
    return strokes + fills


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    image = cv2.imread(str(args.image), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError('Cannot read reference')
    strokes = trace(image)
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
