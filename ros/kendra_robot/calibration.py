"""Calibrate a planar camera-to-base mapping from measured correspondences.

The output stays on the host. It is intentionally never stored in Git.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time


def calibrate_intrinsics(images: list[str], columns: int, rows: int,
                         square_m: float) -> tuple[list, list, float]:
    import cv2
    import numpy as np
    pattern = np.zeros((columns * rows, 3), np.float32)
    pattern[:, :2] = np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_m
    object_points, image_points = [], []
    size = None
    for file in images:
        frame = cv2.imread(file)
        if frame is None:
            raise ValueError(f"cannot read checkerboard image: {file}")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        size = gray.shape[::-1]
        found, corners = cv2.findChessboardCorners(gray, (columns, rows))
        if found:
            object_points.append(pattern)
            image_points.append(cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                                 (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)))
    if len(object_points) < 10:
        raise ValueError("at least ten checkerboard views required")
    error, matrix, distortion, _, _ = cv2.calibrateCamera(object_points, image_points, size, None, None)
    if error > 1.0:
        raise ValueError(f"lens calibration RMS {error:.2f} px is too high")
    return matrix.tolist(), distortion.tolist(), error


def marker_centers(image_path: str, ids: list[int]) -> list[list[float]]:
    import cv2
    frame = cv2.imread(image_path)
    if frame is None:
        raise ValueError("marker image unavailable")
    aruco = cv2.aruco
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    corners, found_ids, _ = aruco.detectMarkers(frame, dictionary)
    if found_ids is None:
        raise ValueError("no ArUco markers detected")
    centers = {int(identifier): corner[0].mean(axis=0).tolist()
               for identifier, corner in zip(found_ids.flatten(), corners)}
    if len(ids) < 4 or any(identifier not in centers for identifier in ids):
        raise ValueError("all four configured markers must be visible")
    return [centers[identifier] for identifier in ids]


def undistort_points(pixels: list[list[float]], matrix: list, distortion: list) -> list[list[float]]:
    import cv2
    import numpy as np
    points = np.asarray(pixels, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(points, np.asarray(matrix), np.asarray(distortion),
                               P=np.asarray(matrix)).reshape(-1, 2).tolist()


def fit_homography(pixels: list[list[float]], table_xy: list[list[float]]) -> list[list[float]]:
    import cv2
    import numpy as np
    if len(pixels) < 4 or len(pixels) != len(table_xy):
        raise ValueError("at least four matched non-collinear points required")
    h, mask = cv2.findHomography(np.asarray(pixels, dtype=np.float64),
                                  np.asarray(table_xy, dtype=np.float64), method=0)
    if h is None or mask is None:
        raise ValueError("homography fit failed")
    return h.tolist()


def project(h: list[list[float]], u: float, v: float) -> tuple[float, float]:
    denominator = h[2][0] * u + h[2][1] * v + h[2][2]
    if abs(denominator) < 1e-9:
        raise ValueError("projection singularity")
    return ((h[0][0] * u + h[0][1] * v + h[0][2]) / denominator,
            (h[1][0] * u + h[1][1] * v + h[1][2]) / denominator)


def validate(h: list[list[float]], pixels: list[list[float]],
             measured_xy: list[list[float]], max_error_m: float = 0.010) -> float:
    import math
    if not pixels or len(pixels) != len(measured_xy):
        raise ValueError("independent validation points required")
    worst = max(math.dist(project(h, *p), tuple(q)) for p, q in zip(pixels, measured_xy))
    if worst > max_error_m:
        raise ValueError(f"table error {worst:.4f} m exceeds {max_error_m:.4f} m")
    return worst


def main() -> None:
    parser = argparse.ArgumentParser(description="Fit and validate camera-to-base table mapping")
    parser.add_argument("measurements", type=Path, help="JSON with fit_pixels, fit_xy_m, check_pixels, check_xy_m")
    parser.add_argument("output", type=Path, help="ignored host-local calibration JSON")
    args = parser.parse_args()
    raw = json.loads(args.measurements.read_text())
    matrix, distortion, rms = calibrate_intrinsics(raw["checkerboard_images"],
                                                    raw["checkerboard_columns"],
                                                    raw["checkerboard_rows"],
                                                    raw["checkerboard_square_m"])
    fit_pixels = marker_centers(raw["marker_image"], raw["marker_ids"])
    fit_pixels = undistort_points(fit_pixels, matrix, distortion)
    check_pixels = undistort_points(raw["check_pixels"], matrix, distortion)
    h = fit_homography(fit_pixels, raw["marker_xy_m"])
    worst = validate(h, check_pixels, raw["check_xy_m"])
    result = {"homography": h, "worst_validation_error_m": worst,
              "camera_matrix": matrix, "distortion": distortion, "lens_rms_px": rms,
              "camera_device": raw["camera_device"], "width": raw["width"],
              "height": raw["height"], "created_unix": time.time(), "valid": True}
    args.output.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2))
    args.output.chmod(0o600)
    print(f"Validated independent worst error: {worst:.4f} m")


if __name__ == "__main__":
    main()
