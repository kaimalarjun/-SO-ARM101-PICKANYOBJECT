# Overhead table calibration

Use a fixed camera and measured robot-base/table coordinates. All measured files stay outside Git (for example, `/var/lib/kendra-robot`).

1. Fix four ArUco markers at known table positions outside the pick envelope. Measure each marker centre in `base_link` metres. Photograph them with the exact camera mode used at runtime.
2. Calibrate lens intrinsics with OpenCV checkerboard images from the mounted camera and save the matrix/distortion coefficients in the restricted host directory. Undistort points before fitting the planar homography.
3. Hold out at least three independent measured points throughout the pick zone. A measurements JSON has keys `checkerboard_images` (at least ten paths), `checkerboard_columns`, `checkerboard_rows`, `checkerboard_square_m`, `marker_image`, `marker_ids` (four IDs in the 4x4_50 dictionary), `marker_xy_m` (base-frame centres in the same order), `check_pixels`, `check_xy_m`, `camera_device`, `width`, `height`.
4. Run `kendra-robot-calibrate measurements.json /var/lib/kendra-robot/camera-calibration.json`. The CLI rejects a worst held-out error greater than 10 mm and saves a file with mode 0600. Recheck after any camera, table, base or resolution change and after reboot.
5. Measure table height, gripper contact plane and safe approach height separately. A planar homography gives x/y only; object height is not inferred from a single overhead frame.

Treat missing, stale, mismatched or failed calibration as a hard motion gate. Do not use `config/camera.example.yaml` values as measurements.
