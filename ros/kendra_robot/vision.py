"""Fresh camera frame and private vision-service client."""
from __future__ import annotations

import base64
import json
from pathlib import Path
import time
import urllib.request
import uuid

from .calibration import project, undistort_points
from .control import Rejected


class VisionClient:
    def __init__(self, camera: dict, vision: dict) -> None:
        self.camera = camera
        self.vision = vision

    def capture(self) -> bytes:
        # usb_cam is the sole camera owner; subscribe to one fresh ROS image.
        import cv2
        import rclpy
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
        node = rclpy.create_node("kendra_robot_capture")
        frames = []
        def receive(msg: Image) -> None:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            if abs(time.time() - stamp) <= self.camera["max_frame_age_s"]:
                frames.append(msg)
        subscription = node.create_subscription(Image, "/overhead_camera/image_raw", receive, 1)
        try:
            deadline = time.monotonic() + self.camera["max_frame_age_s"]
            while not frames and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            if not frames:
                raise Rejected("camera did not return a fresh ROS frame")
            frame = CvBridge().imgmsg_to_cv2(frames[-1], desired_encoding="bgr8")
            if frame.shape[1] != self.camera["width"] or frame.shape[0] != self.camera["height"]:
                raise Rejected("camera dimensions differ from calibration")
            success, encoded = cv2.imencode(".jpg", frame)
            if not success:
                raise Rejected("camera frame encoding failed")
            return encoded.tobytes()
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()

    def look(self, request_name: str | None = None) -> dict:
        started = time.monotonic()
        frame = self.capture()
        calibration = json.loads(Path(self.camera["calibration_path"]).read_text())
        if not calibration.get("valid") or calibration.get("worst_validation_error_m", 1) > self.camera["max_table_error_m"]:
            raise Rejected("camera calibration invalid")
        if (calibration.get("camera_device") != self.camera["device"] or
            calibration.get("width") != self.camera["width"] or
            calibration.get("height") != self.camera["height"]):
            raise Rejected("camera configuration differs from calibration")
        payload = json.dumps({"image_b64": base64.b64encode(frame).decode("ascii"),
                              "query": request_name,
                              "placement_zone_roi_px": self.camera.get("placement_zone_roi_px")}).encode()
        req = urllib.request.Request(self.vision["url"].rstrip("/") + "/infer", payload,
                                     {"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=self.vision["timeout_s"]) as response:
            found = json.load(response)
        if time.monotonic() - started > self.vision["max_box_age_s"]:
            raise Rejected("vision response is stale")
        objects = []
        for obj in found.get("objects", [])[:self.vision["max_candidates"]]:
            if obj.get("score", 0) < self.vision["min_grounding_score"]:
                continue
            pixel = undistort_points([obj["pixel_center"]], calibration["camera_matrix"],
                                     calibration["distortion"])[0]
            x, y = project(calibration["homography"], *pixel)
            objects.append({"label": obj["label"], "score": obj["score"], "box": obj["box"],
                            "pixel_center": obj["pixel_center"],
                            "point": {"x": x, "y": y, "z": self.camera["grasp_z_m"]}})
        unique = len(objects) == 1 if request_name else False
        return {"id": uuid.uuid4().hex, "monotonic_time": time.monotonic(),
                "objects": objects, "unique": unique, "calibrated": True,
                "approach_clear": unique and found.get("isolated", False),
                "zone_clear": found.get("zone_clear", False),
                "point": objects[0]["point"] if unique else None}
