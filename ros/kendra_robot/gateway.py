"""Loopback-only HTTP API for the Kendra OpenClaw plugin."""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import threading
from typing import Any


from .control import MockBackend, Point, Rejected, RobotController
from .vision import VisionClient


class Gateway:
    def __init__(self, robot: dict, camera: dict | None = None,
                 vision: dict | None = None, backend: Any = None) -> None:
        self.robot = RobotController(robot, backend or MockBackend())
        self.vision = VisionClient(camera, vision) if camera and vision else None
        self._carried_label: str | None = None
        self._lease_pub = None
        try:
            import rclpy
            from std_msgs.msg import Empty
            if not rclpy.ok():
                rclpy.init(args=None)
            self._lease_node = rclpy.create_node("kendra_gateway_lease")
            self._lease_pub = self._lease_node.create_publisher(Empty, "/kendra_robot/lease", 10)
            self._lease_msg = Empty()
        except ImportError:
            if robot["mode"] != "mock":
                raise Rejected("ROS lease publisher unavailable")
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _watchdog(self) -> None:
        import time
        while True:
            self.robot.watchdog_tick()
            if (self._lease_pub and self.robot._busy and not self.robot.cancel.is_set()
                    and time.monotonic() <= self.robot._lease_deadline):
                self._lease_pub.publish(self._lease_msg)
            time.sleep(0.1)

    def dispatch(self, method: str, data: dict[str, Any]) -> dict[str, Any]:
        if method == "status":
            return self.robot.status()
        if method == "stop":
            return self.robot.stop()
        if method == "heartbeat":
            self.robot.heartbeat()
            return {"ok": True}
        if method == "look":
            if not self.vision:
                raise Rejected("camera and vision service not configured")
            observation = self.vision.look(data.get("query"))
            self.robot.set_observation(observation)
            return {k: v for k, v in observation.items() if k != "monotonic_time"}
        if method == "home":
            return self.robot.home()
        if method == "open_gripper":
            return self.robot.gripper(True)
        if method == "close_gripper":
            return self.robot.gripper(False)
        if method == "move_to":
            return self.robot.move_to(Point.parse(data))
        if method == "pick":
            before = self.robot._observation
            result = self.robot.pick(Point.parse(data), str(data.get("observation_id", "")))
            if before and before.get("objects") and self.vision:
                label = before["objects"][0]["label"]
                try:
                    after = self.vision.look(label)
                    lift_roi = self.vision.camera.get("lift_verification_roi_px")
                    if lift_roi and len(after["objects"]) == 1:
                        old = before["objects"][0]["pixel_center"]
                        new = after["objects"][0]["pixel_center"]
                        in_lift = self._in_roi(new, lift_roi)
                        at_old = abs(new[0] - old[0]) < 20 and abs(new[1] - old[1]) < 20
                        if in_lift and not at_old:
                            result["task_success"] = "verified"
                            self._carried_label = label
                        elif at_old:
                            result["task_success"] = "failed"
                except (Rejected, OSError, ValueError):
                    pass
            return result
        if method == "place":
            result = self.robot.place(Point.parse(data))
            if self._carried_label and self.vision:
                try:
                    after = self.vision.look(self._carried_label)
                    zone = self.vision.camera.get("placement_zone_roi_px")
                    if zone and len(after["objects"]) == 1 and self._in_roi(after["objects"][0]["pixel_center"], zone):
                        result["task_success"] = "verified"
                        self._carried_label = None
                except (Rejected, OSError, ValueError):
                    pass
            return result
        raise Rejected("unknown robot method")

    @staticmethod
    def _in_roi(point: list[float], roi: list[int]) -> bool:
        return roi[0] <= point[0] <= roi[2] and roi[1] <= point[1] <= roi[3]


def handler_for(gateway: Gateway) -> type[BaseHTTPRequestHandler]:
    class Handler(BaseHTTPRequestHandler):
        def _reply(self, code: int, data: dict) -> None:
            body = json.dumps(data, allow_nan=False).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self) -> None:
            if self.path == "/health":
                self._reply(200, gateway.robot.status())
            else:
                self._reply(404, {"error": "not found"})

        def do_POST(self) -> None:
            if not self.path.startswith("/robot/"):
                self._reply(404, {"error": "not found"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length > 16384 or length < 0:
                    raise Rejected("request too large")
                data = json.loads(self.rfile.read(length) or b"{}")
                if not isinstance(data, dict):
                    raise Rejected("JSON object required")
                result = gateway.dispatch(self.path.removeprefix("/robot/"), data)
                self._reply(200, result)
            except (Rejected, ValueError, KeyError, OSError) as exc:
                self._reply(422, {"error": str(exc), "motion_success": "failed",
                                  "task_success": "uncertain"})

        def log_message(self, format: str, *args: Any) -> None:
            # Do not log image content or target coordinates to a shared journal.
            pass

    return Handler


def main() -> None:
    import yaml
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", type=Path, required=True)
    parser.add_argument("--camera", type=Path)
    parser.add_argument("--vision", type=Path)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--repo-path", type=Path)
    args = parser.parse_args()
    if bool(args.camera) != bool(args.vision):
        parser.error("camera and vision configs must be supplied together")
    load = lambda p: yaml.safe_load(p.read_text()) if p else None
    config = load(args.robot)
    backend = None
    if config["mode"] == "real":
        if not args.repo_path:
            parser.error("--repo-path required for real MoveIt backend")
        from .ros_backend import RosBackend
        backend = RosBackend(config, args.repo_path)
    server = ThreadingHTTPServer(("127.0.0.1", args.port),
                                 handler_for(Gateway(config, load(args.camera), load(args.vision), backend)))
    server.serve_forever(poll_interval=0.1)


if __name__ == "__main__":
    main()
