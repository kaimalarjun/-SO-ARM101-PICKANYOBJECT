"""Pure-Python safety core shared by the gateway and tests.

Only a backend with explicit operator enable may move physical hardware.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Any, Protocol


class Rejected(RuntimeError):
    """A command failed a safety or readiness gate."""


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    z: float

    @classmethod
    def parse(cls, data: dict[str, Any]) -> "Point":
        try:
            value = cls(*(float(data[k]) for k in ("x", "y", "z")))
        except (KeyError, TypeError, ValueError) as exc:
            raise Rejected("x, y and z must be numeric metres") from exc
        if not all(math.isfinite(v) for v in (value.x, value.y, value.z)):
            raise Rejected("coordinates must be finite")
        return value


class Backend(Protocol):
    def ready(self) -> bool: ...
    def move(self, point: Point, cancel: threading.Event, deadline: float) -> None: ...
    def gripper(self, opened: bool, cancel: threading.Event, deadline: float) -> None: ...
    def stop(self) -> None: ...


class MockBackend:
    """Deterministic backend; never opens a serial device."""

    def __init__(self) -> None:
        self.position = Point(0.15, 0.0, 0.15)
        self.opened = True
        self.stopped = False
        self.commands: list[tuple[str, object]] = []

    def ready(self) -> bool:
        return not self.stopped

    def move(self, point: Point, cancel: threading.Event, deadline: float) -> None:
        if cancel.is_set() or time.monotonic() > deadline:
            raise Rejected("command cancelled or timed out")
        self.position = point
        self.commands.append(("move", point))

    def gripper(self, opened: bool, cancel: threading.Event, deadline: float) -> None:
        if cancel.is_set() or time.monotonic() > deadline:
            raise Rejected("command cancelled or timed out")
        self.opened = opened
        self.commands.append(("gripper", opened))

    def stop(self) -> None:
        self.stopped = True
        self.commands.append(("stop", None))


class RobotController:
    def __init__(self, config: dict[str, Any], backend: Backend) -> None:
        self.config = config
        self.backend = backend
        self.cancel = threading.Event()
        self._lock = threading.Lock()
        self._lease_deadline = 0.0
        self._busy = False
        self._observation: dict[str, Any] | None = None

    def heartbeat(self) -> None:
        self._lease_deadline = time.monotonic() + float(self.config["command_lease_s"])

    def watchdog_tick(self) -> bool:
        if self._busy and time.monotonic() > self._lease_deadline:
            self.stop()
            return True
        return False

    def status(self) -> dict[str, Any]:
        return {"mode": self.config["mode"], "ready": self.backend.ready(),
                "operator_enabled": bool(self.config.get("operator_enabled")),
                "busy": self._busy, "stopped": self.cancel.is_set()}

    def stop(self) -> dict[str, Any]:
        # Never waits for the action lock; this is the queue-bypass path.
        self.cancel.set()
        self.backend.stop()
        return {"motion_success": "uncertain", "task_success": "uncertain", "stopped": True}

    def set_observation(self, observation: dict[str, Any]) -> None:
        self._observation = observation

    def _check(self, point: Point) -> None:
        if self.cancel.is_set():
            raise Rejected("stop latched; restart and re-enable after inspection")
        if not self.backend.ready():
            raise Rejected("joint feedback or controller unavailable")
        if self.config["mode"] != "mock" and not self.config.get("operator_enabled"):
            raise Rejected("physical motion requires operator enable")
        for axis in ("x", "y", "z"):
            low, high = self.config["workspace_m"][axis]
            if not low <= getattr(point, axis) <= high:
                raise Rejected(f"{axis} outside measured workspace")

    def _run(self, action: Any) -> None:
        if not self._lock.acquire(blocking=False):
            raise Rejected("robot is busy")
        self._busy = True
        self.heartbeat()
        try:
            deadline = time.monotonic() + float(self.config["max_command_s"])
            action(deadline)
            if self.cancel.is_set() or time.monotonic() > deadline:
                raise Rejected("command cancelled or timed out")
        finally:
            self._busy = False
            self._lock.release()

    def move_to(self, point: Point) -> dict[str, str]:
        self._check(point)
        self._run(lambda deadline: self.backend.move(point, self.cancel, deadline))
        return {"motion_success": "verified", "task_success": "uncertain"}

    def gripper(self, opened: bool) -> dict[str, str]:
        self._check(Point.parse({"x": 0.15, "y": 0.0, "z": 0.10}))
        self._run(lambda deadline: self.backend.gripper(opened, self.cancel, deadline))
        return {"motion_success": "verified", "task_success": "uncertain"}

    def home(self) -> dict[str, str]:
        return self.move_to(Point.parse(self.config["home_m"]))

    def pick(self, point: Point, observation_id: str) -> dict[str, str]:
        obs = self._observation
        if not obs or obs.get("id") != observation_id:
            raise Rejected("fresh observation ID required")
        if time.monotonic() - obs["monotonic_time"] > self.config["max_observation_age_s"]:
            raise Rejected("observation is stale")
        if not obs.get("unique") or not obs.get("calibrated") or not obs.get("approach_clear"):
            raise Rejected("ambiguous object, invalid calibration, or occupied approach")
        measured = Point.parse(obs["point"])
        if math.dist((point.x, point.y, point.z), (measured.x, measured.y, measured.z)) > 0.01:
            raise Rejected("target differs from observed grasp point")
        self._check(point)
        above = Point(point.x, point.y, point.z + self.config["approach_clearance_m"])
        self._check(above)

        def sequence(deadline: float) -> None:
            self.backend.gripper(True, self.cancel, deadline)
            self.backend.move(above, self.cancel, deadline)
            self.backend.move(point, self.cancel, deadline)
            self.backend.gripper(False, self.cancel, deadline)
            self.backend.move(above, self.cancel, deadline)

        self._run(sequence)
        self._observation = None  # single use
        return {"motion_success": "verified", "task_success": "uncertain"}

    def place(self, point: Point) -> dict[str, str]:
        obs = self._observation
        if (not obs or time.monotonic() - obs["monotonic_time"] > self.config["max_observation_age_s"]
                or not obs.get("zone_clear")):
            raise Rejected("placement zone is occupied or unobserved")
        zones = [Point.parse(zone) for zone in self.config.get("placement_zones", {}).values()]
        if not any(math.dist((point.x, point.y, point.z),
                             (zone.x, zone.y, zone.z)) <= 0.01 for zone in zones):
            raise Rejected("placement target is not a predefined zone")
        self._check(point)
        above = Point(point.x, point.y, point.z + self.config["approach_clearance_m"])
        self._check(above)

        def sequence(deadline: float) -> None:
            self.backend.move(above, self.cancel, deadline)
            self.backend.move(point, self.cancel, deadline)
            self.backend.gripper(True, self.cancel, deadline)
            self.backend.move(above, self.cancel, deadline)

        self._run(sequence)
        return {"motion_success": "verified", "task_success": "uncertain"}
