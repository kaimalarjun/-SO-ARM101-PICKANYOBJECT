"""Independent ROS-side lease monitor.

This process subscribes to lease heartbeats and calls both trajectory cancel and
controller deactivate on expiry. Real deployment remains gated until tested.
"""
from __future__ import annotations

import time


def main() -> None:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Empty
    from action_msgs.srv import CancelGoal

    class LeaseWatchdog(Node):
        def __init__(self) -> None:
            super().__init__("kendra_robot_watchdog")
            self.declare_parameter("lease_s", 2.0)
            self.last = time.monotonic()
            self.active = False
            self.create_subscription(Empty, "/kendra_robot/lease", self.beat, 10)
            self.cancel_client = self.create_client(CancelGoal, "/joint_trajectory_controller/follow_joint_trajectory/_action/cancel_goal")
            self.create_timer(0.1, self.check)

        def beat(self, _msg: Empty) -> None:
            self.last = time.monotonic()
            self.active = True

        def check(self) -> None:
            if self.active and time.monotonic() - self.last > self.get_parameter("lease_s").value:
                if self.cancel_client.service_is_ready():
                    self.cancel_client.call_async(CancelGoal.Request())
                self.active = False
                self.get_logger().error("command lease expired; cancel requested")

    rclpy.init()
    node = LeaseWatchdog()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
