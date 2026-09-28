"""MoveIt-based physical backend. Commission in mock mode before selecting real."""
from __future__ import annotations

import os
from pathlib import Path
import threading
import time

from .control import Point, Rejected


class RosBackend:
    def __init__(self, config: dict, repo_path: Path) -> None:
        if config["mode"] != "real" or not config.get("operator_enabled"):
            raise Rejected("real ROS backend requires explicit operator enable")
        if not os.path.exists(config["follower_port"]):
            raise Rejected("follower stable port absent")
        import rclpy
        from sensor_msgs.msg import JointState
        from moveit.planning import MoveItPy
        import xacro
        import yaml
        from ament_index_python.packages import get_package_share_directory

        self.config = config
        self.feedback_time = 0.0
        self.joint_positions: dict[str, float] = {}
        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("kendra_robot_feedback")
        def feedback(msg: JointState) -> None:
            required = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint",
                        "wrist_flex_joint", "wrist_roll_joint", "gripper_joint"}
            if required.issubset(set(msg.name)):
                self.feedback_time = time.monotonic()
                self.joint_positions = dict(zip(msg.name, msg.position))
        self._node.create_subscription(JointState, "/joint_states", feedback, 10)
        threading.Thread(target=rclpy.spin, args=(self._node,), daemon=True).start()

        description = Path(get_package_share_directory("so_arm101_description"))
        moveit_dir = repo_path / "config" / "moveit"
        robot_description = xacro.process_file(str(description / "urdf" / "so_arm101.urdf.xacro"),
                                                 mappings={"ros2_control_hardware_type": "real",
                                                           "usb_port": config["follower_port"]}).toxml()
        params = {"robot_description": robot_description,
                  "robot_description_semantic": (moveit_dir / "so101.srdf").read_text(),
                  "robot_description_kinematics": yaml.safe_load((moveit_dir / "kinematics.yaml").read_text()),
                  "planning_scene_monitor_options": {"name": "planning_scene_monitor",
                                                     "robot_description": "robot_description",
                                                     "joint_state_topic": "/joint_states",
                                                     "wait_for_initial_state_timeout": 10.0},
                  "plan_request_params": {"planning_attempts": 1,
                                          "planning_pipeline": "ompl",
                                          "max_velocity_scaling_factor": 0.1,
                                          "max_acceleration_scaling_factor": 0.1}}
        params.update(yaml.safe_load((moveit_dir / "ompl_planning.yaml").read_text()))
        params.update(yaml.safe_load((moveit_dir / "moveit_controllers.yaml").read_text()))
        self.robot = MoveItPy(node_name="kendra_robot_moveit", config_dict=params)
        self.arm = self.robot.get_planning_component("manipulator")
        self.gripper_component = self.robot.get_planning_component("gripper")
        self._install_table_collision()

    def _install_table_collision(self) -> None:
        from geometry_msgs.msg import Pose
        from moveit_msgs.msg import CollisionObject
        from shape_msgs.msg import SolidPrimitive
        bounds = self.config["workspace_m"]
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [bounds["x"][1] - bounds["x"][0] + 0.2,
                          bounds["y"][1] - bounds["y"][0] + 0.2, 0.02]
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x = sum(bounds["x"]) / 2
        pose.position.y = sum(bounds["y"]) / 2
        pose.position.z = self.config["table_z_m"] - 0.01
        obj = CollisionObject()
        obj.id = "measured_table"
        obj.header.frame_id = self.config["base_frame"]
        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        monitor = self.robot.get_planning_scene_monitor()
        with monitor.read_write() as scene:
            scene.apply_collision_object(obj)
            scene.current_state.update()

    def ready(self) -> bool:
        return (os.path.exists(self.config["follower_port"])
                and time.monotonic() - self.feedback_time <= 1 / self.config["min_joint_feedback_hz"] * 2)

    def _execute(self, component, cancel: threading.Event, deadline: float) -> None:
        if not self.ready() or cancel.is_set() or time.monotonic() > deadline:
            raise Rejected("feedback stale, cancelled or timed out")
        plan = component.plan()
        if not plan:
            raise Rejected("MoveIt could not find a collision-free trajectory")
        if cancel.is_set() or time.monotonic() > deadline:
            raise Rejected("cancelled before trajectory execution")
        success = self.robot.execute(plan.trajectory, controllers=[])
        if not success or cancel.is_set() or time.monotonic() > deadline:
            raise Rejected("trajectory execution failed or timed out")

    def move(self, point: Point, cancel: threading.Event, deadline: float) -> None:
        from geometry_msgs.msg import PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = self.config["base_frame"]
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point.x, point.y, point.z
        orientation = self.config["tool_orientation_xyzw"]
        pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = orientation
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(pose_stamped_msg=pose, pose_link=self.config["tool_frame"])
        self._execute(self.arm, cancel, deadline)

    def gripper(self, opened: bool, cancel: threading.Event, deadline: float) -> None:
        from moveit.core.robot_state import RobotState
        state = RobotState(self.robot.get_robot_model())
        state.joint_positions = {"gripper_joint": self.config["gripper_open_rad" if opened else "gripper_closed_rad"]}
        self.gripper_component.set_start_state_to_current_state()
        self.gripper_component.set_goal_state(robot_state=state)
        self._execute(self.gripper_component, cancel, deadline)

    def guarded_joint_step(self, desired: dict[str, float], cancel: threading.Event,
                           deadline: float) -> None:
        """Leader-guided demonstration step through MoveIt, never a raw servo write."""
        from moveit.core.robot_state import RobotState
        joints = ("shoulder_pan_joint", "shoulder_lift_joint", "elbow_flex_joint",
                  "wrist_flex_joint", "wrist_roll_joint", "gripper_joint")
        if set(desired) != set(joints) or not self.ready():
            raise Rejected("leader joint mapping or feedback invalid")
        import math
        if any(not math.isfinite(desired[name]) or
               abs(desired[name] - self.joint_positions[name]) > self.config["max_joint_step_rad"]
               for name in joints):
            raise Rejected("leader command exceeds per-step joint limit")
        state = RobotState(self.robot.get_robot_model())
        state.joint_positions = {name: desired[name] for name in joints}
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(robot_state=state)
        self._execute(self.arm, cancel, deadline)
        self.gripper_component.set_start_state_to_current_state()
        self.gripper_component.set_goal_state(robot_state=state)
        self._execute(self.gripper_component, cancel, deadline)

    def stop(self) -> None:
        self.robot.get_trajectory_execution_manager().stop_execution()
