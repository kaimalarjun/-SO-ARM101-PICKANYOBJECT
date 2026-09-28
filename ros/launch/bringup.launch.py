"""SO-101 controller, MoveIt and camera; mock mode by default."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    enabled = LaunchConfiguration("operator_enabled").perform(context).lower() == "true"
    if mode not in ("mock", "real"):
        raise RuntimeError("mode must be mock or real")
    if mode == "real" and not enabled:
        raise RuntimeError("real mode requires operator_enabled:=true after physical preflight")
    description = Path(get_package_share_directory("so_arm101_description"))
    repo = Path(LaunchConfiguration("repo_path").perform(context))
    moveit_dir = repo / "config" / "moveit"
    usb_port = LaunchConfiguration("usb_port").perform(context)
    model = xacro.process_file(str(description / "urdf" / "so_arm101.urdf.xacro"),
                               mappings={"ros2_control_hardware_type": "real" if mode == "real" else "mock_components",
                                         "usb_port": usb_port}).toxml()
    params = {"robot_description": model,
              "robot_description_semantic": (moveit_dir / "so101.srdf").read_text(),
              "robot_description_kinematics": yaml.safe_load((moveit_dir / "kinematics.yaml").read_text()),
              "planning_pipelines": ["ompl"],
              "default_planning_pipeline": "ompl"}
    params.update(yaml.safe_load((moveit_dir / "ompl_planning.yaml").read_text()))
    params.update(yaml.safe_load((moveit_dir / "moveit_controllers.yaml").read_text()))
    controller_launch = description / "launch" / "controllers_bringup.launch.py"
    nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(controller_launch)),
                                 launch_arguments={"hardware_type": "real" if mode == "real" else "mock_components",
                                                   "usb_port": usb_port}.items()),
        Node(package="moveit_ros_move_group", executable="move_group", parameters=[params], output="screen"),
        Node(package="kendra_robot", executable="kendra-robot-watchdog", output="screen"),
    ]
    camera_config = LaunchConfiguration("camera_config").perform(context)
    if camera_config:
        camera = yaml.safe_load(Path(camera_config).read_text())
        nodes.append(Node(package="usb_cam", executable="usb_cam_node_exe", name="overhead_camera",
                          parameters=[{"video_device": camera["device"],
                                       "image_width": camera["width"],
                                       "image_height": camera["height"],
                                       "framerate": camera["fps"],
                                       "camera_frame_id": camera["frame_id"]}]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="mock"),
        DeclareLaunchArgument("operator_enabled", default_value="false"),
        DeclareLaunchArgument("usb_port", default_value="/dev/LeRobotFollower"),
        DeclareLaunchArgument("camera_config", default_value=""),
        DeclareLaunchArgument("repo_path", description="Absolute path of public repository checkout"),
        OpaqueFunction(function=_setup),
    ])
