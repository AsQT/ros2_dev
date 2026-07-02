"""Publish a static TF base_link -> aruco_world from a YAML config.

TEMPORARY calibration — see config/aruco_world_to_base.yaml and
Reports/vision_box_height_and_base_tf_report.md for how/why these numbers
were chosen and how to replace them with measured values later.

This does NOT assume aruco_world == base_link: it publishes the actual
(currently estimated) offset between the two frames as a real TF, so any
downstream tf2 lookup (e.g. MoveTargetRl transforming /vision/wood_objects
and /vision/box_objects poses) resolves correctly instead of failing with
"frame does not exist".
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory("robot_bringup"),
        "config",
        "aruco_world_to_base.yaml",
    )

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    parent_frame = str(cfg["parent_frame"])
    child_frame = str(cfg["child_frame"])
    translation = cfg["translation"]
    rotation_rpy = cfg["rotation_rpy"]

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="aruco_world_static_tf_publisher",
        output="screen",
        arguments=[
            "--x", str(translation["x"]),
            "--y", str(translation["y"]),
            "--z", str(translation["z"]),
            "--roll", str(rotation_rpy["roll"]),
            "--pitch", str(rotation_rpy["pitch"]),
            "--yaw", str(rotation_rpy["yaw"]),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
    )

    return LaunchDescription([
        LogInfo(
            msg=(
                f"[aruco_world_static_tf] Publishing TEMPORARY static TF "
                f"{parent_frame} -> {child_frame} from {config_path} "
                f"(x={translation['x']}, y={translation['y']}, z={translation['z']})"
            )
        ),
        static_tf_node,
    ])
