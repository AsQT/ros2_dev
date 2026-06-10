"""Launch the mock environment node for simulation testing.

Publishes synthetic target/object data so the DRL inference node can run
without hardware or a RealSense camera.

Usage:
  ros2 launch robot_drl mock_environment.launch.py

  # Simulate "target" class (YOLO model must be retrained to detect it)
  ros2 launch robot_drl mock_environment.launch.py \\
      target_class_name:=target

  # Custom target position
  ros2 launch robot_drl mock_environment.launch.py \\
      target_x:=0.2 target_y:=-0.3 target_z:=0.4 \\
      target_class_name:=box

Parameters:
  publish_rate_hz     — publish rate in Hz
  target_class_name  — "box" or "target" (must match trained YOLO class)
  target_x/y/z       — world-frame position of the object
  frame_id           — TF frame ID (default world)
  distance_m         — fake depth from camera (m)
  bbox_width_px      — fake bbox width (px)
  bbox_height_px     — fake bbox height (px)
  confidence         — fake YOLO confidence (0-1)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    args = [
        DeclareLaunchArgument(
            "publish_rate_hz", default_value="10.0",
            description="Publish rate in Hz"
        ),
        DeclareLaunchArgument(
            "target_class_name", default_value="box",
            description="YOLO class name: 'box' or 'target' (must match trained model)"
        ),
        DeclareLaunchArgument(
            "target_x", default_value="0.0",
            description="Target X position in world frame (m)"
        ),
        DeclareLaunchArgument(
            "target_y", default_value="-0.3",
            description="Target Y position in world frame (m)"
        ),
        DeclareLaunchArgument(
            "target_z", default_value="0.4",
            description="Target Z position in world frame (m)"
        ),
        DeclareLaunchArgument(
            "frame_id", default_value="world",
            description="TF frame ID for published poses"
        ),
        DeclareLaunchArgument(
            "distance_m", default_value="0.5",
            description="Fake distance in BoxDetection (m)"
        ),
        DeclareLaunchArgument(
            "bbox_width_px", default_value="200",
            description="Fake bbox width in pixels"
        ),
        DeclareLaunchArgument(
            "bbox_height_px", default_value="200",
            description="Fake bbox height in pixels"
        ),
        DeclareLaunchArgument(
            "confidence", default_value="0.95",
            description="Fake YOLO confidence score"
        ),
    ]

    node = Node(
        package="robot_drl",
        executable="mock_environment_node",
        name="mock_environment_node",
        output="screen",
        parameters=[{
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
            "target_class_name": LaunchConfiguration("target_class_name"),
            "target_x": LaunchConfiguration("target_x"),
            "target_y": LaunchConfiguration("target_y"),
            "target_z": LaunchConfiguration("target_z"),
            "frame_id": LaunchConfiguration("frame_id"),
            "distance_m": LaunchConfiguration("distance_m"),
            "bbox_width_px": LaunchConfiguration("bbox_width_px"),
            "bbox_height_px": LaunchConfiguration("bbox_height_px"),
            "confidence": LaunchConfiguration("confidence"),
        }],
    )

    return LaunchDescription(args + [node])
