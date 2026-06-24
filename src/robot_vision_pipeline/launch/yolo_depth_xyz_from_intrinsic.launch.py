from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_vision_pipeline")

    return LaunchDescription(
        [
            Node(
                package="robot_vision_pipeline",
                executable="yolo_depth_xyz_from_intrinsic",
                name="yolo_depth_xyz_from_intrinsic",
                output="screen",
                parameters=[
                    {
                        "model_path": PathJoinSubstitution([
                            pkg_share,
                            "models",
                            "yolov8.pt",
                        ]),
                        "intrinsic_yaml_path": PathJoinSubstitution([
                            pkg_share,
                            "config",
                            "Intrinsic.yaml",
                        ]),
                    }
                ],
            )
        ]
    )
