from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("robot_vision_pipeline")

    default_param_file = PathJoinSubstitution([
        pkg_share,
        "config",
        "yolo_detect.yaml",
    ])

    param_file_arg = DeclareLaunchArgument(
        "param_file",
        default_value=default_param_file,
        description="Path to YOLO detection parameter file",
    )

    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Override YOLO model path. Empty means use YAML config.",
    )

    image_topic_arg = DeclareLaunchArgument(
        "image_topic",
        default_value="",
        description="Override input image topic. Empty means use YAML config.",
    )

    node = Node(
        package="robot_vision_pipeline",
        executable="yolo_detect_node",
        name="yolo_detect_node",
        output="screen",
        # Ép node chạy bằng Python trong venv ros_env
        prefix="/home/minhquang/venvs/ros_yolo/bin/python3",
        parameters=[
            LaunchConfiguration("param_file"),
            {
                "model_path_override": LaunchConfiguration("model_path"),
                "image_topic_override": LaunchConfiguration("image_topic"),
            },
        ],
    )

    return LaunchDescription([
        param_file_arg,
        model_path_arg,
        image_topic_arg,
        node,
    ])