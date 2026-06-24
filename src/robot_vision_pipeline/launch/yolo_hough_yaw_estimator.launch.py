from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "image_topic",
            default_value="/camera/camera/color/image_raw",
            description="Input color image topic.",
        ),
        DeclareLaunchArgument(
            "yolo_json_topic",
            default_value="/vision/yolo/detections_json",
            description="Input YOLO detections JSON topic.",
        ),
        DeclareLaunchArgument(
            "output_json_topic",
            default_value="/vision/yolo/hough_yaw_json",
            description="Output yaw JSON topic.",
        ),
        DeclareLaunchArgument(
            "debug_image_topic",
            default_value="/vision/debug_hough_yaw_image",
        ),
        DeclareLaunchArgument(
            "debug_edges_topic",
            default_value="/vision/debug_hough_edges",
        ),
        DeclareLaunchArgument("target_class", default_value="wood"),
        DeclareLaunchArgument("bbox_padding", default_value="25"),
        DeclareLaunchArgument("canny_low", default_value="30"),
        DeclareLaunchArgument("canny_high", default_value="100"),
        DeclareLaunchArgument("hough_threshold", default_value="10"),
        DeclareLaunchArgument("min_line_length", default_value="10"),
        DeclareLaunchArgument("max_line_gap", default_value="8"),
        DeclareLaunchArgument("arrow_length", default_value="60"),
        DeclareLaunchArgument("publish_debug_image", default_value="true"),
    ]

    node = Node(
        package="robot_vision_pipeline",
        executable="yolo_hough_yaw_estimator_node",
        name="yolo_hough_yaw_estimator_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "yolo_json_topic": LaunchConfiguration("yolo_json_topic"),
                "output_json_topic": LaunchConfiguration("output_json_topic"),
                "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                "debug_edges_topic": LaunchConfiguration("debug_edges_topic"),
                "target_class": LaunchConfiguration("target_class"),
                "bbox_padding": LaunchConfiguration("bbox_padding"),
                "canny_low": LaunchConfiguration("canny_low"),
                "canny_high": LaunchConfiguration("canny_high"),
                "hough_threshold": LaunchConfiguration("hough_threshold"),
                "min_line_length": LaunchConfiguration("min_line_length"),
                "max_line_gap": LaunchConfiguration("max_line_gap"),
                "arrow_length": LaunchConfiguration("arrow_length"),
                "publish_debug_image": LaunchConfiguration("publish_debug_image"),
            }
        ],
    )

    return LaunchDescription(args + [node])
