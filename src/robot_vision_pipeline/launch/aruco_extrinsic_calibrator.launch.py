from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("robot_vision_pipeline")
    default_layout_file = PathJoinSubstitution(
        [pkg_share, "config", "aruco_board_layout.yaml"]
    )
    default_result_file = PathJoinSubstitution(
        [pkg_share, "config", "aruco_extrinsic_result.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "board_layout_file",
                default_value=default_layout_file,
                description="Path to ArUco board layout YAML",
            ),
            DeclareLaunchArgument(
                "result_yaml_path",
                default_value=default_result_file,
                description="Path to save the camera pose result YAML",
            ),
            Node(
                package="robot_vision_pipeline",
                executable="aruco_extrinsic_calibrator_node",
                name="aruco_extrinsic_calibrator_node",
                output="screen",
                parameters=[
                    {
                        "board_layout_file": LaunchConfiguration("board_layout_file"),
                        "result_yaml_path": LaunchConfiguration("result_yaml_path"),
                    }
                ],
            ),
        ]
    )
