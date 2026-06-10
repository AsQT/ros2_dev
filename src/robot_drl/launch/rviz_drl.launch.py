"""Launch RViz configured for DRL trajectory markers."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description() -> LaunchDescription:
    rviz_config = PathJoinSubstitution([
        FindPackageShare("robot_drl"),
        "rviz",
        "drl_markers.rviz",
    ])

    return LaunchDescription([
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_drl",
            output="screen",
            arguments=["-d", rviz_config],
        )
    ])
