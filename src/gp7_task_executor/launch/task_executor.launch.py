"""Launch file to start the `gp7_task_executor` C++ node.

This launch file starts the task executor node that provides three services:
- /move_to_named_target: Move to a named SRDF target (e.g., "home")
- /move_to_joint_target: Move to a joint-space goal
- /move_to_pose_target: Move to a pose goal
- /move_sequence: Execute a sequence of predefined joint waypoints

Assumes MoveIt is already running separately.

Typical usage:
    ros2 launch gp7_task_executor task_executor.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_share = PathJoinSubstitution([
        FindPackageShare("gp7_task_executor"),
    ])

    launch_args = [
        DeclareLaunchArgument(
            "move_group_name",
            default_value="arm",
            description="MoveIt planning group name"),
        DeclareLaunchArgument(
            "base_frame",
            default_value="base_link",
            description="Base reference frame for planning (MUST be base_link — do NOT use world)"),
        DeclareLaunchArgument(
            "ee_link",
            default_value="tool0",
            description="End-effector link name"),
        DeclareLaunchArgument(
            "waypoints_config",
            default_value=[pkg_share, "/config/joint_waypoints.yaml"],
            description="Path to the YAML file containing joint waypoints"),
        DeclareLaunchArgument(
            "cartesian_points_config",
            default_value=[pkg_share, "/config/cartesian_points.yaml"],
            description="Path to the YAML file containing Cartesian points (xyz only, fixed orientation)"),
        DeclareLaunchArgument(
            "pose_waypoints_config",
            default_value=[pkg_share, "/config/pose_waypoints.yaml"],
            description="Path to the YAML file containing named pose waypoints (position + RPY)"),
        DeclareLaunchArgument(
            "planning_time",
            default_value="2.0",
            description="Default planning time (seconds)"),
        DeclareLaunchArgument(
            "num_planning_attempts",
            default_value="5",
            description="Number of planning attempts"),
        DeclareLaunchArgument(
            "max_velocity_scaling_factor",
            default_value="0.5",
            description="Max velocity scaling factor (0-1)"),
        DeclareLaunchArgument(
            "max_acceleration_scaling_factor",
            default_value="0.5",
            description="Max acceleration scaling factor (0-1)"),
    ]

    node = Node(
        package="gp7_task_executor",
        executable="task_executor_node",
        name="task_executor_node",
        output="screen",
        parameters=[{
            "move_group_name": LaunchConfiguration("move_group_name"),
            "base_frame": LaunchConfiguration("base_frame"),
            "ee_link": LaunchConfiguration("ee_link"),
            "waypoints_config_path": LaunchConfiguration("waypoints_config"),
            "cartesian_points_config_path": LaunchConfiguration("cartesian_points_config"),
            "pose_waypoints_config_path": LaunchConfiguration("pose_waypoints_config"),
            "planning_time": LaunchConfiguration("planning_time"),
            "num_planning_attempts": LaunchConfiguration("num_planning_attempts"),
            "max_velocity_scaling_factor": LaunchConfiguration("max_velocity_scaling_factor"),
            "max_acceleration_scaling_factor": LaunchConfiguration("max_acceleration_scaling_factor"),
        }],
    )

    return LaunchDescription(launch_args + [node])
