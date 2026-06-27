"""Launch the DRL-focused Cartesian pose sequence executor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    robot_description_pkg = get_package_share_directory("robot_description")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="robot_moveit")
        .robot_description(
            file_path=os.path.join(robot_description_pkg, "urdf", "robot.urdf.xacro"),
            mappings={
                "use_sim": LaunchConfiguration("use_sim_time"),
                "use_mock_hardware": "true",
            },
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    launch_args = [
        DeclareLaunchArgument("move_group_name", default_value="arm"),
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("ee_link", default_value="tcp_link"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("planning_time", default_value="2.0"),
        DeclareLaunchArgument("num_planning_attempts", default_value="5"),
        DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.1"),
        DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.5"),
        DeclareLaunchArgument(
            "cartesian_pose_sequence_service_name",
            default_value="/move_cartesian_pose_sequence",
        ),
        DeclareLaunchArgument("cartesian_eef_step", default_value="0.01"),
        DeclareLaunchArgument("cartesian_jump_threshold", default_value="0.0"),
        DeclareLaunchArgument("cartesian_success_threshold", default_value="0.95"),
    ]

    node = Node(
        package="robot_drl_executor",
        executable="robot_drl_executor_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "move_group_name": LaunchConfiguration("move_group_name"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "base_frame": LaunchConfiguration("base_frame"),
                "ee_link": LaunchConfiguration("ee_link"),
                "planning_time": LaunchConfiguration("planning_time"),
                "num_planning_attempts": LaunchConfiguration("num_planning_attempts"),
                "max_velocity_scaling_factor": LaunchConfiguration(
                    "max_velocity_scaling_factor"
                ),
                "max_acceleration_scaling_factor": LaunchConfiguration(
                    "max_acceleration_scaling_factor"
                ),
                "cartesian_pose_sequence_service_name": LaunchConfiguration(
                    "cartesian_pose_sequence_service_name"
                ),
                "cartesian_eef_step": LaunchConfiguration("cartesian_eef_step"),
                "cartesian_jump_threshold": LaunchConfiguration("cartesian_jump_threshold"),
                "cartesian_success_threshold": LaunchConfiguration(
                    "cartesian_success_threshold"
                ),
            },
        ],
    )

    return LaunchDescription(launch_args + [node])
