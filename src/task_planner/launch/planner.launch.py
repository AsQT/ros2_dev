import os

from launch import LaunchDescription
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="robot_moveit")
        .to_moveit_configs()
    )

    planning_tutorial_node = Node(
        package="task_planner",
        executable="planning_tutorial",
        name="planning_tutorial",
        output="screen",
        parameters=[
            moveit_config.to_dict()
        ],
    )

    return LaunchDescription([
        planning_tutorial_node
    ])