"""Launch Gazebo DRL stack and run synchronized wood-block obstacle tests."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    execute_arg = DeclareLaunchArgument(
        "execute",
        default_value="true",
        description="Execute the validated DRL trajectory in Gazebo",
    )
    case_count_arg = DeclareLaunchArgument(
        "case_count",
        default_value="1",
        description="Number of random Gazebo cases to run",
    )
    random_seed_arg = DeclareLaunchArgument(
        "random_seed",
        default_value="2",
        description="Random seed for repeatable Gazebo obstacle tests; use -1 for entropy",
    )
    obstacle_mode_arg = DeclareLaunchArgument(
        "obstacle_mode",
        default_value="block_direct_path",
        description="Obstacle mode: random_free or block_direct_path",
    )
    max_random_attempts_arg = DeclareLaunchArgument(
        "max_random_attempts",
        default_value="5",
        description="Retry random Gazebo scenes until the case passes",
    )
    timeout_arg = DeclareLaunchArgument(
        "timeout_sec",
        default_value="75.0",
        description="Timeout for Gazebo, PlanningScene, planning, and execution checks",
    )
    output_file_arg = DeclareLaunchArgument(
        "output_file",
        default_value="/tmp/robot_drl_gazebo_obstacle_test.npz",
        description="NPZ evidence file for captured Gazebo test results",
    )
    test_delay_arg = DeclareLaunchArgument(
        "test_start_delay_sec",
        default_value="30.0",
        description="Delay before starting the Gazebo obstacle runner",
    )

    drl_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_drl"),
                "launch",
                "drl_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "input_mode": "manual",
            "auto_plan_on_start": "false",
            "manual_prompt_on_start": "false",
            "auto_execute_after_plan": "false",
            "use_rviz": "true",
        }.items(),
    )

    test_node = Node(
        package="robot_drl",
        executable="gazebo_obstacle_test",
        name="drl_gazebo_obstacle_test",
        output="screen",
        parameters=[{
            "execute": LaunchConfiguration("execute"),
            "case_count": LaunchConfiguration("case_count"),
            "random_seed": LaunchConfiguration("random_seed"),
            "obstacle_mode": LaunchConfiguration("obstacle_mode"),
            "max_random_attempts": LaunchConfiguration("max_random_attempts"),
            "timeout_sec": LaunchConfiguration("timeout_sec"),
            "output_file": LaunchConfiguration("output_file"),
        }],
    )

    return LaunchDescription([
        LogInfo(msg="[drl_gazebo_obstacle_test] Starting Gazebo DRL obstacle test"),
        execute_arg,
        case_count_arg,
        random_seed_arg,
        obstacle_mode_arg,
        max_random_attempts_arg,
        timeout_arg,
        output_file_arg,
        test_delay_arg,
        drl_gazebo,
        TimerAction(
            period=LaunchConfiguration("test_start_delay_sec"),
            actions=[
                LogInfo(msg="[drl_gazebo_obstacle_test] Running obstacle client..."),
                test_node,
            ],
        ),
    ])
