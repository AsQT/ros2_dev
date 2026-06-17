"""Launch the mock-hardware DRL stack and run the obstacle regression client.

This is the one-command path for checking that the DRL planner reads MoveIt
PlanningScene obstacles, changes its path, validates clearance, and optionally
executes the resulting trajectory on ros2_control mock hardware.

Usage:
    ros2 launch robot_drl drl_mock_hw_obstacle_test.launch.py

    ros2 launch robot_drl drl_mock_hw_obstacle_test.launch.py \
        execute:=false randomize_obstacle_count:=true obstacle_count_max:=10
"""

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
        description="Execute the validated obstacle-avoidance trajectory",
    )
    randomize_target_arg = DeclareLaunchArgument(
        "randomize_target",
        default_value="true",
        description="Sample a target inside target_min_base/target_max_base",
    )
    randomize_start_arg = DeclareLaunchArgument(
        "randomize_start",
        default_value="true",
        description="Sample the planner start TCP inside start_min_base/start_max_base",
    )
    random_seed_arg = DeclareLaunchArgument(
        "random_seed",
        default_value="2",
        description="Random seed for repeatable target/obstacle placement; use -1 for entropy",
    )
    case_count_arg = DeclareLaunchArgument(
        "case_count",
        default_value="1",
        description="Number of random mock-hardware cases to run in one launch",
    )
    obstacle_count_arg = DeclareLaunchArgument(
        "obstacle_count",
        default_value="3",
        description="Number of PlanningScene box obstacles to add",
    )
    randomize_obstacle_count_arg = DeclareLaunchArgument(
        "randomize_obstacle_count",
        default_value="true",
        description="Sample obstacle_count in [obstacle_count_min, obstacle_count_max]",
    )
    obstacle_count_min_arg = DeclareLaunchArgument(
        "obstacle_count_min",
        default_value="2",
        description="Minimum random obstacle count",
    )
    obstacle_count_max_arg = DeclareLaunchArgument(
        "obstacle_count_max",
        default_value="4",
        description="Maximum random obstacle count",
    )
    obstacle_size_arg = DeclareLaunchArgument(
        "obstacle_size",
        default_value="[0.02, 0.02, 0.02]",
        description="Fixed obstacle box size [x, y, z] when randomize_obstacle_size is false",
    )
    randomize_obstacle_size_arg = DeclareLaunchArgument(
        "randomize_obstacle_size",
        default_value="true",
        description="Randomize each obstacle size inside obstacle_size_min/max",
    )
    obstacle_size_min_arg = DeclareLaunchArgument(
        "obstacle_size_min",
        default_value="[0.018, 0.018, 0.018]",
        description="Minimum random obstacle size [x, y, z] in metres",
    )
    obstacle_size_max_arg = DeclareLaunchArgument(
        "obstacle_size_max",
        default_value="[0.045, 0.045, 0.045]",
        description="Maximum random obstacle size [x, y, z] in metres",
    )
    randomize_obstacle_positions_arg = DeclareLaunchArgument(
        "randomize_obstacle_positions",
        default_value="true",
        description="Randomize obstacle centers around the start-target segment",
    )
    randomize_obstacle_orientation_arg = DeclareLaunchArgument(
        "randomize_obstacle_orientation",
        default_value="true",
        description="Randomize obstacle orientation before adding it to PlanningScene",
    )
    obstacle_roll_pitch_max_arg = DeclareLaunchArgument(
        "obstacle_roll_pitch_max_rad",
        default_value="0.25",
        description="Maximum absolute random obstacle roll/pitch angle in radians",
    )
    obstacle_yaw_min_arg = DeclareLaunchArgument(
        "obstacle_yaw_min_rad",
        default_value="-3.141592653589793",
        description="Minimum random obstacle yaw angle in radians",
    )
    obstacle_yaw_max_arg = DeclareLaunchArgument(
        "obstacle_yaw_max_rad",
        default_value="3.141592653589793",
        description="Maximum random obstacle yaw angle in radians",
    )
    max_random_attempts_arg = DeclareLaunchArgument(
        "max_random_attempts",
        default_value="5",
        description="Retry random obstacle scenes until Test B passes or attempts are exhausted",
    )
    retry_cooldown_arg = DeclareLaunchArgument(
        "retry_cooldown_sec",
        default_value="5.0",
        description="Wait after a failed random plan before sending the next plan request",
    )
    obstacle_lateral_offset_max_arg = DeclareLaunchArgument(
        "obstacle_lateral_offset_max_m",
        default_value="0.150",
        description="Maximum lateral obstacle offset from the start-target segment",
    )
    obstacle_lateral_min_arg = DeclareLaunchArgument(
        "obstacle_lateral_min_m",
        default_value="0.090",
        description="Minimum lateral offset for random obstacle centers",
    )
    obstacle_primary_lateral_min_arg = DeclareLaunchArgument(
        "obstacle_primary_lateral_min_m",
        default_value="0.100",
        description="Minimum lateral offset for the policy-selected obstacle",
    )
    obstacle_path_clearance_arg = DeclareLaunchArgument(
        "obstacle_path_clearance_m",
        default_value="0.035",
        description="Minimum estimated clearance from each random obstacle AABB to the start-target segment",
    )
    obstacle_path_fraction_min_arg = DeclareLaunchArgument(
        "obstacle_path_fraction_min",
        default_value="0.35",
        description="Minimum random obstacle fraction along the start-target segment",
    )
    obstacle_path_fraction_max_arg = DeclareLaunchArgument(
        "obstacle_path_fraction_max",
        default_value="0.80",
        description="Maximum random obstacle fraction along the start-target segment",
    )
    obstacle_endpoint_clearance_arg = DeclareLaunchArgument(
        "obstacle_endpoint_clearance_m",
        default_value="0.080",
        description="Extra distance from start/target when sampling obstacle centers",
    )
    obstacle_min_separation_arg = DeclareLaunchArgument(
        "obstacle_min_separation_m",
        default_value="0.060",
        description="Minimum center-to-center distance between sampled obstacles",
    )
    start_base_arg = DeclareLaunchArgument(
        "start_base",
        default_value="[0.375, 0.000, 0.250]",
        description="Fallback start [x, y, z] when randomize_start is false",
    )
    start_min_arg = DeclareLaunchArgument(
        "start_min_base",
        default_value="[0.33, -0.08, 0.20]",
        description="Minimum random start [x, y, z]",
    )
    start_max_arg = DeclareLaunchArgument(
        "start_max_base",
        default_value="[0.40, 0.08, 0.27]",
        description="Maximum random start [x, y, z]",
    )
    target_base_arg = DeclareLaunchArgument(
        "target_base",
        default_value="[0.45, 0.05, 0.12]",
        description="Fallback target [x, y, z] when randomize_target is false",
    )
    target_min_arg = DeclareLaunchArgument(
        "target_min_base",
        default_value="[0.43, -0.08, 0.09]",
        description="Minimum random target [x, y, z]",
    )
    target_max_arg = DeclareLaunchArgument(
        "target_max_base",
        default_value="[0.49, 0.10, 0.16]",
        description="Maximum random target [x, y, z]",
    )
    random_target_min_distance_arg = DeclareLaunchArgument(
        "random_target_min_distance_m",
        default_value="0.13",
        description="Minimum distance between random start and random target",
    )
    timeout_arg = DeclareLaunchArgument(
        "timeout_sec",
        default_value="45.0",
        description="Timeout for planner, MoveIt services, and execution checks",
    )
    output_file_arg = DeclareLaunchArgument(
        "output_file",
        default_value="/tmp/robot_drl_mock_hw_obstacle_test.npz",
        description="NPZ file for captured A/B trajectories and observations",
    )
    test_delay_arg = DeclareLaunchArgument(
        "test_start_delay_sec",
        default_value="18.0",
        description="Delay before starting the obstacle test client",
    )

    stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_drl"),
                "launch",
                "drl_mock_hw.launch.py",
            )
        ),
        launch_arguments={
            "input_mode": "manual",
            "auto_plan_on_start": "false",
            "manual_prompt_on_start": "false",
            "auto_execute_after_plan": "false",
        }.items(),
    )

    obstacle_test_node = Node(
        package="robot_drl",
        executable="mock_hw_obstacle_test",
        name="drl_mock_hw_obstacle_test",
        output="screen",
        parameters=[{
            "execute": LaunchConfiguration("execute"),
            "randomize_start": LaunchConfiguration("randomize_start"),
            "randomize_target": LaunchConfiguration("randomize_target"),
            "random_seed": LaunchConfiguration("random_seed"),
            "case_count": LaunchConfiguration("case_count"),
            "obstacle_count": LaunchConfiguration("obstacle_count"),
            "randomize_obstacle_count": LaunchConfiguration("randomize_obstacle_count"),
            "obstacle_count_min": LaunchConfiguration("obstacle_count_min"),
            "obstacle_count_max": LaunchConfiguration("obstacle_count_max"),
            "obstacle_size": LaunchConfiguration("obstacle_size"),
            "randomize_obstacle_size": LaunchConfiguration("randomize_obstacle_size"),
            "obstacle_size_min": LaunchConfiguration("obstacle_size_min"),
            "obstacle_size_max": LaunchConfiguration("obstacle_size_max"),
            "randomize_obstacle_positions": LaunchConfiguration("randomize_obstacle_positions"),
            "randomize_obstacle_orientation": LaunchConfiguration("randomize_obstacle_orientation"),
            "obstacle_roll_pitch_max_rad": LaunchConfiguration("obstacle_roll_pitch_max_rad"),
            "obstacle_yaw_min_rad": LaunchConfiguration("obstacle_yaw_min_rad"),
            "obstacle_yaw_max_rad": LaunchConfiguration("obstacle_yaw_max_rad"),
            "max_random_attempts": LaunchConfiguration("max_random_attempts"),
            "retry_cooldown_sec": LaunchConfiguration("retry_cooldown_sec"),
            "obstacle_lateral_offset_max_m": LaunchConfiguration("obstacle_lateral_offset_max_m"),
            "obstacle_lateral_min_m": LaunchConfiguration("obstacle_lateral_min_m"),
            "obstacle_primary_lateral_min_m": LaunchConfiguration("obstacle_primary_lateral_min_m"),
            "obstacle_path_clearance_m": LaunchConfiguration("obstacle_path_clearance_m"),
            "obstacle_path_fraction_min": LaunchConfiguration("obstacle_path_fraction_min"),
            "obstacle_path_fraction_max": LaunchConfiguration("obstacle_path_fraction_max"),
            "obstacle_endpoint_clearance_m": LaunchConfiguration("obstacle_endpoint_clearance_m"),
            "obstacle_min_separation_m": LaunchConfiguration("obstacle_min_separation_m"),
            "start_base": LaunchConfiguration("start_base"),
            "start_min_base": LaunchConfiguration("start_min_base"),
            "start_max_base": LaunchConfiguration("start_max_base"),
            "target_base": LaunchConfiguration("target_base"),
            "target_min_base": LaunchConfiguration("target_min_base"),
            "target_max_base": LaunchConfiguration("target_max_base"),
            "random_target_min_distance_m": LaunchConfiguration("random_target_min_distance_m"),
            "timeout_sec": LaunchConfiguration("timeout_sec"),
            "output_file": LaunchConfiguration("output_file"),
        }],
    )

    return LaunchDescription([
        LogInfo(msg="[drl_mock_hw_obstacle_test] Starting mock hardware DRL obstacle test"),
        execute_arg,
        randomize_start_arg,
        randomize_target_arg,
        random_seed_arg,
        case_count_arg,
        obstacle_count_arg,
        randomize_obstacle_count_arg,
        obstacle_count_min_arg,
        obstacle_count_max_arg,
        obstacle_size_arg,
        randomize_obstacle_size_arg,
        obstacle_size_min_arg,
        obstacle_size_max_arg,
        randomize_obstacle_positions_arg,
        randomize_obstacle_orientation_arg,
        obstacle_roll_pitch_max_arg,
        obstacle_yaw_min_arg,
        obstacle_yaw_max_arg,
        max_random_attempts_arg,
        retry_cooldown_arg,
        obstacle_lateral_offset_max_arg,
        obstacle_lateral_min_arg,
        obstacle_primary_lateral_min_arg,
        obstacle_path_clearance_arg,
        obstacle_path_fraction_min_arg,
        obstacle_path_fraction_max_arg,
        obstacle_endpoint_clearance_arg,
        obstacle_min_separation_arg,
        start_base_arg,
        start_min_arg,
        start_max_arg,
        target_base_arg,
        target_min_arg,
        target_max_arg,
        random_target_min_distance_arg,
        timeout_arg,
        output_file_arg,
        test_delay_arg,
        stack_launch,
        TimerAction(
            period=LaunchConfiguration("test_start_delay_sec"),
            actions=[
                LogInfo(msg="[drl_mock_hw_obstacle_test] Running obstacle client..."),
                obstacle_test_node,
            ],
        ),
    ])
