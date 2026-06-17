"""Launch Gazebo simulation stack plus the DRL planner.

This is the Gazebo counterpart to ``drl_mock_hw.launch.py``.  The robot model,
Gazebo, controller manager, MoveIt, RViz and ``robot_task_executor`` are started
by ``robot_bringup/launch/sim.launch.py``; this launch only adds the mock vision
environment and the DRL unified planner.

Usage:
    ros2 launch robot_drl drl_gazebo.launch.py

    ros2 launch robot_drl drl_gazebo.launch.py input_mode:=vision

    ros2 launch robot_drl drl_gazebo.launch.py \
        auto_execute_after_plan:=true target_x:=0.450 target_y:=-0.100 target_z:=0.120
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    input_mode_arg = DeclareLaunchArgument(
        "input_mode",
        default_value="manual",
        description="Input mode: 'manual' or 'vision'",
    )
    auto_plan_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Plan automatically on startup",
    )
    manual_prompt_arg = DeclareLaunchArgument(
        "manual_prompt_on_start",
        default_value="false",
        description="Prompt terminal for manual target input on startup",
    )
    auto_exec_arg = DeclareLaunchArgument(
        "auto_execute_after_plan",
        default_value="true",
        description="Execute the DRL trajectory after planning",
    )
    start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.375, 0.000, 0.250]",
        description="Start TCP in base_link frame [x, y, z] in metres",
    )
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start MoveIt RViz from robot_bringup sim.launch.py",
    )
    target_x_arg = DeclareLaunchArgument("target_x", default_value="0.450")
    target_y_arg = DeclareLaunchArgument("target_y", default_value="0.050")
    target_z_arg = DeclareLaunchArgument("target_z", default_value="0.120")
    target_class_arg = DeclareLaunchArgument("target_class_name", default_value="box")

    use_sim_time = "true"

    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_bringup"),
                "launch",
                "sim.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_rviz": LaunchConfiguration("use_rviz"),
            "start_task_executor": "true",
            "spawn_demo_woods": "false",
        }.items(),
    )

    task_executor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_task_executor"),
                "launch",
                "task_executor.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "move_group_name": "arm",
            "base_frame": "base_link",
            "ee_link": "tcp_link",
            "planning_time": "2.0",
            "num_planning_attempts": "5",
            "max_velocity_scaling_factor": "0.5",
            "max_acceleration_scaling_factor": "0.5",
        }.items(),
    )

    mock_env_node = Node(
        package="robot_drl",
        executable="mock_environment_node",
        name="mock_environment_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "publish_rate_hz": 10.0,
            "target_class_name": LaunchConfiguration("target_class_name"),
            "target_x": LaunchConfiguration("target_x"),
            "target_y": LaunchConfiguration("target_y"),
            "target_z": LaunchConfiguration("target_z"),
            "frame_id": "base_link",
            "distance_m": 0.5,
            "confidence": 0.95,
        }],
    )

    drl_node = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "calibrated_start_tcp_base": LaunchConfiguration("calibrated_start_tcp_base"),
            "input_mode": LaunchConfiguration("input_mode"),
            "auto_plan_on_start": LaunchConfiguration("auto_plan_on_start"),
            "manual_prompt_on_start": LaunchConfiguration("manual_prompt_on_start"),
            "auto_execute_after_plan": LaunchConfiguration("auto_execute_after_plan"),
            "manual_default_target": ParameterValue([
                "[",
                LaunchConfiguration("target_x"),
                ", ",
                LaunchConfiguration("target_y"),
                ", ",
                LaunchConfiguration("target_z"),
                "]",
            ], value_type=list[float]),
            "manual_default_obstacle_size": [0.0, 0.0, 0.0],
            "workspace_min_base": [0.250, -0.150, 0.020],
            "workspace_max_base": [0.500, 0.150, 0.300],
            "use_current_tcp_orientation_for_execution": True,
            "start_pose_tolerance": 0.005,
            "preposition_before_plan": True,
            "preposition_tcp_base": [0.375, 0.000, 0.250],
            "preposition_clamp_to_workspace": True,
            "preposition_verify_timeout_sec": 3.0,
            "update_start_tcp_from_tf_before_plan": True,
            "fallback_to_final_pose_on_execute_failure": False,
            "execute_final_pose_only": False,
            "use_planning_scene_obstacles": True,
            "planning_scene_service_name": "/get_planning_scene",
            "planning_scene_frame": "base_link",
            "path_collision_check_step_m": 0.01,
            "path_collision_clearance_margin_m": 0.05,
            "obstacle_safety_filter_enabled": True,
            "obstacle_safety_margin_m": 0.05,
            "obstacle_safety_check_step_m": 0.005,
            "execute_collision_check_step_m": 0.01,
            "execute_collision_clearance_margin_m": 0.05,
        }],
    )

    return LaunchDescription([
        LogInfo(msg="[drl_gazebo] Gazebo sim + task executor + DRL"),
        input_mode_arg,
        auto_plan_arg,
        manual_prompt_arg,
        auto_exec_arg,
        start_tcp_arg,
        use_rviz_arg,
        target_x_arg,
        target_y_arg,
        target_z_arg,
        target_class_arg,
        sim_bringup,
        TimerAction(period=12.0, actions=[
            LogInfo(msg="[drl_gazebo] Starting task_executor_node..."),
            task_executor_launch,
        ]),
        TimerAction(period=14.0, actions=[
            LogInfo(msg="[drl_gazebo] Starting mock environment and DRL planner..."),
            mock_env_node,
            drl_node,
        ]),
    ])
