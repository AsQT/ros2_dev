"""Gazebo demo: 3 cm box + sim ground-truth perception + RL pick-place."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    task_executor_share = get_package_share_directory("robot_task_executor")

    args = [
        DeclareLaunchArgument("randomize_box", default_value="false"),
        DeclareLaunchArgument("box_x", default_value="0.42"),
        DeclareLaunchArgument("box_y", default_value="0.00"),
        DeclareLaunchArgument("box_size", default_value="0.03"),
        DeclareLaunchArgument(
            "table_height",
            default_value="-1.0",
            description="Negative value means infer from the table world SDF",
        ),
        DeclareLaunchArgument("robot_base_world_z", default_value="1.02"),
        DeclareLaunchArgument("gripper_close_width", default_value="0.025"),
        DeclareLaunchArgument(
            "pick_z_offset",
            default_value="0.06",
            description="Added to the perceived box center Z before sending target_pick",
        ),
        DeclareLaunchArgument("place_xyz", default_value="[0.34, -0.10, 0.12]"),
        DeclareLaunchArgument("run_demo_client", default_value="true"),
        DeclareLaunchArgument("spawn_startup_delay", default_value="3.0"),
        DeclareLaunchArgument("demo_client_delay", default_value="50.0"),
    ]

    sim_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "sim.launch.py")
        ),
        launch_arguments={
            "spawn_demo_woods": "false",
        }.items(),
    )

    task_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(task_executor_share, "launch", "task_executor.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "move_group_name": "arm",
            "base_frame": "base_link",
            "ee_link": "tcp_link",
            "planning_time": "2.0",
            "num_planning_attempts": "5",
            "max_velocity_scaling_factor": "0.5",
            "max_acceleration_scaling_factor": "0.5",
        }.items(),
    )

    pick_box = Node(
        package="robot_description",
        executable="spawn_pick_box.py",
        name="pick_box_spawner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "object_name": "pick_box",
            "frame_id": "base_link",
            "info_topic": "/sim/pick_box_info",
            "box_size": ParameterValue(LaunchConfiguration("box_size"), value_type=float),
            "table_height": ParameterValue(LaunchConfiguration("table_height"), value_type=float),
            "robot_base_world_z": ParameterValue(LaunchConfiguration("robot_base_world_z"), value_type=float),
            "x": ParameterValue(LaunchConfiguration("box_x"), value_type=float),
            "y": ParameterValue(LaunchConfiguration("box_y"), value_type=float),
            "randomize": ParameterValue(LaunchConfiguration("randomize_box"), value_type=bool),
            "x_min": 0.35,
            "x_max": 0.50,
            "y_min": -0.12,
            "y_max": 0.12,
            "startup_delay": LaunchConfiguration("spawn_startup_delay"),
            "publish_rate_hz": 2.0,
        }],
    )

    drl_node = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "input_mode": "manual",
            "auto_plan_on_start": False,
            "manual_prompt_on_start": False,
            "auto_execute_after_plan": False,
            "manual_default_target": [0.42, 0.0, 0.08],
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

    demo_client = Node(
        package="robot_task_manager",
        executable="drl_pick_place_box_demo_client.py",
        name="drl_pick_place_box_demo_client",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "object_info_topic": "/sim/pick_box_info",
            "action_name": "drl_pickplace",
            "frame_id": "base_link",
            "place_xyz": ParameterValue(LaunchConfiguration("place_xyz"), value_type=list[float]),
            "pick_z_offset_m": ParameterValue(LaunchConfiguration("pick_z_offset"), value_type=float),
            "gripper_close_width_m": ParameterValue(LaunchConfiguration("gripper_close_width"), value_type=float),
            "min_pick_z_m": 0.025,
            "object_timeout_sec": 60.0,
            "action_server_timeout_sec": 120.0,
            "goal_timeout_sec": 420.0,
        }],
        condition=IfCondition(LaunchConfiguration("run_demo_client")),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[rl_pick_place_box_gazebo_demo] Starting Gazebo + MoveIt + task servers"),
        sim_stack,
        TimerAction(period=10.0, actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Starting robot_task_executor"),
            task_executor,
        ]),
        TimerAction(period=14.0, actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Starting DRL planner"),
            drl_node,
        ]),
        TimerAction(period=16.0, actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Spawning 3 cm box and publishing /sim/pick_box_info"),
            pick_box,
        ]),
        TimerAction(period=LaunchConfiguration("demo_client_delay"), actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Sending RL pick-place goal from sim perception"),
            demo_client,
        ]),
    ])
