"""Gazebo demo: pick_wood object + obstacle_box ground truth + RL pick-place."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

RL_PYTHON = "/home/minhquang/venvs/ros_rl/bin/python3"


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    drl_executor_share = get_package_share_directory("robot_drl_executor")

    args = [
        DeclareLaunchArgument("randomize_objects", default_value="true"),
        DeclareLaunchArgument("wood_x", default_value="0.44"),
        DeclareLaunchArgument("wood_y", default_value="0.06"),
        DeclareLaunchArgument("wood_size", default_value="[0.03, 0.03, 0.03]"),
        DeclareLaunchArgument("box_x", default_value="0.34"),
        DeclareLaunchArgument("box_y", default_value="-0.09"),
        DeclareLaunchArgument("box_size", default_value="[0.10, 0.10, 0.10]"),
        DeclareLaunchArgument("randomize_box_size", default_value="true"),
        DeclareLaunchArgument("box_size_min", default_value="[0.05, 0.05, 0.10]"),
        DeclareLaunchArgument("box_size_max", default_value="[0.10, 0.09, 0.13]"),
        DeclareLaunchArgument(
            "table_height",
            default_value="-1.0",
            description="Negative value means infer from the table world SDF",
        ),
        DeclareLaunchArgument("robot_base_world_z", default_value="1.02"),
        DeclareLaunchArgument("gripper_close_width", default_value="0.025"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument(
            "grasp_tcp_offset_z",
            default_value="0.015",
            description="TCP Z above wood center when fingers contact wood (tune empirically)",
        ),
        DeclareLaunchArgument("place_xyz", default_value="[0.46, 0.12, 0.12]"),
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
            "enable_drl_backend": "false",
        }.items(),
    )

    drl_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drl_executor_share, "launch", "robot_drl_executor.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "move_group_name": "arm",
            "base_frame": "base_link",
            "ee_link": "tcp_link",
            "planning_time": "2.0",
            "num_planning_attempts": "5",
            "max_velocity_scaling_factor": "1.0",
            "max_acceleration_scaling_factor": "1.0",
        }.items(),
    )

    sim_objects = Node(
        package="robot_gazebo",
        executable="spawn_pick_wood_obstacle_box.py",
        name="pick_wood_obstacle_box_spawner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "frame_id": "base_link",
            "wood_name": "pick_wood",
            "wood_info_topic": "/sim/pick_wood_info",
            "wood_size": ParameterValue(LaunchConfiguration("wood_size"), value_type=list[float]),
            "wood_x": ParameterValue(LaunchConfiguration("wood_x"), value_type=float),
            "wood_y": ParameterValue(LaunchConfiguration("wood_y"), value_type=float),
            "box_name": "obstacle_box",
            "box_info_topic": "/sim/obstacle_box_info",
            "box_size": ParameterValue(LaunchConfiguration("box_size"), value_type=list[float]),
            "randomize_box_size": ParameterValue(LaunchConfiguration("randomize_box_size"), value_type=bool),
            "box_size_min": ParameterValue(LaunchConfiguration("box_size_min"), value_type=list[float]),
            "box_size_max": ParameterValue(LaunchConfiguration("box_size_max"), value_type=list[float]),
            "box_x": ParameterValue(LaunchConfiguration("box_x"), value_type=float),
            "box_y": ParameterValue(LaunchConfiguration("box_y"), value_type=float),
            "place_xyz": ParameterValue(LaunchConfiguration("place_xyz"), value_type=list[float]),
            "place_info_topic": "/sim/place_pose_info",
            "table_height": ParameterValue(LaunchConfiguration("table_height"), value_type=float),
            "robot_base_world_z": ParameterValue(LaunchConfiguration("robot_base_world_z"), value_type=float),
            "randomize": ParameterValue(LaunchConfiguration("randomize_objects"), value_type=bool),
            "corridor_randomization": True,
            "wood_x_min": 0.42,
            "wood_x_max": 0.48,
            "wood_y_min": 0.08,
            "wood_y_max": 0.13,
            "start_info_topic": "/sim/start_pose_info",
            "start_x_min": 0.35,
            "start_x_max": 0.39,
            "start_y_abs_min": 0.08,
            "start_y_abs_max": 0.13,
            "place_x_min": 0.43,
            "place_x_max": 0.48,
            "place_y_abs_min": 0.08,
            "place_y_abs_max": 0.15,
            "box_x_min": 0.36,
            "box_x_max": 0.46,
            "box_y_center_min": -0.025,
            "box_y_center_max": 0.025,
            "box_midpoint_noise_x": 0.015,
            "min_xy_separation": 0.10,
            "avoidance_margin_m": 0.02,
            "startup_delay": LaunchConfiguration("spawn_startup_delay"),
            "publish_rate_hz": 2.0,
        }],
    )

    drl_node = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        prefix=RL_PYTHON,
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
            "task_executor_result_timeout_sec": 300.0,
        }],
    )

    demo_client = Node(
        package="robot_task_manager",
        executable="drl_pick_place_wood_box_demo_client.py",
        name="drl_pick_place_wood_box_demo_client",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "wood_info_topic": "/sim/pick_wood_info",
            "box_info_topic": "/sim/obstacle_box_info",
            "start_info_topic": "/sim/start_pose_info",
            "place_info_topic": "/sim/place_pose_info",
            "use_scene_start_pose": True,
            "use_scene_place_pose": True,
            "action_name": "drl_pickplace",
            "planner_node_name": "/drl_unified_planner_node",
            "frame_id": "base_link",
            "place_xyz": ParameterValue(LaunchConfiguration("place_xyz"), value_type=list[float]),
            "grasp_tcp_offset_z": ParameterValue(LaunchConfiguration("grasp_tcp_offset_z"), value_type=float),
            "gripper_close_width_m": ParameterValue(LaunchConfiguration("gripper_close_width"), value_type=float),
            "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
            "obstacle_id": "obstacle_box",
            "min_pick_z_m": 0.025,
            "object_timeout_sec": 60.0,
            "action_server_timeout_sec": 120.0,
            "planning_scene_timeout_sec": 20.0,
            "goal_timeout_sec": 900.0,
        }],
        condition=IfCondition(LaunchConfiguration("run_demo_client")),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[timing] launch start: Gazebo + MoveIt + task servers"),
        sim_stack,
        TimerAction(period=10.0, actions=[
            LogInfo(msg="[timing] +10.0s starting robot_drl_executor; execution_velocity_scale=1.0"),
            drl_executor,
        ]),
        TimerAction(period=14.0, actions=[
            LogInfo(msg="[timing] +14.0s starting DRL planner"),
            drl_node,
        ]),
        TimerAction(period=16.0, actions=[
            LogInfo(msg="[timing] +16.0s spawning pick_wood and obstacle_box"),
            sim_objects,
        ]),
        TimerAction(period=LaunchConfiguration("demo_client_delay"), actions=[
            LogInfo(msg="[timing] demo_client_delay elapsed; sending RL pick-place goal"),
            demo_client,
        ]),
    ])
