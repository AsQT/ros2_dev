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
        DeclareLaunchArgument("randomize_objects", default_value="false"),
        DeclareLaunchArgument("wood_x", default_value="0.44"),
        DeclareLaunchArgument("wood_y", default_value="0.06"),
        DeclareLaunchArgument("wood_size", default_value="[0.03, 0.03, 0.03]"),
        DeclareLaunchArgument("box_x", default_value="0.34"),
        DeclareLaunchArgument("box_y", default_value="-0.09"),
        DeclareLaunchArgument("box_size", default_value="[0.10, 0.10, 0.10]"),
        DeclareLaunchArgument("randomize_box_size", default_value="true"),
        DeclareLaunchArgument("box_size_min", default_value="[0.05, 0.05, 0.05]"),
        DeclareLaunchArgument("box_size_max", default_value="[0.15, 0.15, 0.15]"),
        DeclareLaunchArgument(
            "table_height",
            default_value="-1.0",
            description="Negative value means infer from the table world SDF",
        ),
        DeclareLaunchArgument("robot_base_world_z", default_value="1.02"),
        DeclareLaunchArgument("gripper_close_width", default_value="0.025"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument(
            "pick_z_offset",
            default_value="0.06",
            description="Added to the pick_wood center Z before sending target_pick",
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
            "max_velocity_scaling_factor": "0.5",
            "max_acceleration_scaling_factor": "0.5",
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
            "table_height": ParameterValue(LaunchConfiguration("table_height"), value_type=float),
            "robot_base_world_z": ParameterValue(LaunchConfiguration("robot_base_world_z"), value_type=float),
            "randomize": ParameterValue(LaunchConfiguration("randomize_objects"), value_type=bool),
            "wood_x_min": 0.38,
            "wood_x_max": 0.48,
            "wood_y_min": 0.02,
            "wood_y_max": 0.12,
            "box_x_min": 0.34,
            "box_x_max": 0.43,
            "box_y_min": -0.12,
            "box_y_max": -0.05,
            "min_xy_separation": 0.09,
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
            "action_name": "drl_pickplace",
            "frame_id": "base_link",
            "place_xyz": ParameterValue(LaunchConfiguration("place_xyz"), value_type=list[float]),
            "pick_z_offset_m": ParameterValue(LaunchConfiguration("pick_z_offset"), value_type=float),
            "gripper_close_width_m": ParameterValue(LaunchConfiguration("gripper_close_width"), value_type=float),
            "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
            "obstacle_id": "obstacle_box",
            "min_pick_z_m": 0.025,
            "object_timeout_sec": 60.0,
            "action_server_timeout_sec": 120.0,
            "planning_scene_timeout_sec": 20.0,
            "goal_timeout_sec": 420.0,
        }],
        condition=IfCondition(LaunchConfiguration("run_demo_client")),
    )

    return LaunchDescription([
        *args,
        LogInfo(msg="[rl_pick_place_box_gazebo_demo] Starting Gazebo + MoveIt + task servers"),
        sim_stack,
        TimerAction(period=10.0, actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Starting robot_drl_executor"),
            drl_executor,
        ]),
        TimerAction(period=14.0, actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Starting DRL planner"),
            drl_node,
        ]),
        TimerAction(period=16.0, actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Spawning pick_wood and obstacle_box from Gazebo ground truth"),
            sim_objects,
        ]),
        TimerAction(period=LaunchConfiguration("demo_client_delay"), actions=[
            LogInfo(msg="[rl_pick_place_box_gazebo_demo] Sending RL pick-place goal from wood pose and box obstacle"),
            demo_client,
        ]),
    ])
