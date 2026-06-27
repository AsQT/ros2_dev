import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    robot_description_pkg = get_package_share_directory("robot_description")
    moveit_config = (
        MoveItConfigsBuilder(robot_name="robot", package_name="robot_moveit")
        .robot_description(
            file_path=os.path.join(robot_description_pkg, "urdf", "robot.urdf.xacro"),
            mappings={
                "use_sim": "false",
                "use_mock_hardware": "true",
            }
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    common_parameters = [
        moveit_config.to_dict(),
        {"planning_group": "arm"},
        {"home_target": "home"},
        {"base_frame": "world"},
    ]
    gripper_parameters = [
        moveit_config.to_dict(),
        {"planning_group": "gripper"},
        {"base_frame": "link_6"},
    ]

    enable_drl_backend_arg = DeclareLaunchArgument(
        "enable_drl_backend",
        default_value="true",
        description=(
            "Launch robot_drl/drl_unified_planner_node for /move_pose_rl and "
            "/drl_pickplace. Set false when an external DRL backend is already running."
        ),
    )
    planner_node_name_arg = DeclareLaunchArgument(
        "planner_node_name",
        default_value="/drl_unified_planner_node",
        description="Planner node name used for the DRL parameter service.",
    )
    drl_calibrated_start_tcp_arg = DeclareLaunchArgument(
        "drl_calibrated_start_tcp_base",
        default_value="[0.375, 0.000, 0.250]",
        description="Default DRL start TCP in base_link frame [x, y, z].",
    )

    drl_backend = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("enable_drl_backend")),
        parameters=[{
            "use_sim_time": False,
            "calibrated_start_tcp_base": LaunchConfiguration("drl_calibrated_start_tcp_base"),
            "input_mode": "manual",
            "auto_plan_on_start": False,
            "manual_prompt_on_start": False,
            "auto_execute_after_plan": False,
            "manual_default_obstacle_size": [0.0, 0.0, 0.0],
            "workspace_min_base": [0.250, -0.150, 0.020],
            "workspace_max_base": [0.500, 0.150, 0.450],
            "use_current_tcp_orientation_for_execution": True,
            "preposition_before_plan": False,
            "update_start_tcp_from_tf_before_plan": True,
            "fallback_to_final_pose_on_execute_failure": False,
            "execute_final_pose_only": False,
            "use_planning_scene_obstacles": False,
            "planning_scene_service_name": "/get_planning_scene",
            "planning_scene_frame": "base_link",
            "validate_path_with_moveit_ik": False,
            "path_collision_check_step_m": 0.01,
            "path_collision_clearance_margin_m": 0.05,
            "obstacle_safety_filter_enabled": True,
            "obstacle_safety_margin_m": 0.05,
            "obstacle_safety_check_step_m": 0.005,
            "execute_collision_check_step_m": 0.01,
            "execute_collision_clearance_margin_m": 0.05,
        }],
    )

    gohome_server = Node(
        package="robot_task_manager",                          
        executable="gohome_server",
        name="gohome_server",
        output="screen",
        parameters=common_parameters,
    )

    move_to_pose_server = Node(
        package="robot_task_manager",
        executable="move_to_pose_server",
        name="move_to_pose_server",
        output="screen",
        parameters=common_parameters,
    )

    move_pose_cartesian_server = Node(
        package="robot_task_manager",
        executable="move_pose_cartesian_server",
        name="move_pose_cartesian_server",
        output="screen",
        parameters=common_parameters,
    )

    checker_board = Node(
        package="robot_task_manager",
        executable="checker_board_server",
        name="checker_board_server",
        output="screen",
        parameters=common_parameters + [{
            "measurement_settle_time_s": 2.0,
        }],
    )

    mode_gripper = Node(
        package="robot_task_manager",
        executable="move_gripper_server",
        name="move_gripper_server",
        output="screen",
        parameters=gripper_parameters,
    )

    pickplace = Node(
        package="robot_task_manager",
        executable="pickplace_server",
        name="pickplace_action_server",
        output="screen",
        parameters=common_parameters,
    )

    drl_pickplace = Node(
        package="robot_task_manager",
        executable="drl_pickplace_server",
        name="drl_pickplace_action_server",
        output="screen",
        parameters=common_parameters + [{
            "planning_frame": "base_link",
            "ee_link": "tcp_link",
            "planner_node_name": LaunchConfiguration("planner_node_name"),
        }],
    )

    move_pose_rl = Node(
        package="robot_task_manager",
        executable="move_pose_rl_server",
        name="move_pose_rl_action_server",
        output="screen",
        parameters=common_parameters + [{
            "planning_frame": "base_link",
            "ee_link": "tcp_link",
            "position_tolerance_m": 0.01,
            "orientation_tolerance_rad": 0.10,
            "drl_timeout_sec": 120.0,
            "drl_trajectory_endpoint_tolerance_m": 0.015,
            "drl_plan_attempts": 3,
            "tf_timeout_sec": 2.0,
            "sub_action_timeout_sec": 60.0,
            "planner_node_name": LaunchConfiguration("planner_node_name"),
        }],
    )

    repeatability_test = Node(
        package="robot_task_manager",
        executable="repeatability_test_server",
        name="repeatability_test_action_server",
        output="screen",
        parameters=common_parameters + [{
            "fast_velocity_scale": 0.7,
        }],
    )

    return LaunchDescription([
        enable_drl_backend_arg,
        planner_node_name_arg,
        drl_calibrated_start_tcp_arg,
        LogInfo(
            msg=[
                "[task_servers] Starting action servers; enable_drl_backend=",
                LaunchConfiguration("enable_drl_backend"),
            ]
        ),
        drl_backend,
        gohome_server,
        move_to_pose_server,
        move_pose_cartesian_server,
        checker_board,
        mode_gripper,
        pickplace,
        drl_pickplace,
        move_pose_rl,
        repeatability_test,
    ])
