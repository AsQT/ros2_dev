import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

RL_PYTHON = "/home/minhquang/venvs/ros_rl/bin/python3"


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
        {
            "enable_executor_logging": LaunchConfiguration("enable_executor_logging"),
            "runtime_mode": LaunchConfiguration("runtime_mode"),
            "enable_debug_logging": LaunchConfiguration("enable_debug_logging"),
            "enable_standard_logging": LaunchConfiguration("enable_standard_logging"),
            "log_root_dir": LaunchConfiguration("log_root_dir"),
            "executor_log_dir": LaunchConfiguration("executor_log_dir"),
            "executor_sample_rate_hz": LaunchConfiguration("executor_sample_rate_hz"),
            "executor_base_frame": LaunchConfiguration("executor_base_frame"),
            "executor_tcp_frame": LaunchConfiguration("executor_tcp_frame"),
        },
    ]
    gripper_parameters = [
        moveit_config.to_dict(),
        {"planning_group": "gripper"},
        {"base_frame": "link_6"},
        {
            "runtime_mode": LaunchConfiguration("runtime_mode"),
            "enable_standard_logging": LaunchConfiguration("enable_standard_logging"),
            "log_root_dir": LaunchConfiguration("log_root_dir"),
        },
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
    enable_executor_logging_arg = DeclareLaunchArgument(
        "enable_executor_logging",
        default_value="false",
        description="Enable CSV executor experiment logging in MoveItExecutor-based servers.",
    )
    runtime_mode_arg = DeclareLaunchArgument(
        "runtime_mode",
        default_value="mock",
        description="Runtime log branch: mock or real.",
    )
    enable_standard_logging_arg = DeclareLaunchArgument(
        "enable_standard_logging",
        default_value="true",
        description="Enable standard summary/events/metadata logging for summary-only actions.",
    )
    enable_debug_logging_arg = DeclareLaunchArgument(
        "enable_debug_logging",
        default_value="false",
        description="Enable standalone debug logs for child/helper actions.",
    )
    log_root_dir_arg = DeclareLaunchArgument(
        "log_root_dir",
        default_value="/home/minhquang/ros2_dev/Log_robot_data",
        description="Unified root directory for robot log data.",
    )
    executor_log_dir_arg = DeclareLaunchArgument(
        "executor_log_dir",
        default_value="/home/minhquang/ros2_dev/Log_robot_data/mock/baseline/executor_internal",
        description="Directory for executor experiment logs.",
    )
    executor_sample_rate_hz_arg = DeclareLaunchArgument(
        "executor_sample_rate_hz",
        default_value="50.0",
        description="Sampling rate for executor actual data.",
    )
    executor_base_frame_arg = DeclareLaunchArgument(
        "executor_base_frame",
        default_value="base_link",
        description="Base frame for executor TCP TF lookup.",
    )
    executor_tcp_frame_arg = DeclareLaunchArgument(
        "executor_tcp_frame",
        default_value="tcp_link",
        description="TCP frame for executor TCP TF lookup.",
    )

    drl_backend = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        prefix=RL_PYTHON,
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

    gohome2_server = Node(
        package="robot_task_manager",
        executable="gohome_server",
        name="gohome2_server",
        output="screen",
        parameters=common_parameters + [{
            "home_target": "home_2",
            "action_name": "gohome_2",
        }],
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
            # codex.md: PickPlaceRL plans PLAN_TO_PRE_PICK from the current TCP; do
            # NOT drive the arm to the fixed preposition_tcp_base first.
            "use_preposition_before_pre_pick": False,
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

    move_target_rl = Node(
        package="robot_task_manager",
        executable="move_target_rl_server",
        name="move_target_rl_action_server",
        output="screen",
        parameters=common_parameters + [{
            "planning_frame": "base_link",
            "ee_link": "tcp_link",
            "target_class": "wood",
            "obstacle_class": "box",
            "vision_timeout_sec": 1.0,
            "default_box_size": [0.10, 0.10, 0.10],
            "target_position_tolerance_m": 0.02,
            "wood_objects_topic": "/vision/wood_objects",
            "box_objects_topic": "/vision/box_objects",
            "position_tolerance_m": 0.01,
            "drl_timeout_sec": 120.0,
            "drl_trajectory_endpoint_tolerance_m": 0.015,
            "drl_plan_attempts": 3,
            "tf_timeout_sec": 2.0,
            "sub_action_timeout_sec": 60.0,
            "drl_planner_node_name": LaunchConfiguration("planner_node_name"),
        }],
    )

    move_to_pose_obstacle = Node(
        package="robot_task_manager",
        executable="move_to_pose_obstacle_server",
        name="move_to_pose_obstacle_action_server",
        output="screen",
        parameters=common_parameters + [{
            "obstacle_frame": "base_link",
            "ee_link": "tcp_link",
            "obstacle_class": "box",
            "vision_timeout_sec": 1.0,
            "box_objects_topic": "/vision/box_objects",
            "tf_timeout_sec": 2.0,
            "collision_object_id": "move_to_pose_obstacle_box",
        }],
    )

    repeatability_test = Node(
        package="robot_task_manager",
        executable="repeatability_test_server",
        name="repeatability_test_action_server",
        output="screen",
        parameters=common_parameters + [{
            "fast_velocity_scale": 0.1,
        }],
    )

    return LaunchDescription([
        enable_drl_backend_arg,
        planner_node_name_arg,
        drl_calibrated_start_tcp_arg,
        enable_executor_logging_arg,
        runtime_mode_arg,
        enable_standard_logging_arg,
        enable_debug_logging_arg,
        log_root_dir_arg,
        executor_log_dir_arg,
        executor_sample_rate_hz_arg,
        executor_base_frame_arg,
        executor_tcp_frame_arg,
        LogInfo(
            msg=[
                "[task_servers] Starting action servers; enable_drl_backend=",
                LaunchConfiguration("enable_drl_backend"),
            ]
        ),
        drl_backend,
        gohome_server,
        gohome2_server,
        move_to_pose_server,
        move_pose_cartesian_server,
        checker_board,
        mode_gripper,
        pickplace,
        drl_pickplace,
        move_pose_rl,
        move_target_rl,
        move_to_pose_obstacle,
        repeatability_test,
    ])
