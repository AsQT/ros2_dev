"""Mock-only launch for the robot_drl DRL planner stack.

Runs ONLY mock_hardware-compatible nodes — NO Gazebo, NO MoveIt, NO robot_bringup.

Launches:
  1. mock_environment_node   — synthetic target pose / box detection at 10 Hz
  2. drl_unified_planner_node — DRL planning (manual or vision mode)

Intended for testing DRL model inference and trajectory visualization
without any hardware or physics simulation.

Usage::

    # Manual mode (terminal prompt for target)
    ros2 launch robot_drl mock_drl.launch.py

    # Vision mode (reads from mock_environment_node)
    ros2 launch robot_drl mock_drl.launch.py input_mode:=vision

    # Auto-plan with defaults (no terminal prompt)
    ros2 launch robot_drl mock_drl.launch.py \\
        input_mode:=manual \\
        auto_plan_on_start:=true \\
        manual_prompt_on_start:=false

    # Custom target position
    ros2 launch robot_drl mock_drl.launch.py \\
        target_x:=0.575 target_y:=0.050 target_z:=0.120

Arguments:
  input_mode                  — "manual" (terminal) or "vision" (default: manual)
  auto_plan_on_start         — auto-plan on startup (default: true)
  manual_prompt_on_start     — prompt terminal on startup (default: true)
  auto_execute_after_plan    — auto-execute after planning (default: false)
  use_sim_time               — use simulation time (default: false)
  calibrated_start_tcp_base  — start TCP in BASE frame [x, y, z] in metres
  target_x/y/z              — mock target position in world frame
  target_class_name          — "box" or "target" (default: box)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_drl = "robot_drl"

    share_drl = get_package_share_directory(pkg_drl)

    # --- Launch arguments ---
    input_mode_arg = DeclareLaunchArgument(
        "input_mode",
        default_value="manual",
        description="Input mode: 'manual' (terminal prompt) or 'vision' (from mock_environment)",
    )
    auto_plan_on_start_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Automatically plan on startup",
    )
    manual_prompt_on_start_arg = DeclareLaunchArgument(
        "manual_prompt_on_start",
        default_value="true",
        description="Prompt for manual target on startup. Set false to use default parameters.",
    )
    auto_execute_arg = DeclareLaunchArgument(
        "auto_execute_after_plan",
        default_value="false",
        description="Execute the planned trajectory after planning.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (set true if running with sim clock)",
    )
    calibrated_start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.5241, 0.000, 0.315]",
        description="Calibrated start TCP in BASE frame [x, y, z] in metres",
    )

    # Mock environment args
    target_x_arg = DeclareLaunchArgument(
        "target_x",
        default_value="0.575",
        description="Mock target X in world frame (metres)",
    )
    target_y_arg = DeclareLaunchArgument(
        "target_y",
        default_value="0.050",
        description="Mock target Y in world frame (metres)",
    )
    target_z_arg = DeclareLaunchArgument(
        "target_z",
        default_value="0.120",
        description="Mock target Z in world frame (metres)",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class_name",
        default_value="box",
        description="Mock object class: 'box' or 'target'",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    # --- Node 1: mock_environment_node ---
    mock_env_node = Node(
        package=pkg_drl,
        executable="mock_environment_node",
        name="mock_environment_node",
        output="screen",
        parameters=[{
            "publish_rate_hz": 10.0,
            "target_class_name": LaunchConfiguration("target_class_name"),
            "target_x": LaunchConfiguration("target_x"),
            "target_y": LaunchConfiguration("target_y"),
            "target_z": LaunchConfiguration("target_z"),
            "frame_id": "world",
            "distance_m": 0.5,
            "confidence": 0.95,
        }],
    )

    # --- Node 2: drl_unified_planner_node ---
    planner_node = Node(
        package=pkg_drl,
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "calibrated_start_tcp_base": LaunchConfiguration("calibrated_start_tcp_base"),
            "input_mode": LaunchConfiguration("input_mode"),
            "auto_plan_on_start": LaunchConfiguration("auto_plan_on_start"),
            "auto_execute_after_plan": LaunchConfiguration("auto_execute_after_plan"),
            "manual_prompt_on_start": LaunchConfiguration("manual_prompt_on_start"),
        }],
    )

    # Delay planner 3s so mock environment is publishing first
    delayed_planner = TimerAction(
        period=3.0,
        actions=[planner_node],
    )

    return LaunchDescription([
        LogInfo(msg="[mock_drl] robot_drl mock-only launch (NO Gazebo, NO robot_bringup)"),
        LogInfo(msg="[mock_drl] Launching mock_environment + drl_unified_planner_node..."),

        # Args
        input_mode_arg,
        auto_plan_on_start_arg,
        manual_prompt_on_start_arg,
        auto_execute_arg,
        use_sim_time_arg,
        calibrated_start_tcp_arg,
        target_x_arg,
        target_y_arg,
        target_z_arg,
        target_class_arg,

        # Nodes
        mock_env_node,
        delayed_planner,
    ])
