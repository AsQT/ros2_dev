"""Launch file for the unified DRL planner node.

Launches the robot bringup (Gazebo + MoveIt + RViz) and starts the
drl_unified_planner_node which supports both manual and vision input modes.

Usage::

    # Manual mode (default)
    ros2 launch robot_drl drl_unified_planner.launch.py

    # Vision mode
    ros2 launch robot_drl drl_unified_planner.launch.py \\
        input_mode:=vision auto_plan_on_start:=false

    # Override defaults
    ros2 launch robot_drl drl_unified_planner.launch.py \\
        input_mode:=manual auto_plan_on_start:=true \\
        calibrated_start_tcp_base:="[0.5241, 0.000, 0.315]"

Arguments:
  input_mode                  — "manual" or "vision" (default: manual)
  auto_plan_on_start          — auto-plan on startup in manual mode (default: true)
  model                       — DRL model filename (default: run/model/best_model.zip)
  use_sim_time               — use /clock for simulation time (default: true)
  calibrated_start_tcp_base   — start TCP in BASE frame [x, y, z] in metres
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_pkg = "robot_bringup"
    bringup_share = get_package_share_directory(bringup_pkg)

    # --- Launch arguments ---
    input_mode_arg = DeclareLaunchArgument(
        "input_mode",
        default_value="manual",
        description="Input mode: 'manual' (terminal prompt) or 'vision' (from /vision/* topics)",
    )
    auto_plan_on_start_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Automatically prompt and plan on startup (manual mode only)",
    )
    manual_prompt_on_start_arg = DeclareLaunchArgument(
        "manual_prompt_on_start",
        default_value="true",
        description="Prompt for manual target on startup. Set false to use default parameters.",
    )
    auto_execute_arg = DeclareLaunchArgument(
        "auto_execute_after_plan",
        default_value="false",
        description="Execute the planned forward trajectory after planning.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock for simulation time",
    )
    calibrated_start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.5241, 0.000, 0.315]",
        description="Calibrated start TCP in BASE frame [x, y, z] in metres.",
    )

    # --- Include robot_bringup sim launch (robot + MoveIt + task servers) ---
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [bringup_share, "/launch/sim.launch.py"]
        )
    )

    # --- Unified planner node ---
    # Use string paths with substitution syntax so LaunchConfiguration is resolved
    # at launch time rather than at import/os.path.join time.
    unified_node = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "calibrated_start_tcp_base": LaunchConfiguration(
                    "calibrated_start_tcp_base"
                ),
                "input_mode": LaunchConfiguration("input_mode"),
                "auto_plan_on_start": LaunchConfiguration("auto_plan_on_start"),
                "auto_execute_after_plan": LaunchConfiguration("auto_execute_after_plan"),
                "manual_prompt_on_start": LaunchConfiguration("manual_prompt_on_start"),
                "use_planning_scene_obstacles": True,
                "planning_scene_service_name": "/get_planning_scene",
                "planning_scene_frame": "base_link",
                "path_collision_check_step_m": 0.01,
                "path_collision_clearance_margin_m": 0.05,
                "obstacle_safety_filter_enabled": True,
                "obstacle_safety_margin_m": 0.05,
                "obstacle_safety_check_step_m": 0.005,
                "fallback_to_final_pose_on_execute_failure": False,
                "execute_final_pose_only": False,
                "execute_collision_check_step_m": 0.01,
                "execute_collision_clearance_margin_m": 0.05,
            },
        ],
    )

    # Delay the unified node until bringup is ready (~10 s after launch starts)
    delayed_unified_node = TimerAction(
        period=10.0, actions=[unified_node]
    )

    return LaunchDescription([
        LogInfo(
            msg="[drl_unified] Launching robot + RViz + MoveIt + Unified DRL Planner..."
        ),
        LogInfo(
            msg="[drl_unified] Start TCP: calibrated_start_tcp_base=[0.5241, 0.000, 0.315] "
                "(calibrated, RPY=[pi,0,0])"
        ),
        LogInfo(
            msg="[drl_unified] Mode: input_mode=$(var input_mode) | "
                "auto_plan_on_start=$(var auto_plan_on_start)"
        ),
        input_mode_arg,
        auto_plan_on_start_arg,
        auto_execute_arg,
        manual_prompt_on_start_arg,
        use_sim_time_arg,
        calibrated_start_tcp_arg,
        sim_bringup,
        delayed_unified_node,
    ])
