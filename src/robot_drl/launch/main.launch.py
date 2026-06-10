"""Top-level launch file for the unified DRL planner stack.

Orchestrates all layers in the correct startup order:
  1. Robot bringup  (sim via robot_bringup)
  2. Vision         (mock via mock_environment_node, or real via robot_vision_pipeline)
  3. DRL planner    (drl_unified_planner_node — handles planning and execution)

Usage:
  # Manual mode (terminal prompt)
  ros2 launch robot_drl main.launch.py

  # Vision mode (live camera detections)
  ros2 launch robot_drl main.launch.py input_mode:=vision

  # With mock vision only (no camera needed)
  ros2 launch robot_drl main.launch.py vision:=mock

  # Custom start position
  ros2 launch robot_drl main.launch.py \\
      calibrated_start_tcp_base:="[0.5241, 0.000, 0.315]"

Arguments:
  input_mode                — "manual" (terminal prompt) or "vision" (live detections)
  auto_plan_on_start       — auto-plan on startup in manual mode (default: true)
  vision                   — "mock" (synthetic data) or "real" (robot_vision_pipeline)
  use_sim_time             — use /clock for simulation time (default: true)
  calibrated_start_tcp_base — start TCP in BASE frame [x, y, z] in metres
  start_scene_visualization — launch scene_visualization_node (default: true)
  show_table               — show table marker in RViz (default: true)
"""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = "robot_bringup"
    pkg_drl = "robot_drl"
    pkg_vision = "robot_vision_pipeline"

    share_bringup = get_package_share_directory(pkg_bringup)
    share_drl = get_package_share_directory(pkg_drl)
    share_vision = get_package_share_directory(pkg_vision)

    # Launch arguments
    input_mode_arg = DeclareLaunchArgument(
        "input_mode",
        default_value="manual",
        description="Input mode: 'manual' (terminal prompt) or 'vision' (live detections)",
    )
    auto_plan_on_start_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Automatically plan on startup (manual mode only)",
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
    vision_arg = DeclareLaunchArgument(
        "vision",
        default_value="mock",
        description="'mock' (synthetic data) or 'real' (robot_vision_pipeline)",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock for simulation time",
    )
    calibrated_start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.5241, 0.000, 0.315]",
        description="Calibrated start TCP in BASE frame [x, y, z] in metres",
    )
    start_scene_viz_arg = DeclareLaunchArgument(
        "start_scene_visualization",
        default_value="true",
        description="Launch scene_visualization_node for centralized /scene/markers",
    )
    show_table_arg = DeclareLaunchArgument(
        "show_table",
        default_value="true",
        description="Show table marker in RViz",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    vision = LaunchConfiguration("vision")

    # Step 1: Robot bringup (sim)
    sim_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [share_bringup, "/launch/sim.launch.py"]
        )
    )

    # Step 2: Vision layer
    mock_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [share_drl, "/launch/mock_environment.launch.py"]
        ),
        launch_arguments={
            "target_x": "0.575",
            "target_y": "0.050",
            "target_z": "0.120",
            "frame_id": "world",
        }.items(),
        condition=IfCondition(
            ['"', vision, '" == "mock"']
        ),
    )
    real_vision = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [share_vision, "/launch/vision_full_pipeline.launch.py"]
        ),
        condition=IfCondition(
            ['"', vision, '" == "real"']
        ),
    )

    # Step 3: Unified DRL planner node
    unified_node = Node(
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

    # Delay planner until bringup is ready (~8 s)
    delayed_unified = TimerAction(
        period=8.0,
        actions=[unified_node],
    )

    return LaunchDescription([
        LogInfo(msg="[main] robot DRL unified planner stack"),
        LogInfo(msg="[main] input_mode=$(var input_mode) | vision=$(var vision) | use_sim_time=$(var use_sim_time)"),

        input_mode_arg,
        auto_plan_on_start_arg,
        auto_execute_arg,
        manual_prompt_on_start_arg,
        vision_arg,
        use_sim_time_arg,
        calibrated_start_tcp_arg,
        start_scene_viz_arg,
        show_table_arg,

        # Step 1: Robot bringup
        LogInfo(msg="[main] Step 1: Robot bringup (sim)..."),
        sim_bringup,

        # Step 2: Vision (delayed 3s so bringup is up first)
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg="[main] Step 2: Vision layer..."),
                mock_vision,
                real_vision,
            ],
        ),

        # Step 3: DRL unified planner (delayed 8s)
        LogInfo(msg="[main] Step 3: DRL unified planner..."),
        delayed_unified,
    ])
