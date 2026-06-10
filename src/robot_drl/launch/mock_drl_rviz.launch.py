"""Launch file: robot_drl mock stack + RViz with DRL trajectory display.

Launches everything in one command:
  1. mock_environment_node        — synthetic target / box detection at 10 Hz
  2. drl_unified_planner_node    — DRL planning
  3. RViz2                       — visualize trajectory markers

Usage::

    # Full stack + RViz (recommended)
    ros2 launch robot_drl mock_drl_rviz.launch.py

    # With custom target
    ros2 launch robot_drl mock_drl_rviz.launch.py \
        target_x:=0.600 target_y:=-0.100 target_z:=0.200

    # Vision mode
    ros2 launch robot_drl mock_drl_rviz.launch.py input_mode:=vision

Arguments (all inherited from mock_drl.launch.py):
  input_mode, auto_plan_on_start, manual_prompt_on_start,
  auto_execute_after_plan, use_sim_time, calibrated_start_tcp_base,
  target_x, target_y, target_z, target_class_name
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_drl = "robot_drl"

    # --- Shared arguments (mirrored from mock_drl.launch.py) ---
    input_mode_arg = DeclareLaunchArgument(
        "input_mode",
        default_value="manual",
        description="Input mode: 'manual' (terminal) or 'vision' (from mock_environment)",
    )
    auto_plan_on_start_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Automatically plan on startup",
    )
    manual_prompt_on_start_arg = DeclareLaunchArgument(
        "manual_prompt_on_start",
        default_value="true",
        description="Prompt for manual target on startup",
    )
    auto_execute_arg = DeclareLaunchArgument(
        "auto_execute_after_plan",
        default_value="false",
        description="Execute trajectory after planning",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time",
    )
    calibrated_start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.5241, 0.000, 0.315]",
        description="Calibrated start TCP in BASE frame [x, y, z] (metres)",
    )
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

    delayed_planner = TimerAction(
        period=3.0,
        actions=[planner_node],
    )

    # --- Node 3: RViz2 ---
    share_drl = get_package_share_directory(pkg_drl)
    rviz_config_candidates = [
        os.path.join(share_drl, "rviz", "drl_markers.rviz"),
        os.path.join(share_drl, "rviz", "DRL_Rviz.rviz"),
    ]
    rviz_config = None
    for candidate in rviz_config_candidates:
        if os.path.exists(candidate):
            rviz_config = candidate
            break

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if rviz_config else [],
    )

    # Delay RViz so trajectory is already published when it opens
    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node],
    )

    return LaunchDescription([
        LogInfo(msg="[mock_drl_rviz] robot_drl mock stack + RViz2"),
        LogInfo(msg="[mock_drl_rviz] NO Gazebo, NO robot_bringup"),
        LogInfo(
            msg="[mock_drl_rviz] Starting: mock_environment + drl_unified_planner + RViz2..."
        ),

        # Arguments
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
        delayed_rviz,
    ])
