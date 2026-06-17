"""Unified launch: Gazebo sim + MoveIt + Task Executor + DRL Planner.

Full robot simulation stack for DRL-based pick-and-place:
  1. Gazebo world + robot spawn + bridges + gz_ros2_control
  2. MoveIt (move_group) — waits 4 s for Gazebo to initialise
  3. Task executor (MoveIt client) — provides /move_cartesian_pose_sequence
  4. DRL unified planner node — DRL inference + trajectory execution via task executor
  5. RViz — trajectory visualisation

Note: No separate controller spawners are needed.
The gz_ros2_control plugin in Gazebo provides the /controller_manager service
internally. The DRL node uses MoveIt's move_group for planning and execution
via the /move_cartesian_pose_sequence service, which does not require
external controller spawners.

Usage::

    # Full stack with auto-plan and auto-execute
    ros2 launch robot_bringup drl_test.launch.py \\
        auto_plan_on_start:=true \\
        manual_prompt_on_start:=false \\
        auto_execute_after_plan:=true

    # Manual plan + execute
    ros2 launch robot_bringup drl_test.launch.py

    # Custom start TCP
    ros2 launch robot_bringup drl_test.launch.py \\
        calibrated_start_tcp_base:="[0.5241, 0.000, 0.315]"

Arguments:
  auto_plan_on_start          — plan automatically on startup (default: true)
  manual_prompt_on_start    — prompt terminal for target input (default: false)
  auto_execute_after_plan    — execute trajectory after planning (default: false)
  use_rviz                — launch RViz for trajectory visualisation (default: true)
  calibrated_start_tcp_base — start TCP in BASE frame [x, y, z] metres
"""

import os

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


def generate_launch_description():
    pkg_drl = "robot_drl"

    share_desc = get_package_share_directory("robot_description")
    share_drl = get_package_share_directory(pkg_drl)

    # ── Arguments ────────────────────────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            "auto_plan_on_start",
            default_value="true",
            description="Plan automatically on startup",
        ),
        DeclareLaunchArgument(
            "manual_prompt_on_start",
            default_value="false",
            description="Prompt terminal for target input on startup",
        ),
        DeclareLaunchArgument(
            "auto_execute_after_plan",
            default_value="false",
            description="Execute the planned trajectory after planning",
        ),
        DeclareLaunchArgument(
            "calibrated_start_tcp_base",
            default_value="[0.5241, 0.000, 0.315]",
            description="Start TCP in BASE frame [x, y, z] (metres)",
        ),
    ]

    auto_plan     = LaunchConfiguration("auto_plan_on_start")
    manual_prompt = LaunchConfiguration("manual_prompt_on_start")
    auto_exec     = LaunchConfiguration("auto_execute_after_plan")
    start_tcp     = LaunchConfiguration("calibrated_start_tcp_base")

    # ── Step 1: Gazebo (gz_ros2_control plugin provides /controller_manager) ──
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([share_desc, "/launch/gazebo.launch.py"])
    )

    # ── Step 2: MoveIt (start_controller_manager=false to avoid spawner conflicts)
    # gz_ros2_control in Gazebo provides the /controller_manager service.
    # Setting start_controller_manager=false prevents MoveIt's controllers.launch.py
    # from starting its own spawners which would fail trying to reach /controller_manager.
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("robot_moveit"),
            "/launch/moveit.launch.py",
        ]),
        launch_arguments={
            "use_sim_time": "true",
            "use_mock": "true",
            "start_controller_manager": "false",
        }.items(),
    )

    # ── Step 3: Task executor (provides /move_cartesian_pose_sequence)
    # This is the C++ node from robot_task_executor package.
    # It connects to move_group and provides MoveIt's Cartesian pose execution.
    task_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("robot_task_executor"),
            "/launch/task_executor.launch.py",
        ]),
        launch_arguments={
            "use_sim_time": "true",
            "base_frame": "base_link",
        }.items(),
    )

    # ── Step 4: DRL planner node ───────────────────────────────────────────
    drl_node = Node(
        package=pkg_drl,
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "calibrated_start_tcp_base": start_tcp,
            "input_mode": "manual",
            "auto_plan_on_start": auto_plan,
            "manual_prompt_on_start": manual_prompt,
            "auto_execute_after_plan": auto_exec,
        }],
    )

    # ── Step 5: RViz ────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_drl",
        output="screen",
        arguments=["-d", os.path.join(share_drl, "rviz", "drl_markers.rviz")],
    )

    return LaunchDescription([
        *args,

        # Step 1: Gazebo immediately
        LogInfo(msg="[drl_test] === Unified DRL Robot Stack ==="),
        LogInfo(msg="[drl_test] Step 1: Gazebo + gz_ros2_control + bridges..."),
        gazebo,

        # Step 2: MoveIt (after 5 s — needs time for gz_ros2_control to init)
        TimerAction(period=5.0, actions=[
            LogInfo(msg="[drl_test] Step 2: MoveIt (start_controller_manager=false)..."),
            moveit,
        ]),

        # Step 3: Task executor (after 9 s — needs time for move_group to be ready)
        TimerAction(period=9.0, actions=[
            LogInfo(msg="[drl_test] Step 3: Task executor (/move_cartesian_pose_sequence)..."),
            task_executor,
        ]),

        # Step 4: DRL planner (after 13 s)
        TimerAction(period=13.0, actions=[
            LogInfo(msg="[drl_test] Step 4: DRL unified planner..."),
            drl_node,
        ]),

        # Step 5: RViz (after 15 s)
        TimerAction(period=15.0, actions=[
            LogInfo(msg="[drl_test] Step 5: RViz..."),
            #rviz_node,
        ]),
    ])
