"""Launch robot simulation, MoveIt, DRL planner, task executor, and RViz."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup_share = get_package_share_directory("robot_bringup")
    task_executor_share = get_package_share_directory("robot_task_executor")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo /clock time.",
    )
    auto_plan_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Plan with the default manual target after startup.",
    )
    auto_execute_arg = DeclareLaunchArgument(
        "auto_execute_after_plan",
        default_value="true",
        description="Execute the planned forward trajectory after planning.",
    )
    manual_prompt_arg = DeclareLaunchArgument(
        "manual_prompt_on_start",
        default_value="false",
        description="Prompt in terminal for manual target instead of using defaults.",
    )
    calibrated_start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.5241, 0.000, 0.315]",
        description="Start TCP in base_link frame [x, y, z].",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_share, "/launch/sim.launch.py"])
    )

    task_executor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [task_executor_share, "/launch/task_executor.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "base_frame": "base_link",
            "ee_link": "tcp_link",
        }.items(),
    )

    drl_node = Node(
        package="robot_drl",
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "input_mode": "manual",
            "auto_plan_on_start": LaunchConfiguration("auto_plan_on_start"),
            "auto_execute_after_plan": LaunchConfiguration("auto_execute_after_plan"),
            "manual_prompt_on_start": LaunchConfiguration("manual_prompt_on_start"),
            "calibrated_start_tcp_base": LaunchConfiguration(
                "calibrated_start_tcp_base"
            ),
        }],
    )

    move_to_start_pose = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/move_to_named_pose_target",
            "robot_task_executor_msgs/srv/MoveToNamedPoseTarget",
            "{target_name: pose_A, execute: true}",
        ],
        output="screen",
    )
    start_drl_after_start_pose = RegisterEventHandler(
        OnProcessExit(
            target_action=move_to_start_pose,
            on_exit=[
                LogInfo(msg="[rl_sim_rviz] Start pose reached; starting DRL planner..."),
                drl_node,
            ],
        )
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory("robot_drl"),
            "/launch/rviz_drl.launch.py",
        ])
    )

    return LaunchDescription([
        use_sim_time_arg,
        auto_plan_arg,
        auto_execute_arg,
        manual_prompt_arg,
        calibrated_start_tcp_arg,
        LogInfo(msg="[rl_sim_rviz] Starting Gazebo robot sim + MoveIt..."),
        bringup,
        TimerAction(
            period=7.0,
            actions=[
                LogInfo(msg="[rl_sim_rviz] Starting robot_task_executor..."),
                task_executor,
            ],
        ),
        TimerAction(
            period=16.0,
            actions=[
                LogInfo(msg="[rl_sim_rviz] Moving robot to DRL start pose pose_A..."),
                move_to_start_pose,
            ],
        ),
        start_drl_after_start_pose,
        TimerAction(
            period=14.0,
            actions=[
                LogInfo(msg="[rl_sim_rviz] Opening RViz..."),
                rviz,
            ],
        ),
    ])
