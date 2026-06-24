import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_gazebo_arg = DeclareLaunchArgument(
        "use_gazebo",
        default_value="false",
        description="Use Gazebo simulation stack instead of the default mock hardware stack",
    )
    number_of_trials_arg = DeclareLaunchArgument("number_of_trials", default_value="20")
    random_seed_arg = DeclareLaunchArgument("random_seed", default_value="0")
    gripper_close_width_arg = DeclareLaunchArgument(
        "gripper_close_width_m",
        default_value="0.028",
    )

    drl_mock_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_drl"),
                "launch",
                "drl_mock_hw.launch.py",
            )
        ),
        launch_arguments={
            "input_mode": "manual",
            "auto_plan_on_start": "false",
            "manual_prompt_on_start": "false",
            "auto_execute_after_plan": "false",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("use_gazebo")),
    )

    drl_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_drl"),
                "launch",
                "drl_gazebo.launch.py",
            )
        ),
        launch_arguments={
            "input_mode": "manual",
            "auto_plan_on_start": "false",
            "manual_prompt_on_start": "false",
            "auto_execute_after_plan": "false",
            "use_rviz": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    task_servers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_task_manager"),
                "launch",
                "task_servers.launch.py",
            )
        )
    )

    mock_random_client = Node(
        package="robot_task_manager",
        executable="drl_pick_place_random_test_client.py",
        name="drl_pick_place_random_test_client",
        output="screen",
        parameters=[{
            "number_of_trials": LaunchConfiguration("number_of_trials"),
            "random_seed": LaunchConfiguration("random_seed"),
            "gripper_close_width_m": LaunchConfiguration("gripper_close_width_m"),
        }],
        condition=UnlessCondition(LaunchConfiguration("use_gazebo")),
    )

    gazebo_random_client = Node(
        package="robot_task_manager",
        executable="drl_pick_place_random_test_client.py",
        name="drl_pick_place_random_test_client",
        output="screen",
        parameters=[{
            "number_of_trials": LaunchConfiguration("number_of_trials"),
            "random_seed": LaunchConfiguration("random_seed"),
            "gripper_close_width_m": LaunchConfiguration("gripper_close_width_m"),
            "workspace_min": [0.32, -0.10, 0.10],
            "workspace_max": [0.46, 0.10, 0.18],
            "start_min": [0.35, -0.06, 0.22],
            "start_max": [0.40, 0.06, 0.27],
            "goal_timeout_sec": 420.0,
            "setup_timeout_sec": 90.0,
            "action_server_timeout_sec": 120.0,
            "wait_for_joint_states": True,
            "joint_states_timeout_sec": 90.0,
        }],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    return LaunchDescription([
        use_gazebo_arg,
        number_of_trials_arg,
        random_seed_arg,
        gripper_close_width_arg,
        LogInfo(
            msg="[drl_pick_place_random_test] starting mock HW + DRL stack",
            condition=UnlessCondition(LaunchConfiguration("use_gazebo")),
        ),
        LogInfo(
            msg="[drl_pick_place_random_test] starting Gazebo + DRL stack",
            condition=IfCondition(LaunchConfiguration("use_gazebo")),
        ),
        drl_mock_hw,
        drl_gazebo,
        TimerAction(period=10.0, actions=[
            LogInfo(msg="[drl_pick_place_random_test] starting task action servers"),
            task_servers,
        ], condition=UnlessCondition(LaunchConfiguration("use_gazebo"))),
        TimerAction(period=22.0, actions=[
            LogInfo(msg="[drl_pick_place_random_test] starting random action client"),
            mock_random_client,
        ], condition=UnlessCondition(LaunchConfiguration("use_gazebo"))),
        TimerAction(period=45.0, actions=[
            LogInfo(msg="[drl_pick_place_random_test] starting Gazebo random action client"),
            gazebo_random_client,
        ], condition=IfCondition(LaunchConfiguration("use_gazebo"))),
    ])
