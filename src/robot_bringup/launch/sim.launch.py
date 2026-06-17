import os

from launch                             import LaunchDescription
from launch.actions                     import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, RegisterEventHandler, TimerAction
from launch.event_handlers              import OnProcessExit
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration
from launch_ros.actions                import Node
from ament_index_python.packages        import get_package_share_directory


def generate_launch_description():
    robot_description_pkg = get_package_share_directory("robot_description")
    robot_moveit_pkg     = get_package_share_directory("robot_moveit")
    robot_task_pkg       = get_package_share_directory("robot_task_manager")
    robot_control_pkg    = get_package_share_directory("robot_control")

    controllers_yaml = os.path.join(robot_control_pkg, "config", "robot_controllers.yaml")
    spawn_demo_woods_arg = DeclareLaunchArgument(
        "spawn_demo_woods",
        default_value="true",
        description="Spawn the legacy random demo wood blocks from robot_description",
    )

    # 1) Gazebo bringup
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_description_pkg, "launch", "gazebo.launch.py")),
                launch_arguments={
                    "spawn_demo_woods": LaunchConfiguration("spawn_demo_woods"),
                }.items())

    # 2) MoveIt (no local controller manager — Gazebo provides one via gz_ros2_control plugin)
    moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_moveit_pkg, "launch", "moveit.launch.py")),
                launch_arguments={
                    "use_sim_time": "true",
                    "use_mock": "false",
                    "start_controller_manager": "false",
                }.items())

    # 3) Task executor (MoveIt client — talks to move_group)
    task_executor = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(robot_task_pkg, "launch", "task_servers_sim.launch.py")))

    # 4) Controller spawners (talk to the /controller_manager provided by gz_ros2_control plugin)
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "15",
        ],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "15",
        ],
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
            "--controller-manager-timeout", "15",
        ],
        output="screen",
    )

    # Spawn arm after broadcaster, spawn gripper after arm
    start_arm_after_broadcaster = RegisterEventHandler(
        OnProcessExit(target_action=joint_state_broadcaster,
                       on_exit=[arm_controller_spawner]))
    start_gripper_after_arm = RegisterEventHandler(
        OnProcessExit(target_action=arm_controller_spawner,
                       on_exit=[gripper_controller_spawner]))

    # MoveIt after 4 s, task executor after 5 s
    return LaunchDescription([
        spawn_demo_woods_arg,
        LogInfo(msg="[sim.launch] Starting: Gazebo + robot spawn + bridges"),
        gazebo,

        TimerAction(period=4.0, actions=[
            LogInfo(msg="[sim.launch] Starting: MoveIt + controller spawners"),
            moveit,
            joint_state_broadcaster,
            start_arm_after_broadcaster,
            start_gripper_after_arm,
        ]),

        TimerAction(period=6.0, actions=[
            LogInfo(msg="[sim.launch] Starting: Task executor (MoveIt client)"),
            task_executor,
        ]),
    ])
