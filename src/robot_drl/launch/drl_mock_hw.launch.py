"""Mock Hardware + MoveIt + DRL launch — NO Gazebo.

Full robot stack without physics simulation:
  1. robot_state_publisher + ros2_control_node (mock hardware, GenericSystem)
  2. Controller spawners (joint_state_broadcaster, arm_controller, gripper_controller)
  3. MoveIt move_group
  4. Task executor (/move_cartesian_pose_sequence service)
  5. DRL planner (planning + trajectory visualization)
  6. RViz (robot model + trajectory markers)

Usage::

    # Quick start with auto-plan
    ros2 launch robot_drl drl_mock_hw.launch.py

    # Vision mode
    ros2 launch robot_drl drl_mock_hw.launch.py \\
        input_mode:=vision auto_plan_on_start:=false

    # With auto-execute after planning
    ros2 launch robot_drl drl_mock_hw.launch.py \\
        auto_plan_on_start:=true \\
        manual_prompt_on_start:=false \\
        auto_execute_after_plan:=true

    # Custom target
    ros2 launch robot_drl drl_mock_hw.launch.py \\
        auto_plan_on_start:=true \\
        manual_prompt_on_start:=false \\
        target_x:=0.600 target_y:=-0.100 target_z:=0.200

Arguments:
  input_mode                  — "manual" or "vision" (default: manual)
  auto_plan_on_start          — auto-plan on startup (default: true)
  manual_prompt_on_start      — prompt terminal on startup (default: false)
  auto_execute_after_plan     — auto-execute after planning (default: false)
  calibrated_start_tcp_base   — start TCP in BASE frame [x, y, z] (metres)
  target_x/y/z                — mock target position (default: 0.575/0.050/0.120)
  target_class_name           — "box" or "target" (default: box)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def _make_params(params_dict: dict, use_sim_time_bool: bool) -> list[dict]:
    """Strip use_sim_time from params and prepend it as a proper bool.

    MoveItConfigsBuilder.robot_description() adds use_sim_time as a string
    "false" which causes rclcpp nodes to abort on launch.  We extract it and
    re-add it as a native Python bool at the top of the parameter list so
    that rclcpp accepts it before the string version is seen.
    """
    params_out = [{"use_sim_time": use_sim_time_bool}]
    for k, v in params_dict.items():
        if k == "use_sim_time":
            continue
        params_out.append({k: v})
    return params_out


def generate_launch_description():
    pkg_drl     = "robot_drl"
    pkg_control = "robot_control"
    pkg_desc    = "robot_description"
    pkg_moveit  = "robot_moveit"

    share_desc    = get_package_share_directory(pkg_desc)
    share_control = get_package_share_directory(pkg_control)
    share_drl     = get_package_share_directory(pkg_drl)
    share_moveit  = get_package_share_directory(pkg_moveit)

    xacro_file        = os.path.join(share_desc, "urdf", "robot.urdf.xacro")
    controllers_yaml  = os.path.join(share_control, "config", "robot_controllers.yaml")

    # ── Launch arguments ────────────────────────────────────────────────────────
    input_mode_arg = DeclareLaunchArgument(
        "input_mode",
        default_value="manual",
        description="Input mode: 'manual' (terminal) or 'vision' (from mock_environment)",
    )
    auto_plan_arg = DeclareLaunchArgument(
        "auto_plan_on_start",
        default_value="true",
        description="Plan automatically on startup",
    )
    manual_prompt_arg = DeclareLaunchArgument(
        "manual_prompt_on_start",
        default_value="false",
        description="Prompt terminal for target input on startup",
    )
    auto_exec_arg = DeclareLaunchArgument(
        "auto_execute_after_plan",
        default_value="false",
        description="Execute the planned trajectory after planning",
    )
    start_tcp_arg = DeclareLaunchArgument(
        "calibrated_start_tcp_base",
        default_value="[0.5241, 0.000, 0.315]",
        description="Start TCP in BASE frame [x, y, z] (metres)",
    )
    target_x_arg = DeclareLaunchArgument(
        "target_x", default_value="0.575",
        description="Mock target X in world frame (metres)",
    )
    target_y_arg = DeclareLaunchArgument(
        "target_y", default_value="0.050",
        description="Mock target Y in world frame (metres)",
    )
    target_z_arg = DeclareLaunchArgument(
        "target_z", default_value="0.120",
        description="Mock target Z in world frame (metres)",
    )
    target_class_arg = DeclareLaunchArgument(
        "target_class_name",
        default_value="box",
        description="Mock object class: 'box' or 'target'",
    )

    auto_plan     = LaunchConfiguration("auto_plan_on_start")
    manual_prompt = LaunchConfiguration("manual_prompt_on_start")
    auto_exec     = LaunchConfiguration("auto_execute_after_plan")
    start_tcp     = LaunchConfiguration("calibrated_start_tcp_base")
    input_mode    = LaunchConfiguration("input_mode")

    # use_sim_time=False for mock hardware (no /clock needed)
    use_sim_time_bool = False

    # ── MoveIt config ─────────────────────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name=pkg_moveit)
        .robot_description(
            file_path=xacro_file,
            mappings={
                "use_sim":           "false",
                "use_mock_hardware": "true",
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(share_moveit, "config", "robot.srdf")
        )
        .trajectory_execution(
            file_path=os.path.join(share_moveit, "config", "moveit_controllers.yaml")
        )
        .to_moveit_configs()
    )

    # robot_description params — strip the bad use_sim_time string and prepend bool
    rsp_params = _make_params(moveit_config.robot_description, use_sim_time_bool)

    # move_group params
    move_group_params = _make_params(moveit_config.to_dict(), use_sim_time_bool)

    # ── Step 1: robot_state_publisher ─────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=rsp_params,
    )

    # ── Step 2: ros2_control_node (mock hardware) ───────────────────────────────
    ros2_control_params = _make_params(
        {**moveit_config.robot_description, **controllers_yaml_to_dict(controllers_yaml)},
        use_sim_time_bool,
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=ros2_control_params,
    )

    # ── Step 3: Controller spawners ───────────────────────────────────────────
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--param-file",         controllers_yaml,
            "--controller-manager-timeout", "15",
            "--switch-timeout",      "15",
        ],
        output="screen",
    )
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file",         controllers_yaml,
            "--controller-manager-timeout", "15",
            "--switch-timeout",      "15",
        ],
        output="screen",
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
            "--param-file",         controllers_yaml,
            "--controller-manager-timeout", "15",
            "--switch-timeout",      "15",
        ],
        output="screen",
    )

    start_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller_spawner],
        )
    )
    start_gripper = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # ── Step 4: MoveIt move_group ─────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
    )

    # ── Step 5: Static TF world -> base_link ──────────────────────────────────
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        output="log",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
    )

    # ── Step 6: Task executor (MoveIt client) ──────────────────────────────────
    task_executor_node = Node(
        package="robot_task_executor",
        executable="task_executor_node",
        name="task_executor_node",
        output="screen",
        parameters=_make_params({
            **moveit_config.robot_description,
            **moveit_config.robot_description_semantic,
            "use_sim_time":                use_sim_time_bool,
            "move_group_name":             "arm",
            "base_frame":                  "base_link",
            "ee_link":                     "tcp_link",
            "planning_time":               2.0,
            "num_planning_attempts":        5,
            "max_velocity_scaling_factor":  0.5,
            "max_acceleration_scaling_factor": 0.5,
        }, use_sim_time_bool),
    )

    # ── Step 7: mock_environment_node ──────────────────────────────────────────
    mock_env_node = Node(
        package=pkg_drl,
        executable="mock_environment_node",
        name="mock_environment_node",
        output="screen",
        parameters=[{
            "publish_rate_hz":   10.0,
            "target_class_name": LaunchConfiguration("target_class_name"),
            "target_x":          LaunchConfiguration("target_x"),
            "target_y":          LaunchConfiguration("target_y"),
            "target_z":          LaunchConfiguration("target_z"),
            "frame_id":          "base_link",
            "distance_m":        0.5,
            "confidence":         0.95,
        }],
    )

    # manual_default_target must be a proper double array.
    # Since target_x/y/z are LaunchConfiguration strings, we declare them as
    # launch args and reference them directly — DrlUnifiedPlannerNode already
    # reads target_x/y/z from the mock_environment_node topic, so this param
    # is only a fallback.  We omit it and let the node use its code defaults.
    #
    # The DRL node picks up target from:
    #   - mock_environment_node via /vision/target_position (vision mode)
    #   - manual_default_target param or terminal prompt (manual mode)
    drl_node = Node(
        package=pkg_drl,
        executable="drl_unified_planner_node",
        name="drl_unified_planner_node",
        output="screen",
        parameters=[{
            "use_sim_time":         use_sim_time_bool,
            "calibrated_start_tcp_base": start_tcp,
            "input_mode":           input_mode,
            "auto_plan_on_start":    auto_plan,
            "manual_prompt_on_start": manual_prompt,
            "auto_execute_after_plan": auto_exec,
        }],
    )

    # ── Step 9: RViz ──────────────────────────────────────────────────────────
    rviz_config = os.path.join(share_drl, "rviz", "drl_markers.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_drl",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=_make_params({
            **moveit_config.robot_description,
            **moveit_config.robot_description_semantic,
        }, use_sim_time_bool),
    )

    # ── Launch sequence ────────────────────────────────────────────────────────
    return LaunchDescription([
        LogInfo(msg="[mock_hw] === Mock Hardware + MoveIt + DRL (no Gazebo) ==="),

        # Arguments
        input_mode_arg,
        auto_plan_arg,
        manual_prompt_arg,
        auto_exec_arg,
        start_tcp_arg,
        target_x_arg,
        target_y_arg,
        target_z_arg,
        target_class_arg,

        # Core nodes
        LogInfo(msg="[mock_hw] Step 1: robot_state_publisher + ros2_control (mock hardware)..."),
        rsp_node,
        ros2_control_node,
        joint_state_broadcaster,
        start_arm,
        start_gripper,

        # MoveIt + TF
        TimerAction(period=5.0, actions=[
            LogInfo(msg="[mock_hw] Step 2: MoveIt move_group + static_tf..."),
            static_tf,
            move_group_node,
        ]),

        # Task executor
        TimerAction(period=8.0, actions=[
            LogInfo(msg="[mock_hw] Step 3: Task executor (MoveIt client)..."),
            task_executor_node,
        ]),

        # Mock environment + DRL planner
        TimerAction(period=11.0, actions=[
            LogInfo(msg="[mock_hw] Step 4: mock_environment + DRL planner..."),
            mock_env_node,
            drl_node,
        ]),

        # RViz
        TimerAction(period=14.0, actions=[
            LogInfo(msg="[mock_hw] Step 5: RViz (robot model + trajectory markers)..."),
            rviz_node,
        ]),
    ])


def controllers_yaml_to_dict(path: str) -> dict:
    """Load a YAML file and return it as a plain Python dict."""
    import yaml
    with open(path) as f:
        return yaml.safe_load(f) or {}
