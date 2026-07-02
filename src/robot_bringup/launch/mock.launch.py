import os

from ament_index_python.packages        import get_package_share_directory
from launch                             import LaunchDescription
from launch.actions                     import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions                  import IfCondition
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import LaunchConfiguration

def generate_launch_description():

    robot_moveit_pkg = get_package_share_directory("robot_moveit")
    robot_task_pkg = get_package_share_directory("robot_task_manager")
    robot_executor_pkg = get_package_share_directory("robot_task_executor")
    robot_vision_pkg = get_package_share_directory("robot_vision_pipeline")
    robot_bringup_pkg = get_package_share_directory("robot_bringup")
    cyclonedds_config = os.path.join(robot_bringup_pkg, "config", "cyclonedds.xml")
    set_cyclonedds_uri = SetEnvironmentVariable(
        name="CYCLONEDDS_URI",
        value=f"file://{cyclonedds_config}",
    )

    moveit_gui = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_moveit_pkg,
                        "launch",
                        "moveit_gui.launch.py",   )    ),
                launch_arguments={ "use_mock": "true",
                                   "use_sim_time": "false",
                                   "start_controller_manager": "true",}.items(),   )
    task_serrver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_task_pkg,
                        "launch",
                        "task_servers.launch.py",   )    ),
                launch_arguments={
                    "enable_executor_logging": LaunchConfiguration("enable_executor_logging"),
                    "log_root_dir": LaunchConfiguration("log_root_dir"),
                    "executor_log_dir": LaunchConfiguration("executor_log_dir"),
                    "executor_sample_rate_hz": LaunchConfiguration("executor_sample_rate_hz"),
                    "executor_base_frame": LaunchConfiguration("executor_base_frame"),
                    "executor_tcp_frame": LaunchConfiguration("executor_tcp_frame"),
                }.items(),   )
    task_executor = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_executor_pkg,
                        "launch",
                        "task_executor.launch.py",   )    ),
                launch_arguments={
                    "enable_executor_logging": LaunchConfiguration("enable_executor_logging"),
                    "log_root_dir": LaunchConfiguration("log_root_dir"),
                    "executor_log_dir": LaunchConfiguration("executor_log_dir"),
                    "executor_sample_rate_hz": LaunchConfiguration("executor_sample_rate_hz"),
                    "executor_base_frame": LaunchConfiguration("executor_base_frame"),
                    "executor_tcp_frame": LaunchConfiguration("executor_tcp_frame"),
                }.items(),   )

    # Vision branch — optional, single flag. Mock hardware must never depend
    # on a physically-connected camera, so this only starts when the user
    # explicitly opts in with use_vision:=true; that one flag brings up both
    # the camera/YOLO/mapper pipeline and the aruco_world static TF together.
    # If the camera is missing or vision nodes fail, that failure is isolated
    # to this branch: it does not stop moveit_gui/task_serrver/task_executor
    # above. vision_full_pipeline.launch.py still has its own use_camera
    # argument internally — it is forwarded here (use_camera = use_vision) so
    # the bringup user never has to see or pass it separately.
    vision_full_pipeline = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_vision_pkg,
                        "launch",
                        "vision_full_pipeline.launch.py",   )    ),
                launch_arguments={
                    "use_camera": LaunchConfiguration("use_vision"),
                }.items(),
                condition=IfCondition(LaunchConfiguration("use_vision")),   )

    aruco_world_static_tf = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_bringup_pkg,
                        "launch",
                        "aruco_world_static_tf.launch.py",   )    ),
                condition=IfCondition(LaunchConfiguration("use_vision")),   )

    return LaunchDescription(
        [
            set_cyclonedds_uri,
            DeclareLaunchArgument(
                "enable_executor_logging",
                default_value="false",
                description="Enable CSV executor experiment logging in task_executor_node"),
            DeclareLaunchArgument(
                "log_root_dir",
                default_value="/home/minhquang/ros2_dev/Log_robot_data",
                description="Unified root directory for robot log data"),
            DeclareLaunchArgument(
                "executor_log_dir",
                default_value="/home/minhquang/ros2_dev/Log_robot_data/executor_logs",
                description="Directory for executor experiment logs"),
            DeclareLaunchArgument(
                "executor_sample_rate_hz",
                default_value="50.0",
                description="Sampling rate for executor actual data"),
            DeclareLaunchArgument(
                "executor_base_frame",
                default_value="base_link",
                description="Base frame for executor TCP TF lookup"),
            DeclareLaunchArgument(
                "executor_tcp_frame",
                default_value="tcp_link",
                description="TCP frame for executor TCP TF lookup"),
            DeclareLaunchArgument(
                "use_vision",
                default_value="false",
                description=(
                    "Start the whole vision branch: RealSense camera + "
                    "vision_full_pipeline.launch.py (YOLO + mapper, publishes "
                    "/vision/wood_objects, /vision/box_objects) + the temporary "
                    "aruco_world static TF. Mock hardware does not need this; "
                    "MoveTargetRl/MoveToPoseObstacle work without it via goal "
                    "fallback fields."),
            ),
            moveit_gui,
            task_serrver,
            task_executor,
            vision_full_pipeline,
            aruco_world_static_tf,
              ]  )
