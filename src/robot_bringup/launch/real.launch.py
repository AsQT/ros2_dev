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
                launch_arguments={ "use_mock": "false",
                                   "use_sim_time": "false",
                                   "start_controller_manager": "true",}.items(),   )
    task_serrver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_task_pkg,
                        "launch",
                        "task_servers.launch.py",   )    ),  )

    # Vision + static TF are optional even on the real robot — default off so
    # a missing/disconnected camera never blocks bringing the arm up. Enable
    # explicitly with use_vision:=true once the camera is actually connected;
    # that single flag brings up both the camera/YOLO/mapper pipeline and the
    # aruco_world static TF together (use_camera is forwarded internally).
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
                "use_vision",
                default_value="false",
                description=(
                    "Start the whole vision branch: RealSense camera + "
                    "vision_full_pipeline.launch.py (YOLO + mapper) + the temporary "
                    "aruco_world static TF. Default off so a missing camera never "
                    "blocks the real robot bringing up."),
            ),
            moveit_gui,
            task_serrver,
            vision_full_pipeline,
            aruco_world_static_tf,
              ]  )
