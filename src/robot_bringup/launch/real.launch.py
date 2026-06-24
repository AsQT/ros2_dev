import os

from ament_index_python.packages        import get_package_share_directory
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource

def generate_launch_description():

    robot_moveit_pkg = get_package_share_directory("robot_moveit")
    robot_task_pkg = get_package_share_directory("robot_task_manager")

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

    return LaunchDescription(
        [
            moveit_gui,
            task_serrver, 
              ]  )
