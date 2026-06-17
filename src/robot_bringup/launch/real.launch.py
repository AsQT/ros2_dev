import os

from ament_index_python.packages        import get_package_share_directory
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_robot_gui = get_package_share_directory("robot_gui")
    robot_moveit_pkg = get_package_share_directory("robot_moveit")
    robot_task_pkg = get_package_share_directory("robot_task_manager")


    gui_node = os.path.join(pkg_robot_gui, "launch", "robot_gui.launch.py")

    Gui = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gui_node),
                launch_arguments={ }.items(),   )      
       
    moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_moveit_pkg,
                        "launch",
                        "moveit.launch.py",   )    ),
                launch_arguments={ "use_mock": "false",
                                   "use_sim_time": "false",}.items(),   )
    task_serrver = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_task_pkg,
                        "launch",
                        "task_servers.launch.py",   )    ),  )

    return LaunchDescription(
        [
            Gui,
            moveit,
            task_serrver, 
              ]  )
