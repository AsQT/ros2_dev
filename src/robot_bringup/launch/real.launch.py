import os

from ament_index_python.packages        import get_package_share_directory
from launch                             import LaunchDescription
from launch.actions                     import IncludeLaunchDescription
from launch.launch_description_sources  import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_robot_hardware = get_package_share_directory("robot_hardware_interface")
    pkg_robot_gui = get_package_share_directory("robot_gui")
    robot_moveit_pkg = get_package_share_directory("robot_moveit")


    hardware_node = os.path.join(pkg_robot_hardware, "launch", "hardware_interface.launch.py")
    gui_node = os.path.join(pkg_robot_gui, "launch", "robot_gui.launch.py")

    Hardware = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(hardware_node),
                    launch_arguments={  }.items(),   )
    
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

    return LaunchDescription(
        [
            Hardware,
            Gui,
            moveit,  ]  )