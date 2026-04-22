import os

from launch                             import LaunchDescription
from launch.actions                     import  IncludeLaunchDescription, TimerAction
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from ament_index_python.packages        import get_package_share_directory


def generate_launch_description():
    robot_description_pkg = get_package_share_directory("robot_description")
    robot_moveit_pkg = get_package_share_directory("robot_moveit")

    # 1) Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_description_pkg,
                        "launch",
                        "gazebo.launch.py",  )   )   )

    # 2) MoveIt 
    moveit = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        robot_moveit_pkg,
                        "launch",
                        "moveit.launch.py",   )    ),
                launch_arguments={ "use_sim_time": "True",  }.items(),   )
    

    return LaunchDescription(
        [
            gazebo,
            TimerAction(
                period=4.0,
                actions=[moveit, ],    ),     ]    )