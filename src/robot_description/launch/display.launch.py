import os
from launch                             import LaunchDescription
from launch.substitutions               import Command, LaunchConfiguration
from launch.actions                     import DeclareLaunchArgument
from launch_ros.actions                 import Node
from launch_ros.parameter_descriptions  import ParameterValue
from ament_index_python.packages        import get_package_share_directory

def generate_launch_description():
    pkg_share           = get_package_share_directory('robot_description')
    model_path          = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    rviz_config_path    = os.path.join(pkg_share, 'rviz/config.rviz')
    
    robot_state_publisher   =Node(
                                package    ='robot_state_publisher',
                                executable ='robot_state_publisher',
                                parameters =[{
                                            'robot_description': ParameterValue(
                                                                    Command(['xacro', ' ', model_path]),
                                                                    value_type =str     )  }]  )

    joint_state_publisher_gui     = Node(
                            package     ='joint_state_publisher_gui',
                            executable  ='joint_state_publisher_gui',
                            name        ='joint_state_publisher_gui',  )

    rviz_node   =Node(
                    package     ='rviz2',
                    executable  ='rviz2',
                    name        ='rviz2',
                    output      ='screen',
                    arguments   =['-d', rviz_config_path], )
    
    return LaunchDescription([
                joint_state_publisher_gui,
                robot_state_publisher,
                rviz_node ])
