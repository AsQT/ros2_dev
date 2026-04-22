from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_hardware_interface",
            executable="rs485_hw_node",
            name="rs485_hw",
            output="screen",
            parameters=["{}/config/params.yaml".format(
                __import__('ament_index_python.packages').packages.get_package_share_directory('robot_hardware_interface')
            )]
        )
    ])
