from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_pick_and_place')
    params_file = os.path.join(pkg_share, 'config', 'aruco_pick_place.yaml')

    return LaunchDescription([
        Node(
            package='robot_pick_and_place',
            executable='aruco_pick_and_place',
            name='aruco_pick_and_place',
            output='screen',
            parameters=[params_file],
        )
    ])
