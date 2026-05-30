from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robot_reachability'),
                'config',
                'reachability_scan.yaml',
            ]),
            description='YAML config file for reachability scan node',
        ),

        Node(
            package='robot_reachability',
            executable='reachability_scan_node',
            name='reachability_scan_node',
            output='screen',
            parameters=[config_file],
        ),
    ])
