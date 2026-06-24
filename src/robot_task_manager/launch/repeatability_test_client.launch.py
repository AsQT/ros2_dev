from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("axis", default_value="0"),
        DeclareLaunchArgument("repeat_count", default_value="3"),
        DeclareLaunchArgument("meas_offset", default_value="0.02"),
        DeclareLaunchArgument("velocity_scale", default_value="0.25"),
        DeclareLaunchArgument("frame_id", default_value="world"),
        Node(
            package="robot_task_manager",
            executable="repeatability_test_client.py",
            name="repeatability_test_client",
            output="screen",
            parameters=[{
                "axis": ParameterValue(LaunchConfiguration("axis"), value_type=int),
                "repeat_count": ParameterValue(LaunchConfiguration("repeat_count"), value_type=int),
                "meas_offset": ParameterValue(LaunchConfiguration("meas_offset"), value_type=float),
                "velocity_scale": ParameterValue(LaunchConfiguration("velocity_scale"), value_type=float),
                "frame_id": LaunchConfiguration("frame_id"),
            }],
        ),
    ])
