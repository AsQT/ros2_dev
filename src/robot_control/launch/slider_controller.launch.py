import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    is_sim = LaunchConfiguration("is_sim")
    is_ignition = LaunchConfiguration("is_ignition")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value="false",
        description="Use simulation time only when /clock exists",
    )

    is_ignition_arg = DeclareLaunchArgument(
        "is_ignition",
        default_value="false",
        description="Use ignition/gazebo backend",
    )

    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="true",
        description="Use mock hardware for ros2_control",
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("panda_controller"),
                "launch",
                "controller.launch.py",
            )
        ),
        launch_arguments={
            "is_sim": is_sim,
            "is_ignition": is_ignition,
            "use_mock_hardware": use_mock_hardware,
        }.items(),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        remappings=[
            ("/joint_states", "/joint_commands"),
        ],
        parameters=[
            {"use_sim_time": is_sim},
        ],
    )

    slider_control_node = Node(
        package="panda_controller",
        executable="slider_controller.py",
        output="screen",
        parameters=[
            {"use_sim_time": is_sim},
        ],
    )

    return LaunchDescription(
        [
            is_sim_arg,
            is_ignition_arg,
            use_mock_hardware_arg,
            controller_launch,
            joint_state_publisher_gui_node,
            slider_control_node,
        ]
    )