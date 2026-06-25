from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    embed_rviz = LaunchConfiguration("embed_rviz")
    initial_page = LaunchConfiguration("initial_page")
    rviz_config_package = LaunchConfiguration("rviz_config_package")
    rviz_config_relative_path = LaunchConfiguration("rviz_config_relative_path")
    config_file = PathJoinSubstitution(
        [FindPackageShare("robot_gui"), "config", "config.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "embed_rviz",
                default_value="false",
                description="Embed RViz native widget inside robot_gui",
            ),
            DeclareLaunchArgument(
                "rviz_config_package",
                default_value="robot_moveit",
                description="Package containing the RViz config",
            ),
            DeclareLaunchArgument(
                "initial_page",
                default_value="-1",
                description="Initial page index; -1 keeps the value from the UI file",
            ),
            DeclareLaunchArgument(
                "rviz_config_relative_path",
                default_value="config/moveit.rviz",
                description="RViz config path relative to the package share directory",
            ),
            Node(
                package="robot_gui",
                executable="robot_gui_node",
                output="screen",
                parameters=[
                    config_file,
                    {
                        "embed_rviz": ParameterValue(embed_rviz, value_type=bool),
                        "initial_page": ParameterValue(initial_page, value_type=int),
                        "rviz_config_package": rviz_config_package,
                        "rviz_config_relative_path": rviz_config_relative_path,
                    }
                ],
            ),
        ]
    )
