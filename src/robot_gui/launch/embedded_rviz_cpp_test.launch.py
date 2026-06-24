import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock = LaunchConfiguration("use_mock")
    robot_description_pkg = get_package_share_directory("robot_description")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="robot_moveit")
        .robot_description(
            file_path=os.path.join(robot_description_pkg, "urdf", "robot.urdf.xacro"),
            mappings={
                "use_sim": use_sim_time,
                "use_mock_hardware": use_mock,
            },
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time},
        ],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": use_sim_time},
        ],
    )

    robot_gui = Node(
        package="robot_gui",
        executable="robot_gui_node",
        output="screen",
        parameters=[
            {
                "embed_rviz": True,
                "rviz_config_package": "robot_moveit",
                "rviz_config_relative_path": "config/moveit.rviz",
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time for the embedded RViz test",
            ),
            DeclareLaunchArgument(
                "use_mock",
                default_value="true",
                description="Use mock hardware in robot_description xacro",
            ),
            robot_state_publisher,
            joint_state_publisher,
            move_group,
            robot_gui,
        ]
    )
