import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    embed_rviz = LaunchConfiguration("embed_rviz")

    robot_description_pkg = get_package_share_directory("robot_description")

    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="robot_moveit")
        .robot_description(
            file_path=os.path.join(robot_description_pkg, "urdf", "robot.urdf.xacro"),
            mappings={
                "use_sim": use_sim_time,
                "use_mock_hardware": "true",
            },
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
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

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_world_to_base",
        output="log",
        arguments=[
            "--x",
            "0",
            "--y",
            "0",
            "--z",
            "0",
            "--roll",
            "0",
            "--pitch",
            "0",
            "--yaw",
            "0",
            "--frame-id",
            "world",
            "--child-frame-id",
            "base_link",
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
        executable="robot_gui",
        name="robot_gui",
        output="screen",
        parameters=[
            {"joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]},
            {"axis_ids": [0, 1, 2, 3, 4, 5]},
            {"robot_ip": "192.168.2.50"},
            {"ping_timeout_ms": 1000},
            {"group_name": "arm"},
            {"base_frame": "base_link"},
            {"ee_link": "tcp_link"},
            {"embed_rviz": embed_rviz},
            {"initial_page": 1},
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("embed_rviz", default_value="true"),
            static_tf,
            robot_state_publisher,
            joint_state_publisher,
            move_group,
            robot_gui,
        ]
    )
