from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(robot_name="robot", package_name="robot_moveit")
        .robot_description(
            file_path="config/robot.urdf.xacro",
            mappings={"is_ignition": "false"}
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    common_parameters = [
        moveit_config.to_dict(),
        {"planning_group": "arm"},
        {"home_target": "home"},
        {"base_frame": "world"},
    ]

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    gohome_server = Node(
        package="robot_task_manager",                          
        executable="gohome_server",
        name="gohome_server",
        output="screen",
        parameters=common_parameters,
    )

    move_to_pose_server = Node(
        package="robot_task_manager",
        executable="move_to_pose_server",
        name="move_to_pose_server",
        output="screen",
        parameters=common_parameters,
    )

    move_pose_cartesian_server = Node(
        package="robot_task_manager",
        executable="move_pose_cartesian_server",
        name="move_pose_cartesian_server",
        output="screen",
        parameters=common_parameters,
    )

    checker_board = Node(
        package="robot_task_manager",
        executable="checker_board_server",
        name="checker_board_server",
        output="screen",
        parameters=common_parameters,
    )

    return LaunchDescription([
        move_group_node,
        gohome_server,
        move_to_pose_server,
        move_pose_cartesian_server,
        checker_board,
    ])
