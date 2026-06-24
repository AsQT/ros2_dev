import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    robot_description_pkg = get_package_share_directory("robot_description")
    moveit_config = (
        MoveItConfigsBuilder(robot_name="robot", package_name="robot_moveit")
        .robot_description(
            file_path=os.path.join(robot_description_pkg, "urdf", "robot.urdf.xacro"),
            mappings={
                "use_sim": "true",
                "use_mock_hardware": "false",
            }
        )
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    common_moveit_params = [
        moveit_config.to_dict(),
        {
            "use_sim_time": True,
            "planning_group": "arm",
            "home_target": "home",
            "base_frame": "world",
        },
    ]

    gripper_parameters = [
        moveit_config.to_dict(),
        {
            "use_sim_time": True,
            "planning_group": "gripper",
            "base_frame": "link_6"
        },
    ]

    gohome_server = Node(
        package="robot_task_manager",
        executable="gohome_server",
        name="gohome_server",
        output="screen",
        parameters=common_moveit_params,
    )

    move_to_pose_server = Node(
        package="robot_task_manager",
        executable="move_to_pose_server",
        name="move_to_pose_server",
        output="screen",
        parameters=common_moveit_params,
    )

    move_pose_cartesian_server = Node(
        package="robot_task_manager",
        executable="move_pose_cartesian_server",
        name="move_pose_cartesian_server",
        output="screen",
        parameters=common_moveit_params,
    )

    checker_board = Node(
        package="robot_task_manager",
        executable="checker_board_server",
        name="checker_board_server",
        output="screen",
        parameters=common_moveit_params,
    )

    move_gripper = Node(
        package="robot_task_manager",
        executable="move_gripper_server",
        name="move_gripper_server",
        output="screen",
        parameters=gripper_parameters,
    )

    pickplace = Node(
        package="robot_task_manager",
        executable="pickplace_server",
        name="pickplace_action_server",
        output="screen",
        parameters=common_moveit_params,
    )

    drl_pickplace = Node(
        package="robot_task_manager",
        executable="drl_pickplace_server",
        name="drl_pickplace_action_server",
        output="screen",
        parameters=common_moveit_params + [{
            "planning_frame": "base_link",
            "ee_link": "tcp_link",
        }],
    )

    repeatability_test = Node(
        package="robot_task_manager",
        executable="repeatability_test_server",
        name="repeatability_test_action_server",
        output="screen",
        parameters=common_moveit_params,
    )

    return LaunchDescription([
        gohome_server,
        move_to_pose_server,
        move_pose_cartesian_server,
        checker_board,
        move_gripper,
        pickplace,
        drl_pickplace,
        repeatability_test,
    ])
