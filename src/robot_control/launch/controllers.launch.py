import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import Command, LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue

from launch_ros.actions import Node


from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # =========================
    # Launch configurations
    # =========================

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock = LaunchConfiguration("use_mock")
    
    robot_controller_pkg = get_package_share_directory("robot_control")
    robot_description_pkg = get_package_share_directory("robot_description")


    xacro_file = os.path.join(
                            robot_description_pkg,
                            "urdf",
                            "robot.urdf.xacro",  )

    controllers_yaml = os.path.join(
                                robot_controller_pkg,
                                "config",
                                "robot_controllers.yaml", )
            
    robot_description = ParameterValue(
                            Command(
                                [
                                    "xacro", " ",   xacro_file,
                                    " ",
                                    "use_sim:=false",
                                    " ",
                                    "use_mock:=",  use_mock,] ),    value_type=str,    )
    

    robot_state_publisher_node = Node(
                                        package     ="robot_state_publisher",
                                        executable  ="robot_state_publisher",
                                        output      ="screen",
                                        parameters  =[
                                                        {
                                                        "robot_description": robot_description,
                                                        "use_sim_time": use_sim_time, } ],  )

    controller_manager_node = Node(
                                        package     ="controller_manager",
                                        executable  ="ros2_control_node",
                                        output      ="screen",
                                        parameters  =[
                                                        {
                                                        "robot_description": robot_description,
                                                        "use_sim_time": use_sim_time,
                                                        "defaults.allow_controller_activation_with_inactive_hardware": True,
                                                        },
                                                        controllers_yaml,  ],  )

    joint_state_broadcaster_spawner = Node(
                                        package     ="controller_manager",
                                        executable  ="spawner",
                                        output      ="screen",
                                        arguments   =[
                                                        "joint_state_broadcaster",
                                                        "--controller-manager",
                                                        "/controller_manager",
                                                        "--param-file",
                                                        controllers_yaml,
                                                        "--controller-manager-timeout",
                                                        "15",
                                                        "--switch-timeout",
                                                        "15", ],  )
    arm_controller_spawner = Node(
                                        package     ="controller_manager",
                                        executable  ="spawner",
                                        output      ="screen",
                                        arguments   =[
                                                        "arm_controller",
                                                        "--controller-manager",
                                                        "/controller_manager",
                                                        "--param-file",
                                                        controllers_yaml,
                                                        "--controller-manager-timeout",
                                                        "15",
                                                        "--switch-timeout",
                                                        "15",  ],   )

    gripper_controller_spawner = Node(
                                        package     ="controller_manager",
                                        executable  ="spawner",
                                        output      ="screen",
                                        arguments   =[
                                                        "gripper_controller",
                                                        "--controller-manager",
                                                        "/controller_manager",
                                                        "--param-file",
                                                        controllers_yaml,
                                                        "--controller-manager-timeout",
                                                        "15",
                                                        "--switch-timeout",
                                                        "15",   ],  )

    start_arm_      = RegisterEventHandler(
                        OnProcessExit(
                            target_action=joint_state_broadcaster_spawner,
                            on_exit=[arm_controller_spawner],  )   )

    start_gripper   = RegisterEventHandler(
                        OnProcessExit(
                            target_action=arm_controller_spawner,
                            on_exit=[gripper_controller_spawner],   )   )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false" ),
            DeclareLaunchArgument("use_mock", default_value="true"),
            robot_state_publisher_node,
            controller_manager_node, 
            joint_state_broadcaster_spawner, 
            start_arm_,
            start_gripper,
        ]
    )