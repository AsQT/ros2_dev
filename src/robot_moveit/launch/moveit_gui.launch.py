import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_mock = LaunchConfiguration("use_mock")
    start_controller_manager = LaunchConfiguration("start_controller_manager")
    gui_delay = LaunchConfiguration("gui_delay")
    initial_page = LaunchConfiguration("initial_page")
    robot_gui_config = PathJoinSubstitution(
        [FindPackageShare("robot_gui"), "config", "config.yaml"]
    )

    use_sim_time            = LaunchConfiguration("use_sim_time")
    use_mock                = LaunchConfiguration("use_mock")
    start_controller_manager = LaunchConfiguration("start_controller_manager")
    robot_description_pkg   = get_package_share_directory("robot_description")

    # 2) MOVEIT CONFIG
    moveit_config = (
                    MoveItConfigsBuilder("robot", package_name="robot_moveit")
                    .robot_description(
                        file_path=os.path.join( robot_description_pkg, "urdf", "robot.urdf.xacro", ),
                        mappings={ 
                                "use_sim": use_sim_time,
                                "use_mock_hardware": use_mock,         },)
                    .robot_description_semantic(file_path="config/robot.srdf")
                    .trajectory_execution(file_path="config/moveit_controllers.yaml")
                    .to_moveit_configs() )
    # 3) MOVE_GROUP
    node_move_group = Node(
                    package="moveit_ros_move_group",
                    executable="move_group",
                    output="screen",
                    parameters=[
                        moveit_config.to_dict(),
                        {"use_sim_time": use_sim_time},  ],  )

    # 5) STATIC TF: world -> base_link
    node_static_tf = Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    name="static_tf_world_to_base",
                    output="log",
                    arguments=[
                        "--x",              "0", 
                        "--y",              "0", 
                        "--z",              "0",
                        "--roll",           "0", 
                        "--pitch",          "0", 
                        "--yaw",            "0",
                        "--frame-id",       "world",
                        "--child-frame-id", "base_link",   ],  )

    controller = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory("robot_control"),
                                "launch",
                                "controllers.launch.py"  )   ),
                            launch_arguments={
                                "use_sim_time": use_sim_time,
                                "use_mock":     use_mock,           }.items(),
                            condition=IfCondition(start_controller_manager)   )


    robot_gui = TimerAction(
        period=gui_delay,
        actions=[
            Node(
                package="robot_gui",
                executable="robot_gui_node",
                output="screen",
                parameters=[
                    robot_gui_config,
                    moveit_config.to_dict(),
                    {
                        "embed_rviz": True,
                        "initial_page": ParameterValue(initial_page, value_type=int),
                        "rviz_config_package": "robot_moveit",
                        "rviz_config_relative_path": "config/moveit.rviz",
                        "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time",
            ),
            DeclareLaunchArgument(
                "use_mock",
                default_value="true",
                description="Use mock hardware in robot_description/control",
            ),
            DeclareLaunchArgument(
                "start_controller_manager",
                default_value="true",
                description="Start ros2_control controller manager",
            ),
            DeclareLaunchArgument(
                "gui_delay",
                default_value="3.0",
                description="Delay robot_gui startup until robot model and move_group are available",
            ),
            DeclareLaunchArgument(
                "initial_page",
                default_value="1",
                description="Initial robot_gui page index; 1 opens MAIN with embedded RViz",
            ),

        node_static_tf,
        node_move_group,
        controller,
        robot_gui,
        ]
    )
