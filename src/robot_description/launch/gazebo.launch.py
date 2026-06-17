import os

from ament_index_python.packages        import get_package_share_directory

from launch                             import LaunchDescription
from launch.actions                     import (
                                                DeclareLaunchArgument,
                                                IncludeLaunchDescription,
                                                SetEnvironmentVariable,
                                                RegisterEventHandler,
                                                TimerAction,)
from launch.conditions                  import IfCondition
from launch.launch_description_sources  import PythonLaunchDescriptionSource
from launch.substitutions               import Command, FindExecutable, LaunchConfiguration
from launch.event_handlers              import OnProcessExit

from launch_ros.actions                 import Node


def generate_launch_description():
    # 0) PATHS / CONSTANTS
    use_sim_time = True
    spawn_demo_woods_arg = DeclareLaunchArgument(
        "spawn_demo_woods",
        default_value="true",
        description="Spawn the legacy random demo wood blocks on Gazebo startup",
    )

    robot_description   = get_package_share_directory("robot_description")
    pkg_share_parent    = os.path.dirname(robot_description)

    worlds_dir          = os.path.join(robot_description, "worlds")
    world_file          = os.path.join(robot_description, "worlds", "table", "arm_on_the_table.sdf")
    xacro_file          = os.path.join(robot_description, "urdf",  "robot.urdf.xacro")

    # 1) ROBOT MODEL 
    robot_description_content   = Command([
                                    FindExecutable(name="xacro"), " ",
                                    xacro_file, " ",
                                    "use_sim:=true", ])

    # 2) ENV for Gazebo (resource/model paths)
    gz_resource_path    = SetEnvironmentVariable(
                                name    ="GZ_SIM_RESOURCE_PATH",
                                value   =":".join([
                                            pkg_share_parent,
                                            robot_description,
                                            worlds_dir,
                                            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),]),   )

    gz_model_path   = SetEnvironmentVariable(
                                name    ="GZ_SIM_MODEL_PATH",
                                value   =":".join([
                                            worlds_dir,
                                            os.environ.get("GZ_SIM_MODEL_PATH", ""), ]), )

    env_actions = [
                    gz_resource_path,
                    gz_model_path, ]

    # 3) CORE SIM: Start Gazebo + TF publisher + Spawn robot
    gazebo = IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                                    os.path.join(
                                        get_package_share_directory("ros_gz_sim"), 
                                        "launch", 
                                        "gz_sim.launch.py")  ),
                        launch_arguments={"gz_args": f"-r {world_file}"}.items(), )

    node_robot_state_publisher = Node(
                                    package     ="robot_state_publisher",
                                    executable  ="robot_state_publisher",
                                    output      ="screen",
                                    parameters  =[{
                                                "use_sim_time":         use_sim_time,
                                                "robot_description":    robot_description_content,}],   )

    spawn_robot = Node(
                    package     ="ros_gz_sim",
                    executable  ="create",
                    output      ="screen",
                    arguments   =[
                                "-string",          robot_description_content,
                                "-name",            "robot",
                                "-x",               "0.0",
                                "-y",               "0.0",
                                "-z",               "1.02",
                                "-allow_renaming",  "true", ], )

    core_actions = [
                    gazebo,
                    node_robot_state_publisher,
                    spawn_robot, 
                    ]

    # 4) BRIDGE: ROS <-> Gazebo topics
    bridge  = Node(
                package     ="ros_gz_bridge",
                executable  ="parameter_bridge",
                output      ="screen",
                arguments   =[
                            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                            "/astra_sim/rgb/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                            "/astra_sim/rgb/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",
                            "/astra_sim/depth/image_raw@sensor_msgs/msg/Image[gz.msgs.Image",
                            "/astra_sim/depth/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo",], )

    
    bridge_actions = [ bridge, ]

    # 6) Spawn wood blocks after robot
    spawn_wood_node  = Node(
                    package     ="robot_description",
                    executable  ="random_wood_blocks.py", #random_wood_blocks wood_blocks_3.py; wood_blocks_3.py
                    output      ="screen",
                    parameters  =[
                                {"world":   "default"},
                                {"count":   5},
                                {"seed":    0},
                                {"x_min":   0.35},
                                {"x_max":   0.55},
                                {"y_min":   -0.3},
                                {"y_max":   0.3},
                                {"z":       1.3} ],  )

    spawn_wood_after_robot = RegisterEventHandler(
                                OnProcessExit(
                                    target_action=spawn_robot,
                                    on_exit=[TimerAction(
                                        period=1.0,
                                        actions=[spawn_wood_node],
                                        condition=IfCondition(LaunchConfiguration("spawn_demo_woods")),
                                    )], ) )

    spawn_wood = [
        spawn_wood_after_robot, ]

    #  Compose launch in functional blocks (clean & readable)
    return LaunchDescription(
        [spawn_demo_woods_arg]
        + env_actions
        + core_actions
        + bridge_actions
        + spawn_wood
    )
