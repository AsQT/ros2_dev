from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("aruco_image_topic", default_value="/aruco/image_annotated"),
        DeclareLaunchArgument("aruco_pose_topic", default_value="/aruco_pose"),
        DeclareLaunchArgument("pick_place_action_name", default_value="/pickplace"),

        DeclareLaunchArgument("place_x", default_value="0.300"),
        DeclareLaunchArgument("place_y", default_value="-0.200"),
        DeclareLaunchArgument("place_z", default_value="0.100"),
        DeclareLaunchArgument("place_qx", default_value="0.7071"),
        DeclareLaunchArgument("place_qy", default_value="0.7071"),
        DeclareLaunchArgument("place_qz", default_value="0.0000"),
        DeclareLaunchArgument("place_qw", default_value="0.0000"),

        DeclareLaunchArgument("use_fixed_pick_orientation", default_value="true"),
        DeclareLaunchArgument("pick_qx", default_value="0.7071"),
        DeclareLaunchArgument("pick_qy", default_value="0.7071"),
        DeclareLaunchArgument("pick_qz", default_value="0.0000"),
        DeclareLaunchArgument("pick_qw", default_value="0.0000"),
        DeclareLaunchArgument("pick_z_offset", default_value="0.000"),
        DeclareLaunchArgument("min_pick_z", default_value="-10.0"),

        DeclareLaunchArgument("gripper", default_value="0.022"),
        DeclareLaunchArgument("velocity_scale", default_value="0.9"),

        Node(
            package="robot_pick_place",
            executable="pick_place_gui",
            name="robot_pick_place_gui",
            output="screen",
            parameters=[{
                "aruco_image_topic": LaunchConfiguration("aruco_image_topic"),
                "aruco_pose_topic": LaunchConfiguration("aruco_pose_topic"),
                "pick_place_action_name": LaunchConfiguration("pick_place_action_name"),

                "place_x": LaunchConfiguration("place_x"),
                "place_y": LaunchConfiguration("place_y"),
                "place_z": LaunchConfiguration("place_z"),
                "place_qx": LaunchConfiguration("place_qx"),
                "place_qy": LaunchConfiguration("place_qy"),
                "place_qz": LaunchConfiguration("place_qz"),
                "place_qw": LaunchConfiguration("place_qw"),

                "use_fixed_pick_orientation": LaunchConfiguration("use_fixed_pick_orientation"),
                "pick_qx": LaunchConfiguration("pick_qx"),
                "pick_qy": LaunchConfiguration("pick_qy"),
                "pick_qz": LaunchConfiguration("pick_qz"),
                "pick_qw": LaunchConfiguration("pick_qw"),
                "pick_z_offset": LaunchConfiguration("pick_z_offset"),
                "min_pick_z": LaunchConfiguration("min_pick_z"),

                "gripper": LaunchConfiguration("gripper"),
                "velocity_scale": LaunchConfiguration("velocity_scale"),
            }],
        ),
    ])
