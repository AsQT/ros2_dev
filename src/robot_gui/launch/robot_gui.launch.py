from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="robot_gui",
            executable="robot_gui",
            name="robot_gui",
            output="screen",
            parameters=[
                {"joint_names": ["joint_1","joint_2","joint_3","joint_4","joint_5","joint_6"]},
                {"axis_ids": [0,1,2,3,4,5]},
                {"robot_ip": "192.168.2.50"},
                {"ping_timeout_ms": 1000},
                {"group_name": "arm"},
                {"base_frame": "base_link"},
                {"ee_link": "tcp_link"},
            ],
        )
    ])
