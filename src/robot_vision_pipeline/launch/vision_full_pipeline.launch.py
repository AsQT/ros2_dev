from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg_share = FindPackageShare("robot_vision_pipeline")
    realsense_pkg_share = FindPackageShare("realsense2_camera")

    yolo_param_file = PathJoinSubstitution([
        pkg_share, "config", "yolo_detect_real.yaml"
    ])
    rs_config_file = PathJoinSubstitution([
        pkg_share, "config", "rs_camera.yaml"
    ])
    adapter_param_file = PathJoinSubstitution([
        pkg_share, "config", "yolo_json_adapter.yaml"
    ])
    mapper_param_file = PathJoinSubstitution([
        pkg_share, "config", "pixel_to_base_mapper.yaml"
    ])
    marker_param_file = PathJoinSubstitution([
        pkg_share, "config", "vision_markers.yaml"
    ])

    use_camera_arg = DeclareLaunchArgument("use_camera", default_value="true")
    use_mapper_arg = DeclareLaunchArgument("use_mapper", default_value="true")
    use_markers_arg = DeclareLaunchArgument("use_markers", default_value="true")
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value=PathJoinSubstitution([
            pkg_share, "models", "yolov8.pt"
        ]),
    )
    image_topic_arg = DeclareLaunchArgument(
        "image_topic", default_value="/camera/camera/color/image_raw"
    )
    use_world_transform_arg = DeclareLaunchArgument(
        "use_world_transform",
        default_value="true",
        description="Transform final object positions and debug labels to aruco_world",
    )
    world_frame_id_arg = DeclareLaunchArgument(
        "world_frame_id",
        default_value="aruco_world",
        description="World frame used when use_world_transform is true",
    )
    extrinsic_yaml_path_arg = DeclareLaunchArgument(
        "extrinsic_yaml_path",
        default_value=PathJoinSubstitution([
            pkg_share, "config", "aruco_extrinsic_result.yaml"
        ]),
        description="YAML containing T_world_camera for camera-to-world output",
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([realsense_pkg_share, "launch", "rs_launch.py"])
        ),
        condition=IfCondition(LaunchConfiguration("use_camera")),
        launch_arguments={"config_file": rs_config_file}.items(),
    )

    yolo_node = Node(
        package="robot_vision_pipeline",
        executable="yolo_detect_node",
        name="yolo_detect_node",
        output="screen",
        parameters=[
            yolo_param_file,
            {
                "model_path_override": LaunchConfiguration("model_path"),
                "image_topic_override": LaunchConfiguration("image_topic"),
                "image_qos": "best_effort",
            },
        ],
    )

    adapter_node = Node(
        package="robot_vision_pipeline",
        executable="yolo_json_to_object_detection_node",
        name="yolo_json_to_object_detection_node",
        output="screen",
        parameters=[
            adapter_param_file,
        ],
    )

    hough_yaw_node = Node(
        package="robot_vision_pipeline",
        executable="yolo_hough_yaw_estimator_node",
        name="yolo_hough_yaw_estimator_node",
        output="screen",
        parameters=[
            {
                "image_topic": LaunchConfiguration("image_topic"),
                "yolo_json_topic": "/vision/yolo/detections_json",
                "output_json_topic": "/vision/yolo/hough_yaw_json",
                "debug_image_topic": "/vision/debug_hough_yaw_image",
                "debug_edges_topic": "/vision/debug_hough_edges",
                "bbox_padding": 25,
                "canny_low": 30,
                "canny_high": 100,
                "hough_threshold": 10,
                "min_line_length": 10,
                "max_line_gap": 8,
                "arrow_length": 60,
                "publish_debug_image": True,
            }
        ],
    )

    mapper_node = Node(
        package="robot_vision_pipeline",
        executable="pixel_to_base_mapper_node",
        name="pixel_to_base_mapper_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_mapper")),
        parameters=[
            mapper_param_file,
            yolo_param_file,
            {
                "hough_yaw_topic": "/vision/yolo/hough_yaw_json",
                "hough_yaw_json_topic": "/vision/yolo/hough_yaw_json",
                "use_hough_yaw": True,
                "use_hough_yaw_for_wood": True,
                "yaw_match_max_dist_px": 40.0,
                "yaw_match_max_center_dist_px": 40.0,
                "yaw_max_age_sec": 0.5,
                "yaw_stale_timeout_sec": 0.5,
                "draw_wood_yaw_arrow": True,
                "yaw_arrow_length_px": 55,
                "use_yaw_arrow_from_hough_json": True,
                "debug_center_alpha": 0.2,
                "use_world_transform": LaunchConfiguration("use_world_transform"),
                "output_frame_id": LaunchConfiguration("world_frame_id"),
                "world_frame_id": LaunchConfiguration("world_frame_id"),
                "extrinsic_yaml_path": LaunchConfiguration("extrinsic_yaml_path"),
            },
        ],
    )

    marker_node = Node(
        package="robot_vision_pipeline",
        executable="vision_detection_marker_node",
        name="vision_detection_marker_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_markers")),
        parameters=[marker_param_file],
    )

    return LaunchDescription([
        LogInfo(msg="[vision_full_pipeline] Starting full vision pipeline..."),
        use_camera_arg,
        use_mapper_arg,
        use_markers_arg,
        model_path_arg,
        image_topic_arg,
        use_world_transform_arg,
        world_frame_id_arg,
        extrinsic_yaml_path_arg,
        realsense_launch,
        yolo_node,
        adapter_node,
        hough_yaw_node,
        mapper_node,
        marker_node,
    ])
