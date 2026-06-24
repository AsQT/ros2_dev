cmake_minimum_required(VERSION 3.8)
project(robot_vision_pipeline)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoPose.msg"
  "msg/ArucoPoseArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_get_python_install_dir(python_install_dir)
install(
  DIRECTORY robot_vision_pipeline/
  DESTINATION "${python_install_dir}/robot_vision_pipeline"
  FILES_MATCHING PATTERN "*.py"
)

install(
  PROGRAMS
  scripts/aruco_distance_from_intrinsic
  scripts/yolo_detect_node
  scripts/yolo_detect_node_v1
  scripts/yolo_depth_to_camera
  scripts/yolo_depth_xyz_from_intrinsic
  scripts/yolo_wood_center_to_world
  scripts/yolo_json_to_object_detection_node
  scripts/yolo_json_to_box_detection_node
  scripts/pixel_to_base_mapper_node
  scripts/yolo_hough_yaw_estimator_node
  scripts/vision_detection_marker_node
  scripts/realsense_depth_debug
  scripts/aruco_detect_node
  scripts/aruco_extrinsic_calibrator_node
  scripts/calib_camera_to_world_aruco_5points
  scripts/vision_gui
  scripts/vision_gui_astra
  scripts/static_image_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY robot_vision_pipeline/calib/
  DESTINATION share/${PROJECT_NAME}/calib
  FILES_MATCHING PATTERN "*.yaml"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
