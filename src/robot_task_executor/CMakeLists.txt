cmake_minimum_required(VERSION 3.8)
project(robot_task_executor)

if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(robot_task_executor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

include_directories(include)

# Reusable CSV execution logger, shared between task_executor_node (via
# /move_cartesian_pose_sequence) and robot_task_manager::MoveItExecutor, so
# both code paths write the exact same CSV format through one implementation.
add_library(executor_experiment_logger SHARED
  src/executor_experiment_logger.cpp
)

target_include_directories(executor_experiment_logger PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(executor_experiment_logger
  rclcpp
  geometry_msgs
  moveit_msgs
  sensor_msgs
  tf2_ros
)

add_executable(task_executor_node
  src/task_executor_node.cpp
  src/planner_utils.cpp
  src/visualization_utils.cpp
  src/waypoint_loader.cpp
  src/transform_utils.cpp
)

ament_target_dependencies(task_executor_node
  rclcpp
  std_srvs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  sensor_msgs
  moveit_ros_planning_interface
  pluginlib
  robot_task_executor_msgs
  yaml_cpp_vendor
)

target_link_libraries(task_executor_node
  executor_experiment_logger
  yaml-cpp::yaml-cpp
)

install(TARGETS executor_experiment_logger
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS task_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp geometry_msgs moveit_msgs sensor_msgs tf2_ros)

ament_package()
