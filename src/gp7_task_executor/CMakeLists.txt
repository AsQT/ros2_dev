cmake_minimum_required(VERSION 3.8)
project(gp7_task_executor)

if(CMAKE_CXX_STANDARD LESS 14)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gp7_task_executor_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)

install(DIRECTORY include/
  DESTINATION include
)

include_directories(include)

add_executable(task_executor_node
  src/task_executor_node.cpp
  src/planner_utils.cpp
  src/visualization_utils.cpp
  src/waypoint_loader.cpp
  src/transform_utils.cpp
)

ament_target_dependencies(task_executor_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  moveit_ros_planning_interface
  pluginlib
  gp7_task_executor_msgs
  yaml_cpp_vendor
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

ament_package()
