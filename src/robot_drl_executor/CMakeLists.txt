cmake_minimum_required(VERSION 3.8)
project(robot_drl_executor)

if(CMAKE_CXX_STANDARD LESS 17)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robot_task_executor_msgs REQUIRED)

include_directories(include)

add_executable(robot_drl_executor_node
  src/robot_drl_executor_node.cpp
  src/executor_experiment_logger.cpp
)

ament_target_dependencies(robot_drl_executor_node
  rclcpp
  std_srvs
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  sensor_msgs
  tf2_ros
  robot_task_executor_msgs
)

install(TARGETS robot_drl_executor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
