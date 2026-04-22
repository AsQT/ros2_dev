cmake_minimum_required(VERSION 3.8)
project(task_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(planning_tutorial src/planning_tutorial.cpp)
target_compile_features(planning_tutorial PUBLIC cxx_std_17)
ament_target_dependencies(planning_tutorial
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
)

install(TARGETS planning_tutorial
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()  
