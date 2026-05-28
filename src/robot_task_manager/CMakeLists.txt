cmake_minimum_required(VERSION 3.8)
project(robot_task_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rosidl_default_runtime REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoHome.action"
  "action/MoveToPose.action"
  "action/MoveToPoseCartesian.action"
  "action/CheckerBoard.action"
  "action/MoveGripper.action"
  "action/PickPlace.action"
  DEPENDENCIES geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(moveit_executor SHARED
  src/moveit_executor.cpp)

ament_target_dependencies(
  moveit_executor
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  moveit_visual_tools
)
target_link_libraries(moveit_executor Eigen3::Eigen)

add_library(gripper_executor SHARED
  src/gripper_executor.cpp)

ament_target_dependencies(
  gripper_executor
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs

)
target_link_libraries(gripper_executor Eigen3::Eigen)

## action 
add_executable(gohome_server src/gohome_server.cpp)
ament_target_dependencies(gohome_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  moveit_visual_tools
)
target_link_libraries(gohome_server
  moveit_executor
  "${cpp_typesupport_target}"
)

add_executable(move_to_pose_server src/move_to_pose_server.cpp)
ament_target_dependencies(move_to_pose_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  moveit_visual_tools
)
target_link_libraries(move_to_pose_server
  moveit_executor
  "${cpp_typesupport_target}"
)

add_executable(move_pose_cartesian_server src/move_to_pose_cartesian_server.cpp)
ament_target_dependencies(move_pose_cartesian_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
)
target_link_libraries(move_pose_cartesian_server
  moveit_executor
  "${cpp_typesupport_target}"
)

add_executable(checker_board_server src/move_checker_board_server.cpp)
ament_target_dependencies(checker_board_server
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
)
target_link_libraries(checker_board_server
  moveit_executor
  "${cpp_typesupport_target}"
)

add_executable(move_gripper_server src/move_gripper_server.cpp)
ament_target_dependencies(move_gripper_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
)
target_link_libraries(move_gripper_server
  gripper_executor
)
rosidl_target_interfaces(move_gripper_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(pickplace_server src/pickplace_server.cpp)
ament_target_dependencies(pickplace_server
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
)
target_link_libraries(pickplace_server
  moveit_executor
  gripper_executor
)
rosidl_target_interfaces(pickplace_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
#_________________________________________________________________
add_executable(task_manager_client src/task_manager_client.cpp)
ament_target_dependencies(task_manager_client
  rclcpp
  rclcpp_action
)
target_link_libraries(task_manager_client
  "${cpp_typesupport_target}"
)



install(TARGETS
  moveit_executor
  gripper_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  gohome_server
  move_to_pose_server
  move_pose_cartesian_server
  checker_board_server
  move_gripper_server
  pickplace_server

  task_manager_client

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()