cmake_minimum_required(VERSION 3.8)
project(robot_task_executor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveToNamedTarget.srv"
  "srv/MoveToJointTarget.srv"
  "srv/MoveToPoseTarget.srv"
  "srv/MoveToNamedPoseTarget.srv"
  "srv/MoveToCartesianTarget.srv"
  "srv/MoveToNamedCartesianTarget.srv"
  "srv/MoveCartesianSequence.srv"
  "srv/MoveCartesianPoseSequence.srv"
  "srv/MoveSequence.srv"
  DEPENDENCIES geometry_msgs
)

ament_package()
