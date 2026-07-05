cmake_minimum_required(VERSION 3.16)
project(robot_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC_SEARCH_PATHS ${CMAKE_CURRENT_SOURCE_DIR}/ui)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(robot_hardware_interface REQUIRED)
find_package(robot_task_manager REQUIRED)
find_package(robot_vision_pipeline_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(SOURCES
  include/robot_gui/main_window.hpp
  include/robot_gui/rviz_panel.hpp
  include/robot_gui/task_action_controller.hpp
  src/main.cpp
  src/main_window.cpp
  src/rviz_panel.cpp
  src/robot_gui_node.cpp
  src/status_led.cpp
  src/task_action_controller.cpp
  ui/robot_gui.ui
)

add_executable(robot_gui_node ${SOURCES})
target_include_directories(robot_gui_node PRIVATE
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)
ament_target_dependencies(robot_gui_node
  rclcpp
  rclcpp_action
  std_srvs
  std_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  robot_hardware_interface
  robot_task_manager
  robot_vision_pipeline_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
  rviz_common
  rviz_rendering
  pluginlib
)
target_link_libraries(robot_gui_node Qt5::Widgets)

install(TARGETS robot_gui_node
  DESTINATION lib/${PROJECT_NAME}
)

# codex.md Phase 5/6: pure-logic self-test for the shared pick/place target
# resolver. No ROS runtime needed — just geometry_msgs headers. Registered as a
# ctest so `colcon test` exercises the MANUAL/AUTO/multi-wood selection rules.
add_executable(pick_place_resolver_selftest test/pick_place_resolver_selftest.cpp)
target_include_directories(pick_place_resolver_selftest PRIVATE include)
ament_target_dependencies(pick_place_resolver_selftest geometry_msgs)
install(TARGETS pick_place_resolver_selftest
  DESTINATION lib/${PROJECT_NAME}
)
if(BUILD_TESTING)
  find_package(ament_cmake_test REQUIRED)
  # The self-test is a plain C++ binary (non-zero exit == failure); it does not
  # emit a JUnit result file, so ask run_test.py to synthesize a passing result
  # from a zero exit code instead of reporting "did not generate a result file".
  ament_add_test(pick_place_resolver_selftest
    COMMAND "$<TARGET_FILE:pick_place_resolver_selftest>"
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    TIMEOUT 60
  )
endif()

install(DIRECTORY launch ui
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
