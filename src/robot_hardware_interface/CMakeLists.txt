cmake_minimum_required(VERSION 3.8)
project(robot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# =========================
# ROS interfaces (Services)
# =========================
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ServoOnAxis.srv"
  "srv/ServoOnAll.srv"
  "srv/Jog.srv"
  "srv/Home.srv"
  "srv/RunAxis.srv"
  "srv/StopAxis.srv"
  "srv/StopAll.srv"
  "msg/AxisFlag.msg"
  "msg/FlagStatus.msg"
  DEPENDENCIES std_msgs
)


include_directories(include)

add_library(rs485_protocol
  src/rs485_protocol.cpp
)
ament_target_dependencies(rs485_protocol
  rclcpp
)
set_target_properties(rs485_protocol PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(rs485_hw_node
  src/rs485_hw_node.cpp
)

target_link_libraries(rs485_hw_node rs485_protocol)
ament_target_dependencies(rs485_hw_node
  rclcpp std_msgs sensor_msgs trajectory_msgs std_srvs
)

# Link against generated service type support
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(rs485_hw_node ${cpp_typesupport_target})

add_library(rs485_system_hardware SHARED
  src/rs485_system_hardware.cpp
)
target_link_libraries(rs485_system_hardware rs485_protocol)
ament_target_dependencies(rs485_system_hardware
  rclcpp rclcpp_lifecycle hardware_interface pluginlib
)

target_include_directories(rs485_system_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

pluginlib_export_plugin_description_file(hardware_interface plugin.xml)

install(TARGETS 
  rs485_protocol 
  rs485_hw_node 
  rs485_system_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  scripts/hardware_gui.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)



ament_export_include_directories(include)
ament_export_libraries(rs485_protocol rs485_system_hardware)
ament_export_dependencies(rosidl_default_runtime)


ament_package()
