cmake_minimum_required(VERSION 3.8)
project(rs_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(msg_manual_teleop REQUIRED)  

add_executable(
  g923_teleop_node
  src/g923_teleop_node.cpp
)

ament_target_dependencies(
  g923_teleop_node
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(
  g923_debug
  src/g923_debug.cpp
)

ament_target_dependencies(
  g923_debug
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(
  xbox_controller
  src/xbox_controller.cpp
)

ament_target_dependencies(
  xbox_controller
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(
  xbox_teleop_node_v2
  src/xbox_teleop_node_v2.cpp
)

ament_target_dependencies(
  xbox_teleop_node_v2
  rclcpp
  sensor_msgs
  msg_manual_teleop
)

add_executable(
  g923_teleop_node_v2
  src/g923_teleop_node_v2.cpp
)

ament_target_dependencies(
  g923_teleop_node_v2
  rclcpp
  sensor_msgs
  msg_manual_teleop
)

add_executable(
  command_gate
  src/command_gate.cpp
)

ament_target_dependencies(
  command_gate
  rclcpp
  msg_manual_teleop
)


install(TARGETS
  g923_teleop_node
  g923_debug
  xbox_controller
  xbox_teleop_node_v2
  g923_teleop_node_v2
  command_gate
  DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS
  src/pointcloud_generator.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_generator
)

ament_package()
