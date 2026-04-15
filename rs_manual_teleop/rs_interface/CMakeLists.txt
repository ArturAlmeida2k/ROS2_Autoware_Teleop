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

install(TARGETS
  g923_teleop_node
  g923_debug
  xbox_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})


ament_package()
