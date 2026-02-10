cmake_minimum_required(VERSION 3.8)
project(manual_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(sensor_msgs REQUIRED)

add_executable(
  teleop_g923_node
  src/teleop_g923_node.cpp
)

ament_target_dependencies(
  teleop_g923_node
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

install(PROGRAMS
  src/PCB_encoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME PCB_encoder
)


install(TARGETS
  teleop_g923_node
  g923_debug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
