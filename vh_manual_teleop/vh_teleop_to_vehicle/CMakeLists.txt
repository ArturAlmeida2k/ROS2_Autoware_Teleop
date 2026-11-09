cmake_minimum_required(VERSION 3.8)
project(vh_teleop_to_vehicle)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(teleop_msgs REQUIRED)

add_executable(control src/control.cpp)
ament_target_dependencies(control rclcpp std_msgs teleop_msgs)

add_executable(vehicle_interface src/vehicle_interface.cpp)
ament_target_dependencies(vehicle_interface rclcpp std_msgs teleop_msgs)

add_executable(vehicle_telemetry src/vehicle_telemetry.cpp)
ament_target_dependencies(vehicle_telemetry rclcpp std_msgs teleop_msgs)

add_executable(safety_gate src/teleop_safety_gate.cpp)
ament_target_dependencies(safety_gate rclcpp std_msgs teleop_msgs)

add_executable(topic_monitor src/teleop_topic_monitor.cpp)
ament_target_dependencies(topic_monitor rclcpp std_msgs teleop_msgs)

install(TARGETS
  control
  vehicle_interface
  vehicle_telemetry
  safety_gate
  topic_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
