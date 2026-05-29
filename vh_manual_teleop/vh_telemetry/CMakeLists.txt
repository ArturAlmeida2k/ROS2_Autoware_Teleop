cmake_minimum_required(VERSION 3.8)
project(vh_telemetry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(msg_manual_teleop REQUIRED)  
find_package(std_msgs REQUIRED)

add_executable(
  telemetry_node 
  src/telemetry_node.cpp
)

ament_target_dependencies(telemetry_node
  rclcpp
  autoware_vehicle_msgs
  autoware_adapi_v1_msgs
  msg_manual_teleop
)

add_executable(
  network_latency_logger 
  src/network_latency_logger.cpp
)

ament_target_dependencies(network_latency_logger
  rclcpp
  std_msgs
)

install(TARGETS 
  telemetry_node
  network_latency_logger
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()