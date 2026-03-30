cmake_minimum_required(VERSION 3.8)
project(vh_telemetry)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(msg_manual_teleop REQUIRED)  

add_executable(telemetry_node src/telemetry_node.cpp)

ament_target_dependencies(telemetry_node
  rclcpp
  autoware_vehicle_msgs
  autoware_adapi_v1_msgs
  msg_manual_teleop
)

install(TARGETS telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()