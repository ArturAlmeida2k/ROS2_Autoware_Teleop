cmake_minimum_required(VERSION 3.8)
project(vh_telemetry)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TelemetryState.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(telemetry_node src/telemetry_node.cpp)

ament_target_dependencies(telemetry_node
  rclcpp
  std_msgs
  autoware_vehicle_msgs
  autoware_adapi_v1_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(telemetry_node "${cpp_typesupport_target}")

install(TARGETS telemetry_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()