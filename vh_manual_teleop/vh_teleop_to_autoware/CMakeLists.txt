cmake_minimum_required(VERSION 3.8)
project(vh_teleop_to_autoware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_control_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(msg_manual_teleop REQUIRED)


add_executable(control src/control.cpp)
ament_target_dependencies(control
  rclcpp
  std_msgs
  tier4_external_api_msgs    
  tier4_control_msgs     
  autoware_control_msgs
  autoware_universe_utils  
  autoware_vehicle_msgs 
  msg_manual_teleop
)

add_executable(safety_gate src/teleop_safety_gate.cpp)
ament_target_dependencies(safety_gate
  rclcpp
  std_msgs
  autoware_control_msgs
  autoware_vehicle_msgs 
)

add_executable(topic_monitor src/teleop_topic_monitor.cpp)
ament_target_dependencies(topic_monitor
  rclcpp
  std_msgs
)

install(TARGETS
  control
  safety_gate
  topic_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
ament_package()