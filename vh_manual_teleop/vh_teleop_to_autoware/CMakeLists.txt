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

add_executable(
  control
  src/control.cpp
)

ament_target_dependencies(
  control
  rclcpp
  std_msgs
  tier4_external_api_msgs    
  tier4_control_msgs     
  autoware_control_msgs
  autoware_universe_utils  
  autoware_vehicle_msgs 
)

install(TARGETS
  control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
