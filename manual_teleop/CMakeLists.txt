cmake_minimum_required(VERSION 3.8)
project(manual_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(autoware_cmake REQUIRED)
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_control_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(
  manual_teleop 
  src/manual_teleop.cpp
)

ament_target_dependencies(
  manual_teleop
  rclcpp
  std_msgs
  tier4_external_api_msgs    
  tier4_control_msgs     
  autoware_control_msgs
  autoware_universe_utils  
  autoware_vehicle_msgs 
)

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
  manual_teleop
  teleop_g923_node
  g923_debug
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
