cmake_minimum_required(VERSION 3.8)
project(rs_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msg_manual_teleop REQUIRED)
find_package(autoware_perception_msgs REQUIRED)

# --- Python nodes ---
install(PROGRAMS
  src/input_teleop_encoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME input_teleop_encoder
)

install(PROGRAMS
  src/telemetry_decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME telemetry_decoder
)

install(PROGRAMS
  src/pointcloud_decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_decoder
)

ament_package()