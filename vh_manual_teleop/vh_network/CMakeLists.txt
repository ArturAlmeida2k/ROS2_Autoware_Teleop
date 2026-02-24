cmake_minimum_required(VERSION 3.8)
project(vh_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


install(PROGRAMS
  src/input_teleop_decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME input_teleop_decoder
)

install(PROGRAMS
  src/video_encoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME video_encoder
)

ament_package()
