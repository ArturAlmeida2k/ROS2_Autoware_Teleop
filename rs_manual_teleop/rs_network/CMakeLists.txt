cmake_minimum_required(VERSION 3.8)
project(rs_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)

install(PROGRAMS
  src/input_teleop_encoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME input_teleop_encoder
)

install(PROGRAMS
  src/video_decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME video_decoder
)

add_executable(
  video_decoder_cpp 
  src/video_decoder_cpp.cpp
)


ament_target_dependencies(
  video_decoder_cpp 
  rclcpp
)

target_link_libraries(
  video_decoder_cpp 
  ${OpenCV_LIBS}
)

install(TARGETS 
  video_decoder_cpp
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

