cmake_minimum_required(VERSION 3.8)
project(vh_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

add_executable(
  video_encoder_cpp 
  src/video_encoder.cpp
)

ament_target_dependencies(
  video_encoder_cpp 
  rclcpp 
  sensor_msgs 
  cv_bridge
)

target_link_libraries(
  video_encoder_cpp
  ${OpenCV_LIBS}
  )

install(TARGETS
  video_encoder_cpp
  DESTINATION lib/${PROJECT_NAME}
)

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
