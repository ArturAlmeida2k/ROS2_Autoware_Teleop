cmake_minimum_required(VERSION 3.8)
project(rs_network)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
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
  src/input_teleop_encoder_v2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME input_teleop_encoder_v2
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

# --- video_decoder_cpp (exibe janela) ---
add_executable(video_decoder_cpp
  src/video_decoder.cpp
)
ament_target_dependencies(video_decoder_cpp
  rclcpp
  sensor_msgs
)
target_link_libraries(video_decoder_cpp
  ${OpenCV_LIBS}
)

# --- video_decoder_cpp_v2 (publica em /camera/front/compressed) ---
add_executable(video_decoder_cpp_v2
  src/video_decoder_v2.cpp
)
ament_target_dependencies(video_decoder_cpp_v2
  rclcpp
  sensor_msgs
)
target_link_libraries(video_decoder_cpp_v2
  ${OpenCV_LIBS}
)


# --- video_decoder_4x.cpp  ---
add_executable(video_decoder_4x_cpp
  src/video_decoder_4x.cpp
)
ament_target_dependencies(video_decoder_4x_cpp
  rclcpp
  sensor_msgs
)
target_link_libraries(video_decoder_4x_cpp
  ${OpenCV_LIBS}
)


# --- Instalar ambos os executáveis ---
install(TARGETS
  video_decoder_cpp
  video_decoder_cpp_v2
  video_decoder_4x_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()