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

# --- Nó Original (1 Câmara) ---
add_executable(
  video_encoder
  src/video_encoder.cpp
)

ament_target_dependencies(
  video_encoder
  rclcpp 
  sensor_msgs 
  cv_bridge
)

target_link_libraries(
  video_encoder
  ${OpenCV_LIBS}
)

# --- Novo Nó (4 Câmaras) ---
add_executable(
  video_encoder_4x
  src/video_encoder_4x.cpp
)

ament_target_dependencies(
  video_encoder_4x
  rclcpp 
  sensor_msgs 
  cv_bridge
)

target_link_libraries(
  video_encoder_4x
  ${OpenCV_LIBS}
)

# --- Instalação de Executáveis C++ ---
install(TARGETS
  video_encoder
  video_encoder_4x
  DESTINATION lib/${PROJECT_NAME}
)

# --- Instalação de Scripts Python ---
install(PROGRAMS
  src/input_teleop_decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME input_teleop_decoder
)

install(PROGRAMS                       
  src/telemetry_encoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME telemetry_encoder
)

install(PROGRAMS                        
  src/teleop_logger.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME teleop_logger
)

ament_package()
