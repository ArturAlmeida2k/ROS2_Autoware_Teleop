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
find_package(msg_manual_teleop REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-video-1.0)

include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})

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
  ${GSTREAMER_LIBRARIES}
)

# --- Instalação de Executáveis C++ ---

install(TARGETS
  video_encoder
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
  src/pointcloud_encoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pointcloud_encoder
)

ament_package()
