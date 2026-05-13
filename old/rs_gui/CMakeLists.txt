cmake_minimum_required(VERSION 3.8)
project(rs_gui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)  # necessário para Qt signals/slots

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(msg_manual_teleop REQUIRED)
find_package(OpenCV REQUIRED)

find_package(Qt5 REQUIRED COMPONENTS Widgets OpenGL)
find_package(OpenGL REQUIRED)

add_executable(rs_gui
  src/main.cpp
  src/ros_bridge.cpp
  src/main_window.cpp
  src/camera_gl_widget.cpp
  src/telemetry_panel.cpp
  src/pointcloud_gl_widget.cpp
  include/ros_bridge.hpp
  include/main_window.hpp
  include/camera_gl_widget.hpp
  include/telemetry_panel.hpp
  include/pointcloud_gl_widget.hpp
)

target_include_directories(rs_gui PRIVATE include)

ament_target_dependencies(rs_gui
  rclcpp
  sensor_msgs
  msg_manual_teleop
)

target_link_libraries(rs_gui
  Qt5::Widgets
  Qt5::OpenGL
  OpenGL::GL
  ${OpenCV_LIBS}
)

install(TARGETS rs_gui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
