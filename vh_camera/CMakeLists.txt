cmake_minimum_required(VERSION 3.8)
project(vh_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(camera_streamer_node src/camera_streamer_node.cpp)

target_include_directories(camera_streamer_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

ament_target_dependencies(camera_streamer_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(camera_streamer_node
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

install(TARGETS
  camera_streamer_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()