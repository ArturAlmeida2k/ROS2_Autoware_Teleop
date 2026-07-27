cmake_minimum_required(VERSION 3.8)
project(vh_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Encontrar as dependências (ROS 2 e OpenCV)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 2. Compilar o Nó de Captura da Câmara (D435)
add_executable(camera_capture_node src/camera_capture_node.cpp)
target_include_directories(camera_capture_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(camera_capture_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(camera_capture_node ${OpenCV_LIBRARIES})

# 3. Compilar o Nó de Encoding (GStreamer)
add_executable(video_encoder_node src/video_encoder_node.cpp)
target_include_directories(video_encoder_node PUBLIC ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(video_encoder_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(video_encoder_node ${OpenCV_LIBRARIES})

# 4. Instalar os executáveis para que o comando 'ros2 run' os encontre
install(TARGETS
  camera_capture_node
  video_encoder_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()