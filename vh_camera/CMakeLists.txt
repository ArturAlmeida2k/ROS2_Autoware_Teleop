cmake_minimum_required(VERSION 3.8)
project(vh_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Adicionado -O3 para maximizar a performance do CPU no processamento de vídeo
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# 1. Encontrar dependências ROS 2 e OpenCV
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# 2. Encontrar dependências do GStreamer usando o PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

# 3. Declarar o executável
add_executable(camera_streamer_node src/camera_streamer_node.cpp)

# 4. Adicionar os caminhos de Inclusão (OpenCV e GStreamer)
target_include_directories(camera_streamer_node PUBLIC 
  ${OpenCV_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
)

# 5. Adicionar as dependências do ROS 2 (Faltava o sensor_msgs)
ament_target_dependencies(camera_streamer_node 
  rclcpp 
  sensor_msgs 
  cv_bridge
)

# 6. Ligar as bibliotecas nativas (OpenCV e GStreamer)
target_link_libraries(camera_streamer_node 
  ${OpenCV_LIBRARIES}
  ${GSTREAMER_LIBRARIES}
)

# 7. Instalar os executáveis para que o comando 'ros2 run' os encontre
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