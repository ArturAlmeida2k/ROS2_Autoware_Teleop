cmake_minimum_required(VERSION 3.8)
project(manual_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependências para integração com Autoware (PC A)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) 
find_package(tier4_external_api_msgs REQUIRED)
find_package(tier4_control_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_universe_utils REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)

# Executável: O nó que fala com o Autoware
add_executable(manual_teleop src/manual_teleop.cpp)
ament_target_dependencies(manual_teleop
  rclcpp
  std_msgs
  tier4_external_api_msgs    
  tier4_control_msgs     
  autoware_control_msgs
  autoware_universe_utils  
  autoware_vehicle_msgs 
)

# Instalação do Binário C++
install(TARGETS
  manual_teleop
  DESTINATION lib/${PROJECT_NAME}
)

# Instalação do Script Python (Decoder)
install(PROGRAMS
  src/PCA_decoder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME PCA_decoder
)

# Instalação dos Launch Files (Lado A)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()