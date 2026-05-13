cmake_minimum_required(VERSION 3.8)
project(vh_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(vh_network REQUIRED)
find_package(vh_telemetry REQUIRED)
find_package(teleop_to_autoware REQUIRED)


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
