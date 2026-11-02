cmake_minimum_required(VERSION 3.8)
project(vh_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()



# find dependencies
find_package(ament_cmake REQUIRED)
# só tem launch e config: os outros pacotes são dependências de execução (package.xml),
# para poder compilar sem o Autoware quando se usa target:=vehicle


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
