cmake_minimum_required(VERSION 3.8)
project(velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}
 src/controller_node.cpp
 src/controller_main.cpp
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)
ament_auto_package()