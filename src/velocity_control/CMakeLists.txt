cmake_minimum_required(VERSION 3.8)
project(velocity_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(velocity_control
  src/controller_node.cpp
  src/controller_main.cpp
)

ament_target_dependencies(velocity_control
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

target_include_directories(velocity_control
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)
install(TARGETS
  velocity_control
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()