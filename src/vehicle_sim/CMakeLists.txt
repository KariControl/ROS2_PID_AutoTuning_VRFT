cmake_minimum_required(VERSION 3.8)
project(vehicle_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(vehicle_sim
  src/vehicle_plant.cpp
  src/vehicle_plant_main.cpp
)

ament_target_dependencies(vehicle_sim
  rclcpp
  std_msgs
  geometry_msgs
)

target_include_directories(vehicle_sim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)
# install(TARGETS
#   vehicle_sim
#   EXPORT export_${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

ament_package()