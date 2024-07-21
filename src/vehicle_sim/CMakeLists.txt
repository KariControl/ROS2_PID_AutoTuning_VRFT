cmake_minimum_required(VERSION 3.8)
project(vehicle_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME} 
  src/vehicle_plant.cpp
  src/vehicle_plant_main.cpp
)

install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package()