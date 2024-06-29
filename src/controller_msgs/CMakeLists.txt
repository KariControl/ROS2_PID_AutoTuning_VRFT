cmake_minimum_required(VERSION 3.8)
project(controller_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlantInfo.msg"
  DEPENDENCIES builtin_interfaces std_msgs # カスタムメッセージの場合は依存関係をここに記述
)
ament_export_dependencies(rosidl_default_runtime) # これがないとROSのメッセージとして認識されずビルドされない
ament_package()