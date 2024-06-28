cmake_minimum_required(VERSION 3.8)
project(pid_tuner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies 必要なメッセージのパッケージを記載
find_package(ament_cmake REQUIRED) # amet_cmakeは必要
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(controller_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# パッケージのsrcの中での必要なcppファイルを記載
add_executable(pid_tuner
  src/pid_tuner_node.cpp
  src/pid_tuner_main.cpp
)

# packageの依存関係(ビルド時に依存する外部のパッケージを記載)
ament_target_dependencies(pid_tuner
  rclcpp
  std_msgs
  geometry_msgs
  Eigen3
  controller_msgs
)

# ヘッダーファイル(include)のパスを通すために必要な処理
target_include_directories(pid_tuner
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

include_directories(${EIGEN3_INCLUDE_DIR})

## installフォルダへlaunchをインストールする処理、これがないと各パッケージをlaunchで実行できない
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)
## installフォルダへのインストール処理(これがないと実行できない)
install(TARGETS
  pid_tuner
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()