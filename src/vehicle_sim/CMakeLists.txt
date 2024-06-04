cmake_minimum_required(VERSION 3.8)
project(vehicle_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies 必要なメッセージのパッケージを記載
find_package(ament_cmake REQUIRED) # amet_cmakeは必要
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# パッケージのsrcの中での必要なcppファイルを記載
add_executable(vehicle_sim
  src/vehicle_plant.cpp
  src/vehicle_plant_main.cpp
)

# packageの依存関係(ビルド時に依存する外部のパッケージを記載)
ament_target_dependencies(vehicle_sim
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
)

# ヘッダーファイル(include)のパスを通すために必要な処理
target_include_directories(vehicle_sim
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

## 以下はビルドファイルのインストール用の処理
## 仕組みはhttps://runebook.dev/ja/docs/cmake/command/install
## install関数では対象ターゲットをinstallファイルの指定ディレクトリへインストールさせる
## この処理がないと、ノードを立ち上げる際にビルド後のファイルを読み込んで実行できない

## installフォルダへincludeファイルをインストールする処理
install(
  DIRECTORY include/
  DESTINATION include
)

## installフォルダへlaunchをインストールする処理、これがないと各パッケージをlaunchで実行できない
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}/
)
## installフォルダへのインストール処理(これがないと実行できない)
install(TARGETS
  vehicle_sim
  DESTINATION lib/${PROJECT_NAME}
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