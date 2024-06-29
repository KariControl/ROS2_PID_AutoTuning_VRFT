#include "rclcpp/rclcpp.hpp"
#include "pid_tuner_node.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PID_Tuner>(); // デフォルトで100データポイントのバッファサイズを使用
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}