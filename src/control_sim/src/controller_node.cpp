#include "control_sim/controller_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

// https://qiita.com/NeK/items/224595987212067f41db
// https://qiita.com/NeK/items/d98a20e1c93993e06b7a
// 上のサイトのやり方じゃないとエラーが出る。

ControllerNode::ControllerNode(
  const rclcpp::NodeOptions& options
): ControllerNode("",options){}

ControllerNode::ControllerNode(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("ControllerNode", name_space, options){
    this->declare_parameter("set_point", 0.1);  // デフォルト値として1.0を設定
    this->declare_parameter("kp", 0.2);         // デフォルト値として0.1を設定
    this->declare_parameter("ki", 0.3);        // デフォルト値として0.01を設定
    this->declare_parameter("kd", 0.05);        // デフォルト値として0.05を設定
    this->declare_parameter("dt", 0.01);        // デフォルト値として0.05を設定

    get_parameter("set_point",setpoint_);
    get_parameter("kp",kp_);
    get_parameter("ki",ki_);
    get_parameter("kd",kd_);
    get_parameter("dt",dt_);

    controller_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>("vehicle_state", 1, std::bind(&ControllerNode::state_callback, this, std::placeholders::_1));
    controller_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("steering", 1);

    using namespace std::literals::chrono_literals; // これが無いと、create_wall_timer等での100msとかの時間単位付きの変数を指定できない
    timer_ = this->create_wall_timer(10ms, std::bind(&ControllerNode::timer_callback, this));
}
void ControllerNode::state_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    yaw_rate_ = msg->angular_velocity.z;
}
void ControllerNode::timer_callback() {
    setpoint_=0.1;
    float error = setpoint_ - yaw_rate_;
    static float pre_integral_ = 0.0;
    static float last_error_ = 0.0;
 
    integral_= pre_integral_+error * dt_;
    float derivative = (error - last_error_) / dt_;
    // float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    float output = kp_ * error + ki_ * integral_;
    // float output = kp_ * error;

    pre_integral_ = integral_;
    last_error_=error;

    geometry_msgs::msg::TwistStamped output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "base_link";
    output_msg.twist.angular.z = output;
    controller_publisher_->publish(output_msg);
}
