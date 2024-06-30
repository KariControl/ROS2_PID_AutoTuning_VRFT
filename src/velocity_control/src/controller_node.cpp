#include "control_sim/controller_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

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

    this->get_parameter("set_point",setpoint_);
    this->get_parameter("kp",kp_);
    this->get_parameter("ki",ki_);
    this->get_parameter("kd",kd_);
    this->get_parameter("dt",dt_);

    controller_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>("vehicle_velocity", 1, std::bind(&ControllerNode::state_callback, this, std::placeholders::_1));
    controller_publisher_ = this->create_publisher<geometry_msgs::msg::AccelStamped>("accel", 1);

    using namespace std::literals::chrono_literals; // これが無いと、create_wall_timer等での100msとかの時間単位付きの変数を指定できない
    timer_ = this->create_wall_timer(100ms, std::bind(&ControllerNode::timer_callback, this));
}
void ControllerNode::state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    velocity_ = msg->twist.linear.x;
}
void ControllerNode::timer_callback() {
    double error = setpoint_ - velocity_;
    static double pre_integral_ = 0.0;
    static double last_error_ = 0.0;
 
    integral_= pre_integral_+error * dt_;
    double derivative = (error - last_error_) / dt_;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    pre_integral_ = integral_;
    last_error_=error;

    geometry_msgs::msg::AccelStamped output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "base_link";
    output_msg.accel.linear.x = output;
    controller_publisher_->publish(output_msg);
}
