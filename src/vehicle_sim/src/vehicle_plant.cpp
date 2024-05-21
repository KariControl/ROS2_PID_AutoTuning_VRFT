#include "vehicle_sim/vehicle_plant.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

// https://qiita.com/NeK/items/224595987212067f41db
// https://qiita.com/NeK/items/d98a20e1c93993e06b7a
// 上のサイトのやり方じゃないとエラーが出る。

VehiclePlant::VehiclePlant(
  const rclcpp::NodeOptions& options
): VehiclePlant("",options){}

VehiclePlant::VehiclePlant(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("VehiclePlant", name_space, options){
    // Publisher
    yaw_rate_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("yaw_rate", 1);// yaw_rate output
    x_publshier_ = this->create_publisher<geometry_msgs::msg::PointStamped>("position_x", 1);// x position output
    y_publshier_ = this->create_publisher<geometry_msgs::msg::PointStamped>("position_y", 1);// y position output

    // Subscriber
    str_angle_subscriber_=this->create_subscription<std_msgs::msg::String>("steering", 1, std::bind(&VehiclePlant::Motion_callback, this, std::placeholders::_1));
}

void VehiclePlant::Motion_callback(const std_msgs::msg::String::SharedPtr msg)
{    
    
    // auto msg = std_msgs::msg::String();
    // このあたりに等価二輪モデルの運動方程式をこの辺りに計算

    // msg.data = "Hello, world ";
    // publisher_->publish(msg);
}