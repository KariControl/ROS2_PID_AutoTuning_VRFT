#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

class VehiclePlant : public rclcpp::Node
{
public:
  VehiclePlant(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  VehiclePlant(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );

private:
  void Motion_callback(const std_msgs::msg::String::SharedPtr msg);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr yaw_rate_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr x_publshier_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr y_publshier_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr str_angle_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
};