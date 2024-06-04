#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
  void Motion_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr postion_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr vehicle_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr str_angle_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  float steering_angle_;
  float yaw_rate_;
  float yaw_angle_;
  float vehicle_speed_;
  float wheel_base_;
  float diff_time_;
  float x_;
  float y_;
};