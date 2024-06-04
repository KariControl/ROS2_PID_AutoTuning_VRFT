#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ControllerNode : public rclcpp::Node
{
public:
  ControllerNode(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  ControllerNode(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );

private:
  void timer_callback();
  // void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void state_callback(const sensor_msgs::msg::Imu::SharedPtr msg);


  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr controller_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr controller_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  float steering_angle_;
  float yaw_rate_;
  float yaw_angle_;
  float kp_;
  float ki_;
  float kd_;
  float dt_;
  float setpoint_;
  float integral_;

};