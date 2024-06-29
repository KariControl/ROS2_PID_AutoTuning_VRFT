#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

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
  void state_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr controller_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr controller_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double steering_angle_;
  double velocity_;
  double kp_;
  double ki_;
  double kd_;
  double dt_;
  double setpoint_;
  double integral_;
};