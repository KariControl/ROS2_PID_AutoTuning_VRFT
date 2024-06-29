#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
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
  void lateral_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
  void Velocity_callback(const geometry_msgs::msg::AccelStamped::SharedPtr msg);
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vehicle_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr yawrate_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr str_angle_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr accel_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  double steering_angle_;
  double yaw_rate_;
  double yaw_angle_;
  double vehicle_speed_;
  double wheel_base_;
  double x_;
  double y_;

  double time_constant_;//参照モデルの時定数
  double DC_gain_;//参照モデルの時定数


  double diff_time_;//差分時間
  double k1;//ルンゲクッタ中間変数1
  double k2;//ルンゲクッタ中間変数2
  double k3;//ルンゲクッタ中間変数3
  double k4;//ルンゲクッタ中間変数4

};