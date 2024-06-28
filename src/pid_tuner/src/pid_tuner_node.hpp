#ifndef PID_TUNER_HPP_
#define PID_TUNER_HPP_

#include <deque>
#include <geometry_msgs/msg/point.hpp>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <controller_msgs/msg/plant_info.hpp>

class PID_Tuner : public rclcpp::Node
{
public:
  PID_Tuner(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );
  PID_Tuner(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions()
  );


private:
    void tuning_callback(const controller_msgs::msg::PlantInfo::SharedPtr msg);
    double compute_Td(double u);
    std::deque<double> x_data_;
    std::deque<double> y_data_;
    double integral_;//積分値
    size_t max_data_points_;
    bool is_fit_calculated_;
    double time_constant_;
    rclcpp::Subscription<controller_msgs::msg::PlantInfo>::SharedPtr subscription_;
    // const controller_msgs::msg::PlantInfo::SharedPtr msg;
    double diff_time_;//差分時間
    double k1;//ルンゲクッタ係数
    double k2;//ルンゲクッタ係数
    double k3;//ルンゲクッタ係数
    double k4;//ルンゲクッタ係数
};
#endif  // PID_TUNER_HPP_
