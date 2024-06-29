#ifndef DATA_SYNCHRONIZER_HPP
#define DATA_SYNCHRONIZER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <geometry_msgs/msg/accel_stamped.hpp>

#include "sensor_msgs/msg/imu.hpp"
#include <controller_msgs/msg/plant_info.hpp>

class DataSynchronizer : public rclcpp::Node {
public:
    DataSynchronizer();

private:
    void callback(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& vel_msg,const geometry_msgs::msg::AccelStamped::ConstSharedPtr& acc_msg);
    
    rclcpp::Publisher<controller_msgs::msg::PlantInfo>::SharedPtr plant_publisher_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> vel_sub_;
    message_filters::Subscriber<geometry_msgs::msg::AccelStamped> input_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<geometry_msgs::msg::TwistStamped,geometry_msgs::msg::AccelStamped>>> sync_;
};

#endif  // DATA_SYNCHRONIZER_HPP
