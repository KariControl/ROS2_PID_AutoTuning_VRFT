#ifndef DATA_SYNCHRONIZER_HPP
#define DATA_SYNCHRONIZER_HPP

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <controller_msgs/msg/plant_info.hpp>

class DataSynchronizer : public rclcpp::Node {
public:
    DataSynchronizer();

private:
    void callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,const geometry_msgs::msg::TwistStamped::ConstSharedPtr& input_msg);
    
    rclcpp::Publisher<controller_msgs::msg::PlantInfo>::SharedPtr plant_publisher_;
    message_filters::Subscriber<sensor_msgs::msg::Imu> imu_sub_;
    message_filters::Subscriber<geometry_msgs::msg::TwistStamped> input_sub_;
    std::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped>>> sync_;

};

#endif  // DATA_SYNCHRONIZER_HPP
