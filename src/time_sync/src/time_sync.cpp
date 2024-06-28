#include "time_sync.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <controller_msgs/msg/plant_info.hpp>

using namespace message_filters;

DataSynchronizer::DataSynchronizer() : Node("data_synchronizer"),
    imu_sub_(this, "vehicle_state"),
    input_sub_(this, "steering")
{
    typedef sync_policies::ApproximateTime<sensor_msgs::msg::Imu, geometry_msgs::msg::TwistStamped> MySyncPolicy;
    sync_ = std::make_shared<Synchronizer<MySyncPolicy>>(MySyncPolicy(10), imu_sub_, input_sub_);
    sync_->registerCallback(std::bind(&DataSynchronizer::callback, this, std::placeholders::_1, std::placeholders::_2));
    plant_publisher_ = this->create_publisher<controller_msgs::msg::PlantInfo>("plant_info", 1);// controller mesage
}

void DataSynchronizer::callback(const sensor_msgs::msg::Imu::ConstSharedPtr& imu_msg,const geometry_msgs::msg::TwistStamped::ConstSharedPtr& input_msg) {
    RCLCPP_INFO(this->get_logger(), "同期されたメッセージを受信しました。");
    RCLCPP_INFO(this->get_logger(), "ヨーレート (z軸): %f rad/s", imu_msg->angular_velocity.z);
    RCLCPP_INFO(this->get_logger(), "角度 (z軸): %f degrees", input_msg->twist.angular.z);

    controller_msgs::msg::PlantInfo output_msg;
    output_msg.header.stamp = imu_msg->header.stamp;
    output_msg.header.frame_id = "base_link";
    output_msg.output = imu_msg->angular_velocity.z; 
    output_msg.input = input_msg->twist.angular.z; 

    plant_publisher_->publish(output_msg);
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DataSynchronizer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}