#include "vehicle_sim/vehicle_plant.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
    this->declare_parameter("vehicle_speed", 15.0/3.6);  // 車速デフォルト設定
    this->declare_parameter("wheel_base", 1.0);     // ホイールベースデフォルト設定
    this->declare_parameter("yaw_angle", 0.0);     // 初期ヨー角
    this->declare_parameter("initial_x", 0.0);     // 初期x
    this->declare_parameter("initial_y", 0.0);     // 初期y

    get_parameter("vehicle_speed",vehicle_speed_); // Paramからの車速読み込み
    get_parameter("wheel_base",wheel_base_); //Paramからのホイールベース読み込み
    get_parameter("initial_x",x_); //Paramからの初期x位置読み込み
    get_parameter("initial_y",y_); //Paramからの初期y位置読み込み

    // Publisher
    vehicle_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("vehicle_state", 1);// yaw_rate output
    postion_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("vehicle_postion", 1);// postion output

    // Subscriber
    str_angle_subscriber_=this->create_subscription<geometry_msgs::msg::TwistStamped>("steering", 1, std::bind(&VehiclePlant::Motion_callback, this, std::placeholders::_1));
}
void VehiclePlant::Motion_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
    // 極低速タスクの簡易的なsimとして、kinematicモデルをプラントモデルに設定

    static float yaw_angle_pre_ = 0.0; //前回値保存の静的変数
    static float x_pre_ = x_; //前回値保存の静的変数
    static float y_pre_ = y_; //前回値保存の静的変数

    static rclcpp::Time pre_time_=this->now(); // 前回時間

    steering_angle_ = msg->twist.angular.z;  // 操舵角度を取得
    yaw_rate_ = (vehicle_speed_ / wheel_base_) * tan(steering_angle_);  // ヨーレートの計算
    diff_time_=this->now().seconds()-pre_time_.seconds(); // 差分時間計算
    yaw_angle_ =yaw_angle_pre_+yaw_rate_*diff_time_; // ヨー角計算
    x_=x_pre_+vehicle_speed_*cos(yaw_angle_)*diff_time_;  //x位置
    y_=y_pre_+vehicle_speed_*sin(yaw_angle_)*diff_time_;  //y位置

    yaw_angle_pre_=yaw_angle_; //前回値保存
    x_pre_=x_;//前回値保存
    y_pre_=y_;//前回値保存

    pre_time_=this->now(); // 前回時間の計算
    // std::cout<<yaw_rate_<<"\n";

    geometry_msgs::msg::TwistStamped output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "base_link";
    output_msg.twist.angular.z = yaw_angle_; 
    output_msg.twist.linear.x = x_;
    output_msg.twist.linear.y = y_;
    output_msg.twist.linear.z = 0;
    postion_publisher_->publish(output_msg);

    sensor_msgs::msg::Imu yaw_rate_msg;
    yaw_rate_msg.header.stamp=output_msg.header.stamp;
    yaw_rate_msg.angular_velocity.z = yaw_rate_;
    vehicle_publisher_->publish(yaw_rate_msg);
}