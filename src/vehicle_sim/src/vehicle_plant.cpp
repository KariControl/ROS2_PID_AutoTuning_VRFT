#include "vehicle_sim/vehicle_plant.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"

// 参考：https://qiita.com/NeK/items/224595987212067f41db

VehiclePlant::VehiclePlant(
  const rclcpp::NodeOptions& options
): VehiclePlant("",options){}

VehiclePlant::VehiclePlant(
  const std::string& name_space, 
  const rclcpp::NodeOptions& options
): Node("VehiclePlant", name_space, options){
    this->declare_parameter("vehicle_speed", 0.0);     // 車速デフォルト設定
    this->declare_parameter("time_constant", 1.0);     // 時定数
    this->declare_parameter("DC_gain", 0.8);     // DCゲイン
    this->declare_parameter("diff_time", 0.01);     // DCゲイン

    this->get_parameter("vehicle_speed",vehicle_speed_); // Paramからの車速読み込み
    this->get_parameter("time_constant",time_constant_); //Paramからの初期x位置読み込み
    this->get_parameter("DC_gain",DC_gain_); //Paramからの初期y位置読み込み
    this->get_parameter("diff_time",diff_time_); //Paramからの初期y位置読み込み

    // Publisher
    vehicle_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("vehicle_velocity", 1);// velocity output

    // Subscriber
    accel_subscriber_=this->create_subscription<geometry_msgs::msg::AccelStamped>("accel", 1, std::bind(&VehiclePlant::Velocity_callback, this, std::placeholders::_1));
}
void VehiclePlant::Velocity_callback(const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
    static double x_1;
    static double x_2=vehicle_speed_;
    double a_x;          // 　実加速度
    double u;          // 　加速度指令
    double k1;//ルンゲクッタ中間変数1
    double k2;//ルンゲクッタ中間変数2
    double k3;//ルンゲクッタ中間変数3
    double k4;//ルンゲクッタ中間変数4

    u=msg->accel.linear.x;
    // dx(t)/dt=Ax(t)+Bu(t)に対する4次ルンゲクッタ
    // 1次遅れ系の計算
    k1 = (-(1.0 / time_constant_) * x_1 + (DC_gain_ / time_constant_) * u);
    k2 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k1 / 2.0) + (DC_gain_ / time_constant_) * u);
    k3 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k2 / 2.0) + (DC_gain_ / time_constant_) * u);
    k4 = (-(1.0 / time_constant_) * (x_1 + diff_time_ * k3) + (DC_gain_ / time_constant_) * u);

    a_x = x_1;                                                        // フィルタ出力計算
    x_1 = x_1 + diff_time_ * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0; // 1サンプル先の状態変数を更新

    // 積分器の計算
    k1 =  a_x;
    k2 =  (a_x + diff_time_ * k1 / 2.0);
    k3 =  (a_x + diff_time_ * k2 / 2.0);
    k4 =  (a_x + diff_time_ * k3) ;

    x_2 = x_2 + diff_time_ * (k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0; // 1サンプル先の状態変数を更新

    geometry_msgs::msg::TwistStamped output_msg;
    output_msg.header.stamp = this->now();
    output_msg.header.frame_id = "base_link";
    output_msg.twist.angular.z = 0.0; 
    output_msg.twist.linear.x = x_2;
    output_msg.twist.linear.y = 0.0;
    output_msg.twist.linear.z = 0.0;
    vehicle_publisher_->publish(output_msg);
}