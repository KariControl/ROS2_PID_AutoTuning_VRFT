#include "pid_tuner_node.hpp"
#include <controller_msgs/msg/plant_info.hpp>
#include <deque>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>

PID_Tuner::PID_Tuner(
  const rclcpp::NodeOptions& options
): PID_Tuner("",options){}

PID_Tuner::PID_Tuner(
    const std::string& name_space, 
    const rclcpp::NodeOptions& options
): Node("PID_Tuner", name_space, options){
    this->declare_parameter("max_data_points", 100);  // デフォルト値を100に設定
    this->get_parameter("max_data_points", max_data_points_);

    this->declare_parameter("time_const", 0.5);  // デフォルト値を0.5に設定
    this->get_parameter("time_const", time_constant_);

    this->declare_parameter("diff_time_", 0.1);  // 数値積分サンプリング時間
    this->get_parameter("diff_time", diff_time_);

    subscription_ = this->create_subscription<controller_msgs::msg::PlantInfo>("points", 10, std::bind(&PID_Tuner::tuning_callback, this, std::placeholders::_1));
}
void PID_Tuner::tuning_callback(const controller_msgs::msg::PlantInfo::SharedPtr msg) {
    x_data_.push_back(msg->output-compute_Td(msg->output));// 受信したデータをx_data_に保存する処理、ここをVRFTに合わせて変更：(1-Td)y
    y_data_.push_back(compute_Td(msg->input));// 受信したデータをy_data_に保存する処理、ここをVRFTに合わせて変更：Tdu

    if (x_data_.size() > max_data_points_ && !is_fit_calculated_) {
        // ここで一度だけパラメータを計算
        // PI制御用に設計
        
        Eigen::MatrixXd A(max_data_points_, 2);
        Eigen::VectorXd b(max_data_points_);
        static float pre_integral_ = 0.0;

        for (size_t i = 0; i < max_data_points_; i++) {
            A(i, 0) = x_data_[i];  // 比例制御の疑似制御入力計算
            A(i, 1) = pre_integral_+x_data_[i] * diff_time_;  // 積分制御の疑似制御入力計算
            b(i) = y_data_[i];
            pre_integral_=A(i, 1);
        }

        Eigen::Vector3d params = (A.transpose() * A).ldlt().solve(A.transpose() * b);
        RCLCPP_INFO(this->get_logger(), "Fitted parameters: kp=%f, ki=%f, kd=%f", params[0], params[1], params[2]);
        // 以降の計算フラグを無効化
        is_fit_calculated_ = true;
    }
}
double PID_Tuner::compute_Td(double u){
    static double x_1; //状態変数
    double y;//　出力変数
    // dx(t)/dt=Ax(t)+Bu(t)に対する4次ルンゲクッタ
    // x_1とuの時刻は同じ
    k1=(-(1/time_constant_)*x_1+(1/time_constant_)*u);
    k2=(-(1/time_constant_)*(x_1+diff_time_*k1/2)+(1/time_constant_)*u);
    k3=(-(1/time_constant_)*(x_1+diff_time_*k2/2)+(1/time_constant_)*u);
    k4=(-(1/time_constant_)*(x_1+diff_time_*k3)+(1/time_constant_)*u);

    y=x_1; //フィルタ出力計算
    x_1=x_1+diff_time_*(k1+2*k2+2*k3+k4)/6;// 1サンプル先の状態変数を更新
    return y;
}