#include "rclcpp/rclcpp.hpp"
#include "vehicle_sim/vehicle_plant.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehiclePlant>());
  rclcpp::shutdown();
  return 0;
}