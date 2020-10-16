#include "rclcpp/rclcpp.hpp"
#include "configuration_from_mocap/marker_sim.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Clock clck();
  auto node = std::make_shared<MarkerSim>(std::make_shared<rclcpp::Clock>());
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
