#include <cstdio>

#include "sim_bridge/SimBridge.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimBridge>());
  rclcpp::shutdown();
  return 0;
}
