#include "sim_bridge/SimBridge.hpp"

SimBridge::SimBridge()
: rclcpp::Node("sim_bridge"), _count(0)
{
    _publisher = this->create_publisher<std_msgs::msg::String>("topic", 10);
    auto timer_callback = 
        [this]() -> void {
            auto message = std_msgs::msg::String();
            message.data = "Hellow, world! " + std::to_string(this->_count++);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            this->_publisher->publish(message);
        };
    _timer = this->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
}