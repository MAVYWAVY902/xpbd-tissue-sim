#include "sim_bridge/SimBridge.hpp"



SimBridge::SimBridge(Sim::VirtuosoSimulation* sim)
: rclcpp::Node("sim_bridge"), _sim(sim)
{
    _tip_position_publisher = this->create_publisher<geometry_msgs::msg::Point>("tip_position", 10);
    auto timer_callback = 
        [this]() -> void {
            const Vec3r tip_pos = this->_sim->activeTipPosition();

            auto message = geometry_msgs::msg::Point();
            message.x = tip_pos[0];
            message.y = tip_pos[1];
            message.z = tip_pos[2];

            // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
            this->_tip_position_publisher->publish(message);
        };

    _sim->addCallback(0.05, timer_callback);
}