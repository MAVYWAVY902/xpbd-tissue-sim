#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "simulation/VirtuosoSimulation.hpp"

#include <chrono>
#include <thread>

class SimBridge : public rclcpp::Node
{
    public:
    SimBridge(Sim::VirtuosoSimulation* sim);
    

    private:
    rclcpp::TimerBase::SharedPtr _timer;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr _tip_position_publisher;
    int _count;

    Sim::VirtuosoSimulation* _sim;
};