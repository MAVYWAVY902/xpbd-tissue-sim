#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "simulation/VirtuosoSimulation.hpp"

#include <chrono>
#include <thread>

class SimBridge : public rclcpp::Node
{
    public:
    SimBridge(Sim::VirtuosoSimulation* sim);

    private:
    std::tuple<double, double, double, double, int> _jointMsgToJointState(sensor_msgs::msg::JointState* msg) const;
    

    private:
    rclcpp::TimerBase::SharedPtr _timer;

    /** Publishers */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm1_frames_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm2_frames_publisher;

    shape_msgs::msg::Mesh _mesh_message;
    rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr _mesh_publisher;

    sensor_msgs::msg::PointCloud2 _mesh_pcl_message;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _mesh_pcl_publisher;

    /** Subscriptions */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm1_joint_state_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm2_joint_state_subscriber;

    Sim::VirtuosoSimulation* _sim;
};