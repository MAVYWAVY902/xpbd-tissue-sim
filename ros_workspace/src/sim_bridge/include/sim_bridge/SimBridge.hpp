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
    /** Parses a ROS JointState message with 5 fields
     * "inner_rotation" - inner tube rotation
     * "outer_rotation" - outer tube rotation
     * "inner_translation" - inner tube translation
     * "outer_translation" - outer tube translation
     * "tool" - tool actuation
     * 
     * into a tuple: (outer_rot, outer_trans, inner_rot, inner_trans, tool)
     */
    std::tuple<double, double, double, double, int> _jointMsgToJointState(sensor_msgs::msg::JointState* msg) const;
    

    private:
    /** Publishers */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm1_frames_publisher;     // publishes coordinate frames along backbone of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm2_frames_publisher;     // publishes coordinate frames along backbone of arm2

    shape_msgs::msg::Mesh _mesh_message;    // pre-allocated mesh ROS message for speed (assuming faces and number of vertices stay the same)
    rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr _mesh_publisher;    // publishes the current tissue mesh (all vertices and surface faces)

    sensor_msgs::msg::PointCloud2 _mesh_pcl_message;    // pre-allocated mesh point cloud ROS message for speed (assuming number of vertices stays the same)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _mesh_pcl_publisher;    // publishes the current mesh vertices as a ROS point cloud (for easy ROS visualization)

    /** Subscriptions */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm1_joint_state_subscriber;     // subscribes to joint state commands for arm1
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm2_joint_state_subscriber;     // subscribes to joint state commands for arm2


    /** Pointer to the actively running Simulation object */
    Sim::VirtuosoSimulation* _sim;
};