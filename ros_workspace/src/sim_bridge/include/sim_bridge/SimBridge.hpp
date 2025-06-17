#ifndef __SIM_BRIDGE_HPP
#define __SIM_BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "shape_msgs/msg/mesh.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "geometry/Mesh.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"

#include <chrono>
#include <thread>

template <typename SimulationType>
class SimBridge : public rclcpp::Node
{
    public:
    SimBridge(SimulationType* sim)
        : rclcpp::Node("sim_bridge"), _sim(sim)
    {
        this->declare_parameter("publish_rate_hz", 30.0);

        // assume that setup() has already been called on the Simulation object
        // then we can probe how many deformable objects are in the Sim
        const typename SimulationType::ObjectVectorType& sim_objects = sim->objects();
        const std::vector<std::unique_ptr<Sim::XPBDMeshObject_Base>>& xpbd_mesh_objs = sim_objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>();

        // allocate space for messages and publishers (one for each XPBD mesh object in the sim)
        _mesh_messages.resize(xpbd_mesh_objs.size());
        _mesh_publishers.resize(xpbd_mesh_objs.size());

        _mesh_pcl_messages.resize(xpbd_mesh_objs.size());
        _mesh_pcl_publishers.resize(xpbd_mesh_objs.size());

        // set up callbacks to publish mesh as Mesh msg and PCL point cloud msg
        for (unsigned i = 0; i < xpbd_mesh_objs.size(); i++)
        {
            const Geometry::Mesh* deformable_mesh = xpbd_mesh_objs[i]->mesh();
            setupDeformableMeshPublisher(i, deformable_mesh);
            setupDeformableMeshPclPublisher(i, deformable_mesh);
        }
    }

    private:

    void setupDeformableMeshPublisher(int index, const Geometry::Mesh* deformable_mesh)
    {
        std::string topic_name = "/output/mesh_" + std::to_string(index);
        _mesh_publishers[index] = this->create_publisher<shape_msgs::msg::Mesh>(topic_name, 3);

        _mesh_messages[index].triangles.resize(deformable_mesh->numFaces());
        for (int i = 0; i < deformable_mesh->numFaces(); i++)
        {
            const Vec3i& face = deformable_mesh->face(i);
            shape_msgs::msg::MeshTriangle tri;
            tri.vertex_indices[0] = face[0];
            tri.vertex_indices[1] = face[1];
            tri.vertex_indices[2] = face[2];

            _mesh_messages[index].triangles[i] = tri;
        }

        _mesh_messages[index].vertices.resize(deformable_mesh->numVertices());

        auto mesh_callback = 
            [this, index, deformable_mesh]() -> void {
                // update vertices
                for (int i = 0; i < deformable_mesh->numVertices(); i++)
                {
                    const Vec3r& vertex = deformable_mesh->vertex(i);
                    this->_mesh_messages[index].vertices[i].x = vertex[0];
                    this->_mesh_messages[index].vertices[i].y = vertex[1];
                    this->_mesh_messages[index].vertices[i].z = vertex[2];
                    
                }

                this->_mesh_publishers[index]->publish(this->_mesh_messages[index]);
            };
        
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mesh_callback);
    }

    void setupDeformableMeshPclPublisher(int index, const Geometry::Mesh* deformable_mesh)
    {
        std::string topic_name = "/output/mesh_vertices_" + std::to_string(index);
        _mesh_pcl_publishers[index] = this->create_publisher<sensor_msgs::msg::PointCloud2>(topic_name, 3);

        // set header
        sensor_msgs::msg::PointCloud2& mesh_pcl_message = _mesh_pcl_messages[index];
        mesh_pcl_message.header.stamp = this->now();
        mesh_pcl_message.header.frame_id = "/world";

        // add point fields
        mesh_pcl_message.fields.resize(3);
        mesh_pcl_message.fields[0].name = "x";
        mesh_pcl_message.fields[0].offset = 0;
        mesh_pcl_message.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[0].count = 1;

        mesh_pcl_message.fields[1].name = "y";
        mesh_pcl_message.fields[1].offset = sizeof(Real);
        mesh_pcl_message.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[1].count = 1;

        mesh_pcl_message.fields[2].name = "z";
        mesh_pcl_message.fields[2].offset = 2*sizeof(Real);
        mesh_pcl_message.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        mesh_pcl_message.fields[2].count = 1;

        mesh_pcl_message.height = 1;
        mesh_pcl_message.width = deformable_mesh->numVertices();
        mesh_pcl_message.is_dense = true;
        mesh_pcl_message.is_bigendian = false;

        mesh_pcl_message.point_step = 3*sizeof(Real);
        mesh_pcl_message.row_step = mesh_pcl_message.point_step * mesh_pcl_message.width;

        mesh_pcl_message.data.resize(mesh_pcl_message.row_step);

        auto mesh_pcl_callback = 
            [this, index, deformable_mesh]() -> void {
                // update vertices
                memcpy(this->_mesh_pcl_messages[index].data.data(), deformable_mesh->vertices().data(), _mesh_pcl_messages[index].data.size());

                this->_mesh_pcl_publishers[index]->publish(this->_mesh_pcl_messages[index]);
            };
        
        _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mesh_pcl_callback);
    }

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
    std::vector<shape_msgs::msg::Mesh> _mesh_messages;    // pre-allocated mesh ROS message for speed (assuming faces and number of vertices stay the same)
    std::vector<rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr> _mesh_publishers;    // publishes the current tissue mesh (all vertices and surface faces)

    std::vector<sensor_msgs::msg::PointCloud2> _mesh_pcl_messages;    // pre-allocated mesh point cloud ROS message for speed (assuming number of vertices stays the same)
    std::vector<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr> _mesh_pcl_publishers;    // publishes the current mesh vertices as a ROS point cloud (for easy ROS visualization)


    /** Pointer to the actively running Simulation object */
    SimulationType* _sim;
};

#endif // __SIM_BRIDGE_HPP