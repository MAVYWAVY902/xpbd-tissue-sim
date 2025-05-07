#include "sim_bridge/SimBridge.hpp"


#include "simulation/VirtuosoTissueGraspingSimulation.hpp"
#include "utils/GeometryUtils.hpp"
#include "geometry/TetMesh.hpp"

#include "std_msgs/msg/header.hpp"

SimBridge::SimBridge(Sim::VirtuosoSimulation* sim)
: rclcpp::Node("sim_bridge"), _sim(sim)
{
    // set up callback to publish frames for arm1 (if it exists)
    if (_sim->virtuosoRobot()->hasArm1())
    {
        std::cout << "Setting up callback for Virtuoso arm 1 frames..." << std::endl;
        _arm1_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("arm1_frames", 10);

        auto arm1_callback = 
            [this]() -> void {
                const Sim::VirtuosoArm* arm1 = this->_sim->virtuosoRobot()->arm1();
                const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = arm1->outerTubeFrames();
                const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm1->innerTubeFrames();


                auto message = geometry_msgs::msg::PoseArray();
                message.header.stamp = this->now();
                message.header.frame_id = "/world";

                for (const auto& frame : ot_frames)
                {
                    geometry_msgs::msg::Pose pose;

                    const Vec3r& origin = frame.origin();
                    const Vec4r& quat = GeometryUtils::matToQuat(frame.transform().rotMat());
                    pose.position.x = origin[0];
                    pose.position.y = origin[1];
                    pose.position.z = origin[2];

                    pose.orientation.x = quat[0];
                    pose.orientation.y = quat[1];
                    pose.orientation.z = quat[2];
                    pose.orientation.w = quat[3];

                    message.poses.push_back(pose);
                }

                for (const auto& frame : it_frames)
                {
                    geometry_msgs::msg::Pose pose;

                    const Vec3r& origin = frame.origin();
                    const Vec4r& quat = GeometryUtils::matToQuat(frame.transform().rotMat());
                    pose.position.x = origin[0];
                    pose.position.y = origin[1];
                    pose.position.z = origin[2];

                    pose.orientation.x = quat[0];
                    pose.orientation.y = quat[1];
                    pose.orientation.z = quat[2];
                    pose.orientation.w = quat[3];

                    message.poses.push_back(pose);
                }
                

                // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                this->_arm1_frames_publisher->publish(message);
            };

        _sim->addCallback(1.0/_publish_rate_Hz, arm1_callback);
    }

    // set up callback to publish frames for arm 2 (if it exists)
    if (_sim->virtuosoRobot()->hasArm2())
    {
        std::cout << "Setting up callback for Virtuoso arm 2 frames..." << std::endl;
        _arm2_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("arm2_frames", 10);
        auto arm2_callback = 
            [this]() -> void {
                const Sim::VirtuosoArm* arm2 = this->_sim->virtuosoRobot()->arm2();
                const Sim::VirtuosoArm::OuterTubeFramesArray& ot_frames = arm2->outerTubeFrames();
                const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm2->innerTubeFrames();


                auto message = geometry_msgs::msg::PoseArray();
                message.header.stamp = this->now();
                message.header.frame_id = "/world";

                for (const auto& frame : ot_frames)
                {
                    geometry_msgs::msg::Pose pose;

                    const Vec3r& origin = frame.origin();
                    const Vec4r& quat = GeometryUtils::matToQuat(frame.transform().rotMat());
                    pose.position.x = origin[0];
                    pose.position.y = origin[1];
                    pose.position.z = origin[2];

                    pose.orientation.x = quat[0];
                    pose.orientation.y = quat[1];
                    pose.orientation.z = quat[2];
                    pose.orientation.w = quat[3];

                    message.poses.push_back(pose);
                }

                for (const auto& frame : it_frames)
                {
                    geometry_msgs::msg::Pose pose;

                    const Vec3r& origin = frame.origin();
                    const Vec4r& quat = GeometryUtils::matToQuat(frame.transform().rotMat());
                    pose.position.x = origin[0];
                    pose.position.y = origin[1];
                    pose.position.z = origin[2];

                    pose.orientation.x = quat[0];
                    pose.orientation.y = quat[1];
                    pose.orientation.z = quat[2];
                    pose.orientation.w = quat[3];

                    message.poses.push_back(pose);
                }
                

                // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
                this->_arm2_frames_publisher->publish(message);
            };

        _sim->addCallback(1.0/_publish_rate_Hz, arm2_callback);
    }

    // set up callback to publish mesh
    if (Sim::VirtuosoTissueGraspingSimulation* tissue_sim = dynamic_cast<Sim::VirtuosoTissueGraspingSimulation*>(_sim))
    {
        _mesh_publisher = this->create_publisher<shape_msgs::msg::Mesh>("tissue_mesh", 3);

        const Geometry::TetMesh* tissue_mesh = tissue_sim->tissueMesh(); 

        
        _mesh_message.triangles.resize(tissue_mesh->numFaces());
        for (int i = 0; i < tissue_mesh->numFaces(); i++)
        {
            const Vec3i& face = tissue_mesh->face(i);
            shape_msgs::msg::MeshTriangle tri;
            tri.vertex_indices[0] = face[0];
            tri.vertex_indices[1] = face[1];
            tri.vertex_indices[2] = face[2];

            _mesh_message.triangles[i] = tri;
        }

        _mesh_message.vertices.resize(tissue_mesh->numVertices());

        auto mesh_callback = 
            [this, tissue_mesh]() -> void {
                // update vertices
                for (int i = 0; i < tissue_mesh->numVertices(); i++)
                {
                    const Vec3r& vertex = tissue_mesh->vertex(i);
                    this->_mesh_message.vertices[i].x = vertex[0];
                    this->_mesh_message.vertices[i].y = vertex[1];
                    this->_mesh_message.vertices[i].z = vertex[2];
                    
                }

                this->_mesh_publisher->publish(this->_mesh_message);
            };
        
        _sim->addCallback(_publish_rate_Hz, mesh_callback);

    }

    // set up callback to publish mesh vertices as a point cloud
    if (Sim::VirtuosoTissueGraspingSimulation* tissue_sim = dynamic_cast<Sim::VirtuosoTissueGraspingSimulation*>(_sim))
    {
        _mesh_pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("tissue_mesh_vertices", 3);

        const Geometry::TetMesh* tissue_mesh = tissue_sim->tissueMesh(); 

        // set header
        _mesh_pcl_message.header.stamp = this->now();
        _mesh_pcl_message.header.frame_id = "/world";

        // add point fields
        _mesh_pcl_message.fields.resize(3);
        _mesh_pcl_message.fields[0].name = "x";
        _mesh_pcl_message.fields[0].offset = 0;
        _mesh_pcl_message.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        _mesh_pcl_message.fields[0].count = 1;

        _mesh_pcl_message.fields[1].name = "y";
        _mesh_pcl_message.fields[1].offset = sizeof(Real);
        _mesh_pcl_message.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        _mesh_pcl_message.fields[1].count = 1;

        _mesh_pcl_message.fields[2].name = "z";
        _mesh_pcl_message.fields[2].offset = 2*sizeof(Real);
        _mesh_pcl_message.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
        _mesh_pcl_message.fields[2].count = 1;

        _mesh_pcl_message.height = 1;
        _mesh_pcl_message.width = tissue_mesh->numVertices();
        _mesh_pcl_message.is_dense = true;
        _mesh_pcl_message.is_bigendian = false;

        _mesh_pcl_message.point_step = 3*sizeof(Real);
        _mesh_pcl_message.row_step = _mesh_pcl_message.point_step * _mesh_pcl_message.width;

        _mesh_pcl_message.data.resize(_mesh_pcl_message.row_step);

        auto mesh_pcl_callback = 
            [this, tissue_mesh]() -> void {
                // update vertices
                memcpy(this->_mesh_pcl_message.data.data(), tissue_mesh->vertices().data(), _mesh_pcl_message.data.size());

                this->_mesh_pcl_publisher->publish(this->_mesh_pcl_message);
            };
        
        _sim->addCallback(1.0/_publish_rate_Hz, mesh_pcl_callback);

    }
}