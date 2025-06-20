#include "sim_bridge/SimBridge.hpp"

#include "simulation/VirtuosoSimulation.hpp"
#include "simulation/VirtuosoTissueGraspingSimulation.hpp"

template <>
class SimBridge<Sim::VirtuosoSimulation> : public rclcpp::Node
{
    public:
    SimBridge(Sim::VirtuosoSimulation* sim)
        : rclcpp::Node("sim_bridge"), _sim(sim)
    {

        this->declare_parameter("publish_rate_hz", 30.0);

        // set up callback to publish frames for arm1 (if it exists)
        if (_sim->virtuosoRobot()->hasArm1())
        {
            std::cout << "Setting up callback for Virtuoso arm 1 frames..." << std::endl;
            _arm1_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/output/arm1_frames", 10);

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

            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm1_callback);
        }

        // set up callback to publish frames for arm 2 (if it exists)
        if (_sim->virtuosoRobot()->hasArm2())
        {
            std::cout << "Setting up callback for Virtuoso arm 2 frames..." << std::endl;
            _arm2_frames_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("/output/arm2_frames", 10);
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

            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm2_callback);
        }

        // set up callback to publish mesh
        if (Sim::VirtuosoTissueGraspingSimulation* tissue_sim = dynamic_cast<Sim::VirtuosoTissueGraspingSimulation*>(_sim))
        {
            _mesh_publisher = this->create_publisher<shape_msgs::msg::Mesh>("/output/tissue_mesh", 3);

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
            
            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mesh_callback);

        }

        // set up callback to publish mesh vertices as a point cloud
        if (Sim::VirtuosoTissueGraspingSimulation* tissue_sim = dynamic_cast<Sim::VirtuosoTissueGraspingSimulation*>(_sim))
        {
            _mesh_pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output/tissue_mesh_vertices", 3);

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
            
            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), mesh_pcl_callback);

        }

        // set up callback to publish mesh vertices as a point cloud
        if (Sim::VirtuosoTissueGraspingSimulation* tissue_sim = dynamic_cast<Sim::VirtuosoTissueGraspingSimulation*>(_sim))
        {
            _partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output/partial_view_pc", 3);
            
            int hfov = 80;
            int vfov = 80;
            Real sample_density = 1.0;

            // set header
            _partial_view_pc_message.header.stamp = this->now();
            _partial_view_pc_message.header.frame_id = "/world";

            // add point fields
            _partial_view_pc_message.fields.resize(3);
            _partial_view_pc_message.fields[0].name = "x";
            _partial_view_pc_message.fields[0].offset = 0;
            _partial_view_pc_message.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
            _partial_view_pc_message.fields[0].count = 1;

            _partial_view_pc_message.fields[1].name = "y";
            _partial_view_pc_message.fields[1].offset = sizeof(Real);
            _partial_view_pc_message.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
            _partial_view_pc_message.fields[1].count = 1;

            _partial_view_pc_message.fields[2].name = "z";
            _partial_view_pc_message.fields[2].offset = 2*sizeof(Real);
            _partial_view_pc_message.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
            _partial_view_pc_message.fields[2].count = 1;

            _partial_view_pc_message.height = 1;
            _partial_view_pc_message.width = hfov * vfov * sample_density * sample_density;
            _partial_view_pc_message.is_dense = true;
            _partial_view_pc_message.is_bigendian = false;

            _partial_view_pc_message.point_step = 3*sizeof(Real);
            _partial_view_pc_message.row_step = _partial_view_pc_message.point_step * _partial_view_pc_message.width;

            _partial_view_pc_message.data.resize(_partial_view_pc_message.row_step);

            auto partial_view_pc_callback = 
                [this, hfov, vfov, sample_density]() -> void {

                    const Vec3r& cam_position = this->_sim->graphicsScene()->cameraPosition();
                    const Vec3r& cam_view_dir = this->_sim->graphicsScene()->cameraViewDirection();
                    const Vec3r& cam_up_dir = this->_sim->graphicsScene()->cameraUpDirection();

                    std::vector<Vec3r> points = this->_sim->embreeScene()->partialViewPointCloud(cam_position, cam_view_dir, cam_up_dir, hfov, vfov, sample_density);
                    this->_partial_view_pc_message.width = points.size();

                    for (unsigned i = 0; i < points.size(); i++)
                    {
                        memcpy((Real*)this->_partial_view_pc_message.data.data() + 3*i, points[i].data(), sizeof(Real)*3);
                    }
                    

                    this->_partial_view_pc_publisher->publish(this->_partial_view_pc_message);
                };
            
            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), partial_view_pc_callback);

        }

        // set up subscriber callback to take in joint state for arm 1
        if (_sim->virtuosoRobot()->hasArm1())
        {
            auto arm1_state_callback = 
                [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
                    const auto [ot_rot, ot_trans, it_rot, it_trans, tool] = this->_jointMsgToJointState(msg.get());

                    this->_sim->setArm1JointState(ot_rot, ot_trans, it_rot, it_trans, tool);
            };

            _arm1_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/input/arm1_joint_state", 10, arm1_state_callback);
        }
        
        // set up subscriber callback to take in joint state for arm 2
        if (_sim->virtuosoRobot()->hasArm2())
        {
            auto arm2_state_callback = 
                [this](sensor_msgs::msg::JointState::UniquePtr msg) -> void {
                    const auto [ot_rot, ot_trans, it_rot, it_trans, tool] = this->_jointMsgToJointState(msg.get());

                    this->_sim->setArm2JointState(ot_rot, ot_trans, it_rot, it_trans, tool);
            };

            _arm2_joint_state_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("/input/arm2_joint_state", 10, arm2_state_callback);
        }
        
    }

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
    std::tuple<double, double, double, double, int> _jointMsgToJointState(sensor_msgs::msg::JointState* msg) const
    {
        // joint states must contain five positions: inner rotation, outer rotation, inner
        // translation, and outer translation, and tool (in any order).
        if (msg->name.size() != 5) {
            // joint state doesn't contain the necessary fields; it is malformed.
            RCLCPP_WARN(
                get_logger(),
                "Joint state contains %lu states in 'name'. Valid joint states contain 5 states.",
                msg->name.size());
            return std::tuple<double, double, double, double, int>();
        } else if (msg->position.size() != 5) {
            // joint state doesn't contain the necessary fields; it is malformed.
            RCLCPP_WARN(
                get_logger(),
                "Joint state contains %lu states in 'position'. Valid joint states contain 5 states.",
                msg->position.size());
            return std::tuple<double, double, double, double, int>();
        }

        bool irSet = false, orSet = false, itSet = false, otSet = false, tSet = false;
        double ot_rot, ot_trans, it_rot, it_trans;
        int tool;

        for (uint32_t idx = 0; idx < 5; ++idx) {
            const auto &name = msg->name[idx];
            const auto scalar = msg->position[idx];

            if (name == "inner_rotation" || name == "innerRotation" || name == "ir") {
                irSet = true;
                it_rot = scalar;
            } else if (name == "outer_rotation" || name == "outerRotation" || name == "or") {
                orSet = true;
                ot_rot = scalar;
            } else if (name == "inner_translation" || name == "innerTranslation" || name == "it") {
                itSet = true;
                it_trans = scalar;
            } else if (name == "outer_translation" || name == "outerTranslation" || name == "ot") {
                otSet = true;
                ot_trans = scalar;
            } else if (name == "tool") {
                tSet = true;
                tool = scalar;
            } else {
                // this joint state was referenced incorrectly
                RCLCPP_WARN(
                get_logger(), "Attempt to set nonexistent joint '%s' failed.", name.c_str());
                return std::tuple<double, double, double, double, int>();
            }
        }

        // ensure that every joint was set
        if (irSet == false || orSet == false || itSet == false || otSet == false || tSet == false) {
            if (irSet == false) {
                RCLCPP_WARN(get_logger(), "Joint state message did not set inner_rotation joint.");
            } else if (orSet == false) {
                RCLCPP_WARN(get_logger(), "Joint state message did not set outer_rotation joint.");
            } else if (itSet == false) {
                RCLCPP_WARN(get_logger(), "Joint state message did not set inner_translation joint.");
            } else if (otSet == false) {
                RCLCPP_WARN(get_logger(), "Joint state message did not set outer_translation joint.");
            } else if (tSet == false) {
                RCLCPP_WARN(get_logger(), "Joint state message did not set tool joint.");
            }
            return std::tuple<double, double, double, double, int>();
        }

        return std::make_tuple(ot_rot, ot_trans, it_rot, it_trans, tool);
    }
    

    private:
    /** Publishers */
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm1_frames_publisher;     // publishes coordinate frames along backbone of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr _arm2_frames_publisher;     // publishes coordinate frames along backbone of arm2

    shape_msgs::msg::Mesh _mesh_message;    // pre-allocated mesh ROS message for speed (assuming faces and number of vertices stay the same)
    rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr _mesh_publisher;    // publishes the current tissue mesh (all vertices and surface faces)

    sensor_msgs::msg::PointCloud2 _mesh_pcl_message;    // pre-allocated mesh point cloud ROS message for speed (assuming number of vertices stays the same)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _mesh_pcl_publisher;    // publishes the current mesh vertices as a ROS point cloud (for easy ROS visualization)

    sensor_msgs::msg::PointCloud2 _partial_view_pc_message;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _partial_view_pc_publisher;

    /** Subscriptions */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm1_joint_state_subscriber;     // subscribes to joint state commands for arm1
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm2_joint_state_subscriber;     // subscribes to joint state commands for arm2


    /** Pointer to the actively running Simulation object */
    Sim::VirtuosoSimulation* _sim;
};