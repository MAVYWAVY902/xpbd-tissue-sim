#include "sim_bridge/SimBridge.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int8.hpp"

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
                    message.header.frame_id = "VB";

                    const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                    for (const auto& frame : ot_frames)
                    {
                        geometry_msgs::msg::Pose pose;

                        const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();

                        const Vec3r& origin = transform.translation();
                        const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
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

                        const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();

                        const Vec3r& origin = transform.translation();
                        const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
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
                    message.header.frame_id = "VB";

                    const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                    for (const auto& frame : ot_frames)
                    {
                        geometry_msgs::msg::Pose pose;

                        const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();

                        const Vec3r& origin = transform.translation();
                        const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
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

                        const Geometry::TransformationMatrix transform = vb_transform_inv * frame.transform();

                        const Vec3r& origin = transform.translation();
                        const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
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

        // set up callback to publish tip frame for arm1 (if it exists)
        if (_sim->virtuosoRobot()->hasArm1())
        {
            std::cout << "Setting up callback for Virtuoso arm 1 tip frame..." << std::endl;
            _arm1_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm1_tip_frame", 10);

            auto arm1_callback = 
                [this]() -> void {
                    const Sim::VirtuosoArm* arm1 = this->_sim->virtuosoRobot()->arm1();
                    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm1->innerTubeFrames();


                    auto message = geometry_msgs::msg::PoseStamped();
                    message.header.stamp = this->now();
                    message.header.frame_id = "VB";

                    const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                    const Geometry::CoordinateFrame& tip_frame = it_frames.back();

                    geometry_msgs::msg::Pose pose;

                    const Geometry::TransformationMatrix transform = vb_transform_inv * tip_frame.transform();

                    const Vec3r& origin = transform.translation();
                    const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
                    message.pose.position.x = origin[0];
                    message.pose.position.y = origin[1];
                    message.pose.position.z = origin[2];

                    message.pose.orientation.x = quat[0];
                    message.pose.orientation.y = quat[1];
                    message.pose.orientation.z = quat[2];
                    message.pose.orientation.w = quat[3];
                    
                    this->_arm1_tip_frame_publisher->publish(message);
                };

            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm1_callback);
        }

        // set up callback to publish tip frame for arm2 (if it exists)
        if (_sim->virtuosoRobot()->hasArm2())
        {
            std::cout << "Setting up callback for Virtuoso arm 2 tip frame..." << std::endl;
            _arm2_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm2_tip_frame", 10);

            auto arm2_callback = 
                [this]() -> void {
                    const Sim::VirtuosoArm* arm2 = this->_sim->virtuosoRobot()->arm2();
                    const Sim::VirtuosoArm::InnerTubeFramesArray& it_frames = arm2->innerTubeFrames();


                    auto message = geometry_msgs::msg::PoseStamped();
                    message.header.stamp = this->now();
                    message.header.frame_id = "VB";

                    const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                    const Geometry::CoordinateFrame& tip_frame = it_frames.back();

                    geometry_msgs::msg::Pose pose;

                    const Geometry::TransformationMatrix transform = vb_transform_inv * tip_frame.transform();

                    const Vec3r& origin = transform.translation();
                    const Vec4r& quat = GeometryUtils::matToQuat(transform.rotMat());
                    message.pose.position.x = origin[0];
                    message.pose.position.y = origin[1];
                    message.pose.position.z = origin[2];

                    message.pose.orientation.x = quat[0];
                    message.pose.orientation.y = quat[1];
                    message.pose.orientation.z = quat[2];
                    message.pose.orientation.w = quat[3];
                    
                    this->_arm2_tip_frame_publisher->publish(message);
                };

            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm2_callback);
        }

        // set up callback to publish commanded tip position for arm1 (if it exists)
        if (_sim->virtuosoRobot()->hasArm1())
        {
            std::cout << "Setting up callback for Virtuoso arm 1 commanded tip frame..." << std::endl;
            _arm1_commanded_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm1_commanded_tip_frame", 10);

            auto arm1_callback = 
                [this]() -> void {
                    const Sim::VirtuosoArm* arm1 = this->_sim->virtuosoRobot()->arm1();
                    const Vec3r& arm1_commanded_position = arm1->commandedTipPosition();


                    auto message = geometry_msgs::msg::PoseStamped();
                    message.header.stamp = this->now();
                    message.header.frame_id = "VB";

                    const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();

                    const Vec3r& origin = vb_transform_inv.rotMat() * arm1_commanded_position + vb_transform_inv.translation();
                    message.pose.position.x = origin[0];
                    message.pose.position.y = origin[1];
                    message.pose.position.z = origin[2];

                    /** commanded tip position has no orientation */
                    message.pose.orientation.x = 0;
                    message.pose.orientation.y = 0;
                    message.pose.orientation.z = 0;
                    message.pose.orientation.w = 1;
                    
                    this->_arm1_commanded_tip_frame_publisher->publish(message);
                };

            _sim->addCallback(1.0/this->get_parameter("publish_rate_hz").as_double(), arm1_callback);
        }

        // set up callback to publish tip frame for arm2 (if it exists)
        if (_sim->virtuosoRobot()->hasArm2())
        {
            std::cout << "Setting up callback for Virtuoso arm 2 commanded tip frame..." << std::endl;
            _arm2_commanded_tip_frame_publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("/output/arm2_commanded_tip_frame", 10);

            auto arm2_callback = 
                [this]() -> void {
                    const Sim::VirtuosoArm* arm2 = this->_sim->virtuosoRobot()->arm2();
                    const Vec3r& arm2_commanded_position = arm2->commandedTipPosition();


                    auto message = geometry_msgs::msg::PoseStamped();
                    message.header.stamp = this->now();
                    message.header.frame_id = "VB";

                    const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                    

                    const Vec3r& origin = vb_transform_inv.rotMat() * arm2_commanded_position + vb_transform_inv.translation();
                    message.pose.position.x = origin[0];
                    message.pose.position.y = origin[1];
                    message.pose.position.z = origin[2];

                    /** commanded tip position has no orientation */
                    message.pose.orientation.x = 0;
                    message.pose.orientation.y = 0;
                    message.pose.orientation.z = 0;
                    message.pose.orientation.w = 1;
                    
                    this->_arm2_commanded_tip_frame_publisher->publish(message);
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
            _mesh_pcl_message.header.frame_id = "sim";

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
            _trachea_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output/trachea_partial_view_pc", 3);
            _tumor_partial_view_pc_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output/tumor_partial_view_pc", 3);
            
            this->declare_parameter("partial_view_pc", true);
            this->declare_parameter("partial_view_pc_hfov", 80.0);
            this->declare_parameter("partial_view_pc_vfov", 30.0);
            this->declare_parameter("partial_view_pc_sample_density", 1.0);

            Real hfov_deg = this->get_parameter("partial_view_pc_hfov").as_double();
            Real vfov_deg = this->get_parameter("partial_view_pc_vfov").as_double();
            Real sample_density = this->get_parameter("partial_view_pc_sample_density").as_double();

            // lambda function to configure point cloud messages
            auto configure_pcl_message = [&](sensor_msgs::msg::PointCloud2& pcl_msg)
            {

                // sensor_msgs::PointCloud2Modifier modifier(pcl_msg);
                // auto datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
                // modifier.setPointCloud2Fields(3,
                //     "x", 1, datatype,
                //     "y", 1, datatype,
                //     "z", 1, datatype
                // );

                // set header
                pcl_msg.header.stamp = this->now();
                pcl_msg.header.frame_id = "VB";

                // // add point fields
                pcl_msg.fields.resize(3);
                pcl_msg.fields[0].name = "x";
                pcl_msg.fields[0].offset = 0;
                pcl_msg.fields[0].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
                pcl_msg.fields[0].count = 1;

                pcl_msg.fields[1].name = "y";
                pcl_msg.fields[1].offset = sizeof(Real);
                pcl_msg.fields[1].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
                pcl_msg.fields[1].count = 1;

                pcl_msg.fields[2].name = "z";
                pcl_msg.fields[2].offset = 2*sizeof(Real);
                pcl_msg.fields[2].datatype = (typeid(Real) == typeid(double)) ? sensor_msgs::msg::PointField::FLOAT64 : sensor_msgs::msg::PointField::FLOAT32;
                pcl_msg.fields[2].count = 1;

                pcl_msg.height = 1;
                pcl_msg.width = hfov_deg * vfov_deg * sample_density * sample_density;
                pcl_msg.is_dense = true;
                pcl_msg.is_bigendian = false;

                pcl_msg.point_step = 3*sizeof(Real);
                pcl_msg.row_step = pcl_msg.point_step * pcl_msg.width;

                pcl_msg.data.resize(pcl_msg.row_step);
            };

            configure_pcl_message(_trachea_partial_view_pc_message);
            configure_pcl_message(_tumor_partial_view_pc_message);
            

            auto partial_view_pc_callback = 
                [this]() -> void {

                    if (!this->get_parameter("partial_view_pc").as_bool())
                        return;

                    const Vec3r& cam_position = this->_sim->graphicsScene()->cameraPosition();
                    const Vec3r& cam_view_dir = this->_sim->graphicsScene()->cameraViewDirection();
                    const Vec3r& cam_up_dir = this->_sim->graphicsScene()->cameraUpDirection();

                    Real hfov_deg = this->get_parameter("partial_view_pc_hfov").as_double();
                    Real vfov_deg = this->get_parameter("partial_view_pc_vfov").as_double();
                    Real sample_density = this->get_parameter("partial_view_pc_sample_density").as_double();

                    this->_sim->updateEmbreeScene();
                    std::vector<Geometry::PointsWithClass> point_clouds = 
                        this->_sim->embreeScene()->partialViewPointCloudsWithClass(cam_position, cam_view_dir, cam_up_dir, hfov_deg, vfov_deg, sample_density);
                    
                    // go through returned point clouds and find the ones that match the trachea and tumor classes
                    for (auto& pc : point_clouds)
                    {
                        // transform points to VB frame
                        const Geometry::CoordinateFrame& vb_frame = this->_sim->virtuosoRobot()->VBFrame();
                        const Geometry::TransformationMatrix vb_transform_inv = vb_frame.transform().inverse();
                        for (unsigned i = 0; i < pc.points.size(); i++)
                        {
                            pc.points[i] = vb_transform_inv.rotMat()*pc.points[i] + vb_transform_inv.translation();
                        }

                        if (pc.classification == Sim::VirtuosoTissueGraspingSimulation::TissueClasses::TRACHEA)
                        {
                            this->_trachea_partial_view_pc_message.header.stamp = this->now();
                            this->_trachea_partial_view_pc_message.width = pc.points.size();
                            this->_trachea_partial_view_pc_message.row_step = this->_trachea_partial_view_pc_message.width * this->_trachea_partial_view_pc_message.point_step;
                            this->_trachea_partial_view_pc_message.data.resize(this->_trachea_partial_view_pc_message.row_step);
                            // make sure we have enough space allocated (this only won't be the case if the user changes the parameters of the partial view point cloud in the middle of running the sim)
                            // size_t data_size = hfov_deg * vfov_deg * sample_density * sample_density * this->_trachea_partial_view_pc_message.point_step;
                            
                            // if (data_size != this->_trachea_partial_view_pc_message.data.size())
                            // {
                            //     this->_trachea_partial_view_pc_message.data.resize(data_size);
                            // }

                            for (unsigned i = 0; i < pc.points.size(); i++)
                            {
                                memcpy((Real*)this->_trachea_partial_view_pc_message.data.data() + 3*i, pc.points[i].data(), sizeof(Real)*3);
                            }
                        }

                        else if (pc.classification == Sim::VirtuosoTissueGraspingSimulation::TissueClasses::TUMOR)
                        {
                            this->_tumor_partial_view_pc_message.header.stamp = this->now();
                            this->_tumor_partial_view_pc_message.width = pc.points.size();
                            this->_tumor_partial_view_pc_message.row_step = this->_tumor_partial_view_pc_message.width * this->_tumor_partial_view_pc_message.point_step;
                            this->_tumor_partial_view_pc_message.data.resize(this->_tumor_partial_view_pc_message.row_step);

                            // make sure we have enough space allocated (this only won't be the case if the user changes the parameters of the partial view point cloud in the middle of running the sim)
                            // size_t data_size = hfov_deg * vfov_deg * sample_density * sample_density * this->_tumor_partial_view_pc_message.point_step;
                            // this->_tumor_partial_view_pc_message.row_step = data_size;
                            // if (data_size != this->_tumor_partial_view_pc_message.data.size())
                            // {
                            //     this->_tumor_partial_view_pc_message.data.resize(data_size);
                            // }

                            for (unsigned i = 0; i < pc.points.size(); i++)
                            {
                                memcpy((Real*)this->_tumor_partial_view_pc_message.data.data() + 3*i, pc.points[i].data(), sizeof(Real)*3);
                            }
                        }
                    }

                    // publish the messages
                    this->_trachea_partial_view_pc_publisher->publish(this->_trachea_partial_view_pc_message);
                    this->_tumor_partial_view_pc_publisher->publish(this->_tumor_partial_view_pc_message);
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

        // set up subscriber callback to take in tip position for arm 1
        if (_sim->virtuosoRobot()->hasArm1())
        {
            auto arm1_tip_pos_callback = 
                [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
                    const Vec3r local_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

                    const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();
                    this->_sim->setArm1TipPosition(global_pos);
            };

            _arm1_tip_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/input/arm1_tip_pos", 10, arm1_tip_pos_callback);
        }

        // set up subscriber callback to take in tip position for arm2
        if (_sim->virtuosoRobot()->hasArm2())
        {
            auto arm2_tip_pos_callback =
                [this](geometry_msgs::msg::PoseStamped::UniquePtr msg) -> void {
                    const Vec3r local_pos(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

                    const Geometry::CoordinateFrame& frame = this->_sim->virtuosoRobot()->VBFrame();
                    const Vec3r global_pos = frame.transform().rotMat()*local_pos + frame.origin();

                    this->_sim->setArm2TipPosition(global_pos);
            };

            _arm2_tip_position_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("/input/arm2_tip_pos", 10, arm2_tip_pos_callback);
        }

        // set up subscriber callback to take in tool state for arm 1
        if (_sim->virtuosoRobot()->hasArm1())
        {
            auto arm1_tool_state_callback = 
                [this](std_msgs::msg::Int8::UniquePtr msg) -> void {
                    this->_sim->setArm1ToolState(msg->data);
            };

            _arm1_tool_state_subscriber = this->create_subscription<std_msgs::msg::Int8>("/input/arm1_tool_state", 10, arm1_tool_state_callback);
        }

        // set up subscriber callback to take in tool state for arm 2
        if (_sim->virtuosoRobot()->hasArm2())
        {
            auto arm2_tool_state_callback = 
                [this](std_msgs::msg::Int8::UniquePtr msg) -> void {
                    this->_sim->setArm2ToolState(msg->data);
            };

            _arm2_tool_state_subscriber = this->create_subscription<std_msgs::msg::Int8>("/intput/arm2_tool_state", 10, arm2_tool_state_callback);
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

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_tip_frame_publisher;          // publishes the tip frame of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_tip_frame_publisher;          // publishes the tip frame of arm2

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_commanded_tip_frame_publisher;  // publishes the commanded tip position of arm1
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_commanded_tip_frame_publisher;  // published the commanded tip position of arm2

    shape_msgs::msg::Mesh _mesh_message;    // pre-allocated mesh ROS message for speed (assuming faces and number of vertices stay the same)
    rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr _mesh_publisher;    // publishes the current tissue mesh (all vertices and surface faces)

    sensor_msgs::msg::PointCloud2 _mesh_pcl_message;    // pre-allocated mesh point cloud ROS message for speed (assuming number of vertices stays the same)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _mesh_pcl_publisher;    // publishes the current mesh vertices as a ROS point cloud (for easy ROS visualization)

    sensor_msgs::msg::PointCloud2 _trachea_partial_view_pc_message;
    sensor_msgs::msg::PointCloud2 _tumor_partial_view_pc_message;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _trachea_partial_view_pc_publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr _tumor_partial_view_pc_publisher;

    /** Subscriptions */
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm1_joint_state_subscriber;     // subscribes to joint state commands for arm1
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _arm2_joint_state_subscriber;     // subscribes to joint state commands for arm2
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _arm1_tip_position_subscriber;     // subscribes to tip position commands for arm1
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _arm2_tip_position_subscriber;     // subscribes to tip position commands for arm2
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _arm1_tool_state_subscriber;              // subscribes to tool state commands for arm1
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _arm2_tool_state_subscriber;              // subscribes to tool state commands for arm2


    /** Pointer to the actively running Simulation object */
    Sim::VirtuosoSimulation* _sim;
};