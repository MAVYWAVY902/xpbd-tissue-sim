#include "TissueGraspingSimulation.hpp"
#include "MeshUtils.hpp"

TissueGraspingSimulation::TissueGraspingSimulation(const std::string& config_filename)
    : OutputSimulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<TissueGraspingSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();


    // extract the stretch velocity and time from the config object
    TissueGraspingSimulationConfig* tissue_grasping_simulation_config = dynamic_cast<TissueGraspingSimulationConfig*>(_config.get());
    _grasp_size = tissue_grasping_simulation_config->graspSize().value();
    _z_scaling = tissue_grasping_simulation_config->zScaling().value();
    _input_device = tissue_grasping_simulation_config->inputDevice().value();

    _grasp_tip_rotation = Eigen::Matrix3d::Identity();

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device..." << RST << std::endl;
        _haptic_device_manager = std::make_unique<HapticDeviceManager>();
    }

    _out_file << "Tissue Grasping Simulation" << std::endl;
}

void TissueGraspingSimulation::setup()
{
    // MeshUtils::createBeamObj("../resource/tissue/tissue_block1x1x0.1_1subdiv.obj", 1, 1, 0.1, 1);

    Simulation::setup();

    // _viewer->enableMouseInteraction(false);
    
    _grasp_tip = new RigidMeshObject("grasp_tip", "../resource/general/Cursor.stl");
    _grasp_tip->moveTo(Eigen::Vector3d(0,0,0));
    _grasp_tip->resize(0.01);
    addObject(_grasp_tip);

    _out_file << toString() << std::endl;

    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        _tissue_block = elastic_mesh_object;
    }
    
    assert(_tissue_block);

    // change camera orientation to 45 degrees looking down at the block of tissue

    // _tissue_block->fixVerticesWithMinZ();
    Eigen::Vector3d min_coords = _tissue_block->bboxMinCoords();
    Eigen::Vector3d max_coords = _tissue_block->bboxMaxCoords();
    unsigned v1 = _tissue_block->getClosestSurfaceVertex(min_coords(0), min_coords(1), min_coords(2));
    unsigned v2 = _tissue_block->getClosestSurfaceVertex(min_coords(0), max_coords(1), min_coords(2));
    unsigned v3 = _tissue_block->getClosestSurfaceVertex(max_coords(0), max_coords(1), min_coords(2));
    unsigned v4 = _tissue_block->getClosestSurfaceVertex(max_coords(0), min_coords(1), min_coords(2));
    unsigned v5 = _tissue_block->getClosestSurfaceVertex(min_coords(0), min_coords(1), max_coords(2));
    unsigned v6 = _tissue_block->getClosestSurfaceVertex(min_coords(0), max_coords(1), max_coords(2));
    unsigned v7 = _tissue_block->getClosestSurfaceVertex(max_coords(0), max_coords(1), max_coords(2));
    unsigned v8 = _tissue_block->getClosestSurfaceVertex(max_coords(0), min_coords(1), max_coords(2));
    _tissue_block->fixVertex(v1);
    _tissue_block->fixVertex(v2);
    _tissue_block->fixVertex(v3);
    _tissue_block->fixVertex(v4);
    _tissue_block->fixVertex(v5);
    _tissue_block->fixVertex(v6);
    _tissue_block->fixVertex(v7);
    _tissue_block->fixVertex(v8);
    



    _out_file << _tissue_block->toString() << std::endl;
    
}

void TissueGraspingSimulation::_updateGraphics()
{   
    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        if (!_grasping && _haptic_device_manager->button1Pressed())
            _toggleTissueGrasping();
        else if (_grasping && !_haptic_device_manager->button1Pressed())
            _toggleTissueGrasping();

        const Eigen::Vector3d haptic_position = _haptic_device_manager->position()/500;
        
        
        _grasp_tip_position = _transformInputPosition(haptic_position);

        // std::cout << "Position: " << _grasp_tip_position(0) << ", " << _grasp_tip_position(1) << ", " << _grasp_tip_position(2) << std::endl;
        
        Eigen::Matrix3d camera_rotation;
        easy3d::vec3 view_dir_easy3d = _viewer->camera()->viewDirection();
        easy3d::vec3 up_vec_easy3d = _viewer->camera()->upVector();
        easy3d::vec3 right_vec_easy3d = -_viewer->camera()->rightVector();
        Eigen::Vector3d view_dir(view_dir_easy3d[0], view_dir_easy3d[1], view_dir_easy3d[2]);
        Eigen::Vector3d up_vec(up_vec_easy3d[0], up_vec_easy3d[1], up_vec_easy3d[2]);
        Eigen::Vector3d right_vec(right_vec_easy3d[0], right_vec_easy3d[1], right_vec_easy3d[2]);
        camera_rotation.row(0) = right_vec;
        camera_rotation.row(1) = up_vec;
        camera_rotation.row(2) = view_dir;

        const Eigen::Matrix3d haptic_orientation = camera_rotation*_haptic_device_manager->orientation();

        _grasp_tip->moveTo(_grasp_tip_position);
        Eigen::Matrix3d rot_transpose = _grasp_tip_rotation.transpose();
        // _grasp_tip->rotate(rot_transpose);
        // _grasp_tip->rotate(haptic_orientation);

        _grasp_tip_rotation = haptic_orientation;

        for (const auto& vd : _grasped_vertex_drivers)
        {
            vd->setPosition(_grasp_tip_position);
        }
    }

    OutputSimulation::_updateGraphics();
}

void TissueGraspingSimulation::printInfo() const
{
    // std::cout << "Button1: " << _haptic_device_manager->button1Pressed() << "\tButton 2: " << _haptic_device_manager->button2Pressed() << std::endl;

    double primary_residual = 0;
    double constraint_residual = 0;
    double dynamics_residual = 0;
    double volume_ratio = 1;
    if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(_tissue_block))
    {
        primary_residual = elastic_mesh_object->primaryResidual();
        constraint_residual = elastic_mesh_object->constraintResidual();
        dynamics_residual = elastic_mesh_object->dynamicsResidual();
        volume_ratio = elastic_mesh_object->volumeRatio();
        // std::cout << "Time: " << _time << std::endl;
        // std::cout << "\tDynamics residual: " << elastic_mesh_object->dynamicsResidual() << std::endl;
        // std::cout << "\tPrimary residual: " << elastic_mesh_object->primaryResidual() << std::endl;
        // std::cout << "\tConstraint residual: " << elastic_mesh_object->constraintResidual() << std::endl;
        // std::cout << "\tVolume ratio: " << elastic_mesh_object->volumeRatio() << std::endl;
    }
    _out_file << _time << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio << std::endl;
}

void TissueGraspingSimulation::notifyMouseButtonPressed(int button, int action, int /* modifiers*/)
{
    if (_input_device != SimulationInputDevice::MOUSE)
    {
        return;
    }

    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    if (button == 0 && action == 1)
    {
        _toggleTissueGrasping();
    }
}

void TissueGraspingSimulation::notifyMouseMoved(double x, double y)
{
    
    if (_input_device != SimulationInputDevice::MOUSE)
    {
        return;
    }

    bool found;
    easy3d::vec3 mouse_pt = _viewer->point_under_pixel(x, y, found);
    Eigen::Vector3d new_mouse_pos;
    new_mouse_pos(0) = mouse_pt.x;
    new_mouse_pos(1) = mouse_pt.y;
    new_mouse_pos(2) = mouse_pt.z;

    for (const auto& vd : _grasped_vertex_drivers)
    {
        Eigen::Vector3d new_position;
        new_position(0) = vd->position()(0);
        new_position(1) = vd->position()(1);
        new_position(2) = vd->position()(2) - _z_scaling*(y - _mouse_pos_2d(1));
        vd->setPosition(new_position);
    }
    // for (const auto& vd : _grasped_vertex_drivers)
    // {
    //     vd->setPosition(new_mouse_pos);
    // }

    _mouse_pos_3d = new_mouse_pos;

    _mouse_pos_2d(0) = x;
    _mouse_pos_2d(1) = y;

    
}

void TissueGraspingSimulation::notifyKeyPressed(int /* key */, int action, int /* modifiers */)
{
    if (action > 0)
    {
        _viewer->camera()->setViewDirection(easy3d::vec3(0.0356192, 0.819275, 0.572293));
        _viewer->camera()->setPosition(easy3d::vec3(0.00371679, -0.163586, 0.261103));
    }
    
}

void TissueGraspingSimulation::_toggleTissueGrasping()
{
    if (_grasping)
    {
        for (const auto& vd : _grasped_vertex_drivers)
        {
            _tissue_block->removeVertexDriver(vd->vertexIndex());
        }
        _grasped_vertex_drivers.clear();

        _grasping = false;
    }
    else
    {
        std::set<unsigned> vertices_to_grasp;// = _getAllVerticesInGraspingArea();
        if (_input_device == SimulationInputDevice::MOUSE)
        {
            vertices_to_grasp.insert(_tissue_block->getClosestSurfaceVertex(_mouse_pos_3d(0), _mouse_pos_3d(1), _mouse_pos_3d(2)));
        }
        if (_input_device == SimulationInputDevice::HAPTIC)
        {
            vertices_to_grasp.insert(_tissue_block->getClosestSurfaceVertex(_grasp_tip_position(0), _grasp_tip_position(1), _grasp_tip_position(2)));
        }
        

        for (const auto& v : vertices_to_grasp)
        {
            // grab middle vertex and lift it up
            
            Eigen::Vector3d vertex_pos = _tissue_block->getVertex(v);


            std::shared_ptr<StaticVertexDriver> vd = std::make_shared<StaticVertexDriver>("tissue grasping", v, vertex_pos);
            _tissue_block->addVertexDriver(vd);

            _grasped_vertex_drivers.push_back(vd);
        }
        

        _grasping = true;
    }
}

std::set<unsigned> TissueGraspingSimulation::_getAllVerticesInGraspingArea()
{
    std::set<unsigned> vertices_to_grasp;
    const double step = 1e-3; // some small value to capture all vertices on the top plane inside the area
    // const double grab_size = 0.05;

    const int num_steps = _grasp_size / step;

    const Eigen::Vector3d& min_coords = _tissue_block->bboxMinCoords();
    const Eigen::Vector3d& max_coords = _tissue_block->bboxMaxCoords();
    Eigen::Vector3d vertex_grab_pos({min_coords(0) + (max_coords(0)-min_coords(0))*0.5, 
                                    min_coords(1) + (max_coords(1)-min_coords(1))*0.5,
                                    max_coords(2)});
    
    for (int i = 0; i <= num_steps; i++)
    {
        for (int j = 0; j <= num_steps; j++)
        {
            // grab middle vertex and lift it up
            
            unsigned vertex_ind = _tissue_block->getClosestVertex(vertex_grab_pos(0) - _grasp_size/2 + i*step, vertex_grab_pos(1) - _grasp_size/2 + j*step, vertex_grab_pos(2));

            vertices_to_grasp.insert(vertex_ind);
        }
    }

    return vertices_to_grasp;
}

Eigen::Vector3d TissueGraspingSimulation::_transformInputPosition(const Eigen::Vector3d& input_position)
{
    Eigen::Vector4d input_position_h(input_position(0), input_position(1), input_position(2), 1);

    Eigen::Matrix4d T_OC = Eigen::Matrix4d::Identity();
    T_OC(0,0) = -1;
    T_OC(2,2) = -1;
    T_OC(2,3) = 0.2; // distance

    easy3d::vec3 view_dir_easy3d = _viewer->camera()->viewDirection();
    easy3d::vec3 up_vec_easy3d = _viewer->camera()->upVector();
    easy3d::vec3 right_vec_easy3d = -_viewer->camera()->rightVector();
    easy3d::vec3 camera_position_easy3d = _viewer->camera()->position();
    Eigen::Vector4d view_dir(view_dir_easy3d[0], view_dir_easy3d[1], view_dir_easy3d[2], 0);
    Eigen::Vector4d up_vec(up_vec_easy3d[0], up_vec_easy3d[1], up_vec_easy3d[2], 0);
    Eigen::Vector4d right_vec(right_vec_easy3d[0], right_vec_easy3d[1], right_vec_easy3d[2], 0);
    Eigen::Vector4d camera_position(camera_position_easy3d[0], camera_position_easy3d[1], camera_position_easy3d[2], 1);

    Eigen::Matrix4d T_CW;
    T_CW.col(0) = right_vec;
    T_CW.col(1) = up_vec;
    T_CW.col(2) = view_dir;
    T_CW.col(3) = camera_position;

    Eigen::Vector4d transformed_position_h = T_CW * T_OC * input_position_h;

    return Eigen::Vector3d(transformed_position_h[0], transformed_position_h[1], transformed_position_h[2]);

}