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

    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        std::cout << BOLD << "Initializing haptic device..." << RST << std::endl;
        _haptic_device_manager = std::make_unique<HapticDeviceManager>();
    }

    _out_file << "Tissue Grasping Simulation" << std::endl;
}

void TissueGraspingSimulation::setup()
{
    MeshUtils::createBeamObj("../resource/tissue/tissue_block1x1x0.1_1subdiv.obj", 1, 1, 0.1, 1);

    Simulation::setup();

    _viewer->enableMouseInteraction(false);
    
    

    _out_file << toString() << std::endl;

    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        _tissue_block = elastic_mesh_object;
    }
    
    assert(_tissue_block);

    // change camera orientation to 45 degrees looking down at the block of tissue
    _viewer->camera()->setViewDirection(easy3d::vec3(1, 0, -1));

    _tissue_block->fixVerticesWithMinZ();

    _out_file << _tissue_block->toString() << std::endl;
    
}

void TissueGraspingSimulation::_timeStep()
{
    if (_input_device == SimulationInputDevice::HAPTIC)
    {
        if (!_grasping && _haptic_device_manager->button1Pressed())
            _toggleTissueGrasping();
        else if (_grasping && !_haptic_device_manager->button1Pressed())
            _toggleTissueGrasping();

        const Eigen::Vector3d haptic_position = _haptic_device_manager->position()/100;
        Eigen::Vector3d position;
        position(0) = -haptic_position(2);
        position(1) = -haptic_position(0);
        position(2) = haptic_position(1);


        for (const auto& vd : _grasped_vertex_drivers)
        {
            vd->setPosition(position);
        }
    }

    OutputSimulation::_timeStep();
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
        std::set<unsigned> vertices_to_grasp = _getAllVerticesInGraspingArea();
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