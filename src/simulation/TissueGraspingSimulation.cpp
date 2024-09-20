#include "TissueGraspingSimulation.hpp"
#include "MeshUtils.hpp"

TissueGraspingSimulation::TissueGraspingSimulation(const std::string& config_filename)
    : OutputSimulation(config_filename)
{
    _out_file << "Tissue Grasping Simulation" << std::endl;
}

void TissueGraspingSimulation::setup()
{
    MeshUtils::createBeamObj("../resource/tissue/tissue_block1x1x0.1_1subdiv.obj", 1, 1, 0.1, 1);

    Simulation::setup();

    _out_file << toString() << std::endl;

    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        _tissue_block = elastic_mesh_object;
    }
    
    assert(_tissue_block);

    _tissue_block->fixVerticesWithMinZ();

    _out_file << _tissue_block->toString() << std::endl;
    
}

void TissueGraspingSimulation::printInfo() const
{
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
    // button = 0 ==> left mouse button
    // button = 1 ==> right mouse button
    // action = 0 ==> mouse up
    // action = 1 ==> mouse down
    
    if (button == 0 && action == 1)
    {
        _toggleTissueGrasping();
    }
}

void TissueGraspingSimulation::_toggleTissueGrasping()
{
    if (_grasping)
    {
        for (const auto& v : _grasped_vertices)
        {
            _tissue_block->removeVertexDriver(v);
        }
        _grasped_vertices.clear();

        _grasping = false;
    }
    else
    {
        // grab middle vertex and lift it up
        const Eigen::Vector3d& min_coords = _tissue_block->bboxMinCoords();
        const Eigen::Vector3d& max_coords = _tissue_block->bboxMaxCoords();
        Eigen::Vector3d vertex_grab_pos({min_coords(0) + (max_coords(0)-min_coords(0))*0.5, 
                                        min_coords(1) + (max_coords(1)-min_coords(1))*0.5,
                                        max_coords(2)});
        unsigned vertex_ind = _tissue_block->getClosestVertex(vertex_grab_pos(0), vertex_grab_pos(1), vertex_grab_pos(2));

        const double cur_time = _time;

        VertexDriver::DriverFunction func = [=] (const double t) {
            return Eigen::Vector3d({vertex_grab_pos(0), vertex_grab_pos(1), vertex_grab_pos(2) + 0.5*(t-cur_time)});
        };

        VertexDriver vd("tissue grasping", vertex_ind, func);
        _tissue_block->addVertexDriver(vd);

        _grasped_vertices.push_back(vertex_ind);

        _grasping = true;
    }
}