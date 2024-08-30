#include "TissueGraspingSimulation.hpp"
#include "MeshUtils.hpp"

TissueGraspingSimulation::TissueGraspingSimulation(const std::string& config_filename)
    : Simulation(config_filename)
{

}

void TissueGraspingSimulation::setup()
{
    MeshUtils::createTissueBlock("../resource/1x1x0.25_tissueblock4-2.obj", 1, 1, 0.25, 4, 2);

    Simulation::setup();

    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        elastic_mesh_object->fixVerticesWithMinZ();

        // grab middle vertex and lift it up
        const Eigen::Vector3d& min_coords = elastic_mesh_object->bboxMinCoords();
        const Eigen::Vector3d& max_coords = elastic_mesh_object->bboxMaxCoords();
        Eigen::Vector3d vertex_grab_pos({min_coords(0) + (max_coords(0)-min_coords(0))*0.5, 
                                        min_coords(1) + (max_coords(1)-min_coords(1))*0.5,
                                        max_coords(2)});
        unsigned vertex_ind = elastic_mesh_object->getClosestVertex(vertex_grab_pos(0), vertex_grab_pos(1), vertex_grab_pos(2));

        VertexDriver::DriverFunction func = [=] (const double t) {
            return Eigen::Vector3d({vertex_grab_pos(0), vertex_grab_pos(1), vertex_grab_pos(2) + 0.5*t});
        };

        VertexDriver vd("tissue grasping", vertex_ind, func);
        elastic_mesh_object->addVertexDriver(vd);

        _out_file << "Material:\n\t" << "Density: " << elastic_mesh_object->material().density() << std::endl;
        _out_file << "\tE: " << elastic_mesh_object->material().E() << std::endl;
        _out_file << "\tnu: " << elastic_mesh_object->material().nu() << std::endl;
        _out_file << "\tmu: " << elastic_mesh_object->material().mu() << std::endl;
        _out_file << "\tlambda: " << elastic_mesh_object->material().lambda() << std::endl;
    }
}

void TissueGraspingSimulation::update()
{
    Simulation::update();
}

void TissueGraspingSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_time - _last_print_sim_time >= 1e-3)
    {
        _printInfo();
        _last_print_sim_time = _time;
    }
}

void TissueGraspingSimulation::_printInfo()
{
    double primary_residual = 0;
    double constraint_residual = 0;
    double dynamics_residual = 0;
    double volume_ratio = 1;
    if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(_mesh_objects[0]))
    {
        primary_residual = elastic_mesh_object->primaryResidual();
        constraint_residual = elastic_mesh_object->constraintResidual();
        dynamics_residual = elastic_mesh_object->dynamicsResidual();
        volume_ratio = elastic_mesh_object->volumeRatio();
        std::cout << "Time: " << _time << std::endl;
        std::cout << "\tDynamics residual: " << elastic_mesh_object->dynamicsResidual() << std::endl;
        std::cout << "\tPrimary residual: " << elastic_mesh_object->primaryResidual() << std::endl;
        std::cout << "\tConstraint residual: " << elastic_mesh_object->constraintResidual() << std::endl;
        std::cout << "\tVolume ratio: " << elastic_mesh_object->volumeRatio() << std::endl;
    }
    // _out_file << _time << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio << std::endl;
}