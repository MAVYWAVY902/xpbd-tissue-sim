#include "BeamStretchSimulation.hpp"
#include "MeshUtils.hpp"

BeamStretchSimulation::BeamStretchSimulation(const std::string& config_filename)
    : Simulation(config_filename)
{

}

void BeamStretchSimulation::setup()
{
    Simulation::setup();

    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        // fix one side of the beam
        elastic_mesh_object->fixVerticesWithMinY();

        // grab middle vertex and lift it up
        const Eigen::Vector3d& min_coords = elastic_mesh_object->bboxMinCoords();
        const Eigen::Vector3d& max_coords = elastic_mesh_object->bboxMaxCoords();

        std::vector<unsigned> free_end_verts = elastic_mesh_object->getVerticesWithY(max_coords(1));
        for (int i = 0; i < free_end_verts.size(); i++)
        {
            Eigen::Vector3d vert = elastic_mesh_object->getVertex(free_end_verts[i]);
            VertexDriver::DriverFunction func = [=] (const double t) {
                return Eigen::Vector3d({vert(0), vert(1)+0.05*t, vert(2)});
            };

            VertexDriver vd("vertex"+std::to_string(i), free_end_verts[i], func);
            elastic_mesh_object->addVertexDriver(vd);
        }

        _out_file << "Material:\n\t" << "Density: " << elastic_mesh_object->material().density() << std::endl;
        _out_file << "\tE: " << elastic_mesh_object->material().E() << std::endl;
        _out_file << "\tnu: " << elastic_mesh_object->material().nu() << std::endl;
        _out_file << "\tmu: " << elastic_mesh_object->material().mu() << std::endl;
        _out_file << "\tlambda: " << elastic_mesh_object->material().lambda() << std::endl;
    }
}

void BeamStretchSimulation::update()
{
    Simulation::update();
}

void BeamStretchSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_time - _last_print_sim_time >= 1e-3)
    {
        _printInfo();
        _last_print_sim_time = _time;
    }
}

void BeamStretchSimulation::_printInfo()
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