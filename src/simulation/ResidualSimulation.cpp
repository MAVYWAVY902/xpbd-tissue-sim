#include "ResidualSimulation.hpp"
#include "MeshUtils.hpp"


ResidualSimulation::ResidualSimulation(const std::string& config_filename)
    : OutputSimulation(config_filename)
{
    _out_file << "Residual Simulation\n";
}

void ResidualSimulation::setup()
{
    Simulation::setup();

    _out_file << toString() << std::endl;
    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        _out_file << elastic_mesh_object->toString() << std::endl;
    }

    _out_file << "\n\nTime(s) DynamicsResidual PrimaryResidual ConstraintResidual VolumeRatio" << std::endl;
}

void ResidualSimulation::printInfo() const
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
    _out_file << _time << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio << std::endl;
}