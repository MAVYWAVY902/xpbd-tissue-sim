#include "ResidualSimulation.hpp"
#include "MeshUtils.hpp"


ResidualSimulation::ResidualSimulation(const std::string& config_filename)
    : Simulation(config_filename), _out_file("../output/out_" + _name + ".txt")
{
    _out_file << "Residual Simulation\n";
}

void ResidualSimulation::setup()
{
    Simulation::setup();

    _out_file << "Time step: " << _time_step << std::endl;
    if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(_mesh_objects[0]))
    {
        // elastic_mesh_object->stretch(0.5, 0.5, 0.5);
        _out_file << "Material:\n\t" << "Density: " << elastic_mesh_object->material().density() << std::endl;
        _out_file << "\tE: " << elastic_mesh_object->material().E() << std::endl;
        _out_file << "\tnu: " << elastic_mesh_object->material().nu() << std::endl;
        _out_file << "\tmu: " << elastic_mesh_object->material().mu() << std::endl;
        _out_file << "\tlambda: " << elastic_mesh_object->material().lambda() << std::endl;
    }

    if (XPBDMeshObject* xpbd_mesh_object = dynamic_cast<XPBDMeshObject*>(_mesh_objects[0]))
    {
        _out_file << "\nXPBD:" << std::endl;
        _out_file << "\tNum solver iters: " << xpbd_mesh_object->numSolverIters() << std::endl;
        _out_file << "\tSolve mode: " << xpbd_mesh_object->solveMode() << std::endl;
    }

    _out_file << "\n\nTime(s) DynamicsResidual PrimaryResidual ConstraintResidual VolumeRatio" << std::endl;
    _printInfo();

    // _updateGraphics();
}

void ResidualSimulation::update()
{
    Simulation::update();
}

void ResidualSimulation::_timeStep()
{
    Simulation::_timeStep();

    if (_time - _last_print_sim_time >= 1e-3)
    {
        _printInfo();
        _last_print_sim_time = _time;
    }

}

void ResidualSimulation::_printInfo()
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