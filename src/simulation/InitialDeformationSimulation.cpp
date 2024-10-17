#include "InitialDeformationSimulation.hpp"

#include "MeshUtils.hpp"

#include "XPBDMeshObject.hpp"

#include <regex>

InitialDeformationSimulation::InitialDeformationSimulation(const std::string& config_filename)
    : OutputSimulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<InitialDeformationSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();

    // extract the stretch velocity and time from the config object
    InitialDeformationSimulationConfig* initial_deformation_simulation_config = dynamic_cast<InitialDeformationSimulationConfig*>(_config.get());
    _deformation_type = initial_deformation_simulation_config->deformationType().value();
    _deformation_factor = initial_deformation_simulation_config->deformationFactor().value();

    // since this is a strictly elastic simulation, make sure g is 0 so there are no external forces
    assert(_g_accel == 0);

    // write some general info about the simulation to file
    _out_file << "Initial Deformation Simulation" << std::endl;
}

std::string InitialDeformationSimulation::toString() const
{
    return Simulation::toString();
}

void InitialDeformationSimulation::setup()
{

    // call the parent setup
    Simulation::setup();

    _out_file << toString() << std::endl;

    for (auto& mesh_object : _mesh_objects) {

        initial_vertices.push_back(mesh_object->vertices());

        // fix the minY face of the beam, and attach drivers to stretch the maxY face of the beam
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object.get()))
        {
            if (_deformation_type == DeformationType::VOLUMETRIC_EXPANSION)
            {
                double scaling = std::cbrt(_deformation_factor);
                elastic_mesh_object->stretch(scaling, scaling, scaling);
            }
            else if (_deformation_type == DeformationType::VOLUMETRIC_COMPRESSION)
            {
                double scaling = 1/std::cbrt(_deformation_factor);
                elastic_mesh_object->stretch(scaling, scaling, scaling);
            }
            

            // write mesh object information to file
            _out_file << elastic_mesh_object->toString() << "\n" << std::endl;
        }
    }

    // write appropriate CSV column headers
    _out_file << "\nTime(s)";
    for (auto& mesh_object : _mesh_objects)
    {
        if (ElasticMeshObject* elastic_mesh_object = dynamic_cast<ElasticMeshObject*>(mesh_object.get()))
        {
            std::regex r("\\s+");
            const std::string& name = std::regex_replace(elastic_mesh_object->name(), r, "");
            _out_file << " "+name+"DynamicsResidual" << " "+name+"PrimaryResidual" << " "+name+"ConstraintResidual" << " "+name+"VolumeRatio";
        }
    }
    _out_file << std::endl;
}

void InitialDeformationSimulation::printInfo() const
{
    double primary_residual = 0;
    double constraint_residual = 0;
    double dynamics_residual = 0;
    double volume_ratio = 1;
    _out_file << _time;
    for (int i = 0; i < _mesh_objects.size(); i++) {
        MeshObject* mesh_object = _mesh_objects[i].get();

        MeshObject::VerticesMat current_vertices = mesh_object->vertices();
        double frob_norm_ss_err = (current_vertices - initial_vertices[i]).norm();
        double ss_err_rms = std::sqrt(frob_norm_ss_err / current_vertices.rows());

        if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(mesh_object))
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
        dynamics_residual = ss_err_rms;
        _out_file << " " << dynamics_residual << " " << primary_residual << " " << constraint_residual << " " << volume_ratio;
    }
    _out_file << std::endl;
}