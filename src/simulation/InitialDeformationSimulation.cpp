#include "simulation/InitialDeformationSimulation.hpp"

#include "utils/MeshUtils.hpp"

#include "simobject/XPBDMeshObject.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"

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

std::string InitialDeformationSimulation::deformationType() const
{
    if (_deformation_type == DeformationType::VOLUMETRIC_EXPANSION)
        return "Volumetric Expansion";
    if (_deformation_type == DeformationType::VOLUMETRIC_COMPRESSION)
        return "Volumetric Compression";
    if (_deformation_type == DeformationType::COLLAPSE_TO_PLANE)
        return "Collapse to Plane";
}

std::string InitialDeformationSimulation::toString() const
{
    return Simulation::toString() + "\n\tDeformation type: " + deformationType() + "\n\tDeformation factor: " + std::to_string(_deformation_factor);
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
            else if (_deformation_type == DeformationType::COLLAPSE_TO_PLANE)
            {
                elastic_mesh_object->stretch(1, 1, 0);
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
            _out_file << " "+name+"VelocityRMS" << " "+name+"VolumeRatio";
        }
    }
    _out_file << std::endl;

    printInfo();
}

void InitialDeformationSimulation::printInfo() const
{
    double volume_ratio = 1;
    _out_file << _time;
    for (int i = 0; i < _mesh_objects.size(); i++) {
        MeshObject* mesh_object = _mesh_objects[i].get();

        MeshObject::VerticesMat velocities = mesh_object->velocities();
        const double frob_norm = velocities.norm();
        const double velocity_rms = std::sqrt(frob_norm/velocities.rows());

        if (XPBDMeshObject* elastic_mesh_object = dynamic_cast<XPBDMeshObject*>(mesh_object))
        {
            // volume_ratio = elastic_mesh_object->volumeRatio();
        }
        _out_file << " " << velocity_rms << " " << volume_ratio;
    }
    _out_file << std::endl;
}