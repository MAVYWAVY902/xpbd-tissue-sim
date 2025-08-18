#include "simulation/InitialDeformationSimulation.hpp"

#include "utils/MeshUtils.hpp"

#include "simobject/XPBDMeshObject.hpp"
#include "simobject/FirstOrderXPBDMeshObject.hpp"

#include "config/simulation/InitialDeformationSimulationConfig.hpp"

#include <regex>


namespace Sim
{

InitialDeformationSimulation::InitialDeformationSimulation(const Config::InitialDeformationSimulationConfig* config)
    : Simulation(config)
{
    _deformation_type = initial_deformation_simulation_config->deformationType().value();
    _deformation_factor = initial_deformation_simulation_config->deformationFactor().value();

    // since this is a strictly elastic simulation, make sure g is 0 so there are no external forces
    assert(_g_accel == 0);
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

    

    // initialize the various states of initial deformation
    if (_deformation_type == DeformationType::VOLUMETRIC_EXPANSION)
    {
        Geometry::AABB bbox = 
    }
}

} // namespace Sim