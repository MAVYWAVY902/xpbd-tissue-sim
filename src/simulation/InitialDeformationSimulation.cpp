#include "simulation/InitialDeformationSimulation.hpp"

#include "utils/MeshUtils.hpp"

#include "config/simulation/InitialDeformationSimulationConfig.hpp"

#include <regex>


namespace Sim
{

InitialDeformationSimulation::InitialDeformationSimulation(const Config::InitialDeformationSimulationConfig* config)
    : Simulation(config)
{
    _deformation_type = config->deformationType();
    _deformation_factor = config->deformationFactor();

    // since this is a strictly elastic simulation, make sure g is 0 so there are no external forces
    assert(_g_accel == 0);
}

std::string InitialDeformationSimulation::deformationType() const
{
    if (_deformation_type == DeformationType::VOLUMETRIC_EXPANSION)
        return "Volumetric Expansion";
    else if (_deformation_type == DeformationType::VOLUMETRIC_COMPRESSION)
        return "Volumetric Compression";
    else if (_deformation_type == DeformationType::COLLAPSE_TO_PLANE)
        return "Collapse to Plane";
    else if (_deformation_type == DeformationType::SCRAMBLE)
        return "Scramble";
    return "";
}

void InitialDeformationSimulation::setup()
{
    std::cout << "Deformation type: " << deformationType() << std::endl;

    // call the parent setup
    Simulation::setup();

    // get all the XPBDMeshObjects
    _objects.for_each_element<std::unique_ptr<XPBDMeshObject_Base>, std::unique_ptr<FirstOrderXPBDMeshObject_Base>>([&](auto& obj)
    {
        _xpbd_objs.push_back(obj.get());
    });

    // add logged variables
    for (auto& xpbd_obj : _xpbd_objs)
    {
        const std::string var_name = xpbd_obj.name() + "_2*t";
        _logger->addOutput(var_name, [this](){
            return 2*_time;
        });
    }

    // initialize the various states of initial deformation for each XPBDMeshObject
    for (auto& xpbd_obj : _xpbd_objs)
    {
        if (_deformation_type == DeformationType::VOLUMETRIC_EXPANSION)
        {
            Geometry::AABB bbox = xpbd_obj.mesh()->boundingBox();
            xpbd_obj.mesh()->resize(bbox.size() * _deformation_factor);
        }
        else if (_deformation_type == DeformationType::VOLUMETRIC_COMPRESSION)
        {
            Geometry::AABB bbox = xpbd_obj.mesh()->boundingBox();
            xpbd_obj.mesh()->resize(bbox.size() / _deformation_factor);
        }
        else if (_deformation_type == DeformationType::COLLAPSE_TO_PLANE)
        {
            for (int i = 0; i < xpbd_obj.mesh()->numVertices(); i++)
            {
                const Vec3r& vi = xpbd_obj.mesh()->vertex(i);
                xpbd_obj.mesh()->setVertex(i, Vec3r(vi[0], vi[1], 0));  // set Z-coordinate of all vertices to 0
            }
        }
        else if (_deformation_type == DeformationType::SCRAMBLE)
        {
            Geometry::AABB bbox = xpbd_obj.mesh()->boundingBox();
            for (int i = 0; i < xpbd_obj.mesh()->numVertices(); i++)
            {
                Real x = (rand()/double(RAND_MAX))*bbox.size()[0] + bbox.min[0];
                Real y = (rand()/double(RAND_MAX))*bbox.size()[1] + bbox.min[1];
                Real z = (rand()/double(RAND_MAX))*bbox.size()[2] + bbox.min[2];
                xpbd_obj.mesh()->setVertex(i,Vec3r(x,y,z));
            }
        }
    }
    
    
}

void InitialDeformationSimulation::notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    // start the simulation after any key press
    if (!_simulation_started)
    {
        _simulation_started = true;
        _wall_time_start = std::chrono::steady_clock::now();
    }

    Simulation::notifyKeyPressed(key, action, modifiers);
}

void InitialDeformationSimulation::_timeStep()
{
    if (!_simulation_started)
        return;
    
    Simulation::_timeStep();
}

} // namespace Sim