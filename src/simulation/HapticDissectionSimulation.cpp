#include "simulation/HapticDissectionSimulation.hpp"
#include "haptics/HaplyInverse3Device.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include <chrono>
#include <thread>

namespace Sim
{

HapticDissectionSimulation::HapticDissectionSimulation(
    const Config::HapticDissectionSimulationConfig* config)
    : PushingSimulation(config)
{
    _haptic_force_scaling   = config->hapticForceScaling();
    _contact_force_stiffness = config->contactForceStiffness();
    _force_filter_alpha     = config->forceFilterAlpha();
    _haptic_workspace_radius = config->hapticWorkspaceRadius();
    _sim_workspace_radius   = config->simWorkspaceRadius();

    // Create the haptic device (attempts connection, graceful fallback)
    _haptic_device = std::make_unique<HaplyInverse3Device>(config->hapticSerialPort());
}

// Destructor must be defined here (not in header) because HaplyInverse3Device
// is only forward-declared in the header — unique_ptr needs the complete type.
HapticDissectionSimulation::~HapticDissectionSimulation() = default;

void HapticDissectionSimulation::setup()
{
    PushingSimulation::setup();

    // Record the knife's initial position as the haptic origin in sim space
    _haptic_origin = _cursor->position();

    // Record the device's rest position if connected
    if (_haptic_device && _haptic_device->isConnected())
    {
        // Use the validated initial position from the constructor
        _haptic_device_origin = _haptic_device->initialPosition();
        std::cout << "[HapticDissection] Haptic device connected. "
                  << "Device origin: (" << _haptic_device_origin.transpose() << ")" << std::endl;
        std::cout << "[HapticDissection] Knife origin in sim: ("
                  << _haptic_origin.transpose() << ")" << std::endl;
    }
    else
    {
        std::cout << "[HapticDissection] No haptic device. Using mouse/keyboard controls." << std::endl;
    }
}

void HapticDissectionSimulation::_timeStep()
{
    // ------------------------------------------------------------------
    // 1. If device connected: poll device and drive the knife
    // ------------------------------------------------------------------
    if (_haptic_device && _haptic_device->isConnected())
    {
        // Synchronous poll: exchange force command for position/velocity
        _haptic_device->poll();

        Vec3r device_pos = _haptic_device->position();
        Vec3r sim_pos = _hapticToSimPosition(device_pos);
        _cursor->setPosition(sim_pos);

        // Debug: log every ~1 second (assuming ~30 fps = every 30 frames)
        static int frame_count = 0;
        if (++frame_count % 30 == 0)
        {
            std::cout << "[HapticDissection] device=(" << device_pos.transpose()
                      << ")  sim=(" << sim_pos.transpose()
                      << ")  knife=(" << _cursor->position().transpose() << ")" << std::endl;
        }
    }
    // else: mouse/keyboard input from PushingSimulation works as-is

    // ------------------------------------------------------------------
    // 2. Run the parent time step (adhesion breaking + pushing + XPBD solve)
    // ------------------------------------------------------------------
    PushingSimulation::_timeStep();

    // ------------------------------------------------------------------
    // 3. If device connected: compute and send force feedback
    // ------------------------------------------------------------------
    if (_haptic_device && _haptic_device->isConnected())
    {
        // a. Contact penalty force from SDF penetration
        Vec3r contact_force = _computeContactForce();

        // b. Net adhesion force (Newton's 3rd law: negate forces on tissue)
        Vec3r adhesion_force = _collectNetAdhesionForce();

        // c. Total sim-space force on the tool
        Vec3r total_force = contact_force + adhesion_force;

        // d. Transform to haptic device frame
        Vec3r haptic_force = _simToHapticForce(total_force);

        // e. Low-pass filter (matches PalpationSimulation pattern)
        Vec3r filtered = _force_filter_alpha * haptic_force
                       + (1.0 - _force_filter_alpha) * _prev_haptic_force;
        _prev_haptic_force = filtered;

        // f. Send to device
        _haptic_device->setForce(filtered);
    }
}

// --------------------------------------------------------------------------
// Contact force: penalty from tissue vertices penetrating the knife SDF
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_computeContactForce()
{
    const Geometry::MeshSDF* knife_sdf = _cursor->SDF();
    if (!knife_sdf) return Vec3r::Zero();

    Vec3r net_force = Vec3r::Zero();

    auto accumulate_contact = [&](auto& mesh_objs)
    {
        for (auto& obj : mesh_objs)
        {
            for (int v = 0; v < obj->mesh()->numVertices(); ++v)
            {
                if (obj->vertexFixed(v)) continue;

                const Vec3r vertex_pos = obj->mesh()->vertex(v);
                Real signed_distance = knife_sdf->evaluate(vertex_pos);

                // Only consider vertices penetrating or in the soft contact zone
                const Real contact_threshold = 0.001; // 1mm
                if (signed_distance < contact_threshold)
                {
                    Real penetration = contact_threshold - signed_distance;
                    Vec3r grad = knife_sdf->gradient(vertex_pos);
                    if (grad.norm() < 1e-6) continue;

                    // Penalty force: F = -stiffness * penetration * normal
                    // This is the reaction force ON the tool (opposite of push direction)
                    Vec3r normal = grad.normalized();
                    net_force -= _contact_force_stiffness * penetration * normal;
                }
            }
        }
    };

    accumulate_contact(
        _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>());
    accumulate_contact(
        _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>());

    return net_force;
}

// --------------------------------------------------------------------------
// Adhesion force: sum constraint forces on tissue, negate for tool reaction
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_collectNetAdhesionForce()
{
    Vec3r net_force = Vec3r::Zero();

    auto accumulate_adhesion = [&](auto& mesh_objs)
    {
        for (auto& obj : mesh_objs)
        {
            int num_verts = obj->mesh()->numVertices();
            std::vector<Vec3r> vertex_forces(num_verts, Vec3r::Zero());
            obj->collectAdhesionForces(vertex_forces);

            for (int v = 0; v < num_verts; ++v)
                net_force += vertex_forces[v];
        }
    };

    accumulate_adhesion(
        _objects.template get<std::unique_ptr<Sim::XPBDMeshObject_Base>>());
    accumulate_adhesion(
        _objects.template get<std::unique_ptr<Sim::FirstOrderXPBDMeshObject_Base>>());

    // Newton's 3rd law: force on tool is opposite to force on tissue
    return -net_force;
}

// --------------------------------------------------------------------------
// Workspace mapping: device → sim
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_hapticToSimPosition(const Vec3r& haptic_pos) const
{
    // Map relative displacement in device space to simulation space
    Vec3r delta = haptic_pos - _haptic_device_origin;

    // Scale from haptic workspace to simulation workspace
    Real scale = _sim_workspace_radius / _haptic_workspace_radius;
    Vec3r sim_delta = delta * scale;

    // Inverse3 coordinate convention: +x right, +y up, +z toward user
    // Simulation convention may differ — apply a simple axis mapping here.
    // Default: direct mapping (can be adjusted if axes differ)
    return _haptic_origin + sim_delta;
}

// --------------------------------------------------------------------------
// Force mapping: sim → device
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_simToHapticForce(const Vec3r& sim_force) const
{
    // Inverse of workspace scaling so that forces feel proportional
    Real scale = _haptic_workspace_radius / _sim_workspace_radius;

    // Apply force scaling factor for haptic amplification
    Vec3r haptic_force = sim_force * scale * _haptic_force_scaling;

    return haptic_force;
}

} // namespace Sim
