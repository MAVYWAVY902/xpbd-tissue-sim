#include "simulation/HapticDissectionSimulation.hpp"
#include "haptics/HaplyInverse3Device.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "utils/GeometryUtils.hpp"
#include <chrono>
#include <thread>
#include <cmath>

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
    _rotation_speed         = config->knifeRotationSpeed();

    // Initialize rotation key tracking
    _rotation_keys_held[SimulationInput::Key::Q] = false;  // yaw -
    _rotation_keys_held[SimulationInput::Key::E] = false;  // yaw +
    _rotation_keys_held[SimulationInput::Key::R] = false;  // pitch +
    _rotation_keys_held[SimulationInput::Key::F] = false;  // pitch -
    _rotation_keys_held[SimulationInput::Key::Z] = false;  // roll -
    _rotation_keys_held[SimulationInput::Key::X] = false;  // roll +

    // Create the haptic device (attempts connection, graceful fallback)
    _haptic_device = std::make_unique<HaplyInverse3Device>(config->hapticSerialPort());
}

// Destructor must be defined here (not in header) because HaplyInverse3Device
// is only forward-declared in the header — unique_ptr needs the complete type.
HapticDissectionSimulation::~HapticDissectionSimulation() = default;

void HapticDissectionSimulation::notifyKeyPressed(
    SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers)
{
    // 'T' key: toggle EndEffectorForce test (3.3N all axes)
    if (key == SimulationInput::Key::T && action == SimulationInput::KeyAction::PRESS)
    {
        if (_haptic_device && _haptic_device->isConnected())
        {
            _haptic_device->toggleTestForce();
            std::cout << "[HapticDissection] Test EndEffectorForce toggled! (3.3N all axes)" << std::endl;
        }
    }

    // 'Y' key: toggle JointTorques test (100 Nmm all motors — bypasses kinematics)
    if (key == SimulationInput::Key::Y && action == SimulationInput::KeyAction::PRESS)
    {
        if (_haptic_device && _haptic_device->isConnected())
        {
            _haptic_device->toggleTestTorque();
            std::cout << "[HapticDissection] Test JointTorques toggled! (100 Nmm all motors)" << std::endl;
        }
    }

    // 'G' key: toggle VerseGrip orientation tracking
    if (key == SimulationInput::Key::G && action == SimulationInput::KeyAction::PRESS)
    {
        _use_grip_orientation = !_use_grip_orientation;
        std::cout << "[HapticDissection] VerseGrip orientation: "
                  << (_use_grip_orientation ? "ENABLED" : "DISABLED") << std::endl;
    }

    // 'M' key: cycle through axis mapping presets for translation
    if (key == SimulationInput::Key::M && action == SimulationInput::KeyAction::PRESS)
    {
        _axis_mapping = (_axis_mapping + 1) % 12;
        const char* labels[] = {
            " X,  Y, -Z",   // 0 (current default)
            " X,  Y,  Z",   // 1
            " X, -Y,  Z",   // 2
            " X, -Y, -Z",   // 3
            "-X,  Y,  Z",   // 4
            "-X,  Y, -Z",   // 5
            " X,  Z, -Y",   // 6 (swap Y/Z)
            " X, -Z,  Y",   // 7
            " X,  Z,  Y",   // 8
            " Z,  Y, -X",   // 9 (swap X/Z)
            "-Z,  Y,  X",   // 10
            " Y,  X, -Z",   // 11 (swap X/Y)
        };
        std::cout << "[HapticDissection] Axis mapping #" << _axis_mapping
                  << ": " << labels[_axis_mapping] << std::endl;
    }

    // Track rotation key held state
    auto it = _rotation_keys_held.find(key);
    if (it != _rotation_keys_held.end())
    {
        it->second = (action == SimulationInput::KeyAction::PRESS);
    }

    // Pass to parent for other key bindings (O for reset, SPACE for depth, etc.)
    PushingSimulation::notifyKeyPressed(key, action, modifiers);
}

void HapticDissectionSimulation::setup()
{
    PushingSimulation::setup();

    // Pushing starts DISABLED — click middle mouse to enable (same as PushingTest)

    // Record the knife's initial position as the haptic origin in sim space
    _haptic_origin = _cursor->position();

    // Record the device's rest position and orientation if connected
    if (_haptic_device && _haptic_device->isConnected())
    {
        _haptic_device_origin = _haptic_device->initialPosition();
        _initial_knife_quat = _cursor->orientation();

        // Wait a moment for VerseGrip thread to get initial orientation
        if (_haptic_device->hasVerseGrip())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            _initial_grip_quat = _haptic_device->orientation();
        }

        std::cout << "[HapticDissection] Haptic device connected." << std::endl;
        std::cout << "  Device origin: (" << _haptic_device_origin.transpose() << ")" << std::endl;
        std::cout << "  Knife origin: (" << _haptic_origin.transpose() << ")" << std::endl;
        std::cout << "  Initial grip quat: (" << _initial_grip_quat.transpose() << ")" << std::endl;
        std::cout << "  Initial knife quat: (" << _initial_knife_quat.transpose() << ")" << std::endl;
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
        // Background thread handles serial I/O — just read latest values.
        // position()/orientation() are thread-safe (mutex-protected).
        Vec3r device_pos = _haptic_device->position();
        Vec3r sim_pos = _hapticToSimPosition(device_pos);
        _cursor->forceSetPosition(sim_pos);

        // Debug: uncomment to log device position every ~1 second
        // static int frame_count = 0;
        // if (++frame_count % 2000 == 0)
        //     std::cout << "[HapticDissection] device=(" << device_pos.transpose()
        //               << ")  sim=(" << sim_pos.transpose() << ")" << std::endl;
    }
    // else: mouse/keyboard input from PushingSimulation works as-is

    // ------------------------------------------------------------------
    // 2. Run the parent time step (adhesion breaking + pushing + XPBD solve)
    // ------------------------------------------------------------------
    PushingSimulation::_timeStep();

    // ------------------------------------------------------------------
    // 3. Force feedback — DISABLED for safety until tested properly
    // ------------------------------------------------------------------
#if 0
    if (_haptic_device && _haptic_device->isConnected())
    {
        Vec3r contact_force = _computeContactForce();
        Vec3r adhesion_force = _collectNetAdhesionForce();
        Vec3r total_force = contact_force + adhesion_force;
        Vec3r haptic_force = _simToHapticForce(total_force);
        Vec3r filtered = _force_filter_alpha * haptic_force
                       + (1.0 - _force_filter_alpha) * _prev_haptic_force;
        _prev_haptic_force = filtered;

        // f. Log forces periodically for debugging
        static int force_log_count = 0;
        if (++force_log_count % 30 == 0)
        {
            if (total_force.norm() > 1e-6)
            {
                std::cout << "[ForceFeedback] contact=(" << contact_force.transpose()
                          << ")  adhesion=(" << adhesion_force.transpose()
                          << ")  haptic=(" << filtered.transpose()
                          << ")  |F|=" << filtered.norm() << " N" << std::endl;
            }
        }

        _haptic_device->setForce(filtered);
    }
#endif  // Force feedback disabled for safety

    // ------------------------------------------------------------------
    // 4. Rotation: VerseGrip (if available) or keyboard fallback
    // ------------------------------------------------------------------
    if (_haptic_device && _haptic_device->hasVerseGrip() && _use_grip_orientation)
    {
        // Compute RELATIVE rotation from initial grip orientation,
        // then apply it to the knife's initial orientation.
        // This way: no grip movement → knife stays at initial orientation.
        Vec4r current_grip = _haptic_device->orientation();

        // q_inverse for XYZW (scalar-last): negate xyz, keep w
        Vec4r inv_initial_grip = Vec4r(
            -_initial_grip_quat[0], -_initial_grip_quat[1],
            -_initial_grip_quat[2],  _initial_grip_quat[3]);

        // delta = current * inverse(initial) → relative rotation
        Vec4r delta_quat = GeometryUtils::quatMult(current_grip, inv_initial_grip);
        delta_quat.normalize();

        // Apply: new_knife = delta * initial_knife
        Vec4r new_knife_quat = GeometryUtils::quatMult(delta_quat, _initial_knife_quat);
        new_knife_quat.normalize();

        _cursor->forceSetOrientation(new_knife_quat);
    }
    else
    {
        // Keyboard fallback: Q/E=yaw, R/F=pitch, Z/X=roll
        Real time_step = dt();
        Real angle_step = _rotation_speed * time_step;

        Real yaw   = 0;  // Y-axis
        Real pitch = 0;  // X-axis
        Real roll  = 0;  // Z-axis

        if (_rotation_keys_held[SimulationInput::Key::Q]) yaw   -= angle_step;
        if (_rotation_keys_held[SimulationInput::Key::E]) yaw   += angle_step;
        if (_rotation_keys_held[SimulationInput::Key::R]) pitch += angle_step;
        if (_rotation_keys_held[SimulationInput::Key::F]) pitch -= angle_step;
        if (_rotation_keys_held[SimulationInput::Key::Z]) roll  -= angle_step;
        if (_rotation_keys_held[SimulationInput::Key::X]) roll  += angle_step;

        if (yaw != 0 || pitch != 0 || roll != 0)
        {
            Vec4r dq = GeometryUtils::eulXYZ2Quat(pitch, yaw, roll);
            Vec4r current_q = _cursor->orientation();
            Vec4r new_q = GeometryUtils::quatMult(dq, current_q);
            new_q.normalize();
            _cursor->forceSetOrientation(new_q);
        }
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

    // Axis mapping: Inverse3 device → simulation frame
    // Cycle through presets with 'M' key to find the correct one.
    Real dx = delta[0], dy = delta[1], dz = delta[2];
    Vec3r sim_delta;
    switch (_axis_mapping)
    {
        case 0:  sim_delta = Vec3r( dx,  dy, -dz) * scale; break;  //  X,  Y, -Z (default)
        case 1:  sim_delta = Vec3r( dx,  dy,  dz) * scale; break;  //  X,  Y,  Z
        case 2:  sim_delta = Vec3r( dx, -dy,  dz) * scale; break;  //  X, -Y,  Z
        case 3:  sim_delta = Vec3r( dx, -dy, -dz) * scale; break;  //  X, -Y, -Z
        case 4:  sim_delta = Vec3r(-dx,  dy,  dz) * scale; break;  // -X,  Y,  Z
        case 5:  sim_delta = Vec3r(-dx,  dy, -dz) * scale; break;  // -X,  Y, -Z
        case 6:  sim_delta = Vec3r( dx,  dz, -dy) * scale; break;  //  X,  Z, -Y
        case 7:  sim_delta = Vec3r( dx, -dz,  dy) * scale; break;  //  X, -Z,  Y
        case 8:  sim_delta = Vec3r( dx,  dz,  dy) * scale; break;  //  X,  Z,  Y
        case 9:  sim_delta = Vec3r( dz,  dy, -dx) * scale; break;  //  Z,  Y, -X
        case 10: sim_delta = Vec3r(-dz,  dy,  dx) * scale; break;  // -Z,  Y,  X
        case 11: sim_delta = Vec3r( dy,  dx, -dz) * scale; break;  //  Y,  X, -Z
        default: sim_delta = Vec3r( dx,  dy, -dz) * scale; break;
    }

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
