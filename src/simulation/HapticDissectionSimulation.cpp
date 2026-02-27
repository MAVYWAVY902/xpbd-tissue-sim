#include "simulation/HapticDissectionSimulation.hpp"
#include "haptics/HaplyInverse3Device.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "utils/GeometryUtils.hpp"
#include <chrono>
#include <thread>
#include <cmath>

namespace Sim
{

// 12 axis-mapping presets (1-based signed: +1=deviceX, -3=-deviceZ, etc.)
static const std::vector<int> kAxisPresets[] = {
    { 1,  2, -3},   //  0:  X,  Y, -Z (default)
    { 1,  2,  3},   //  1:  X,  Y,  Z
    { 1, -2,  3},   //  2:  X, -Y,  Z
    { 1, -2, -3},   //  3:  X, -Y, -Z
    {-1,  2,  3},   //  4: -X,  Y,  Z
    {-1,  2, -3},   //  5: -X,  Y, -Z
    { 1,  3, -2},   //  6:  X,  Z, -Y
    { 1, -3,  2},   //  7:  X, -Z,  Y
    { 1,  3,  2},   //  8:  X,  Z,  Y
    { 3,  2, -1},   //  9:  Z,  Y, -X
    {-3,  2,  1},   // 10: -Z,  Y,  X
    { 2,  1, -3},   // 11:  Y,  X, -Z
};
static constexpr int kNumPresets = 12;

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

    // Initialize axis mapping from config
    _setAxisMapping(config->deviceToSimAxes());

    // Find matching preset index for 'M' cycling
    _axis_mapping = 0;
    for (int i = 0; i < kNumPresets; ++i)
    {
        if (kAxisPresets[i] == config->deviceToSimAxes())
        {
            _axis_mapping = i;
            break;
        }
    }

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

    // 'M' key: cycle through axis mapping presets (translation + orientation)
    if (key == SimulationInput::Key::M && action == SimulationInput::KeyAction::PRESS)
    {
        _axis_mapping = (_axis_mapping + 1) % kNumPresets;
        _setAxisMapping(kAxisPresets[_axis_mapping]);
        const auto& a = kAxisPresets[_axis_mapping];
        std::cout << "[HapticDissection] Axis mapping #" << _axis_mapping
                  << "  config: [" << a[0] << ", " << a[1] << ", " << a[2] << "]"
                  << "  det=" << _det_device_to_camera << std::endl;
    }

    // 'N' key: toggle diagnostic printing
    if (key == SimulationInput::Key::N && action == SimulationInput::KeyAction::PRESS)
    {
        _diag_printing = !_diag_printing;
        std::cout << "[HapticDissection] Diagnostic printing: "
                  << (_diag_printing ? "ON" : "OFF") << std::endl;
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
        std::cout << "  Axis mapping #" << _axis_mapping
                  << "  det=" << _det_device_to_camera << std::endl;
        std::cout << "  Device-to-camera matrix:\n" << _device_to_camera << std::endl;
        std::cout << "  Key bindings:" << std::endl;
        std::cout << "    M = cycle axis mapping preset" << std::endl;
        std::cout << "    N = toggle diagnostic printing" << std::endl;
        std::cout << "    G = toggle VerseGrip orientation" << std::endl;
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

        // Diagnostics: print device/sim coordinates periodically
        if (_diag_printing)
        {
            static int diag_frame = 0;
            if (++diag_frame % 4000 == 0)  // ~every 2s at typical step rates
            {
                Vec3r delta = device_pos - _haptic_device_origin;
                std::cout << "[Diag] device_delta=(" << delta.transpose()
                          << ")  sim_pos=(" << sim_pos.transpose() << ")" << std::endl;
            }
        }
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
        // Compute RELATIVE rotation from initial grip orientation in device frame,
        // transform it to sim frame, then apply to the knife's initial orientation.
        Vec4r current_grip = _haptic_device->orientation();

        // q_inverse for XYZW (scalar-last): negate xyz, keep w
        Vec4r inv_initial_grip = Vec4r(
            -_initial_grip_quat[0], -_initial_grip_quat[1],
            -_initial_grip_quat[2],  _initial_grip_quat[3]);

        // delta = current * inverse(initial) → relative rotation in device frame
        Vec4r delta_device = GeometryUtils::quatMult(current_grip, inv_initial_grip);
        delta_device.normalize();

        // Transform the device-frame delta quaternion to sim frame
        Vec4r delta_sim = _hapticToSimQuaternion(delta_device);

        // Apply: new_knife = delta_sim * initial_knife
        Vec4r new_knife_quat = GeometryUtils::quatMult(delta_sim, _initial_knife_quat);
        new_knife_quat.normalize();

        _cursor->forceSetOrientation(new_knife_quat);

        // Diagnostics for orientation
        if (_diag_printing)
        {
            static int orient_diag_frame = 0;
            if (++orient_diag_frame % 4000 == 0)
            {
                std::cout << "[Diag] grip_quat=(" << current_grip.transpose()
                          << ")  delta_device=(" << delta_device.transpose()
                          << ")  delta_sim=(" << delta_sim.transpose()
                          << ")  knife_quat=(" << new_knife_quat.transpose() << ")" << std::endl;
            }
        }
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
// Workspace mapping: device → sim via camera frame
// device delta → (permutation) → camera-space delta → (camera basis) → world delta
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_hapticToSimPosition(const Vec3r& haptic_pos) const
{
    Vec3r delta = haptic_pos - _haptic_device_origin;
    Real scale = _sim_workspace_radius / _haptic_workspace_radius;

    // Map device axes to camera-space axes (fixed permutation)
    Vec3r delta_camera = _device_to_camera * delta * scale;

    // Transform camera-space to world-space using live camera basis
    const Vec3r cam_right   = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up      = _graphics_scene->cameraUpDirection();
    const Vec3r cam_forward = _graphics_scene->cameraViewDirection();

    Vec3r delta_world = cam_right   * delta_camera[0]
                      + cam_up      * delta_camera[1]
                      + cam_forward * delta_camera[2];

    return _haptic_origin + delta_world;
}

// --------------------------------------------------------------------------
// Force mapping: sim → device via camera frame (inverse of position mapping)
// world force → (project onto camera basis) → camera-space → (inverse perm) → device
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_simToHapticForce(const Vec3r& sim_force) const
{
    // Project world-space force onto camera basis
    const Vec3r cam_right   = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up      = _graphics_scene->cameraUpDirection();
    const Vec3r cam_forward = _graphics_scene->cameraViewDirection();

    Vec3r force_camera(sim_force.dot(cam_right),
                       sim_force.dot(cam_up),
                       sim_force.dot(cam_forward));

    // Camera-space → device-space (inverse permutation = transpose)
    Real scale = _haptic_workspace_radius / _sim_workspace_radius;
    Vec3r device_force = _device_to_camera.transpose() * force_camera;
    return device_force * scale * _haptic_force_scaling;
}

// --------------------------------------------------------------------------
// Quaternion coordinate transform: device frame → sim frame via camera frame
// Permute rotation axis to camera-space, then rotate into world-space.
// --------------------------------------------------------------------------
Vec4r HapticDissectionSimulation::_hapticToSimQuaternion(const Vec4r& device_quat) const
{
    Vec3r v_device(device_quat[0], device_quat[1], device_quat[2]);
    Real w = device_quat[3];

    // Map device rotation axis to camera-space axis
    Vec3r v_camera = _det_device_to_camera * (_device_to_camera * v_device);

    // Rotate axis from camera-space to world-space
    const Vec3r cam_right   = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up      = _graphics_scene->cameraUpDirection();
    const Vec3r cam_forward = _graphics_scene->cameraViewDirection();

    Vec3r v_world = cam_right   * v_camera[0]
                  + cam_up      * v_camera[1]
                  + cam_forward * v_camera[2];

    Vec4r q_world(v_world[0], v_world[1], v_world[2], w);
    q_world.normalize();
    return q_world;
}

// --------------------------------------------------------------------------
// Build the 3x3 signed permutation matrix from a 1-based signed axis spec.
// E.g. {1, 2, -3} means: camRight = +deviceX, camUp = +deviceY, camFwd = -deviceZ
// --------------------------------------------------------------------------
void HapticDissectionSimulation::_setAxisMapping(const std::vector<int>& axes)
{
    _device_to_camera = Mat3r::Zero();
    for (int row = 0; row < 3; ++row)
    {
        int signed_axis = axes[row];
        int col = std::abs(signed_axis) - 1;  // 1-based → 0-based
        Real sign = (signed_axis > 0) ? 1.0 : -1.0;
        _device_to_camera(row, col) = sign;
    }
    _det_device_to_camera = _device_to_camera.determinant();
}

} // namespace Sim
