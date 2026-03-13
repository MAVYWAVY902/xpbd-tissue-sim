#include "simulation/HapticDissectionSimulation.hpp"
#include "haptics/HaplyInverse3Device.hpp"
#include "simobject/XPBDMeshObjectBase.hpp"
#include "utils/GeometryUtils.hpp"
#include <chrono>
#include <thread>
#include <cmath>
#include <string>

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
    _rotation_sensitivity   = config->rotationSensitivity();

    // Initialize device-to-camera transform
    if (config->hasDeviceRotationQuat())
    {
        _setDeviceToCameraFromQuat(config->deviceToCameraQuat());
        std::cout << "[HapticDissection] Using device-to-camera quaternion from config" << std::endl;
    }
    else
    {
        // Legacy: build from signed permutation
        _setAxisMappingFromPermutation(config->deviceToSimAxes());
        std::cout << "[HapticDissection] Using device-to-sim-axes permutation from config" << std::endl;
    }

    // Initialize grip-to-camera transform (for VerseGrip rotation mapping)
    if (config->hasGripRotationQuat())
    {
        _setGripToCameraFromQuat(config->gripToCameraQuat());
        _has_grip_calibration = true;
        std::cout << "[HapticDissection] Using grip-to-camera quaternion from config" << std::endl;
    }

    // Initialize rotation key tracking
    _rotation_keys_held[SimulationInput::Key::Q] = false;  // yaw -
    _rotation_keys_held[SimulationInput::Key::E] = false;  // yaw +
    _rotation_keys_held[SimulationInput::Key::R] = false;  // pitch +
    _rotation_keys_held[SimulationInput::Key::F] = false;  // pitch -
    _rotation_keys_held[SimulationInput::Key::Z] = false;  // roll -
    _rotation_keys_held[SimulationInput::Key::X] = false;  // roll +

    // Create the haptic device only if config says to use it
    if (config->useHapticDevice())
    {
        _haptic_device = std::make_unique<HaplyInverse3Device>(config->hapticSerialPort());
    }
    else
    {
        std::cout << "[HapticDissection] input-device: mouse — skipping haptic device, using mouse/keyboard controls" << std::endl;
    }
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

    // 'C' key: interactive calibration (push device right, then up)
    if (key == SimulationInput::Key::C && action == SimulationInput::KeyAction::PRESS)
    {
        if (_haptic_device && _haptic_device->isConnected())
        {
            _advanceCalibration();
        }
        else
        {
            std::cout << "[HapticDissection] Calibration requires a connected device." << std::endl;
        }
    }

    // 'N' key: toggle diagnostic printing
    if (key == SimulationInput::Key::N && action == SimulationInput::KeyAction::PRESS)
    {
        _diag_printing = !_diag_printing;
        std::cout << "[HapticDissection] Diagnostic printing: "
                  << (_diag_printing ? "ON" : "OFF") << std::endl;
    }

    // 'O' key: rotation calibration when VerseGrip connected, else fall through to parent (reset)
    if (key == SimulationInput::Key::O && action == SimulationInput::KeyAction::PRESS)
    {
        if (_haptic_device && _haptic_device->isConnected() && _haptic_device->hasVerseGrip())
        {
            _advanceRotationCalibration();
            return;  // don't pass to parent (which would reset knife position)
        }
    }

    // 'M' key: cycle rotation axis options (axis cycling calibration)
    if (key == SimulationInput::Key::M && action == SimulationInput::KeyAction::PRESS)
    {
        if (_haptic_device && _haptic_device->isConnected() && _haptic_device->hasVerseGrip())
        {
            _cycleAxisCalib();
            return;
        }
    }

    // comma key: confirm current axis selection
    if (key == SimulationInput::Key::COMMA && action == SimulationInput::KeyAction::PRESS)
    {
        if (_axis_calib_state != AxisCalibState::IDLE)
        {
            _confirmAxisCalib();
            return;
        }
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
        std::cout << "  Device-to-camera matrix (det=" << _det_device_to_camera << "):\n"
                  << _device_to_camera << std::endl;
        if (_has_grip_calibration)
            std::cout << "  Grip-to-camera matrix (det=" << _det_grip_to_camera << "):\n"
                      << _grip_to_camera << std::endl;
        else
            std::cout << "  Grip-to-camera: NOT CALIBRATED (using device-to-camera for rotation)" << std::endl;
        std::cout << "  Key bindings:" << std::endl;
        std::cout << "    C = position calibration (translation mapping)" << std::endl;
        std::cout << "    O = rotation calibration (VerseGrip orientation mapping)" << std::endl;
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
        Vec3r device_pos = _haptic_device->position();

        if (_fixed_base_mode)
        {
            // In fixed-base mode, map device displacement to tip deflection
            // (absolute mapping, not cumulative — device position directly sets deflection)
            Vec3r delta = device_pos - _haptic_device_origin;
            Real scale = _sim_workspace_radius / _haptic_workspace_radius;
            Vec3r delta_camera = _device_to_camera * delta * scale;

            // Scale deflection by tip_sensitivity for comfortable control
            _tip_deflection_camera = delta_camera * _tip_sensitivity;
            // Position/orientation handled by PushingSimulation::_updateFixedBaseKnife()
        }
        else
        {
            // Original absolute position mapping
            Vec3r sim_pos = _hapticToSimPosition(device_pos);
            _cursor->forceSetPosition(sim_pos);
        }

        // Diagnostics: print device/sim coordinates periodically
        if (_diag_printing)
        {
            static int diag_frame = 0;
            if (++diag_frame % 4000 == 0)
            {
                Vec3r delta = device_pos - _haptic_device_origin;
                if (_fixed_base_mode)
                    std::cout << "[Diag] device_delta=(" << delta.transpose()
                              << ")  tip_deflection=(" << _tip_deflection_camera.transpose() << ")" << std::endl;
                else
                    std::cout << "[Diag] device_delta=(" << delta.transpose()
                              << ")  sim_pos=(" << _hapticToSimPosition(device_pos).transpose() << ")" << std::endl;
            }
        }
    }
    // else: mouse/keyboard input from PushingSimulation works as-is

    // ------------------------------------------------------------------
    // 2. Rotation: update knife orientation BEFORE physics step so that
    //    SDF queries (push forces, adhesion interference) use correct orientation
    // ------------------------------------------------------------------
    if (_fixed_base_mode)
    {
        // Orientation fully determined by base→tip pivot — do nothing
    }
    else if (_haptic_device && _haptic_device->hasVerseGrip() && _use_grip_orientation)
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

        // Canonicalize to shortest arc (w >= 0) before any scaling
        if (delta_device[3] < 0)
            delta_device = -delta_device;

        // Quaternion power scaling: amplify rotation angle by _rotation_sensitivity
        // q = (sin(θ/2)*axis, cos(θ/2)) → q^s = (sin(s*θ/2)*axis, cos(s*θ/2))
        if (_rotation_sensitivity != 1.0)
        {
            Vec3r axis(delta_device[0], delta_device[1], delta_device[2]);
            Real axis_norm = axis.norm();
            if (axis_norm > 1e-8)
            {
                axis /= axis_norm;
                Real half_angle = std::atan2(axis_norm, delta_device[3]);
                Real scaled_half = half_angle * _rotation_sensitivity;
                delta_device = Vec4r(
                    std::sin(scaled_half) * axis[0],
                    std::sin(scaled_half) * axis[1],
                    std::sin(scaled_half) * axis[2],
                    std::cos(scaled_half));
            }
        }

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

    // ------------------------------------------------------------------
    // 3. Run the parent time step (adhesion breaking + pushing + XPBD solve)
    // ------------------------------------------------------------------
    PushingSimulation::_timeStep();

    // ------------------------------------------------------------------
    // 4. Force feedback — DISABLED for safety until tested properly
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
// device delta → (device_to_camera rotation) → camera-space delta → (camera basis) → world delta
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_hapticToSimPosition(const Vec3r& haptic_pos) const
{
    Vec3r delta = haptic_pos - _haptic_device_origin;
    Real scale = _sim_workspace_radius / _haptic_workspace_radius;

    // Map device axes to camera-space axes (general rotation)
    Vec3r delta_camera = _device_to_camera * delta * scale;

    // Transform camera-space to world-space using live camera basis.
    // The device-to-camera matrix is a proper rotation (det=+1) whose third axis
    // is right×up = out-of-screen.  cameraViewDirection() points INTO the screen,
    // so we negate it to get the matching right-handed "back" basis vector.
    const Vec3r cam_right = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up    = _graphics_scene->cameraUpDirection();
    const Vec3r cam_back  = -_graphics_scene->cameraViewDirection();

    Vec3r delta_world = cam_right * delta_camera[0]
                      + cam_up    * delta_camera[1]
                      + cam_back  * delta_camera[2];

    return _haptic_origin + delta_world;
}

// --------------------------------------------------------------------------
// Force mapping: sim → device via camera frame (inverse of position mapping)
// world force → (project onto camera basis) → camera-space → (inverse rotation) → device
// --------------------------------------------------------------------------
Vec3r HapticDissectionSimulation::_simToHapticForce(const Vec3r& sim_force) const
{
    // Project world-space force onto right-handed camera basis (right, up, back)
    const Vec3r cam_right = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up    = _graphics_scene->cameraUpDirection();
    const Vec3r cam_back  = -_graphics_scene->cameraViewDirection();

    Vec3r force_camera(sim_force.dot(cam_right),
                       sim_force.dot(cam_up),
                       sim_force.dot(cam_back));

    // Camera-space → device-space (inverse of orthogonal matrix = transpose)
    Real scale = _haptic_workspace_radius / _sim_workspace_radius;
    Vec3r device_force = _device_to_camera.transpose() * force_camera;
    return device_force * scale * _haptic_force_scaling;
}

// --------------------------------------------------------------------------
// Quaternion coordinate transform: device frame → sim frame via camera frame
// Transform rotation axis to camera-space, then rotate into world-space.
// --------------------------------------------------------------------------
Vec4r HapticDissectionSimulation::_hapticToSimQuaternion(const Vec4r& device_quat) const
{
    Vec3r v_device(device_quat[0], device_quat[1], device_quat[2]);
    Real w = device_quat[3];

    // Map device rotation axis to camera-space axis
    // Use grip-specific calibration if available, otherwise fall back to position calibration
    Vec3r v_camera;
    if (_has_grip_calibration)
        v_camera = _det_grip_to_camera * (_grip_to_camera * v_device);
    else
        v_camera = _det_device_to_camera * (_device_to_camera * v_device);

    // Rotate axis from camera-space to world-space (right-handed: right, up, back)
    const Vec3r cam_right = _graphics_scene->cameraRightDirection();
    const Vec3r cam_up    = _graphics_scene->cameraUpDirection();
    const Vec3r cam_back  = -_graphics_scene->cameraViewDirection();

    Vec3r v_world = cam_right * v_camera[0]
                  + cam_up    * v_camera[1]
                  + cam_back  * v_camera[2];

    Vec4r q_world(v_world[0], v_world[1], v_world[2], w);
    q_world.normalize();
    return q_world;
}

// --------------------------------------------------------------------------
// Build the 3x3 matrix from a 1-based signed axis permutation (legacy).
// E.g. {1, 2, -3} means: camRight = +deviceX, camUp = +deviceY, camFwd = -deviceZ
// --------------------------------------------------------------------------
void HapticDissectionSimulation::_setAxisMappingFromPermutation(const std::vector<int>& axes)
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

// --------------------------------------------------------------------------
// Set the device-to-camera matrix from a quaternion (xyzw scalar-last).
// The quaternion represents a proper rotation (det=+1).
// --------------------------------------------------------------------------
void HapticDissectionSimulation::_setDeviceToCameraFromQuat(const Vec4r& quat)
{
    // Convert quaternion (x,y,z,w) to 3x3 rotation matrix
    Real x = quat[0], y = quat[1], z = quat[2], w = quat[3];
    Real x2 = x*x, y2 = y*y, z2 = z*z;
    Real xy = x*y, xz = x*z, yz = y*z;
    Real wx = w*x, wy = w*y, wz = w*z;

    _device_to_camera <<
        1 - 2*(y2 + z2),   2*(xy - wz),       2*(xz + wy),
        2*(xy + wz),       1 - 2*(x2 + z2),   2*(yz - wx),
        2*(xz - wy),       2*(yz + wx),       1 - 2*(x2 + y2);

    _det_device_to_camera = _device_to_camera.determinant();  // should be +1
}

// --------------------------------------------------------------------------
// Interactive calibration: press C three times.
//   1st: start → "push device RIGHT, then press C"
//   2nd: record right direction → "push device UP, then press C"
//   3rd: record up direction → build matrix, done
// --------------------------------------------------------------------------
void HapticDissectionSimulation::_advanceCalibration()
{
    Vec3r device_pos = _haptic_device->position();

    switch (_calib_state)
    {
    case CalibState::IDLE:
    {
        _calib_start_pos = device_pos;
        _calib_state = CalibState::WAIT_RIGHT;
        std::cout << "\n=== CALIBRATION STARTED ===" << std::endl;
        std::cout << "Step 1: Push the device to the RIGHT, then press C" << std::endl;
        break;
    }

    case CalibState::WAIT_RIGHT:
    {
        Vec3r delta = device_pos - _calib_start_pos;
        if (delta.norm() < 1e-4)
        {
            std::cout << "[Calibration] Device hasn't moved enough. Push RIGHT and press C again." << std::endl;
            break;
        }
        _calib_device_right = delta.normalized();
        _calib_start_pos = device_pos;
        _calib_state = CalibState::WAIT_UP;
        std::cout << "  Recorded RIGHT direction in device space: ("
                  << _calib_device_right.transpose() << ")" << std::endl;
        std::cout << "Step 2: Push the device UP, then press C" << std::endl;
        break;
    }

    case CalibState::WAIT_UP:
    {
        Vec3r delta = device_pos - _calib_start_pos;
        if (delta.norm() < 1e-4)
        {
            std::cout << "[Calibration] Device hasn't moved enough. Push UP and press C again." << std::endl;
            break;
        }

        Vec3r d_right = _calib_device_right;
        Vec3r d_up_raw = delta.normalized();

        // Orthogonalize: remove the component of d_up_raw along d_right
        d_up_raw = d_up_raw - d_up_raw.dot(d_right) * d_right;
        if (d_up_raw.norm() < 1e-4)
        {
            std::cout << "[Calibration] UP direction too similar to RIGHT. Try again from Step 2." << std::endl;
            _calib_start_pos = device_pos;
            break;
        }
        Vec3r d_up = d_up_raw.normalized();

        // Third axis via cross product: right × up = out-of-screen (right-handed frame)
        // This gives det=+1 (proper rotation), so the quaternion can faithfully store it.
        // The camera-to-world reconstruction negates this axis to match cameraViewDirection.
        Vec3r d_forward = d_right.cross(d_up);
        d_forward.normalize();

        // Build the matrix: rows are the device-space basis vectors
        // M * device_vector = camera_vector
        // Row 0: device direction that maps to camera-right
        // Row 1: device direction that maps to camera-up
        // Row 2: device direction that maps to camera-forward
        _device_to_camera.row(0) = d_right.transpose();
        _device_to_camera.row(1) = d_up.transpose();
        _device_to_camera.row(2) = d_forward.transpose();
        _det_device_to_camera = _device_to_camera.determinant();

        _calib_state = CalibState::IDLE;

        std::cout << "  Recorded UP direction in device space: ("
                  << d_up.transpose() << ")" << std::endl;
        std::cout << "  Computed FORWARD direction: ("
                  << d_forward.transpose() << ")" << std::endl;
        std::cout << "\n=== CALIBRATION COMPLETE ===" << std::endl;
        std::cout << "  Device-to-camera matrix (det=" << _det_device_to_camera << "):\n"
                  << _device_to_camera << std::endl;

        // Print as quaternion for config file
        // Extract quaternion from rotation matrix (Shepperd's method)
        Real trace = _device_to_camera.trace();
        Vec4r q;
        if (trace > 0)
        {
            Real s = 0.5 / std::sqrt(trace + 1.0);
            q[3] = 0.25 / s;
            q[0] = (_device_to_camera(2,1) - _device_to_camera(1,2)) * s;
            q[1] = (_device_to_camera(0,2) - _device_to_camera(2,0)) * s;
            q[2] = (_device_to_camera(1,0) - _device_to_camera(0,1)) * s;
        }
        else if (_device_to_camera(0,0) > _device_to_camera(1,1) && _device_to_camera(0,0) > _device_to_camera(2,2))
        {
            Real s = 2.0 * std::sqrt(1.0 + _device_to_camera(0,0) - _device_to_camera(1,1) - _device_to_camera(2,2));
            q[3] = (_device_to_camera(2,1) - _device_to_camera(1,2)) / s;
            q[0] = 0.25 * s;
            q[1] = (_device_to_camera(0,1) + _device_to_camera(1,0)) / s;
            q[2] = (_device_to_camera(0,2) + _device_to_camera(2,0)) / s;
        }
        else if (_device_to_camera(1,1) > _device_to_camera(2,2))
        {
            Real s = 2.0 * std::sqrt(1.0 + _device_to_camera(1,1) - _device_to_camera(0,0) - _device_to_camera(2,2));
            q[3] = (_device_to_camera(0,2) - _device_to_camera(2,0)) / s;
            q[0] = (_device_to_camera(0,1) + _device_to_camera(1,0)) / s;
            q[1] = 0.25 * s;
            q[2] = (_device_to_camera(1,2) + _device_to_camera(2,1)) / s;
        }
        else
        {
            Real s = 2.0 * std::sqrt(1.0 + _device_to_camera(2,2) - _device_to_camera(0,0) - _device_to_camera(1,1));
            q[3] = (_device_to_camera(1,0) - _device_to_camera(0,1)) / s;
            q[0] = (_device_to_camera(0,2) + _device_to_camera(2,0)) / s;
            q[1] = (_device_to_camera(1,2) + _device_to_camera(2,1)) / s;
            q[2] = 0.25 * s;
        }
        q.normalize();

        std::cout << "\n  To save this calibration, add to your YAML config:" << std::endl;
        std::cout << "  device-to-camera-rotation: ["
                  << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << std::endl;
        std::cout << std::endl;
        break;
    }
    }
}

// --------------------------------------------------------------------------
// Set the grip-to-camera matrix from a quaternion (same as device-to-camera).
// --------------------------------------------------------------------------
void HapticDissectionSimulation::_setGripToCameraFromQuat(const Vec4r& quat)
{
    Real x = quat[0], y = quat[1], z = quat[2], w = quat[3];
    Real x2 = x*x, y2 = y*y, z2 = z*z;
    Real xy = x*y, xz = x*z, yz = y*z;
    Real wx = w*x, wy = w*y, wz = w*z;

    _grip_to_camera <<
        1 - 2*(y2 + z2),   2*(xy - wz),       2*(xz + wy),
        2*(xy + wz),       1 - 2*(x2 + z2),   2*(yz - wx),
        2*(xz - wy),       2*(yz + wx),       1 - 2*(x2 + y2);

    _det_grip_to_camera = _grip_to_camera.determinant();
}

// --------------------------------------------------------------------------
// Helper: extract quaternion from a 3x3 rotation matrix (Shepperd's method).
// Returns (x, y, z, w) scalar-last.
// --------------------------------------------------------------------------
static Vec4r _matToQuat(const Mat3r& M)
{
    Vec4r q;
    Real trace = M.trace();
    if (trace > 0)
    {
        Real s = 0.5 / std::sqrt(trace + 1.0);
        q[3] = 0.25 / s;
        q[0] = (M(2,1) - M(1,2)) * s;
        q[1] = (M(0,2) - M(2,0)) * s;
        q[2] = (M(1,0) - M(0,1)) * s;
    }
    else if (M(0,0) > M(1,1) && M(0,0) > M(2,2))
    {
        Real s = 2.0 * std::sqrt(1.0 + M(0,0) - M(1,1) - M(2,2));
        q[3] = (M(2,1) - M(1,2)) / s;
        q[0] = 0.25 * s;
        q[1] = (M(0,1) + M(1,0)) / s;
        q[2] = (M(0,2) + M(2,0)) / s;
    }
    else if (M(1,1) > M(2,2))
    {
        Real s = 2.0 * std::sqrt(1.0 + M(1,1) - M(0,0) - M(2,2));
        q[3] = (M(0,2) - M(2,0)) / s;
        q[0] = (M(0,1) + M(1,0)) / s;
        q[1] = 0.25 * s;
        q[2] = (M(1,2) + M(2,1)) / s;
    }
    else
    {
        Real s = 2.0 * std::sqrt(1.0 + M(2,2) - M(0,0) - M(1,1));
        q[3] = (M(1,0) - M(0,1)) / s;
        q[0] = (M(0,2) + M(2,0)) / s;
        q[1] = (M(1,2) + M(2,1)) / s;
        q[2] = 0.25 * s;
    }
    q.normalize();
    return q;
}

// --------------------------------------------------------------------------
// Interactive ROTATION calibration: press O three times.
//   1st: start → "yaw RIGHT (clockwise from above), press O"
//   2nd: record yaw axis → "pitch UP (tilt tip upward), press O"
//   3rd: record pitch axis → build grip_to_camera matrix, done
//
// Yaw = rotation around camera-UP axis
// Pitch = rotation around camera-RIGHT axis
// --------------------------------------------------------------------------
void HapticDissectionSimulation::_advanceRotationCalibration()
{
    Vec4r current_grip = _haptic_device->orientation();

    switch (_rot_calib_state)
    {
    case RotCalibState::IDLE:
    {
        _rot_calib_initial_grip = current_grip;
        _rot_calib_state = RotCalibState::WAIT_YAW;
        std::cout << "\n=== ROTATION CALIBRATION STARTED ===" << std::endl;
        std::cout << "Step 1: YAW RIGHT (turn handle clockwise from above), then press O" << std::endl;
        break;
    }

    case RotCalibState::WAIT_YAW:
    {
        // Compute relative rotation: delta = current * inv(initial)
        Vec4r inv_init = Vec4r(-_rot_calib_initial_grip[0], -_rot_calib_initial_grip[1],
                               -_rot_calib_initial_grip[2],  _rot_calib_initial_grip[3]);
        Vec4r delta = GeometryUtils::quatMult(current_grip, inv_init);
        delta.normalize();

        // Extract rotation axis from quaternion vector part
        Vec3r axis(delta[0], delta[1], delta[2]);
        if (axis.norm() < 1e-4)
        {
            std::cout << "[RotCalibration] Grip hasn't rotated enough. Yaw RIGHT and press O again." << std::endl;
            break;
        }
        // Yaw RIGHT = clockwise from above.  By right-hand rule the quaternion
        // axis points DOWN (-cam_up), but we need the matrix row to represent
        // +cam_up.  Negate so that the dot product gives the correct sign at
        // runtime (yaw right → negative camera-up component → correct direction).
        _rot_calib_yaw_axis = -axis.normalized();

        _rot_calib_initial_grip = current_grip;
        _rot_calib_state = RotCalibState::WAIT_PITCH;
        std::cout << "  Recorded YAW axis in grip space: ("
                  << _rot_calib_yaw_axis.transpose() << ")" << std::endl;
        std::cout << "Step 2: PITCH UP (tilt handle tip upward), then press O" << std::endl;
        break;
    }

    case RotCalibState::WAIT_PITCH:
    {
        Vec4r inv_init = Vec4r(-_rot_calib_initial_grip[0], -_rot_calib_initial_grip[1],
                               -_rot_calib_initial_grip[2],  _rot_calib_initial_grip[3]);
        Vec4r delta = GeometryUtils::quatMult(current_grip, inv_init);
        delta.normalize();

        Vec3r axis(delta[0], delta[1], delta[2]);
        if (axis.norm() < 1e-4)
        {
            std::cout << "[RotCalibration] Grip hasn't rotated enough. Pitch UP and press O again." << std::endl;
            break;
        }
        // Pitch UP = rotation around camera-RIGHT axis
        Vec3r d_pitch = axis.normalized();

        Vec3r d_yaw = _rot_calib_yaw_axis;

        // Orthogonalize: remove yaw component from pitch
        d_pitch = d_pitch - d_pitch.dot(d_yaw) * d_yaw;
        if (d_pitch.norm() < 1e-4)
        {
            std::cout << "[RotCalibration] Pitch axis too similar to yaw axis. Try again from Step 2." << std::endl;
            _rot_calib_initial_grip = current_grip;
            break;
        }
        d_pitch.normalize();

        // Third axis via cross product
        Vec3r d_roll = d_pitch.cross(d_yaw);
        d_roll.normalize();

        // Build the grip-to-camera matrix:
        // Row 0 (camera-right) = pitch axis (pitch = rotation around right)
        // Row 1 (camera-up)    = yaw axis   (yaw = rotation around up)
        // Row 2 (camera-fwd)   = roll axis  (cross product)
        _grip_to_camera.row(0) = d_pitch.transpose();
        _grip_to_camera.row(1) = d_yaw.transpose();
        _grip_to_camera.row(2) = d_roll.transpose();
        _det_grip_to_camera = _grip_to_camera.determinant();
        _has_grip_calibration = true;

        _rot_calib_state = RotCalibState::IDLE;

        std::cout << "  Recorded PITCH axis in grip space: ("
                  << d_pitch.transpose() << ")" << std::endl;
        std::cout << "  Computed ROLL axis: ("
                  << d_roll.transpose() << ")" << std::endl;
        std::cout << "\n=== ROTATION CALIBRATION COMPLETE ===" << std::endl;
        std::cout << "  Grip-to-camera matrix (det=" << _det_grip_to_camera << "):\n"
                  << _grip_to_camera << std::endl;

        // Print as quaternion for config
        Vec4r q = _matToQuat(_grip_to_camera);
        std::cout << "\n  To save this calibration, add to your YAML config:" << std::endl;
        std::cout << "  grip-to-camera-rotation: ["
                  << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << std::endl;
        std::cout << std::endl;

        // Also re-capture initial grip orientation for live use
        _initial_grip_quat = current_grip;
        _initial_knife_quat = _cursor->orientation();
        std::cout << "  (Initial grip/knife orientation recaptured)" << std::endl;
        break;
    }
    }
}

// --------------------------------------------------------------------------
// Axis cycling rotation calibration.
// M key: cycle through axis options.  Comma: confirm current choice.
//
// Step 1 (PICKING_YAW): cycle 6 yaw axis options (±X, ±Y, ±Z of grip → camera-up)
// Step 2 (PICKING_PITCH): cycle 4 pitch axis options (remaining axes → camera-right)
// Roll axis is always cross(pitch, yaw) to guarantee det=+1.
// --------------------------------------------------------------------------

static const Vec3r kYawAxisOptions[6] = {
    Vec3r(1, 0, 0),  Vec3r(-1, 0, 0),   // ±X
    Vec3r(0, 1, 0),  Vec3r(0, -1, 0),   // ±Y
    Vec3r(0, 0, 1),  Vec3r(0, 0, -1),   // ±Z
};
static const char* kYawAxisNames[6] = {
    "+X_grip", "-X_grip", "+Y_grip", "-Y_grip", "+Z_grip", "-Z_grip"
};

void HapticDissectionSimulation::_getPitchOptions(
    int yaw_idx, Vec3r pitch_out[4], std::string names_out[4]) const
{
    // Yaw uses coordinate axis yaw_idx/2 (0=X, 1=Y, 2=Z)
    int yaw_coord = yaw_idx / 2;

    // Find the two remaining coordinate axes
    int rem[2];
    int rc = 0;
    for (int i = 0; i < 3; ++i)
        if (i != yaw_coord) rem[rc++] = i;

    const char* coord_name[3] = {"X", "Y", "Z"};

    // 4 pitch options: rem[0]+, rem[0]-, rem[1]+, rem[1]-
    for (int i = 0; i < 2; ++i)
    {
        pitch_out[i * 2]     = Vec3r::Zero();
        pitch_out[i * 2]    [rem[i]] = 1.0;
        pitch_out[i * 2 + 1] = Vec3r::Zero();
        pitch_out[i * 2 + 1][rem[i]] = -1.0;

        names_out[i * 2]     = std::string("+") + coord_name[rem[i]] + "_grip";
        names_out[i * 2 + 1] = std::string("-") + coord_name[rem[i]] + "_grip";
    }
}

void HapticDissectionSimulation::_applyTrialGripMatrix()
{
    Vec3r yaw_axis = kYawAxisOptions[_axis_calib_yaw_index];

    Vec3r pitch_options[4];
    std::string pitch_names[4];
    _getPitchOptions(_axis_calib_yaw_index, pitch_options, pitch_names);

    Vec3r pitch_axis = pitch_options[_axis_calib_pitch_index];
    Vec3r roll_axis = pitch_axis.cross(yaw_axis);
    roll_axis.normalize();

    // Row 0 = pitch axis (camera-right)
    // Row 1 = yaw axis   (camera-up)
    // Row 2 = roll axis  (camera-back)
    _grip_to_camera.row(0) = pitch_axis.transpose();
    _grip_to_camera.row(1) = yaw_axis.transpose();
    _grip_to_camera.row(2) = roll_axis.transpose();
    _det_grip_to_camera = _grip_to_camera.determinant();
    _has_grip_calibration = true;

    // Re-capture initial orientations to prevent sudden jump
    _initial_grip_quat = _haptic_device->orientation();
    _initial_knife_quat = _cursor->orientation();
}

void HapticDissectionSimulation::_cycleAxisCalib()
{
    switch (_axis_calib_state)
    {
    case AxisCalibState::IDLE:
    {
        _axis_calib_state = AxisCalibState::PICKING_YAW;
        _axis_calib_yaw_index = 0;
        _axis_calib_pitch_index = 0;
        _applyTrialGripMatrix();
        std::cout << "\n=== ROTATION AXIS CALIBRATION ===" << std::endl;
        std::cout << "Step 1: Turn the grip LEFT-RIGHT and press M to cycle yaw axis." << std::endl;
        std::cout << "        Press COMMA to confirm when yaw feels correct." << std::endl;
        std::cout << "  Yaw axis: " << kYawAxisNames[0] << "  (1/6)" << std::endl;
        break;
    }
    case AxisCalibState::PICKING_YAW:
    {
        _axis_calib_yaw_index = (_axis_calib_yaw_index + 1) % 6;
        _axis_calib_pitch_index = 0;  // reset pitch for new yaw
        _applyTrialGripMatrix();
        std::cout << "  Yaw axis: " << kYawAxisNames[_axis_calib_yaw_index]
                  << "  (" << (_axis_calib_yaw_index + 1) << "/6)" << std::endl;
        break;
    }
    case AxisCalibState::PICKING_PITCH:
    {
        _axis_calib_pitch_index = (_axis_calib_pitch_index + 1) % 4;
        _applyTrialGripMatrix();

        Vec3r pitch_options[4];
        std::string pitch_names[4];
        _getPitchOptions(_axis_calib_yaw_index, pitch_options, pitch_names);
        std::cout << "  Pitch axis: " << pitch_names[_axis_calib_pitch_index]
                  << "  (" << (_axis_calib_pitch_index + 1) << "/4)" << std::endl;
        break;
    }
    }
}

void HapticDissectionSimulation::_confirmAxisCalib()
{
    switch (_axis_calib_state)
    {
    case AxisCalibState::IDLE:
        break;

    case AxisCalibState::PICKING_YAW:
    {
        std::cout << "  Yaw axis LOCKED: " << kYawAxisNames[_axis_calib_yaw_index] << std::endl;
        std::cout << "\nStep 2: Tilt the grip UP-DOWN and press M to cycle pitch axis." << std::endl;
        std::cout << "        Press COMMA to confirm when pitch feels correct." << std::endl;

        _axis_calib_state = AxisCalibState::PICKING_PITCH;
        _axis_calib_pitch_index = 0;
        _applyTrialGripMatrix();

        Vec3r pitch_options[4];
        std::string pitch_names[4];
        _getPitchOptions(_axis_calib_yaw_index, pitch_options, pitch_names);
        std::cout << "  Pitch axis: " << pitch_names[0] << "  (1/4)" << std::endl;
        break;
    }

    case AxisCalibState::PICKING_PITCH:
    {
        _axis_calib_state = AxisCalibState::IDLE;

        Vec3r pitch_options[4];
        std::string pitch_names[4];
        _getPitchOptions(_axis_calib_yaw_index, pitch_options, pitch_names);

        std::cout << "  Pitch axis LOCKED: " << pitch_names[_axis_calib_pitch_index] << std::endl;
        std::cout << "\n=== ROTATION AXIS CALIBRATION COMPLETE ===" << std::endl;
        std::cout << "  Grip-to-camera matrix (det=" << _det_grip_to_camera << "):\n"
                  << _grip_to_camera << std::endl;

        // Print quaternion for saving to config
        Vec4r q = _matToQuat(_grip_to_camera);
        std::cout << "\n  To save this calibration, add to your YAML config:" << std::endl;
        std::cout << "  grip-to-camera-rotation: ["
                  << q[0] << ", " << q[1] << ", " << q[2] << ", " << q[3] << "]" << std::endl;
        std::cout << std::endl;
        break;
    }
    }
}

} // namespace Sim
