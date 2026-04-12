#pragma once

#include "simulation/PushingSimulation.hpp"
#include "config/simulation/HapticDissectionSimulationConfig.hpp"
#include <memory>
#include <map>
#include <string>
#include <vector>

class HaplyInverse3Device;

namespace Sim
{

/**
 * @brief Extends PushingSimulation with Haply Inverse3 haptic device integration.
 *
 * When the device is connected, the knife tool tracks the end-effector position
 * and the user receives force feedback from tissue contact, adhesion resistance,
 * and the snap of cutting adhesion bonds.
 *
 * When no device is detected, falls back to mouse/keyboard input inherited from
 * PushingSimulation.
 */
class HapticDissectionSimulation : public PushingSimulation
{
public:
    HapticDissectionSimulation(const Config::HapticDissectionSimulationConfig* config);
    ~HapticDissectionSimulation();

    void setup() override;

    void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;

protected:
    void _timeStep() override;

private:
    /// Compute penalty contact force from tissue vertices penetrating the knife SDF.
    Vec3r _computeContactForce();

    /// Sum adhesion constraint forces across all deformable objects (Newton's 3rd law → negate).
    Vec3r _collectNetAdhesionForce();

    /// Map a device position [m] to simulation-space knife position.
    Vec3r _hapticToSimPosition(const Vec3r& haptic_pos) const;

    /// Map a simulation-space force to device-frame force [N].
    Vec3r _simToHapticForce(const Vec3r& sim_force) const;

    /// Transform a device-frame quaternion (xyzw) to sim frame.
    Vec4r _hapticToSimQuaternion(const Vec4r& device_quat) const;

    /// Build the 3x3 matrix from a 1-based signed axis permutation (legacy).
    void _setAxisMappingFromPermutation(const std::vector<int>& axes);

    /// Set the device-to-camera matrix from a quaternion (xyzw scalar-last).
    void _setDeviceToCameraFromQuat(const Vec4r& quat);

    /// Run one step of the interactive position calibration state machine (C key).
    void _advanceCalibration();

    /// Run one step of the interactive rotation calibration state machine (O key).
    void _advanceRotationCalibration();

    /// Set the grip-to-camera matrix from a quaternion (xyzw scalar-last).
    void _setGripToCameraFromQuat(const Vec4r& quat);

    /// Axis cycling rotation calibration (M key to cycle, comma to confirm).
    void _cycleAxisCalib();
    void _confirmAxisCalib();
    void _applyTrialGripMatrix();
    void _getPitchOptions(int yaw_idx, Vec3r pitch_out[4], std::string names_out[4]) const;

    std::unique_ptr<HaplyInverse3Device> _haptic_device;

    Vec3r _haptic_origin = Vec3r::Zero();          ///< knife starting position in sim frame
    Vec3r _haptic_device_origin = Vec3r::Zero();   ///< device rest position at startup
    Vec3r _prev_haptic_force = Vec3r::Zero();      ///< previous filtered force (for low-pass filter)
    Vec4r _initial_grip_quat = Vec4r(0,0,0,1);    ///< VerseGrip orientation at startup
    Vec4r _initial_knife_quat = Vec4r(0,0,0,1);   ///< knife orientation at startup
    bool _use_grip_orientation = true;              ///< toggle with 'G' key
    Mat3r _device_to_camera = Mat3r::Identity();     ///< orthogonal matrix for position (device→camera-local)
    Real _det_device_to_camera = 1.0;               ///< determinant of _device_to_camera (+1 or -1)
    Mat3r _grip_to_camera = Mat3r::Identity();       ///< orthogonal matrix for rotation (grip→camera-local)
    Real _det_grip_to_camera = 1.0;                  ///< determinant of _grip_to_camera
    bool _has_grip_calibration = false;              ///< true if grip rotation calibration is available
    bool _diag_printing = false;                    ///< toggle with 'N' key

    // Position calibration state (C key)
    enum class CalibState { IDLE, WAIT_RIGHT, WAIT_UP };
    CalibState _calib_state = CalibState::IDLE;
    Vec3r _calib_start_pos = Vec3r::Zero();        ///< device position at start of calibration step
    Vec3r _calib_device_right = Vec3r::Zero();     ///< measured device-space "right" direction

    // Rotation calibration state (O key)
    enum class RotCalibState { IDLE, WAIT_YAW, WAIT_PITCH };
    RotCalibState _rot_calib_state = RotCalibState::IDLE;
    Vec4r _rot_calib_initial_grip = Vec4r(0,0,0,1); ///< grip orientation at start of rotation calibration step
    Vec3r _rot_calib_yaw_axis = Vec3r::Zero();       ///< measured device-space yaw rotation axis

    // Axis cycling rotation calibration state (M key)
    enum class AxisCalibState { IDLE, PICKING_YAW, PICKING_PITCH };
    AxisCalibState _axis_calib_state = AxisCalibState::IDLE;
    int _axis_calib_yaw_index = 0;     ///< 0-5: ±X, ±Y, ±Z
    int _axis_calib_pitch_index = 0;   ///< 0-3: remaining axis options

    // Config parameters
    Real _haptic_force_scaling = 5.0;
    Real _contact_force_stiffness = 500.0;
    Real _force_filter_alpha = 0.3;
    Real _haptic_workspace_radius = 0.08;
    Real _sim_workspace_radius = 0.1;

    Real _rotation_sensitivity = 1.0;  ///< VerseGrip rotation amplification (>1 = more sensitive)

    // Keyboard rotation state
    Real _rotation_speed = 1.0;  ///< radians per second
    std::map<SimulationInput::Key, bool> _rotation_keys_held;
};

} // namespace Sim
