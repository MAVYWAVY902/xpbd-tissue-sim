#pragma once

#include "config/simulation/PushingSimulationConfig.hpp"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace Config
{

/**
 * @brief Configuration class for HapticDissectionSimulation.
 *
 * Extends PushingSimulationConfig with parameters for the Haply Inverse3
 * haptic device integration (force scaling, workspace mapping, filtering).
 */
class HapticDissectionSimulationConfig : public PushingSimulationConfig
{
public:
    HapticDissectionSimulationConfig(const YAML::Node& config_node)
        : PushingSimulationConfig(config_node)
    {
        if (config_node["haptic-force-scaling"])
            _haptic_force_scaling = config_node["haptic-force-scaling"].as<Real>();

        if (config_node["haptic-serial-port"])
            _haptic_serial_port = config_node["haptic-serial-port"].as<std::string>();

        if (config_node["contact-force-stiffness"])
            _contact_force_stiffness = config_node["contact-force-stiffness"].as<Real>();

        if (config_node["force-filter-alpha"])
            _force_filter_alpha = config_node["force-filter-alpha"].as<Real>();

        if (config_node["haptic-workspace-radius"])
            _haptic_workspace_radius = config_node["haptic-workspace-radius"].as<Real>();

        if (config_node["sim-workspace-radius"])
            _sim_workspace_radius = config_node["sim-workspace-radius"].as<Real>();

        if (config_node["knife-rotation-speed"])
            _knife_rotation_speed = config_node["knife-rotation-speed"].as<Real>();

        if (config_node["rotation-sensitivity"])
            _rotation_sensitivity = config_node["rotation-sensitivity"].as<Real>();

        // device-to-camera-rotation: quaternion [x,y,z,w] — takes priority over device-to-sim-axes
        if (config_node["device-to-camera-rotation"])
        {
            auto quat_node = config_node["device-to-camera-rotation"];
            if (quat_node.IsSequence() && quat_node.size() == 4)
            {
                _device_to_camera_quat[0] = quat_node[0].as<Real>();
                _device_to_camera_quat[1] = quat_node[1].as<Real>();
                _device_to_camera_quat[2] = quat_node[2].as<Real>();
                _device_to_camera_quat[3] = quat_node[3].as<Real>();
                _device_to_camera_quat.normalize();
                _has_device_rotation_quat = true;
            }
        }

        // grip-to-camera-rotation: quaternion [x,y,z,w] for VerseGrip orientation mapping
        if (config_node["grip-to-camera-rotation"])
        {
            auto quat_node = config_node["grip-to-camera-rotation"];
            if (quat_node.IsSequence() && quat_node.size() == 4)
            {
                _grip_to_camera_quat[0] = quat_node[0].as<Real>();
                _grip_to_camera_quat[1] = quat_node[1].as<Real>();
                _grip_to_camera_quat[2] = quat_node[2].as<Real>();
                _grip_to_camera_quat[3] = quat_node[3].as<Real>();
                _grip_to_camera_quat.normalize();
                _has_grip_rotation_quat = true;
            }
        }

        // input-device: "haptic" (default) or "mouse" (skip device, use mouse/keyboard like PushingTest)
        if (config_node["input-device"])
        {
            std::string mode = config_node["input-device"].as<std::string>();
            // normalize to lowercase
            std::transform(mode.begin(), mode.end(), mode.begin(), ::tolower);
            if (mode == "mouse" || mode == "keyboard" || mode == "mouse+keyboard")
                _use_haptic_device = false;
            else
                _use_haptic_device = true;  // "haptic", "device", or anything else → use device
        }

        // Legacy: device-to-sim-axes (signed permutation) — used only if quaternion not specified
        if (config_node["device-to-sim-axes"])
        {
            auto axes_node = config_node["device-to-sim-axes"];
            if (axes_node.IsSequence() && axes_node.size() == 3)
            {
                _device_to_sim_axes.clear();
                for (int i = 0; i < 3; ++i)
                    _device_to_sim_axes.push_back(axes_node[i].as<int>());

                // Validate: must be a signed permutation of {1,2,3}
                std::vector<int> abs_vals;
                for (int v : _device_to_sim_axes) abs_vals.push_back(std::abs(v));
                std::sort(abs_vals.begin(), abs_vals.end());
                if (abs_vals[0] != 1 || abs_vals[1] != 2 || abs_vals[2] != 3)
                {
                    std::cerr << "[HapticDissectionConfig] Invalid device-to-sim-axes: "
                              << "must be a signed permutation of {1,2,3}. Using default."
                              << std::endl;
                    _device_to_sim_axes = {1, 2, -3};
                }
            }
        }
    }

    Real hapticForceScaling() const { return _haptic_force_scaling; }
    const std::string& hapticSerialPort() const { return _haptic_serial_port; }
    Real contactForceStiffness() const { return _contact_force_stiffness; }
    Real forceFilterAlpha() const { return _force_filter_alpha; }
    Real hapticWorkspaceRadius() const { return _haptic_workspace_radius; }
    Real simWorkspaceRadius() const { return _sim_workspace_radius; }
    Real knifeRotationSpeed() const { return _knife_rotation_speed; }
    Real rotationSensitivity() const { return _rotation_sensitivity; }
    const std::vector<int>& deviceToSimAxes() const { return _device_to_sim_axes; }
    bool hasDeviceRotationQuat() const { return _has_device_rotation_quat; }
    Vec4r deviceToCameraQuat() const { return _device_to_camera_quat; }
    bool hasGripRotationQuat() const { return _has_grip_rotation_quat; }
    Vec4r gripToCameraQuat() const { return _grip_to_camera_quat; }
    bool useHapticDevice() const { return _use_haptic_device; }

private:
    Real _haptic_force_scaling = 5.0;         ///< multiplier on forces sent to device
    std::string _haptic_serial_port = "";     ///< serial port ("" = auto-detect)
    Real _contact_force_stiffness = 500.0;    ///< penalty stiffness for SDF contact feedback [N/m]
    Real _force_filter_alpha = 0.3;           ///< low-pass filter coefficient (0-1)
    Real _haptic_workspace_radius = 0.08;     ///< Inverse3 physical workspace radius [m]
    Real _sim_workspace_radius = 0.1;         ///< simulation workspace radius to map into [m]
    Real _knife_rotation_speed = 1.0;        ///< keyboard rotation speed [rad/s]
    Real _rotation_sensitivity = 1.0;        ///< VerseGrip rotation amplification (>1 = more sensitive)
    std::vector<int> _device_to_sim_axes = {1, 2, -3}; ///< signed permutation: device→sim axis mapping (legacy)
    Vec4r _device_to_camera_quat = Vec4r(0, 0, 0, 1);  ///< device-to-camera rotation quaternion (xyzw)
    bool _has_device_rotation_quat = false;              ///< true if quaternion was specified in config
    Vec4r _grip_to_camera_quat = Vec4r(0, 0, 0, 1);    ///< grip-to-camera rotation quaternion (xyzw)
    bool _has_grip_rotation_quat = false;                ///< true if grip quaternion was specified in config
    bool _use_haptic_device = true;                      ///< false → mouse/keyboard only (no device connection)
};

} // namespace Config
