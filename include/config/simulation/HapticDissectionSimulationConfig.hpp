#pragma once

#include "config/simulation/PushingSimulationConfig.hpp"

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
    }

    Real hapticForceScaling() const { return _haptic_force_scaling; }
    const std::string& hapticSerialPort() const { return _haptic_serial_port; }
    Real contactForceStiffness() const { return _contact_force_stiffness; }
    Real forceFilterAlpha() const { return _force_filter_alpha; }
    Real hapticWorkspaceRadius() const { return _haptic_workspace_radius; }
    Real simWorkspaceRadius() const { return _sim_workspace_radius; }
    Real knifeRotationSpeed() const { return _knife_rotation_speed; }

private:
    Real _haptic_force_scaling = 5.0;         ///< multiplier on forces sent to device
    std::string _haptic_serial_port = "";     ///< serial port ("" = auto-detect)
    Real _contact_force_stiffness = 500.0;    ///< penalty stiffness for SDF contact feedback [N/m]
    Real _force_filter_alpha = 0.3;           ///< low-pass filter coefficient (0-1)
    Real _haptic_workspace_radius = 0.08;     ///< Inverse3 physical workspace radius [m]
    Real _sim_workspace_radius = 0.1;         ///< simulation workspace radius to map into [m]
    Real _knife_rotation_speed = 1.0;        ///< keyboard rotation speed [rad/s]
};

} // namespace Config
