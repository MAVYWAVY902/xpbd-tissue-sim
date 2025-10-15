#pragma once

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

/**
 * @brief Configuration class for PushingSimulation
 */
class PushingSimulationConfig : public SimulationConfig
{
public:
    PushingSimulationConfig(const YAML::Node& config_node) : SimulationConfig(config_node) 
    {
        if (config_node["tool-radius"])
            _tool_radius = config_node["tool-radius"].as<Real>();
        
        if (config_node["push-stiffness"])
            _push_stiffness = config_node["push-stiffness"].as<Real>();
            
        if (config_node["max-push-force"])
            _max_push_force = config_node["max-push-force"].as<Real>();
            
        if (config_node["fix-min-z"])
            _fix_min_z = config_node["fix-min-z"].as<bool>();
    }

    Real toolRadius() const { return _tool_radius; }
    Real pushStiffness() const { return _push_stiffness; }
    Real maxPushForce() const { return _max_push_force; }
    bool fixMinZ() const { return _fix_min_z; }

private:
    Real _tool_radius = 0.2;        ///< default tool radius [m]
    Real _push_stiffness = 1000.0;  ///< default push stiffness [N/m]
    Real _max_push_force = 50.0;    ///< default max push force [N]
    bool _fix_min_z = true;         ///< default: fix bottom vertices
};

} // namespace Config