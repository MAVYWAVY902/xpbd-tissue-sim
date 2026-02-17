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
            
        if (config_node["push-damping"])
            _push_damping = config_node["push-damping"].as<Real>();
            
        if (config_node["max-push-force"])
            _max_push_force = config_node["max-push-force"].as<Real>();
            
        if (config_node["fix-min-z"])
            _fix_min_z = config_node["fix-min-z"].as<bool>();
            
        if (config_node["fix-max-z"])
            _fix_max_z = config_node["fix-max-z"].as<bool>();
            
        if (config_node["knife-scale-x"])
            _knife_scale_x = config_node["knife-scale-x"].as<Real>();
            
        if (config_node["knife-scale-y"])
            _knife_scale_y = config_node["knife-scale-y"].as<Real>();
            
        if (config_node["knife-scale-z"])
            _knife_scale_z = config_node["knife-scale-z"].as<Real>();
    }

    Real toolRadius() const { return _tool_radius; }
    Real pushStiffness() const { return _push_stiffness; }
    Real pushDamping() const { return _push_damping; }
    Real maxPushForce() const { return _max_push_force; }
    bool fixMinZ() const { return _fix_min_z; }
    bool fixMaxZ() const { return _fix_max_z; }
    Real knifeScaleX() const { return _knife_scale_x; }
    Real knifeScaleY() const { return _knife_scale_y; }
    Real knifeScaleZ() const { return _knife_scale_z; }

private:
    Real _tool_radius = 0.2;        ///< default tool radius [m]
    Real _push_stiffness = 1000.0;  ///< default push stiffness [N/m]
    Real _push_damping = 0.5;       ///< default push damping coefficient (0-1, higher = more damping)
    Real _max_push_force = 50.0;    ///< default max push force [N]
    bool _fix_min_z = true;         ///< default: fix bottom vertices
    bool _fix_max_z = false;        ///< default: don't fix top vertices
    Real _knife_scale_x = -1.0;     ///< knife X scale (-1 = use tool_radius for uniform scaling)
    Real _knife_scale_y = -1.0;     ///< knife Y scale (-1 = use tool_radius for uniform scaling)
    Real _knife_scale_z = -1.0;     ///< knife Z scale (-1 = use tool_radius for uniform scaling)
};

} // namespace Config