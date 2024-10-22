#ifndef __STATIC_LOADING_SIMULATION_CONFIG_HPP
#define __STATIC_LOADING_SIMULATION_CONFIG_HPP

#include "OutputSimulationConfig.hpp"

class StaticLoadingSimulationConfig : public OutputSimulationConfig
{
    static std::optional<double>& DEFAULT_CENTER_FORCE() { static std::optional<double> center_force(0); return center_force; }

    public:
    explicit StaticLoadingSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {
        _extractParameter("center-force", node, _center_force, DEFAULT_CENTER_FORCE());
    }

    std::optional<double> centerForce() const { return _center_force.value; }

    protected:
    ConfigParameter<double> _center_force;
};


#endif // __STATIC_LOADING_SIMULATION_CONFIG_HPP