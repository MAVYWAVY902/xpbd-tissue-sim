#ifndef __RESIDUAL_SIMULATION_CONFIG_HPP
#define __RESIDUAL_SIMULATION_CONFIG_HPP

#include "config/OutputSimulationConfig.hpp"

class ResidualSimulationConfig : public OutputSimulationConfig
{
    public:
    explicit ResidualSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {}
};

#endif // __RESIDUAL_SIMULATION_CONFIG_HPP