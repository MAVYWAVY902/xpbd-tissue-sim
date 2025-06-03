#ifndef __RESIDUAL_SIMULATION_CONFIG_HPP
#define __RESIDUAL_SIMULATION_CONFIG_HPP

#include "config/simulation/OutputSimulationConfig.hpp"

namespace Config
{

class ResidualSimulationConfig : public OutputSimulationConfig
{
    public:
    explicit ResidualSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {}
};

} // namespace Config

#endif // __RESIDUAL_SIMULATION_CONFIG_HPP