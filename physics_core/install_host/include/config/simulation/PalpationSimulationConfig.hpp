#ifndef __PALPATION_SIMULATION_CONFIG_HPP
#define __PALPATION_SIMULATION_CONFIG_HPP

#include "config/simulation/VirtuosoSimulationConfig.hpp"

namespace Config
{

class PalpationSimulationConfig : public VirtuosoSimulationConfig
{
public:
    explicit PalpationSimulationConfig(const YAML::Node& node)
        : VirtuosoSimulationConfig(node)
    {}
};

} // namespace Config

#endif // __PALPATION_SIMULATION_CONFIG_HPP