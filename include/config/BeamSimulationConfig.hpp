#ifndef __BEAM_SIMULATION_CONFIG
#define __BEAM_SIMULATION_CONFIG

#include "config/ResidualSimulationConfig.hpp"

class BeamSimulationConfig : public ResidualSimulationConfig
{
    public:
    explicit BeamSimulationConfig(const YAML::Node& node)
        : ResidualSimulationConfig(node)
    {}
};


#endif // __BEAM_SIMULATION_CONFIG