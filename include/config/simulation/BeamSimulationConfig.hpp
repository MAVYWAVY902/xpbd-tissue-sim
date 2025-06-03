#ifndef __BEAM_SIMULATION_CONFIG
#define __BEAM_SIMULATION_CONFIG

#include "config/simulation/ResidualSimulationConfig.hpp"

namespace Config
{

class BeamSimulationConfig : public ResidualSimulationConfig
{
    public:
    explicit BeamSimulationConfig(const YAML::Node& node)
        : ResidualSimulationConfig(node)
    {}
};

} // namespace Config


#endif // __BEAM_SIMULATION_CONFIG