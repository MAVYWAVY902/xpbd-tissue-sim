#ifndef __BEAM_SIMULATION_CONFIG
#define __BEAM_SIMULATION_CONFIG

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class BeamSimulationConfig : public SimulationConfig
{
    public:
    explicit BeamSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {}
};

} // namespace Config


#endif // __BEAM_SIMULATION_CONFIG