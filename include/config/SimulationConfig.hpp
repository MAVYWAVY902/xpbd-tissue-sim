#ifndef __SIMULATION_CONFIG_HPP
#define __SIMULATION_CONFIG_HPP

#include "config/MeshObjectConfig.hpp"

class SimulationConfig : public Config
{
    public:
    explicit SimulationConfig(const YAML::Node& node);

    std::optional<double> timeStep() const { return _time_step.value; }
    std::optional<double> endTime() const { return _end_time.value; }

    const std::vector<std::unique_ptr<MeshObjectConfig> >& meshObjectConfigs() const { return _mesh_object_configs; }

    protected:
    ConfigParameter<double> _time_step;
    ConfigParameter<double> _end_time; 

    std::vector<std::unique_ptr<MeshObjectConfig>> _mesh_object_configs;

};

#endif // __SIMULATION_CONFIG_HPP