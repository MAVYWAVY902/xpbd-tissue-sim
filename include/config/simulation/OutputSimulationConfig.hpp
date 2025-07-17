#ifndef __OUTPUT_SIMULATION_CONFIG_HPP
#define __OUTPUT_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class OutputSimulationConfig : public SimulationConfig
{
    public:
    explicit OutputSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("print-interval", node, _print_interval);
        _extractParameter("output-folder", node, _output_folder);
    }

    Real printInterval() const { return _print_interval.value; }
    std::string outputFolder() const { return _output_folder.value; }

    protected:
    ConfigParameter<Real> _print_interval = ConfigParameter<Real>(1e-3); // s
    ConfigParameter<std::string> _output_folder = ConfigParameter<std::string>("../output");
};

} // namespace Config

#endif // __OUTPUT_SIMULATION_CONFIG_HPP