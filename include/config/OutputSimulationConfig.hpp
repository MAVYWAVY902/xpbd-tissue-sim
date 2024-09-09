#ifndef __OUTPUT_SIMULATION_CONFIG_HPP
#define __OUTPUT_SIMULATION_CONFIG_HPP

#include "SimulationConfig.hpp"

class OutputSimulationConfig : public SimulationConfig
{
    /** Static predefined default for simulation time step */
    static std::optional<double>& DEFAULT_PRINT_INTERVAL() { static std::optional<double> print_interval(1e-3); return print_interval; }

    public:
    explicit OutputSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("print-interval", node, _print_interval, DEFAULT_PRINT_INTERVAL());
    }

    std::optional<double> printInterval() const { return _print_interval.value; }

    protected:
    ConfigParameter<double> _print_interval; // s
};

#endif // __OUTPUT_SIMULATION_CONFIG_HPP