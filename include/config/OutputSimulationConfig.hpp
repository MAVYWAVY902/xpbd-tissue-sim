#ifndef __OUTPUT_SIMULATION_CONFIG_HPP
#define __OUTPUT_SIMULATION_CONFIG_HPP

#include "SimulationConfig.hpp"

class OutputSimulationConfig : public SimulationConfig
{
    /** Static predefined default for simulation time step */
    static std::optional<double>& DEFAULT_PRINT_INTERVAL() { static std::optional<double> print_interval(1e-3); return print_interval; }
    /** Static predefined efault for output file folder location */
    static std::optional<std::string>& DEFAULT_OUTPUT_FOLDER() { static std::optional<std::string> output_folder("../output"); return output_folder; }

    public:
    explicit OutputSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("print-interval", node, _print_interval, DEFAULT_PRINT_INTERVAL());
        _extractParameter("output-folder", node, _output_folder, DEFAULT_OUTPUT_FOLDER());
    }

    std::optional<double> printInterval() const { return _print_interval.value; }
    std::optional<std::string> outputFolder() const { return _output_folder.value; }

    protected:
    ConfigParameter<double> _print_interval; // s
    ConfigParameter<std::string> _output_folder;
};

#endif // __OUTPUT_SIMULATION_CONFIG_HPP