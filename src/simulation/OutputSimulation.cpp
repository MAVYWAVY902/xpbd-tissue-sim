#include "simulation/OutputSimulation.hpp"
#include "config/OutputSimulationConfig.hpp"
#include "utils/MeshUtils.hpp"

#include "simobject/XPBDMeshObject.hpp"

namespace Sim
{

void OutputSimulation::_init()
{
    Simulation::_init();

    _last_print_sim_time = 0;

    // extract the stretch velocity and time from the config object
    OutputSimulationConfig* output_simulation_config = dynamic_cast<OutputSimulationConfig*>(_config.get());
    _print_interval_s = output_simulation_config->printInterval().value();

    std::string output_folder = output_simulation_config->outputFolder().value();
    if (output_folder.back() != '/')
    {
        output_folder += "/";
    }
    // initialize the output file
    _out_file = std::ofstream(output_folder + _name + "_output.txt");
}

OutputSimulation::OutputSimulation()
    : Simulation()
{

}

OutputSimulation::OutputSimulation(const std::string& config_filename)
    : Simulation()
{
    // create a more specialized config object specifically for BeamStretchSimulations
    _config = std::make_unique<OutputSimulationConfig>(YAML::LoadFile(config_filename));

    // initialize quantities using config object
    _init();
}

void OutputSimulation::_timeStep()
{
    Simulation::_timeStep();

    const double eps = 1e-12;
    if (_time - _last_print_sim_time + eps >= _print_interval_s)
    {
        printInfo();
        _last_print_sim_time = _time;
    }

}

} // namespace Sim