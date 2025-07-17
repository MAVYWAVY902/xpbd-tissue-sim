#include "simulation/OutputSimulation.hpp"
#include "config/simulation/OutputSimulationConfig.hpp"
#include "utils/MeshUtils.hpp"

#include "simobject/XPBDMeshObject.hpp"

namespace Sim
{

OutputSimulation::OutputSimulation(const Config::OutputSimulationConfig* config)
    : Simulation(config)
{
    _last_print_sim_time = 0;

    // extract the stretch velocity and time from the config object
    _print_interval_s = config->printInterval();

    std::string output_folder = config->outputFolder();
    if (output_folder.back() != '/')
    {
        output_folder += "/";
    }
    // initialize the output file
    _out_file = std::ofstream(output_folder + _name + "_output.txt");
}

OutputSimulation::~OutputSimulation()
{
    
}

void OutputSimulation::_timeStep()
{
    Simulation::_timeStep();

    const Real eps = 1e-12;
    if (_time - _last_print_sim_time + eps >= _print_interval_s)
    {
        printInfo();
        _last_print_sim_time = _time;
    }

}

} // namespace Sim