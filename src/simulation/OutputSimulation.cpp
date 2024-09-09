#include "OutputSimulation.hpp"
#include "OutputSimulationConfig.hpp"
#include "MeshUtils.hpp"


void OutputSimulation::_init()
{
    Simulation::_init();

    _last_print_sim_time = 0;

    // extract the stretch velocity and time from the config object
    OutputSimulationConfig* output_simulation_config = dynamic_cast<OutputSimulationConfig*>(_config.get());
    _print_interval_s = output_simulation_config->printInterval().value();

    // initialize the output file
    _out_file = std::ofstream("../output/beam_stretch/out_" + _name + ".txt");
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

    if (_time - _last_print_sim_time >= _print_interval_s)
    {
        printInfo();
        _last_print_sim_time = _time;
    }

}