#include "OutputSimulation.hpp"
#include "MeshUtils.hpp"


void OutputSimulation::_init()
{
    Simulation::_init();

    // initialize the output file
    _out_file = std::ofstream("../output/beam_stretch/out_" + _name + ".txt");
}

OutputSimulation::OutputSimulation()
    : Simulation()
{

}

OutputSimulation::OutputSimulation(const std::string& config_filename)
    : Simulation(config_filename), _out_file("../output/out_" + _name + ".txt")
{
    _print_interval_s = 1e-3;
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