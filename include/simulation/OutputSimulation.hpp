#ifndef __OUTPUT_SIMULATION_HPP
#define __OUTPUT_SIMULATION_HPP

#include "Simulation.hpp"
#include "common/types.hpp"

#include <fstream>

namespace Sim
{

#include <fstream>

namespace Sim
{

class OutputSimulation : public Simulation
{
    public:
    explicit OutputSimulation(const std::string& config_filename);

    ~OutputSimulation();

    protected:
    explicit OutputSimulation();

    public:
    virtual std::string type() const override { return "OutputSimulation"; }
    virtual void printInfo() const = 0;

    protected:
    virtual void _timeStep() override;

    virtual void _init() override;

    
    Real _print_interval_s;
    Real _last_print_sim_time;

    mutable std::ofstream _out_file;
};

} // namespace Sim

#endif // __OUTPUT_SIMULATION_HPP