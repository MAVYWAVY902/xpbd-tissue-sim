#ifndef __OUTPUT_SIMULATION_HPP
#define __OUTPUT_SIMULATION_HPP

#include "Simulation.hpp"
#include <Eigen/Dense>

class OutputSimulation : public Simulation
{
    public:
    explicit OutputSimulation(const std::string& config_filename);

    protected:
    explicit OutputSimulation();

    public:
    virtual std::string type() const override { return "OutputSimulation"; }
    virtual void printInfo() const = 0;

    protected:
    virtual void _timeStep() override;

    virtual void _init() override;

    
    double _print_interval_s;
    double _last_print_sim_time;

    mutable std::ofstream _out_file;
};

#endif // __OUTPUT_SIMULATION_HPP