#ifndef __RESIDUAL_SIMULATION_HPP
#define __RESIDUAL_SIMULATION_HPP

#include "Simulation.hpp"
#include <Eigen/Dense>

class ResidualSimulation : public Simulation
{
    public:

    explicit ResidualSimulation(const std::string& config_filename);

    virtual void setup() override;

    virtual void update() override;

    protected:
    virtual void _timeStep() override;

    void _printInfo();

    double _last_print_sim_time;

    std::ofstream _out_file;
};

#endif // __RESIDUAL_SIMULATION_HPP