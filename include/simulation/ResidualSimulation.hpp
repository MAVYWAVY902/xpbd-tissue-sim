#ifndef __RESIDUAL_SIMULATION_HPP
#define __RESIDUAL_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include <Eigen/Dense>

class ResidualSimulation : public OutputSimulation
{
    public:

    explicit ResidualSimulation(const std::string& config_filename);

    virtual void setup() override;

    void printInfo() const override;
};

#endif // __RESIDUAL_SIMULATION_HPP