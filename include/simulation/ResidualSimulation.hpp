#ifndef __RESIDUAL_SIMULATION_HPP
#define __RESIDUAL_SIMULATION_HPP

#include "simulation/OutputSimulation.hpp"
#include "config/simulation/ResidualSimulationConfig.hpp"
#include "common/types.hpp"

namespace Sim
{

class ResidualSimulation : public OutputSimulation
{
    public:

    explicit ResidualSimulation(const Config::ResidualSimulationConfig* config);
    
    virtual std::string type() const override { return "ResidualSimulation"; }

    virtual void setup() override;

    void printInfo() const override;
};

} // namespace Sim

#endif // __RESIDUAL_SIMULATION_HPP