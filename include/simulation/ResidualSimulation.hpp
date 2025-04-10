#ifndef __RESIDUAL_SIMULATION_HPP
#define __RESIDUAL_SIMULATION_HPP

#include "simulation/OutputSimulation.hpp"
#include "common/types.hpp"

namespace Sim
{

namespace Sim
{

class ResidualSimulation : public OutputSimulation
{
    public:

    explicit ResidualSimulation(const std::string& config_filename);
    
    virtual std::string type() const override { return "ResidualSimulation"; }

    virtual void setup() override;

    void printInfo() const override;
};

} // namespace Sim

#endif // __RESIDUAL_SIMULATION_HPP