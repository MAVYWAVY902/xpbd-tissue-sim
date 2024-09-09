#ifndef __TISSUE_GRASPING_SIMULATION_HPP
#define __TISSUE_GRASPING_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include <Eigen/Dense>

class TissueGraspingSimulation : public OutputSimulation
{
    public:

    explicit TissueGraspingSimulation(const std::string& config_filename);

    virtual void setup() override;

    virtual void printInfo() const override;
};

#endif // __TISSUE_GRASPING_SIMULATION_HPP