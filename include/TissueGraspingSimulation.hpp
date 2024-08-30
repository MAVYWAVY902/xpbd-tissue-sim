#ifndef __TISSUE_GRASPING_SIMULATION_HPP
#define __TISSUE_GRASPING_SIMULATION_HPP

#include "Simulation.hpp"
#include <Eigen/Dense>

class TissueGraspingSimulation : public Simulation
{
    public:

    explicit TissueGraspingSimulation(const std::string& config_filename);

    virtual void setup() override;

    virtual void update() override;

    protected:
    virtual void _timeStep() override;

    void _printInfo();

    double _last_print_sim_time;

    std::ofstream _out_file;
};

#endif // __TISSUE_GRASPING_SIMULATION_HPP