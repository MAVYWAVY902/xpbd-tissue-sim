#ifndef __BEAM_STRETCH_SIMULATION_HPP
#define __BEAM_STRETCH_SIMULATION_HPP

#include "Simulation.hpp"
#include <Eigen/Dense>

class BeamStretchSimulation : public Simulation
{
    public:

    explicit BeamStretchSimulation(const std::string& config_filename);

    virtual void setup() override;

    virtual void update() override;

    protected:
    virtual void _timeStep() override;

    void _printInfo();

    double _last_print_sim_time;

    double _stretch_velocity;
    double _stretch_time;

    std::ofstream _out_file;
};

#endif // __BEAM_STRETCH_SIMULATION_HPP