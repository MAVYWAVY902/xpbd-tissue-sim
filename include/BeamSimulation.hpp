#ifndef __BEAM_SIMULATION_HPP
#define __BEAM_SIMULATION_HPP

#include "Simulation.hpp"
#include <Eigen/Dense>

class BeamSimulation : public Simulation
{
    public:

    explicit BeamSimulation(const std::string& config_filename);

    virtual void setup() override;

    virtual void update() override;

    protected:
    virtual void _timeStep() override;

    void _printInfo();

    unsigned _beam_tip_vertex;
    Eigen::Vector3d _beam_tip_start;
    double _last_print_sim_time;

    std::ofstream _out_file;
};

#endif // __BEAM_SIMULATION_HPP