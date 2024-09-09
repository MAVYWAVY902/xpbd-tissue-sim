#ifndef __BEAM_SIMULATION_HPP
#define __BEAM_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include <Eigen/Dense>

class BeamSimulation : public OutputSimulation
{
    public:

    explicit BeamSimulation(const std::string& config_filename);

    virtual void setup() override;

    virtual void printInfo() const;

    unsigned _beam_tip_vertex;
    Eigen::Vector3d _beam_tip_start;
};

#endif // __BEAM_SIMULATION_HPP