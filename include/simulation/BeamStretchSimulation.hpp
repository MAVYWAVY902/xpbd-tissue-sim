#ifndef __BEAM_STRETCH_SIMULATION_HPP
#define __BEAM_STRETCH_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include <Eigen/Dense>

class BeamStretchSimulation : public OutputSimulation
{
    public:

    explicit BeamStretchSimulation(const std::string& config_filename);

    virtual std::string toString() const override;
    virtual std::string type() const override { return "BeamStretchSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const override;

    double _stretch_velocity;
    double _stretch_time;
};

#endif // __BEAM_STRETCH_SIMULATION_HPP