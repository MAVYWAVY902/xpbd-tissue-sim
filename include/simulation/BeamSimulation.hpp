#ifndef __BEAM_SIMULATION_HPP
#define __BEAM_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include <Eigen/Dense>

namespace Sim
{

class BeamSimulation : public OutputSimulation
{
    public:

    explicit BeamSimulation(const std::string& config_filename);

    virtual std::string type() const override { return "BeamSimulation"; }

    virtual void setup() override;

    virtual void printInfo() const;

    std::vector<unsigned> _beams_tip_vertex;
    std::vector<Eigen::Vector3d> _beams_tip_start;
};

} // namespace Sim

#endif // __BEAM_SIMULATION_HPP