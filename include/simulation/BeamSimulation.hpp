#ifndef __BEAM_SIMULATION_HPP
#define __BEAM_SIMULATION_HPP

#include "OutputSimulation.hpp"
#include "common/types.hpp"

namespace Sim
{

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
    std::vector<Vec3r> _beams_tip_start;
};

} // namespace Sim

#endif // __BEAM_SIMULATION_HPP