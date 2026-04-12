#ifndef __BEAM_SIMULATION_HPP
#define __BEAM_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "config/simulation/BeamSimulationConfig.hpp"
#include "common/types.hpp"

namespace Sim
{

class BeamSimulation : public Simulation
{
public:

    explicit BeamSimulation(const Config::BeamSimulationConfig* config);

    virtual std::string type() const override { return "BeamSimulation"; }

    virtual void setup() override;

private:
    /** Keep track of which vertex we're taking as each beam's tips. */
    std::vector<unsigned> _beams_tip_vertex;
    /** Keep track of where each beam's tip started initially. */
    std::vector<Vec3r> _beams_tip_start;

    /** The deformable "beam" objects in the simulation. */
    std::vector<XPBDMeshObject_BasePtrWrapper> _xpbd_objs;
};

} // namespace Sim

#endif // __BEAM_SIMULATION_HPP