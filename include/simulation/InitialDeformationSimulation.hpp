#ifndef __INITIAL_DEFORMATION_SIMULATION_HPP
#define __INITIAL_DEFORMATION_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

namespace Config
{
    class InitialDeformationSimulationConfig;
}

namespace Sim
{

class InitialDeformationSimulation : public Simulation
{
public:
    enum class DeformationType
    {
        VOLUMETRIC_EXPANSION=0,
        VOLUMETRIC_COMPRESSION,
        COLLAPSE_TO_PLANE,
        SCRAMBLE
    };

    explicit InitialDeformationSimulation(const Config::InitialDeformationSimulationConfig* config);

    std::string deformationType() const;
    virtual std::string type() const override { return "InitialDeformationSimulation"; }

    virtual void setup() override;

    virtual void notifyKeyPressed(SimulationInput::Key key, SimulationInput::KeyAction action, int modifiers) override;
protected:
    virtual void _timeStep() override;

protected:
    /** The type of initial deformation to apply. */
    DeformationType _deformation_type;

    /** The magnitude of applied deformation. Applicable for volumetric expansion and volumetric compression. */
    Real _deformation_factor;

    /** The objects that we will apply initial deformation to. */
    std::vector<XPBDMeshObject_BasePtrWrapper> _xpbd_objs;

    /** The whether or not the simulation has been started.
     * The user presses a key to start the simulation.
     */
    bool _simulation_started = false;
};

} // namespace Sim

#endif // __INITIAL_DEFORMATION_SIMULATION_HPP