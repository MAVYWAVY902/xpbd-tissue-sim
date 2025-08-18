#ifndef __INITIAL_DEFORMATION_SIMULATION_HPP
#define __INITIAL_DEFORMATION_SIMULATION_HPP

#include "simulation/Simulation.hpp"
#include "simobject/XPBDMeshObjectBaseWrapper.hpp"

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
    virtual std::string toString() const override;
    virtual std::string type() const override { return "InitialDeformationSimulation"; }

    protected:
    DeformationType _deformation_type;
    Real _deformation_factor;

    /** The objects that we will apply initial deformation to. */
    std::vector<XPBDMeshObject_BasePtrWrapper> _xpbd_objs;
};

} // namespace Sim

#endif // __INITIAL_DEFORMATION_SIMULATION_HPP