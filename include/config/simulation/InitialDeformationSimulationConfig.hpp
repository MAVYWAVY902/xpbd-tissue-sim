#ifndef __INITIAL_DEFORMATION_SIMULATION_CONFIG_HPP
#define __INITIAL_DEFORMATION_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

#include "simulation/InitialDeformationSimulation.hpp"

namespace Config
{

class InitialDeformationSimulationConfig : public SimulationConfig
{
    /** Static predifined options for the simulation mode. Maps strings to the Simulation mode enum. */
    static std::map<std::string, Sim::InitialDeformationSimulation::DeformationType> DEFORMATION_TYPE_OPTIONS()
    {
        static std::map<std::string, Sim::InitialDeformationSimulation::DeformationType> deformation_type_options{
            {"volumetric-expansion", Sim::InitialDeformationSimulation::DeformationType::VOLUMETRIC_EXPANSION},
            {"volumetric-compression", Sim::InitialDeformationSimulation::DeformationType::VOLUMETRIC_COMPRESSION},
            {"collapse-to-plane", Sim::InitialDeformationSimulation::DeformationType::COLLAPSE_TO_PLANE},
            {"scramble", Sim::InitialDeformationSimulation::DeformationType::SCRAMBLE}};
        return deformation_type_options;
    }

    public:
    explicit InitialDeformationSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        // extract the parameters from the config file
        _extractParameterWithOptions("deformation-type", node,_deformation_type, DEFORMATION_TYPE_OPTIONS());
        _extractParameter("deformation-factor", node, _deformation_factor);
    }

    // getters
    Real deformationFactor() const { return _deformation_factor.value; }
    Sim::InitialDeformationSimulation::DeformationType deformationType() const { return _deformation_type.value; }

    protected:
    /** The scale factor of the initial deformation.
     * I.e. for a deformation type of "volumetric-expansion" and a deformation size of 2, this means a volumetric expansion to two times the volume.
     */
    ConfigParameter<Real> _deformation_factor = ConfigParameter<Real>(1);

    /** The type of initial deformation to perform. */
    ConfigParameter<Sim::InitialDeformationSimulation::DeformationType> _deformation_type 
        = ConfigParameter<Sim::InitialDeformationSimulation::DeformationType>(Sim::InitialDeformationSimulation::DeformationType::VOLUMETRIC_EXPANSION);
};

} // namespace Config

#endif // __INITIAL_DEFORMATION_SIMULATION_CONFIG_HPP