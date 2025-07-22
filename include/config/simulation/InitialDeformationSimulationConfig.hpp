#ifndef __INITIAL_DEFORMATION_SIMULATION_CONFIG_HPP
#define __INITIAL_DEFORMATION_SIMULATION_CONFIG_HPP

#include "config/simulationOutputSimulationConfig.hpp"

namespace Config
{

enum class DeformationType
{
    VOLUMETRIC_EXPANSION=0,
    VOLUMETRIC_COMPRESSION,
    COLLAPSE_TO_PLANE
};

class InitialDeformationSimulationConfig : public OutputSimulationConfig
{
    /** Static predifined options for the simulation mode. Maps strings to the Simulation mode enum. */
    static std::map<std::string, DeformationType> DEFORMATION_TYPE_OPTIONS()
    {
        static std::map<std::string, DeformationType> deformation_type_options{{"Volumetric-Expansion", DeformationType::VOLUMETRIC_EXPANSION},
                                                                               {"Volumetric-Compression", DeformationType::VOLUMETRIC_COMPRESSION},
                                                                               {"Collapse-To-Plane", DeformationType::COLLAPSE_TO_PLANE}};
        return deformation_type_options;
    }

    public:
    explicit InitialDeformationSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {
        // extract the parameters from the config file
        _extractParameterWithOptions("deformation-type", node,_deformation_type, DEFORMATION_TYPE_OPTIONS());
        _extractParameter("deformation-factor", node, _deformation_factor);
    }

    // getters
    Real deformationFactor() const { return _deformation_factor.value; }
    DeformationType deformationType() const { return _deformation_type.value; }

    protected:
    /** The scale factor of the initial deformation.
     * I.e. for a deformation type of "volumetric-expansion" and a deformation size of 2, this means a volumetric expansion to two times the volume.
     */
    ConfigParameter<Real> _deformation_factor = ConfigParameter<Real>(1);

    /** The type of initial deformation to perform. */
    ConfigParameter<DeformationType> _deformation_type = ConfigParameter<DeformationType>(DeformationType::VOLUMETRIC_EXPANSION);
};

} // namespace Config

#endif // __INITIAL_DEFORMATION_SIMULATION_CONFIG_HPP