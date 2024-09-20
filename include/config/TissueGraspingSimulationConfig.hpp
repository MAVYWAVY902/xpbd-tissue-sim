#ifndef __TISSUE_GRASPING_SIMULATION_CONFIG_HPP
#define __TISSUE_GRASPING_SIMULATION_CONFIG_HPP

#include "OutputSimulationConfig.hpp"

class TissueGraspingSimulationConfig : public OutputSimulationConfig
{
    /** Static predefined default for simulation time step */
    static std::optional<double>& DEFAULT_GRASP_SIZE() { static std::optional<double> grasp_size(0.001); return grasp_size; }
    /** Static predefined efault for stretch time */
    static std::optional<double>& DEFAULT_Z_SCALING() { static std::optional<double> z_scaling(0.001); return z_scaling; }

    public:
    explicit TissueGraspingSimulationConfig(const YAML::Node& node)
        : OutputSimulationConfig(node)
    {
        _extractParameter("grasp-size", node, _grasp_size, DEFAULT_GRASP_SIZE());
        _extractParameter("z-scaling", node, _z_scaling, DEFAULT_Z_SCALING());
    }

    std::optional<double> graspSize() const { return _grasp_size.value; }
    std::optional<double> zScaling() const { return _z_scaling.value; }

    protected:
    ConfigParameter<double> _grasp_size; // in m
    ConfigParameter<double> _z_scaling;
};

#endif // __TISSUE_GRASPING_SIMULATION_CONFIG_HPP