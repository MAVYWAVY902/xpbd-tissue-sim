#ifndef __GRASPING_SIMULATION_CONFIG_HPP
#define __GRASPING_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class GraspingSimulationConfig : public SimulationConfig
{

    public:
    explicit GraspingSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("grasp-radius", node, _grasp_radius);
        _extractParameter("fix-min-z", node, _fix_min_z);
        
    }

    Real graspRadius() const { return _grasp_radius.value; }
    bool fixMinZ() const { return _fix_min_z.value; }

    protected:
    ConfigParameter<Real> _grasp_radius = ConfigParameter<Real>(0.1);
    ConfigParameter<bool> _fix_min_z = ConfigParameter<bool>(true);
};

} // namespace Config

#endif // __GRASPING_SIMULATION_CONFIG_HPP