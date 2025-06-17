#ifndef __GRASPING_SIMULATION_CONFIG_HPP
#define __GRASPING_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class GraspingSimulationConfig : public SimulationConfig
{
    static std::optional<Real>& DEFAULT_GRASP_RADIUS() { static std::optional<Real> rad(0.1); return rad; }
    static std::optional<bool>& DEFAULT_FIX_MIN_Z() { static std::optional<bool> fix(true); return fix; }

    public:
    explicit GraspingSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("grasp-radius", node, _grasp_radius, DEFAULT_GRASP_RADIUS());
        _extractParameter("fix-min-z", node, _fix_min_z, DEFAULT_FIX_MIN_Z());
        
    }

    Real graspRadius() const { return _grasp_radius.value.value(); }
    bool fixMinZ() const { return _fix_min_z.value.value(); }

    protected:
    ConfigParameter<Real> _grasp_radius;
    ConfigParameter<bool> _fix_min_z;
};

} // namespace Config

#endif // __GRASPING_SIMULATION_CONFIG_HPP