#ifndef __GRASPING_SIMULATION_CONFIG_HPP
#define __GRASPING_SIMULATION_CONFIG_HPP

#include "config/simulation/SimulationConfig.hpp"

namespace Config
{

class GraspingSimulationConfig : public SimulationConfig
{
    static std::optional<Real>& DEFAULT_GRASP_RADIUS() { static std::optional<Real> rad(0.1); return rad; }

    public:
    explicit GraspingSimulationConfig(const YAML::Node& node)
        : SimulationConfig(node)
    {
        _extractParameter("grasp-radius", node, _grasp_radius, DEFAULT_GRASP_RADIUS());
    }

    Real graspRadius() const { return _grasp_radius.value.value(); }

    protected:
    ConfigParameter<Real> _grasp_radius;
};

} // namespace Config

#endif // __GRASPING_SIMULATION_CONFIG_HPP