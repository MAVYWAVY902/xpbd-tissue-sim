#ifndef __RIGID_OBJECT_CONFIG_HPP
#define __RIGID_OBJECT_CONFIG_HPP

#include "config/ObjectConfig.hpp"

class RigidObjectConfig : public ObjectConfig
{
    public:
    static std::optional<Eigen::Vector3d>& DEFAULT_ANGULAR_VELOCITY() { static std::optional<Eigen::Vector3d> ang_vel({0,0,0}); return ang_vel; }
    static std::optional<double>& DEFAULT_DENSITY() { static std::optional<double> density(1000); return density; }
    static std::optional<bool>& DEFAULT_FIXED() { static std::optional<bool> fixed(false); return fixed; }

    explicit RigidObjectConfig(const YAML::Node& node)
        : ObjectConfig(node)
    {
        // extract parameters from Config
        _extractParameter("angular-velocity", node, _initial_ang_velocity, DEFAULT_ANGULAR_VELOCITY());
        _extractParameter("density", node, _density, DEFAULT_DENSITY());
        _extractParameter("fixed", node, _fixed, DEFAULT_FIXED());
    }

    Eigen::Vector3d initialAngularVelocity() const { return _initial_ang_velocity.value.value(); }
    double density() const { return _density.value.value(); }
    bool fixed() const { return _fixed.value.value(); }

    protected:
    ConfigParameter<Eigen::Vector3d> _initial_ang_velocity;
    ConfigParameter<double> _density;
    ConfigParameter<bool> _fixed;

};

#endif // __RIGID_OBJECT_CONFIG_HPP